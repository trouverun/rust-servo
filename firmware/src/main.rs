#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use servo_firmware as _;
mod bsp;
mod boards;
mod types;
mod calibration;
mod utils;

pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use defmt::{info};
    use {defmt_rtt as _};
    use crate::{bsp::HallFeedback, calibration::{CalibrationInputs, CalibrationRunner, StageResult}, types::ButtonState};
    use crate::bsp::{self, Acceleration, AdcFeedback, Memory, PwmOutput};
    use crate::types::{OperatingMode, BoardStatus};
    use crate::boards::*;
    use crate::types::*;
    use crate::utils::PhaseCurrentFilter;
    use field_oriented::{
        AngleType, ConstantMotorParameters, FOC, FocConfig, FocInput, FocInputType, HasRotorFeedback,
        MotorParamEstimator, MotorParamsEstimate, PhaseValues, RotorFeedback, compute_current_pi_controller_gains
    };

    #[shared]
    struct Shared {
        mode: OperatingMode,
        board_status: BoardStatus,
        config: FirmwareConfig,
        runtime_values: RuntimeValues,
        hall_feedback: HallFeedback,
        pwm_output: PwmOutput,
        memory: Memory,
        foc: FOC,
        motor_parameters: ConstantMotorParameters,
        debug_mappings: DebugMappings,
        button_state: ButtonState
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        current_filter: PhaseCurrentFilter,
        calibration: CalibrationRunner,
        acceleration: Acceleration,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let p = bsp::init();
        let (
            adc_mappings, 
            hall_mappings, 
            pwm_mappings,
            accel_mappings,
            memory_mappings,
            debug_mappings
        ) = map_peripherals(p);
        
        let pwm_output = bsp::PwmOutput::new(pwm_mappings);
        pwm_output.wait_break2_ready(); // Shows active low for first N cycles, wait it out
        let mut adc_feedback = bsp::AdcFeedback::new(adc_mappings);
        adc_feedback.sample_sector(0); // Kick off the ADC ISR loop
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings, 1000.0);
        let acceleration = bsp::Acceleration::new(accel_mappings);
        let memory = bsp::Memory::new(memory_mappings);

        let mut config = if let Some(firmware_config) = memory.read_firmware_config() {
            firmware_config
        } else {
            FirmwareConfig::default()
        };
        config.current_limit = 2.5;
        let current_filter = PhaseCurrentFilter::new(2500.0, config.current_limit);

        let calibration = CalibrationRunner::new();
        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.1
        };
        let mut foc = FOC::new(foc_cfg, PWM_FREQ.0 as f32);

        // Try to read calibrations and configurations from memory:
        let motor_parameters = if let Some(saved_parameters) = memory.read_motor_parameters() {
            ConstantMotorParameters::from_other(saved_parameters)
        } else {
            ConstantMotorParameters {
                params: MotorParamsEstimate::new_empty()
            }
        };
        if let Some(hall_calibrations) = memory.read_hall_calibrations() {
            hall_feedback.set_calibration(hall_calibrations);
        }
        if let Some(controller_parameters) = memory.read_controller_parameters() {
            foc.set_pi_gains(controller_parameters);
        }

        (Shared {
            mode: OperatingMode::Idle,
            board_status: BoardStatus {dc_bus_voltage: None, temperature: None},
            config,
            runtime_values: RuntimeValues::default(),
            hall_feedback,
            pwm_output,
            memory,
            foc,
            motor_parameters,
            debug_mappings,
            button_state: ButtonState::Waiting,
        },
        Local {
            adc_feedback,
            current_filter,
            calibration,
            acceleration,
        })
    }

    #[task(binds = TIM3, shared = [hall_feedback], priority = 2)]
    fn on_tim3(mut cx: on_tim3::Context) {
        cx.shared.hall_feedback.lock(|hall_feedback| {
            hall_feedback.on_interrupt();
        });
    }

    #[task(
        binds = ADC3, 
        local = [adc_feedback, current_filter, calibration, acceleration],
        shared = [
            hall_feedback, pwm_output, memory, foc, motor_parameters,
            mode, board_status, config, runtime_values, debug_mappings
        ],
        priority = 3
    )]
    fn currents_adc_isr(mut cx: currents_adc_isr::Context) {
        let mode = cx.shared.mode.lock(|mode| *mode);
        let calibrating = matches!(mode, OperatingMode::Calibration);
        let active = calibrating || matches!(mode, OperatingMode::TorqueControl | OperatingMode::VelocityControl);
        let mut voltage_hexagon_sector = 0;

        cx.shared.debug_mappings.lock(|dm| dm.la_pin.set_high());

        // if FOC ISR (sampled phase currents):
        if let Some(phase_currents) = cx.local.adc_feedback.read_currents() {

            // Check for overcurrent:
            cx.local.current_filter.update(phase_currents);
            let overcurrent = cx.local.current_filter.check_overcurrent();
            if overcurrent {
                cx.shared.mode.lock(|mode| {
                     *mode = mode.on_command(Command::AssertFault { cause: FaultCause::Overcurrent } )
                });
            }

            let dc_bus_reading = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage);
            let (rotor_feedback, hall_pattern) = cx.shared.hall_feedback.lock(|hf| {
                (hf.read(), hf.get_pattern())
            });
            
            // Inactive or blocked from running, set idle outputs:
            if !active || overcurrent || rotor_feedback.is_err() || dc_bus_reading.is_none() {
                // TODO: need to come here also if: 
                // PI gains not configured
                // motor params not fully populated
                cx.shared.pwm_output.lock(|pwm| {
                    pwm.set_duty_cycles(PhaseValues { u: 0.5, v: 0.5, w: 0.5 })
                });
            } else {
                let RotorFeedback { theta, omega } = rotor_feedback.ok().expect("Already checked for Err above");
                let dc_bus_voltage = dc_bus_reading.expect("Already checked for None above");
                let mut calibration_result = None;

                cx.shared.motor_parameters.lock(|active_params| {
                    let mut estimator: &mut dyn MotorParamEstimator = active_params;

                    // Determine FOC inputs based on operating mode (calibration or normal):
                    let (theta, foc_command) = if calibrating {
                        let (target_voltage, target_current, target_omega) = cx.shared.config.lock(|cfg| {
                            (cfg.calibration_voltage, cfg.calibration_current, cfg.calibration_omega)
                        });
                        let inputs = CalibrationInputs {
                            dc_bus_voltage,
                            theta, hall_pattern,
                            num_pole_pairs: active_params.get_estimate().num_pole_pairs,
                            target_voltage, target_current, target_omega
                        };
                        let (output, stage_result) = cx.local.calibration.step(inputs);
                        estimator = cx.local.calibration.get_estimator();
                        calibration_result = stage_result;
                        (output.theta, output.foc_command)
                    } else {
                        let torque = cx.shared.runtime_values.lock(|rtv| rtv.target_torque);
                        (theta, FocInputType::TargetTorque(torque))
                    };

                    // Do the FOC computations:
                    let foc_input = FocInput { 
                        command: foc_command, dc_bus_voltage, 
                        angle_type: AngleType::Electrical,
                        theta, omega, phase_currents 
                    };
                    let foc_result = cx.shared.foc.lock(|foc| {
                        foc.compute(foc_input, estimator.get_estimate(), cx.local.acceleration)
                    });

                    // Apply the computed PWM duty cycles:
                    cx.shared.pwm_output.lock(|pwm| {
                        match foc_result {
                            Ok(result) => {
                                pwm.set_duty_cycles(result.duty_cycles);
                                estimator.after_foc_iteration(result);
                                voltage_hexagon_sector = result.voltage_hexagon_sector;
                            }
                            Err(fault) => {
                                pwm.set_duty_cycles(PhaseValues { u: 0.0, v: 0.0, w: 0.0 });
                                cx.shared.mode.lock(|mode: &mut OperatingMode| {
                                    *mode = mode.on_command(Command::AssertFault { cause: fault.into() })
                                });
                            }
                        }
                    });
                });
                        
                // Process calibration stage results:
                if calibration_result.is_some() {
                    cx.shared.foc.lock(|foc| {
                        match calibration_result {
                            Some(StageResult::HallCalibration { angle_table }) => {
                                info!("Angle table {}", angle_table);
                                cx.shared.hall_feedback.lock(|hf| hf.set_calibration(angle_table));
                                foc.clear_windup();
                            }
                            Some(StageResult::ResetRequest) => {
                                foc.clear_windup();
                            }
                            Some(StageResult::TuningRequest) => {
                                // Tune controllers in a separate task to avoid locking up a high priority ISR:
                                let _ = tune_pi::spawn(cx.local.calibration.get_estimator().get_estimate());
                                cx.shared.mode.lock(|mode: &mut OperatingMode| *mode = mode.on_command(Command::StartTuning));
                            }
                            Some(StageResult::MotorParameters { motor_params }) => {
                                info!("Params {}", motor_params);
                                cx.shared.motor_parameters.lock(|active_params| {
                                    active_params.copy_other(motor_params);
                                });
                                cx.shared.mode.lock(|mode| *mode = mode.on_command(Command::FinishCalibration));
                                foc.clear_windup();
                            }
                            Some(StageResult::Failure { cause }) => {
                                cx.shared.mode.lock(|mode: &mut OperatingMode| {
                                    *mode = mode.on_command(Command::AssertFault { cause: cause.into() })
                                });                       
                                foc.clear_windup();
                            }
                            _ => {} // Calibration step still in progress
                        }
                    });
                }
            }

            // Always sample something to keep the ADC EOC ISRs running:
            cx.local.adc_feedback.sample_sector(voltage_hexagon_sector);
        }
        
        // if board status ISR (sampled DC bus voltage and board temperature):
        if let Some((vbus, tboard)) = cx.local.adc_feedback.read_board_info() {
            cx.shared.board_status.lock(|bs| {
                bs.dc_bus_voltage = Some(vbus);
                bs.temperature = Some(tboard);
            });
        }

        cx.shared.debug_mappings.lock(|dm| dm.la_pin.set_low());
    }

    #[task(priority = 1, shared = [mode, foc])]
    async fn tune_pi(mut cx: tune_pi::Context, estimate: MotorParamsEstimate) {
        let result = compute_current_pi_controller_gains::<50>(estimate, PWM_FREQ.0 as f32);
        info!("PI gains {}", result);
        match result {
            Some(pi_gains) => {
                cx.shared.foc.lock(|foc| {
                    foc.set_pi_gains(pi_gains);
                    foc.clear_windup();
                });
                cx.shared.mode.lock(|mode| {
                    *mode = mode.on_command(Command::FinishTuning);
                });
            }
            None => {
                cx.shared.mode.lock(|mode| {
                    *mode = mode.on_command(Command::AssertFault { cause: FaultCause::ControllerTuningFail });
                });
            }
        }    
    }

    #[task(binds = TIM8_BRK, shared = [pwm_output], priority = 1)]
    fn on_tim8(mut cx: on_tim8::Context) {
         let bk1_cleared = cx.shared.pwm_output.lock(|pwm| {
            pwm.check_break1()
        });
        let bk2_cleared = cx.shared.pwm_output.lock(|pwm| {
            pwm.check_break2()
        });

        if bk1_cleared {
            info!("Break1 triggered!");
        }
        if bk2_cleared {
            info!("Break2 triggered!");
        }
    }

    #[idle(shared=[debug_mappings])]
    fn idle(mut cx: idle::Context) -> ! {
        loop { 
            cortex_m::asm::wfi();
        }
    }

    // ---------------------- Debug user inputs: -----------------------------------
    #[task(binds = EXTI15_10, shared=[debug_mappings, button_state], priority = 1)]
    fn on_exti15_10(mut cx: on_exti15_10::Context) {
        let edge = cx.shared.debug_mappings.lock(|dm| {
            dm.user_btn.clear_pending();
            if dm.user_btn.is_low() { EdgeType::Rising } else { EdgeType::Falling }
        });
        cx.shared.button_state.lock(|bs| {
            *bs = bs.on_edge(edge);
        });
        if matches!(edge, EdgeType::Rising) {
            cx.shared.debug_mappings.lock(|dm| {
                dm.btn_timer.start();
            });
        }
    }

    #[task(binds = TIM2, shared=[debug_mappings, button_state, runtime_values, mode], priority = 2)]
    fn on_tim2(mut cx: on_tim2::Context) {
        let press_type = cx.shared.button_state.lock(|bs| *bs);
        cx.shared.mode.lock(|mode| {
            match press_type {
                ButtonState::ShortPress => {
                    *mode = mode.on_command(Command::Idle)
                }
                ButtonState::DoublePress => {
                    *mode = mode.on_command(Command::EnableTorqueControl);
                    info!("Starting torque ctrl");
                }
                ButtonState::LongPress => {
                    *mode = mode.on_command(Command::StartCalibration);
                    info!("Starting calibration");
                }
                _ => {}
            }
        });
        cx.shared.debug_mappings.lock(|dm| {
            dm.btn_timer.clear_update_interrupt();
            dm.btn_timer.stop();
            dm.btn_timer.reset();
        });
        cx.shared.button_state.lock(|bs| *bs = ButtonState::Waiting);
    }

    const POT_RECIPROCAL: f32 = 0.05 / ((1 << 12) - 1) as f32;
    #[task(binds = ADC1_2, shared=[debug_mappings, runtime_values], priority = 1)]
    fn on_adc12(mut cx: on_adc12::Context) {
        let adc_reading = cx.shared.debug_mappings.lock(|dm| {
            if dm.pot_adc.check_eoc() {
                Some((dm.pot_adc.read() as f32 * POT_RECIPROCAL))
            } else {
                None
            }
        });

        if let Some(val) = adc_reading {
            cx.shared.runtime_values.lock(|rc| {
                rc.target_torque = val;
            });
        }
    }
}
