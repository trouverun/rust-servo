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
    use crate::{bsp::HallFeedback, calibration::{CalibrationInputs, CalibrationRunner, FailureCause, StageResult}, types::ButtonState};
    use crate::bsp::{self, Acceleration, AdcFeedback, Memory, PwmOutput};
    use crate::types::{OperatingMode, BoardStatus};
    use crate::boards::*;
    use crate::types::*;
    use crate::utils::PhaseCurrentFilter;
    use field_oriented::{
        AngleType, ConstantMotorParameters, ControllerParameters, FOC, FocConfig, FocInput, FocInputType, HasRotorFeedback, MotorParamEstimator, MotorParamsEstimate, PhaseValues, RotorFeedback, compute_current_pi_controller_gains
    };

    #[shared]
    struct Shared {
        mode: OperatingMode,
        board_status: BoardStatus,
        config: FirmwareConfig,
        runtime_values: RuntimeValues,
        hall_feedback: HallFeedback,
        pwm_output: PwmOutput,
        acceleration: Acceleration,
        memory: Memory,
        motor_parameters: ConstantMotorParameters,
        tuning_result: Option<Option<ControllerParameters>>,
        debug_mappings: DebugMappings,
        button_state: ButtonState,
        debug_data: Option<(f32, f32)>,
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        calibration: CalibrationRunner,
        foc: FOC,
        current_filter: PhaseCurrentFilter,
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
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings);
        let acceleration = bsp::Acceleration::new(accel_mappings);
        let memory = bsp::Memory::new(memory_mappings);

        let mut config = if let Some(firmware_config) = memory.read_firmware_config() {
            firmware_config
        } else {
            FirmwareConfig::default()
        };
        config.current_limit = 2.5;
        let current_filter = PhaseCurrentFilter::new(5000.0, config.current_limit);

        let calibration = CalibrationRunner::new();
        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.1
        };
        let mut foc = FOC::new(foc_cfg, PWM_FREQ.0 as f32);

        // Try to read calibrations and configurations from memory:
        let mut motor_parameters = if let Some(saved_parameters) = memory.read_motor_parameters() {
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

        // Testing:
        let mut hall_calibrations = [0.0; 6];
        hall_calibrations[0] = 3.64261;
        hall_calibrations[1] = 5.705433;
        hall_calibrations[2] = 4.6740837;
        hall_calibrations[3] = 1.523397;
        hall_calibrations[4] = 2.5618694;
        hall_calibrations[5] = 0.5235335;
        hall_feedback.set_calibration(hall_calibrations);
        motor_parameters.params.num_pole_pairs = Some(2);
        motor_parameters.params.stator_resistance = Some(0.48448467);
        motor_parameters.params.d_inductance = Some(0.0038107522);
        motor_parameters.params.q_inductance = Some(0.0038107522);
        motor_parameters.params.pm_flux_linkage = Some(0.012887827);
        let gains = compute_current_pi_controller_gains::<50>(motor_parameters.params, PWM_FREQ.0 as f32);
        info!("Gains {}", gains);
        foc.set_pi_gains(gains.unwrap());

        (Shared {
            mode: OperatingMode::Idle,
            board_status: BoardStatus {dc_bus_voltage: None, temperature: 0.0},
            config,
            runtime_values: RuntimeValues::default(),
            hall_feedback,
            pwm_output,
            acceleration,
            memory,
            motor_parameters,
            tuning_result: None,
            debug_mappings,
            button_state: ButtonState::Waiting,
            debug_data: Some((0.0, 0.0))
        },
        Local {
            adc_feedback,
            calibration,
            foc,
            current_filter
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
        local = [adc_feedback, calibration, foc, current_filter, debug_ctr: u32 = 0],
        shared = [
            hall_feedback, pwm_output, acceleration, memory, motor_parameters, tuning_result,
            mode, board_status, config, runtime_values, debug_mappings, debug_data
        ],
        priority = 3
    )]
    fn currents_adc_isr(mut cx: currents_adc_isr::Context) {
        let tuning_result: Option<Option<ControllerParameters>> = cx.shared.tuning_result.lock(|tr| tr.take());
        let mode = cx.shared.mode.lock(|mode| { 
            if matches!(mode, OperatingMode::Tuning) {
                if let Some(result) = tuning_result {
                    match result {
                        Some(pi_gains) => {
                            cx.local.foc.set_pi_gains(pi_gains);
                            *mode = mode.on_command(Command::FinishTuning);
                        }
                        None => {
                            *mode = mode.on_command(Command::Fault);
                        }
                    }    
                }
            }
            *mode
        });
        let calibrating = matches!(mode, OperatingMode::Calibration);
        let active = calibrating || matches!(mode, OperatingMode::TorqueControl | OperatingMode::VelocityControl);
        
        cx.shared.debug_mappings.lock(|dm| dm.la_pin.set_high());

        // ADC injected EOC (FOC ISR):
        let mut voltage_hexagon_sector = 0;
        if let Some(phase_currents) = cx.local.adc_feedback.read_currents() {
            cx.local.current_filter.update(phase_currents);
            let overcurrent = cx.local.current_filter.overcurrent();
            if overcurrent && !matches!(mode, OperatingMode::Fault) {
                cx.shared.mode.lock(|mode| *mode = mode.on_command(Command::Fault));
                info!("Software overcurrent!");
            }
            let dc_bus_reading = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage);
            let (rotor_feedback, hall_pattern) = cx.shared.hall_feedback.lock(|hf| {
                (hf.read(), hf.get_pattern())
            });
            if !active || overcurrent || rotor_feedback.is_err() || dc_bus_reading.is_none() {
                // TODO: need to come here also if: 
                // motor params not fully populated OR
                // PI gains invalid OR
                cx.shared.pwm_output.lock(|pwm| {
                    pwm.set_duty_cycles(PhaseValues { u: 0.5, v: 0.5, w: 0.5 })
                });
                cx.local.foc.reset();
            } else if let (
                Ok(RotorFeedback { theta, omega }), Some(dc_bus_voltage)
            ) = (rotor_feedback, dc_bus_reading) {
                cx.shared.motor_parameters.lock(|active_params| {
                    // Determine FOC inputs based on operating mode:
                    let mut estimator: &mut dyn MotorParamEstimator = active_params;
                    let motor_params = estimator.get_estimate();
                    let mut calibration_result = None;
                    let (theta, omega, foc_command) = if calibrating {
                        let (target_voltage, target_current, target_omega) = cx.shared.config.lock(|cfg| {
                            (cfg.calibration_voltage, cfg.calibration_current, cfg.calibration_omega)
                        });
                        let inputs = CalibrationInputs {
                            dc_bus_voltage,
                            theta, hall_pattern,
                            num_pole_pairs: active_params.get_estimate().num_pole_pairs,
                            target_voltage, target_current, target_omega
                        };
                        let (output, stage_result) = cx.local.calibration.tick(inputs);
                        estimator = cx.local.calibration.get_estimator();
                        calibration_result = stage_result;
                        (output.theta, omega, output.foc_command)
                    } else {
                        let torque = cx.shared.runtime_values.lock(|rtv| rtv.target_torque);
                        (theta, omega, FocInputType::TargetTorque(torque))
                    };

                    // FOC computations:
                    let foc_input = FocInput { 
                        command: foc_command, dc_bus_voltage, 
                        angle_type: AngleType::Electrical,
                        theta,
                        omega,
                        phase_currents 
                    };
                    let foc_result = cx.shared.acceleration.lock(|accelerator| {
                        cx.local.foc.compute(foc_input, motor_params, accelerator)
                    });

                    let mut i_q = 0.0;
                    cx.shared.pwm_output.lock(|pwm| {
                    match foc_result {
                        Ok(result) => {
                            pwm.set_duty_cycles(result.duty_cycles);
                            estimator.after_foc_iteration(result);
                            voltage_hexagon_sector = result.voltage_hexagon_sector;
                            i_q = result.measured_i_dq.q;
                        }
                        Err(e) => {
                            pwm.set_duty_cycles(PhaseValues { u: 0.0, v: 0.0, w: 0.0 });
                            info!("FOC fault!");
                            // Move to fault or idle state
                        }
                    }});

                    *cx.local.debug_ctr += 1;
                    if *cx.local.debug_ctr >= 15000 {
                        *cx.local.debug_ctr = 0;
                        cx.shared.debug_data.lock(|v| if v.is_none() { *v = Some((omega, i_q)) });
                    }
                        
                    // Apply calibration stage results:
                    match calibration_result {
                        Some(StageResult::HallCalibration { angle_table }) => {
                            info!("Angle table {}", angle_table);
                            cx.shared.hall_feedback.lock(|hf| hf.set_calibration(angle_table));
                            cx.local.foc.reset();
                        }
                        Some(StageResult::ResetRequest) => {
                            cx.local.foc.reset();
                        }
                        Some(StageResult::TuningRequest) => {
                            let _ = tune_pi::spawn(estimator.get_estimate());
                            cx.shared.mode.lock(|mode: &mut OperatingMode| *mode = mode.on_command(Command::StartTuning));
                        }
                        Some(StageResult::MotorParameters { motor_params }) => {
                            info!("Params {}", motor_params);
                            active_params.copy_other(motor_params);
                            cx.local.foc.reset();
                            cx.shared.mode.lock(|mode| *mode = mode.on_command(Command::FinishCalibration));
                        }
                        Some(StageResult::Failure { cause }) => {
                            info!("Calibration failure! ({})", cause);
                            match cause {
                                FailureCause::MotorParameterEstimation { fault } => {
                                    info!("Root cause {}", fault);
                                },
                                _ => {}
                            }
                            cx.shared.mode.lock(|mode| *mode = mode.on_command(Command::Fault));
                        }
                        _ => {} // Calibration step still in progress
                    }
                });
            }

            // Always sample something to keep the ADC EOC ISRs running:
            cx.local.adc_feedback.sample_sector(voltage_hexagon_sector);

        }
        
        // ADC regular EOC:
        if let Some((vbus, tboard)) = cx.local.adc_feedback.read_board_info() {
            cx.shared.board_status.lock(|bs| {
                bs.dc_bus_voltage = Some(vbus);
                bs.temperature = tboard;
            });
        }


        cx.shared.debug_mappings.lock(|dm| dm.la_pin.set_low());
    }

    #[task(priority = 1, shared = [tuning_result])]
    async fn tune_pi(mut cx: tune_pi::Context, estimate: MotorParamsEstimate) {
        info!("STARTED TUNING TASK");
        let result = compute_current_pi_controller_gains::<50>(estimate, PWM_FREQ.0 as f32);
        info!("PI gains {}", result);
        cx.shared.tuning_result.lock(|r| *r = Some(result));
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

    #[idle(shared=[debug_mappings, debug_data])]
    fn idle(mut cx: idle::Context) -> ! {
        loop { 
            let debug = cx.shared.debug_data.lock(|v| v.take());
            if let Some((omega, i_d)) = debug {
                info!("omega {}, i_q {}", omega, i_d);
            }
            cortex_m::asm::wfi();
        }
    }

    // ---------------------- Debug user inputs: -----------------------------------
    #[task(binds = EXTI15_10, shared=[debug_mappings, button_state, acceleration], priority = 1)]
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
