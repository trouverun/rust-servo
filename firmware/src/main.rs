#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use servo_firmware as _;
mod boards;
mod bsp;
mod calibration;
mod types;
mod utils;
pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [SPI2, SPI3])]
mod app {
    use crate::boards::*;
    use crate::bsp::{
        self, Acceleration, AdcFeedback, AmtEncoder, HallFeedback, Memory,
        PwmOutput,
    };
    use crate::calibration::StageResult;
    use crate::types::*;
    use crate::types::{BoardStatus, OperatingMode};
    use crate::utils::PhaseCurrentFilter;
    use crate::{
        calibration::{CalibrationInputs},
        types::ButtonState,
    };
    use core::ops::Not;
    use defmt::info;
    use defmt_rtt as _;
    use embassy_stm32::{peripherals::TIM2, rcc};
    use field_oriented::{
        compute_current_pi_controller_gains, AngleType, ConstantMotorParameters, FocConfig,
        FocInput, FocInputType, HasRotorFeedback, MotorParamEstimator, MotorParamsEstimate,
        PhaseValues, RotorFeedback, FOC,
    };
    use rtic_monotonics::stm32::{ExtU64, Tim2 as Mono};

    #[shared]
    struct Shared {
        mode: OperatingMode,
        board_status: BoardStatus,
        config: FirmwareConfig,
        runtime_values: RuntimeValues,
        hall_feedback: HallFeedback,
        amt_encoder: AmtEncoder,
        pwm_output: PwmOutput,
        memory: Memory,
        foc: FOC,
        motor_parameters: ConstantMotorParameters,
        debug_mappings: DebugMappings,
        button_state: ButtonState,
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        current_filter: PhaseCurrentFilter,
        acceleration: Acceleration,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let p = bsp::init();
        let (
            adc_mappings,
            hall_mappings,
            encoder_mappings,
            pwm_mappings,
            accel_mappings,
            memory_mappings,
            debug_mappings,
        ) = map_peripherals(p);

        let pwm_output = bsp::PwmOutput::new(pwm_mappings);
        pwm_output.wait_break2_ready(); // Shows active low for first N cycles, wait it out
        let mut adc_feedback = bsp::AdcFeedback::new(adc_mappings);
        adc_feedback.sample_sector(0); // Kick off the ADC ISR loop
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings, 1000.0);
        let amt_encoder = bsp::AmtEncoder::new(encoder_mappings, 25_000.0, 1000.0);
        let acceleration = bsp::Acceleration::new(accel_mappings);
        let memory = bsp::Memory::new(memory_mappings);

        let mut config = if let Some(firmware_config) = memory.read_firmware_config() {
            firmware_config
        } else {
            FirmwareConfig::default()
        };
        config.current_limit = 2.5;
        let current_filter = PhaseCurrentFilter::new(2500.0, config.current_limit);

        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.1,
        };
        let mut foc = FOC::new(foc_cfg, PWM_FREQ.0 as f32);

        // Try to read calibrations and configurations from memory:
        let mut motor_parameters = if let Some(saved_parameters) = memory.read_motor_parameters() {
            ConstantMotorParameters::from_other(saved_parameters)
        } else {
            ConstantMotorParameters {
                params: MotorParamsEstimate::new_empty(),
            }
        };
        motor_parameters.params.num_pole_pairs = Some(2);

        if let Some(hall_calibrations) = memory.read_hall_calibrations() {
            hall_feedback.set_calibration(hall_calibrations);
        }
        if let Some(controller_parameters) = memory.read_controller_parameters() {
            foc.set_pi_gains(controller_parameters);
        }

        let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        Mono::start(rcc::frequency::<TIM2>().0, token);
        debug_print::spawn().unwrap();

        (
            Shared {
                mode: OperatingMode::Idle,
                board_status: BoardStatus {
                    dc_bus_voltage: None,
                    temperature: None,
                },
                config,
                runtime_values: RuntimeValues::default(),
                hall_feedback,
                amt_encoder,
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
                acceleration
            },
        )
    }

    #[task(
        binds = ADC3,
        local = [adc_feedback, current_filter, acceleration],
        shared = [
            hall_feedback, pwm_output, foc, motor_parameters,
            mode, board_status, config, runtime_values, debug_mappings
        ],
        priority = 3
    )]
    fn currents_adc_isr(mut cx: currents_adc_isr::Context) {
        cx.shared.mode.lock(|mode| {
            let calibrating = matches!(mode, OperatingMode::Calibration {..});
            let controlling = matches!(mode, OperatingMode::TorqueControl | OperatingMode::VelocityControl);
            let active = calibrating || controlling;
            let mut voltage_hexagon_sector = 0;

            // cx.shared.debug_mappings.lock(|dm| dm.la_pin.set_high());

            // if FOC ISR (sampled phase currents):
            if let Some(phase_currents) = cx.local.adc_feedback.read_currents() {
                // Check for overcurrent:
                cx.local.current_filter.update(phase_currents);
                let overcurrent = cx.local.current_filter.check_overcurrent();
                if overcurrent {
                    mode.on_command(Command::AssertFault {
                        cause: FaultCause::Overcurrent,
                    });
                }

                let dc_bus_reading = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage);
                let (rotor_feedback, hall_pattern) = cx.shared.hall_feedback.lock(|hf| {
                    (hf.read(), hf.get_pattern())
                });
                let mut rotor_feedback_fault = rotor_feedback.is_err();
                // Rotor feedback being invalid is not an issue during hall calibration:
                if let OperatingMode::Calibration { calibrator, .. } = mode {
                    rotor_feedback_fault = rotor_feedback_fault && matches!(calibrator.phase, CalibrationPhase::EncoderZeroing {..} | CalibrationPhase::HallCalibration).not();
                }

                // Inactive or blocked from running, set idle outputs:
                if !active || overcurrent || rotor_feedback_fault || dc_bus_reading.is_none() {
                    // TODO: need to come here also if:
                    // PI gains not configured
                    cx.shared.pwm_output.lock(|pwm| {
                        pwm.set_duty_cycles(PhaseValues::safe())
                    });
                } else {
                    // During hall calibration there is no valid rotor feedback, but its not used so we can default to zero values:
                    let RotorFeedback { theta, omega } = rotor_feedback.ok().unwrap_or(RotorFeedback { theta: 0.0, omega: 0.0 });
                    let dc_bus_voltage = dc_bus_reading.expect("Already checked for None above");
                    let mut calibration_result = None;

                    cx.shared.motor_parameters.lock(|active_params| {
                        let mut estimator: &mut dyn MotorParamEstimator = active_params;

                        // Determine FOC inputs based on operating mode (calibration or normal):
                        let (theta, foc_command) = if let OperatingMode::Calibration{ calibrator } = mode {
                            let (target_voltage, target_current, target_omega) = cx.shared.config.lock(|cfg| {
                                (cfg.calibration_voltage, cfg.calibration_current, cfg.calibration_omega)
                            });
                            let inputs = CalibrationInputs {
                                dc_bus_voltage,
                                theta,
                                hall_pattern,
                                target_voltage,
                                target_current,
                                target_omega,
                            };
                            let (output, stage_result) = calibrator.step(inputs);
                            estimator = calibrator.get_estimator();
                            calibration_result = stage_result;
                            (output.theta, output.foc_command)
                        } else {
                            let torque = cx.shared.runtime_values.lock(|rtv| rtv.target_torque);
                            (theta, FocInputType::TargetTorque(torque))
                        };

                        // Do the FOC computations:
                        let foc_input = FocInput {
                            command: foc_command,
                            dc_bus_voltage,
                            angle_type: AngleType::Electrical,
                            theta,
                            omega,
                            phase_currents,
                        };
                        let foc_result = cx.shared.foc.lock(|foc| {
                            foc.compute(foc_input, estimator.get_estimate(), cx.local.acceleration)
                        });

                        // Apply the computed PWM duty cycles:
                        let mut fault_command = None;
                        cx.shared.pwm_output.lock(|pwm| match foc_result {
                            Ok(result) => {
                                pwm.set_duty_cycles(result.duty_cycles);
                                estimator.after_foc_iteration(result);
                                voltage_hexagon_sector = result.voltage_hexagon_sector;
                            }
                            Err(fault) => {
                                pwm.set_duty_cycles(PhaseValues::safe());
                                fault_command = Some(Command::AssertFault { cause: fault.into() })
                            }
                        });
                        if let Some(command) = fault_command {
                            mode.on_command(command);
                        }
                    });

                    // Process calibration stage results:
                    if calibration_result.is_some() {
                        cx.shared.foc.lock(|foc| {
                            match calibration_result {
                                Some(StageResult::ZeroEncoderRequest) => {
                                    let _ = reset_encoder::spawn();
                                }
                                Some(StageResult::HallCalibration { angle_table }) => {
                                    info!("Angle table {}", angle_table);
                                    cx.shared.hall_feedback.lock(|hf| hf.set_calibration(angle_table));
                                    foc.clear_windup();
                                }
                                Some(StageResult::UnwindRequest) => {
                                    foc.clear_windup();
                                }
                                Some(StageResult::TuningRequest { params_estimate} ) => {
                                    // Tune controllers in a separate task to avoid locking up a high priority ISR:
                                    let _ = tune_pi::spawn(params_estimate);
                                }
                                Some(StageResult::MotorParameters { motor_params }) => {
                                    info!("Params {}", motor_params);
                                    cx.shared.motor_parameters.lock(|active_params| {
                                        active_params.copy_other(motor_params);
                                    });
                                    mode.on_command(Command::FinishCalibration);
                                    foc.clear_windup();
                                }
                                Some(StageResult::Failure { cause }) => {
                                    mode.on_command(Command::AssertFault { cause: cause.into() });
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
        });

        // if board status ISR (sampled DC bus voltage and board temperature):
        if let Some((vbus, tboard)) = cx.local.adc_feedback.read_board_info() {
            cx.shared.board_status.lock(|bs| {
                bs.dc_bus_voltage = Some(vbus);
                bs.temperature = Some(tboard);
            });
        }

        // cx.shared.debug_mappings.lock(|dm| dm.la_pin.set_low());
    }

    #[task(priority = 1, shared = [amt_encoder])]
    async fn reset_encoder(mut cx: reset_encoder::Context) {
        cx.shared.amt_encoder.lock(|ae| {
            ae.stop();
        });
        // After 100ms we can be sure there is no active dma request:
        Mono::delay(100.millis()).await;

        cx.shared.amt_encoder.lock(|ae| {
            ae.send_reset_requests();
        });

        // After 250ms the encoder should be ready again:
        Mono::delay(250.millis()).await;

        cx.shared.amt_encoder.lock(|ae| {
            ae.normal_mode();
        });
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
                    mode.on_command(Command::FinishTuning);
                });
            }
            None => {
                cx.shared.mode.lock(|mode| {
                    mode.on_command(Command::AssertFault {
                        cause: FaultCause::ControllerTuningFail,
                    });
                });
            }
        }
    }

    #[task(binds = TIM8_BRK, shared = [mode, pwm_output], priority = 1)]
    fn on_tim8(mut cx: on_tim8::Context) {
        let bk1_cleared = cx.shared.pwm_output.lock(|pwm| pwm.check_break1());
        let bk2_cleared = cx.shared.pwm_output.lock(|pwm| pwm.check_break2());

        if bk1_cleared || bk2_cleared {
            cx.shared.mode.lock(|mode| {
                if bk1_cleared {
                    mode.on_command(Command::AssertFault {
                        cause: FaultCause::Break1,
                    });
                }
                if bk2_cleared {
                    mode.on_command(Command::AssertFault {
                        cause: FaultCause::Break2,
                    });
                }
            });
        }
    }

    #[task(binds = TIM3, shared = [hall_feedback], priority = 2)]
    fn on_tim3(mut cx: on_tim3::Context) {
        cx.shared.hall_feedback.lock(|hall_feedback| {
            hall_feedback.on_interrupt();
        });
    }

    #[task(binds = DMA1_CHANNEL4, shared=[amt_encoder], priority = 2)]
    fn on_amt22_receive(mut cx: on_amt22_receive::Context) {
        cx.shared.amt_encoder.lock(|ae| {
            ae.on_transaction_complete();
        });
    }

    #[task(shared=[amt_encoder], priority = 1)]
    async fn debug_print(mut cx: debug_print::Context) {
        loop {
            cx.shared.amt_encoder.lock(|ae| {
                let data = ae.read();
                match data {
                    Ok(values) => { info!("Enc value: {}", values.theta); },
                    Err(..) => { info!("Enc not OK"); }
                }
            });
            Mono::delay(100.millis()).await;
        }
    }

    #[idle(shared=[amt_encoder, debug_mappings])]
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
            if dm.user_btn.is_low() {
                EdgeType::Rising
            } else {
                EdgeType::Falling
            }
        });
        cx.shared.button_state.lock(|bs| {
            if matches!(bs, ButtonState::Waiting) && matches!(edge, EdgeType::Rising) {
                let _ = button_timeout::spawn();
            }
            *bs = bs.on_edge(edge);
        });
    }

    #[task(shared=[debug_mappings, button_state, runtime_values, mode, motor_parameters], priority = 1)]
    async fn button_timeout(mut cx: button_timeout::Context) {
        Mono::delay(1000.millis()).await;
        let press_type = cx.shared.button_state.lock(|bs| *bs);
        let command = match press_type {
            ButtonState::ShortPress => Command::Idle,
            ButtonState::DoublePress => Command::EnableTorqueControl,
            ButtonState::LongPress => {
                let params_estimate = cx.shared.motor_parameters.lock(|mp| mp.get_estimate());
                if let Some(num_pole_pairs) = params_estimate.num_pole_pairs {
                    Command::StartCalibration {num_pole_pairs}
                } else {
                    Command::AssertFault { cause: FaultCause::MissingMotorParams }
                }
            },
            _ => {return}
        };
        cx.shared.mode.lock(|mode| {
            mode.on_command(command);
        });
        cx.shared.button_state.lock(|bs| *bs = ButtonState::Waiting);
    }

    const POT_RECIPROCAL: f32 = 0.05 / ((1 << 12) - 1) as f32;
    #[task(binds = ADC1_2, shared=[debug_mappings, runtime_values], priority = 1)]
    fn on_adc12(mut cx: on_adc12::Context) {
        let adc_reading = cx.shared.debug_mappings.lock(|dm| {
            if dm.pot_adc.check_eoc() {
                Some(dm.pot_adc.read() as f32 * POT_RECIPROCAL)
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
