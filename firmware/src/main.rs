#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use servo_firmware as _;
mod boards;
mod bsp;
mod calibration;
mod can;
mod types;
mod utils;
pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [SPI2, SPI3, UART5])]
mod app {
    use crate::boards::*;
    use crate::bsp::{
        self, Acceleration, AdcFeedback, AmtEncoder, CanBus, HallFeedback, Memory,
        PwmOutput, Watchdog
    };
    use crate::calibration::StageResult;
    use crate::types::{CalibrationPhase, *};
    use crate::types::{BoardStatus, OperatingMode};
    use crate::utils::{PhaseCurrentFilter, FeedbackArbitrator};
    use crate::{
        calibration::{CalibrationInputs},
        types::ButtonState,
    };
use core::ops::Not;
    use defmt::info;
    use defmt_rtt as _;
    use crate::can::messages::MotorCurrents;
    use crate::can::transport::IntoFrame;
    use embassy_stm32::can::{
        BufferedCanReceiver, BufferedCanSender, IT0InterruptHandler,
    };
    use embassy_stm32::interrupt::typelevel::Handler;
    use embassy_stm32::{peripherals::TIM2, rcc};
    use field_oriented::{
        AngleType, ConstantMotorParameters, FOC, FocConfig, FocInput, FocInputType, 
        HasRotorFeedback, MotorParamEstimator, MotorParamsEstimate, PhaseValues, RotorFeedback, 
        compute_current_pi_controller_gains
    };
    use rtic_monotonics::stm32::{ExtU64, Tim2 as Mono};
    use rtic_monotonics::Monotonic;

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
        current_loop_snapshot: CurrentLoopSnapshot,
        feedback_arbitrator: FeedbackArbitrator,
        watchdog: Watchdog,
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        current_filter: PhaseCurrentFilter,
        acceleration: Acceleration,
        can_tx: BufferedCanSender,
        can_rx: BufferedCanReceiver,
        _can_bus: CanBus,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let (
            adc_mappings,
            hall_mappings,
            encoder_mappings,
            pwm_mappings,
            accel_mappings,
            memory_mappings,
            can_mappings,
            watchdog_mappings,
            debug_mappings,
        ) = map_peripherals();

        let pwm_output = bsp::PwmOutput::new(pwm_mappings);
        pwm_output.wait_break2_ready(); // Shows active low for first N cycles, wait it out
        let mut adc_feedback = bsp::AdcFeedback::new(adc_mappings);
        adc_feedback.sample_sector(0); // Kick off the ADC ISR loop
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings, 1000.0);
        let amt_encoder = bsp::AmtEncoder::new(encoder_mappings, 1000.0);
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

        let can_bus = bsp::CanBus::new(can_mappings, 1_000_000);
        let can_tx = can_bus.writer();
        let can_rx = can_bus.reader();

        let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        Mono::start(rcc::frequency::<TIM2>().0, token);
        //can_tx_task::spawn().unwrap();
        //can_rx_task::spawn().unwrap();
        debug_print::spawn().unwrap();

        (Shared {
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
            current_loop_snapshot: CurrentLoopSnapshot::default(),
            feedback_arbitrator: FeedbackArbitrator::new(),
            watchdog: Watchdog::new(watchdog_mappings)
        },
        Local {
            adc_feedback,
            current_filter,
            acceleration,
            can_tx,
            can_rx,
            _can_bus: can_bus,
        })
    }

    #[task(priority = 4, binds = TIM7_DAC, shared = [watchdog, mode])]
    fn watchdog_isr(mut cx: watchdog_isr::Context) {
        cx.shared.watchdog.lock(|wd| wd.acknowledge_fault());
        cx.shared.mode.lock(|mode| {
            mode.on_command(Command::AssertFault { cause: FaultCause::Watchdog });
        });
    }

    #[task(
        priority = 3, binds = ADC3,
        local = [adc_feedback, current_filter, acceleration],
        shared = [
            pwm_output, foc, motor_parameters, feedback_arbitrator,
            mode, board_status, config, runtime_values, debug_mappings,
            current_loop_snapshot, watchdog
        ]
    )]
    fn currents_adc_isr(mut cx: currents_adc_isr::Context) {
        cx.shared.debug_mappings.lock(|dm| dm.la_a.set_high());

        cx.shared.mode.lock(|mode| {
            let calibrating = match mode {
                // The calibration state machine should not be stepped in the wait phases:
                OperatingMode::Calibration { calibrator } => !matches!(
                    calibrator.phase,
                    CalibrationPhase::WaitingHallCompletion | CalibrationPhase::WaitingTuning
                ),
                _ => false,
            };
            let controlling = matches!(mode, OperatingMode::TorqueControl | OperatingMode::VelocityControl);
            let active = calibrating || controlling;
            let mut voltage_hexagon_sector = 0;

            // if FOC ISR (sampled phase currents):
            if let Some(phase_currents) = cx.local.adc_feedback.read_currents() {
                cx.shared.watchdog.lock(|wd| wd.feed());
                // Check for overcurrent:
                cx.local.current_filter.update(phase_currents);
                let overcurrent = cx.local.current_filter.check_overcurrent();
                if overcurrent {
                    mode.on_command(Command::AssertFault {
                        cause: FaultCause::Overcurrent,
                    });
                }

                let dc_bus_reading = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage);
                let (rotor_feedback, hall_pattern) = cx.shared.feedback_arbitrator.lock(|fa| {
                    (fa.read(), fa.get_hall_pattern())
                });

                let mut rotor_feedback_fault = rotor_feedback.is_err();
                // Rotor feedback being invalid is not an issue during encoder zeroing or hall calibration:
                if let OperatingMode::Calibration { calibrator, .. } = mode {
                    let invalid_ok = matches!(calibrator.phase, CalibrationPhase::EncoderZeroing {..} | CalibrationPhase::HallCalibration {..});
                    rotor_feedback_fault = rotor_feedback_fault && !invalid_ok;
                }

                // Inactive or blocked from running, set idle outputs:
                if !active || overcurrent || rotor_feedback_fault || dc_bus_reading.is_none() {
                    // TODO: need to come here also if:
                    // PI gains not configured
                    cx.shared.pwm_output.lock(|pwm| {
                        pwm.set_duty_cycles(PhaseValues::safe())
                    });
                } else {
                    // During encoder zeroing or hall calibration there may be no valid rotor feedback, 
                    // but feedback is not used anyways, so we can safely default to zero values:
                    let RotorFeedback { angle_type, theta, omega } = rotor_feedback.ok()
                        .unwrap_or(RotorFeedback { angle_type: AngleType::Electrical, theta: 0.0, omega: 0.0 });
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
                                theta: theta,
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
                            angle_type,
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
                                if calibration_result.is_none() {
                                    estimator.after_foc_iteration(result);
                                }
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
                                    let _ = update_hall_table::spawn(angle_table);
                                    foc.clear_windup();
                                }
                                Some(StageResult::UnwindRequest) => {
                                    foc.clear_windup();
                                }
                                Some(StageResult::TuningRequest { params_estimate} ) => {
                                    let _ = tune_pi::spawn(params_estimate);
                                }
                                Some(StageResult::MotorParameters { motor_params }) => {
                                    let _ = update_motor_params::spawn(motor_params);
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

        cx.shared.debug_mappings.lock(|dm| dm.la_a.set_low());
    }

    #[task(priority = 1, shared = [amt_encoder])]
    async fn reset_encoder(mut cx: reset_encoder::Context) {
        cx.shared.amt_encoder.lock(|ae| {
            ae.stop();
        });
        // After 100ms we can be sure there is no active dma request,
        // and we can safely change the DMA source buffer values:
        Mono::delay(100.millis()).await;

        cx.shared.amt_encoder.lock(|ae| {
            ae.send_zero_request();
        });
        // After 300ms the encoder will be ready to respond again,
        // and we can safely revert the DMA source buffer values:
        Mono::delay(300.millis()).await;

        cx.shared.amt_encoder.lock(|ae| {
            ae.normal_mode();
        });
    }

    #[task(priority = 1, shared = [mode, hall_feedback])]
    async fn update_hall_table(mut cx: update_hall_table::Context, angle_table: [f32; 6]) {
        info!("Angle table {}", angle_table);
        cx.shared.hall_feedback.lock(|hf| hf.set_calibration(angle_table));
        cx.shared.mode.lock(|mode| {
            mode.on_command(Command::ResumeCalibration);
        });
    }

    #[task(priority = 1, shared = [mode, foc])]
    async fn tune_pi(mut cx: tune_pi::Context, estimate: MotorParamsEstimate) {
        let result = compute_current_pi_controller_gains::<50>(estimate, PWM_FREQ.0 as f32);
        info!("PI gains {}", result);
        match result {
            Ok(pi_gains) => {
                cx.shared.foc.lock(|foc| {
                    foc.set_pi_gains(pi_gains);
                    foc.clear_windup();
                });
                cx.shared.mode.lock(|mode| {
                    mode.on_command(Command::ResumeCalibration);
                });
            }
            Err(fault) => {
                cx.shared.mode.lock(|mode| {
                    mode.on_command(Command::AssertFault {
                        cause: fault.into(),
                    });
                });
            }
        }
    }

    #[task(priority = 1, shared = [motor_parameters, mode])]
    async fn update_motor_params(mut cx: update_motor_params::Context, parameters: MotorParamsEstimate) {
        info!("Params {}", parameters);
        cx.shared.motor_parameters.lock(|active_params| {
            active_params.copy_other(parameters);
        });
        cx.shared.mode.lock(|mode| {
            mode.on_command(Command::FinishCalibration);
        });
    }

    #[task(priority = 1, binds = TIM8_BRK, shared = [mode, pwm_output])]
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

    #[task(priority = 2, binds = TIM3, shared = [hall_feedback])]
    fn on_tim3(mut cx: on_tim3::Context) {
        cx.shared.hall_feedback.lock(|hall_feedback| {
            hall_feedback.on_hall_interrupt();
        });
    }

    #[task(priority = 2, binds = TIM5, shared = [hall_feedback, feedback_arbitrator, debug_mappings])]
    fn on_tim5(mut cx: on_tim5::Context) {
        cx.shared.debug_mappings.lock(|dm| dm.la_b.toggle());
        let (hall_feedback, hall_pattern) = cx.shared.hall_feedback.lock(|hall_feedback| {
            hall_feedback.on_read_interrupt();
            (hall_feedback.read(), hall_feedback.get_pattern())
        });
        cx.shared.feedback_arbitrator.lock(|fa| {
            fa.update_hall(hall_feedback, hall_pattern);
        })
    }

    #[task(priority = 2, binds = DMA1_CHANNEL4, shared=[amt_encoder, feedback_arbitrator, debug_mappings])]
    fn on_amt22_receive(mut cx: on_amt22_receive::Context) {
        cx.shared.debug_mappings.lock(|dm| dm.la_c.toggle());
        let amt_feedback = cx.shared.amt_encoder.lock(|ae| {
            ae.on_transaction_complete();
            ae.read()
        });
        cx.shared.feedback_arbitrator.lock(|fa| {
            fa.update_encoder(amt_feedback);
        })
    }

    #[task(priority = 1, local = [can_tx], shared = [current_loop_snapshot])]
    async fn can_tx_task(mut cx: can_tx_task::Context) {
        const fn hz_us(hz: u64) -> u64 { 1_000_000 / hz }
        const PERIOD_CURRENTS_US: u64 = hz_us(2500);

        let now = Mono::now();
        let mut due_currents = now + PERIOD_CURRENTS_US.micros();

        loop {
            Mono::delay_until(due_currents).await;
            let now = Mono::now();

            if now >= due_currents {
                let snap = cx.shared.current_loop_snapshot.lock(|s| *s);
                if let Ok(m) = MotorCurrents::new(
                    snap.iq_meas, snap.id_meas, snap.iq_target, snap.id_target,
                ) {
                    cx.local.can_tx.write(m.into_frame()).await;
                }
                due_currents += PERIOD_CURRENTS_US.micros();
            }
        }
    }

    #[task(priority = 1, local = [can_rx])]
    async fn can_rx_task(mut cx: can_rx_task::Context) {
        loop {
            let _ = cx.local.can_rx.receive().await;
        }
    }

    #[task(priority = 2, binds = FDCAN1_IT0)]
    fn fdcan1_it0(_: fdcan1_it0::Context) {
        unsafe { <IT0InterruptHandler<CanPeriph> as Handler<_>>::on_interrupt(); }
    }

    #[idle(shared=[amt_encoder, debug_mappings])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    // ---------------------- Debug user inputs: -----------------------------------
    #[task(priority = 1, binds = EXTI15_10, shared=[debug_mappings, button_state])]
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

    #[task(priority = 1, shared=[debug_mappings, button_state, runtime_values, mode, motor_parameters])]
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
    #[task(priority = 1, binds = ADC1_2, shared=[debug_mappings, runtime_values])]
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

    #[task(priority = 1, shared=[feedback_arbitrator])]
    async fn debug_print(mut cx: debug_print::Context) {
        loop {
            /*let data = cx.shared.feedback_arbitrator.lock(|fa| {
                fa.read()
            });

            if let Ok(vals) = data {
                info!("Angle: {}, Omega {}", vals.theta, vals.omega);
            } */
            Mono::delay(100.millis()).await;
        }
    }
}
