#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use servo_firmware as _;
mod bsp;
mod boards;
mod types;
mod calibration;

pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use core::{f32::consts::PI};
    use defmt::{info};
    use {defmt_rtt as _};
    use crate::calibration::{CalibrationInputs, CalibrationRunner, StageResult};
    use crate::{bsp::{self, Acceleration, AdcFeedback, HallFeedback, Memory, PwmOutput}, types::{OperatingMode, BoardStatus}};
    use crate::boards::*;
    use crate::types::*;
    use field_oriented::{
        ConstantMotorParameters, FOC, FocConfig, FocInput, FocInputType, 
        HasRotorFeedback, MotorParamEstimator, MotorParams, PhaseValues
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
        debug_mappings: DebugMappings,
        button_state: ButtonState,
    }

    #[local]
    struct Local {
        adc_feedback: AdcFeedback,
        calibration: CalibrationRunner,
        foc: FOC,
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
        let adc_feedback = bsp::AdcFeedback::new(adc_mappings);
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings);
        let acceleration = bsp::Acceleration::new(accel_mappings);
        let memory = bsp::Memory::new(memory_mappings);

        let config = if let Some(firmware_config) = memory.read_firmware_config() {
            firmware_config
        } else {
            FirmwareConfig::default()
        };

        let calibration = CalibrationRunner::new();
        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.1
        };
        let mut foc = FOC::new(foc_cfg, PWM_FREQ.0 as f32);

        // Try to read calibrations and configurations from memory:
        let motor_parameters = if let Some(saved_parameters) = memory.read_motor_parameters() {
            ConstantMotorParameters::new(saved_parameters)
        } else {
            ConstantMotorParameters { 
                params: MotorParams {
                    num_pole_pairs: 0,
                    stator_resistance: 0.0,
                    d_inductance: 0.0, q_inductance: 0.0,
                    pm_flux_linkage: 0.0
                } 
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
            board_status: BoardStatus {dc_bus_voltage: 0.0, temperature: 0.0},
            config,
            runtime_values: RuntimeValues::default(),
            hall_feedback,
            pwm_output,
            acceleration,
            memory,
            motor_parameters,
            debug_mappings,
            button_state: ButtonState::Waiting,
        },
        Local {
            adc_feedback,
            calibration,
            foc
        })
    }

    // FOC interrupts:
    #[task(binds = TIM3, shared = [hall_feedback], priority = 2)]
    fn on_tim3(mut cx: on_tim3::Context) {
        cx.shared.hall_feedback.lock(|hall_feedback| {
            hall_feedback.on_interrupt();
        });
    }

    #[task(
        binds = ADC3, 
        local = [adc_feedback, calibration, foc],
        shared = [hall_feedback, pwm_output, acceleration, motor_parameters, mode, board_status, config, runtime_values, memory],
        priority = 3
    )]
    fn currents_adc_isr(mut cx: currents_adc_isr::Context) {
        let mode = cx.shared.mode.lock(|mode| *mode);
        let calibrating = matches!(mode, OperatingMode::Calibration);
        let active = calibrating || matches!(mode, OperatingMode::TorqueControl | OperatingMode::VelocityControl);

        // ADC injected EOC (FOC ISR):
        let mut voltage_hexagon_sector = 0;
        if let Some(phase_currents) = cx.local.adc_feedback.read_currents() {
            if !active {
                cx.shared.pwm_output.lock(|pwm| {
                    pwm.set_duty_cycles(PhaseValues { u: 0.0, v: 0.0, w: 0.0 })
                });
            } else {
                let dc_bus_voltage = cx.shared.board_status.lock(|bs| bs.dc_bus_voltage);

                // Determine FOC inputs based on calibration vs torque/velocity control mode:
                let (theta, omega, foc_command, calibration_result) = if calibrating {
                    let (theta, omega, hall_pattern) = cx.shared.hall_feedback.lock(|hf| {
                        let feedback = hf.read();
                        (feedback.angle, feedback.velocity, hf.get_pattern())
                    });
                    let (target_voltage, target_speed) = cx.shared.config.lock(|cfg| {
                        (cfg.calibration_voltage, cfg.calibration_speed_rad_s)
                    });
                    let inputs = CalibrationInputs { hall_angle: theta, hall_pattern, target_voltage, target_speed };
                    let (output, stage_result) = cx.local.calibration.tick(inputs);
                    (output.rotor_angle_rad, omega, output.foc_command, stage_result)
                } else {
                    let (theta, omega) = cx.shared.hall_feedback.lock(|hf| {
                        let feedback = hf.read();
                        (feedback.angle, feedback.velocity)
                    });                    
                    let torque = cx.shared.runtime_values.lock(|rtv| rtv.target_torque);
                    (theta, omega, FocInputType::TargetTorque(torque), None)
                };

                // FOC computations:
                let foc_input = FocInput { 
                    command: foc_command, dc_bus_voltage, 
                    rotor_angle_rad: theta, 
                    rotor_angular_velocity_rad_s: omega,
                    phase_currents 
                };
                let foc_result = cx.shared.motor_parameters.lock(|params| {
                    cx.shared.acceleration.lock(|acc| {
                        cx.local.foc.compute(foc_input, acc, params)
                    })
                });
                cx.shared.pwm_output.lock(|pwm| pwm.set_duty_cycles(foc_result.duty_cycles));
                voltage_hexagon_sector = foc_result.voltage_hexagon_sector;

                // Apply calibration stage results:
                match calibration_result {
                    // TODO: pole_count needs to somehow get to the shared motor_params (and persist there)
                    Some(StageResult::HallCalibration { angle_table, pole_count }) => {
                        cx.shared.hall_feedback.lock(|hf| hf.set_calibration(angle_table));
                        cx.shared.memory.lock(|m| {
                            m.write_hall_calibrations(angle_table);
                        });
                    }
                    Some(StageResult::MotorParameters { motor_params, pi_gains }) => {
                        cx.shared.motor_parameters.lock(|active_params| {
                            active_params.from_other(&cx.local.calibration.motor_estimator);
                        });
                        cx.local.foc.set_pi_gains(pi_gains);
                        cx.shared.memory.lock(|m| {
                            m.write_motor_parameters(motor_params);
                            m.write_controller_parameters(pi_gains);
                        });
                        cx.shared.mode.lock(|mode| *mode = mode.on_command(Command::FinishCalibration));
                    }
                    Some(StageResult::Failure) => {

                    }
                    None => {}
                }
            }

            // Always sample something to keep the ADC EOC ISRs running:
            cx.local.adc_feedback.sample_sector(voltage_hexagon_sector);
        }
        
        // ADC regular EOC:
        if let Some((vbus, tboard)) = cx.local.adc_feedback.read_board_info() {
            cx.shared.board_status.lock(|bs| {
                bs.dc_bus_voltage = vbus;
                bs.temperature = tboard;
            });
        }
    }/*

    #[task(binds = TIM8_BRK, shared = [pwm], priority = 1)]
    fn on_tim8(mut cx: on_tim8::Context) {
        let bk1_cleared = cx.shared.pwm.lock(|pwm| {
            pwm.acknowledge_break1()
        });
        let bk2_cleared = cx.shared.pwm.lock(|pwm| {
            pwm.acknowledge_break2()
        });

        if bk1_cleared {
            info!("Break1 triggered!");
        }
        if bk2_cleared {
            info!("Break2 triggered!");
        }
    }*/

    #[idle(shared=[debug_mappings, acceleration])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    // ---------------------- Debug user inputs: -----------------------------------
    #[task(binds = EXTI15_10, shared=[debug_mappings, button_state], priority = 1)]
    fn on_exti15_10(mut cx: on_exti15_10::Context) {
        let rising = cx.shared.debug_mappings.lock(|dm| {
            dm.user_btn.clear_pending();
            dm.user_btn.is_low()
        });
        cx.shared.button_state.lock(|bs| {
            *bs = bs.on_edge(rising);
        });
        if rising {
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
                ButtonState::LongPress => {
                    *mode = mode.on_command(Command::StartCalibration);
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

    const POT_RECIPROCAL: f32 = 50.0 / ((1 << 12) - 1) as f32;
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
                rc.target_velocity = val;
            });
        }
    }
}
