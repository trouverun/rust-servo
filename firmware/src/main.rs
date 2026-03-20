#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use servo_firmware as _;
mod bsp;
mod boards;
mod types;

pub mod pac {
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

#[rtic::app(device = crate::pac, peripherals = false, dispatchers = [SPI1, SPI2, SPI3])]
mod app {
    use defmt::{info};
    use {defmt_rtt as _};
    use crate::bsp::{self, Acceleration, CurrentFeedback, HallFeedback, Memory, PwmOutput};
    use crate::boards::*;
    use crate::types::*;
    use field_oriented::{DoesFocMath, FOC, FocConfig, FocInput, HasRotorFeedback, MotorParams, NominalParameters};

    #[shared]
    struct Shared {
        runtime_config: RuntimeConfig,
        hall_feedback: HallFeedback,
        pwm_output: PwmOutput,
        acceleration: Acceleration,
        //memory: Memory,
        debug_mappings: DebugMappings,
        button_state: ButtonState,
    }

    #[local]
    struct Local {  
        current_feedback: CurrentFeedback,
        calibration_state: CalibrationState,
        estimator: NominalParameters,
        foc: FOC,
    }

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let p = bsp::init();
        let (
            current_mappings, 
            hall_mappings, 
            pwm_mappings,
            accel_mappings,
            memory_mappings,
            debug_mappings
        ) = map_peripherals(p);
        
        let pwm_output = bsp::PwmOutput::new(pwm_mappings);
        pwm_output.wait_break2_ready();
        let current_feedback = bsp::CurrentFeedback::new(current_mappings);
        let mut hall_feedback = bsp::HallFeedback::new(hall_mappings);
        let acceleration = bsp::Acceleration::new(accel_mappings);
        /*let memory = bsp::Memory::new(memory_mappings);

        if let Some(calibrations) = memory.read_hall_calibrations() {
            hall_feedback.set_calibration(calibrations);
        }*/

        let foc_cfg = FocConfig {
            num_poles: 2,
            saturation_d_ratio: 0.1
        };
        let foc = FOC::new(foc_cfg);

        let estimator = NominalParameters {
            params: MotorParams {
                pm_flux_linkage: 0.0
            }
        };

        (Shared { 
            runtime_config: RuntimeConfig::default(),
            hall_feedback,
            pwm_output,
            acceleration,
            // memory,
            debug_mappings,
            button_state: ButtonState::Waiting
        }, 
        Local { 
            current_feedback,
            calibration_state: CalibrationState::default(),
            estimator,            
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
        local = [current_feedback, calibration_state, foc, estimator], 
        shared = [hall_feedback, pwm_output, runtime_config, acceleration], 
        priority = 3
    )]
    fn on_adc3(mut cx: on_adc3::Context) {
        let application_state = cx.shared.runtime_config.lock(|rc| rc.application_state);
        
        if let Some(phase_currents) = cx.local.current_feedback.read_currents() {
            let mut sector = 0;
            if matches!(application_state, ApplicationState::Calibration | ApplicationState::Running) {
                // Get the "rotation angle" for park transform as either:
                // a) the slowly rotating hall calibration angle (align rotor to desired position)
                // b) the estimated rotor angle offset by 90 deg (maximize torque in real operation)
                let rotor_angle_rad = if matches!(application_state, ApplicationState::Calibration) {
                    let pattern = cx.shared.hall_feedback.lock(|hall_feedback| {
                        hall_feedback.get_pattern()
                    });
                    cx.local.calibration_state.step(pattern)
                } else {
                    cx.shared.hall_feedback.lock(|hall_feedback| {
                        hall_feedback.read().angle
                    })
                };

                let foc_result = cx.shared.acceleration.lock(|accelerator| {
                    let input = FocInput {
                        bus_voltage: 0.0, rotor_angle_rad, phase_currents, target_torque: 0.0
                    };
                    cx.local.foc.compute(input, accelerator, cx.local.estimator)
                });

                cx.shared.pwm_output.lock(|pwm| pwm.set_duty_cycles(foc_result.duty_cycles));
                sector = foc_result.hexagon_sector;

                if matches!(application_state, ApplicationState::Calibration) && cx.local.calibration_state.done() {
                    // cx.shared.memory.lock(|memory| memory.write_hall_calibrations(cx.local.calibration_state.hall_pattern_to_angle));
                    cx.shared.hall_feedback.lock(|hall_feedback| hall_feedback.set_calibration(cx.local.calibration_state.hall_pattern_to_angle));
                    cx.shared.runtime_config.lock(|rc| rc.application_state.on_command(UserCommand::FinishCalibration));
                }
            }
            // Always sample something to keep the loop running
            cx.local.current_feedback.sample_sector(sector);
        }
        
        { // If adc3 regular eso
            // Read VBUS and temperature
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

    #[task(binds = TIM2, shared=[debug_mappings, button_state, runtime_config], priority = 2)]
    fn on_tim2(mut cx: on_tim2::Context) {
        let press_type = cx.shared.button_state.lock(|bs| *bs);
        cx.shared.runtime_config.lock(|rc| {
            match press_type {
                ButtonState::LongPress => {
                    rc.application_state = rc.application_state.on_command(UserCommand::StartCalibration);
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
    #[task(binds = ADC1_2, shared=[debug_mappings, runtime_config], priority = 1)]
    fn on_adc12(mut cx: on_adc12::Context) {
        let adc_reading = cx.shared.debug_mappings.lock(|dm| {
            if dm.pot_adc.check_eoc() {
                Some((dm.pot_adc.read() as f32 * POT_RECIPROCAL))
            } else {
                None
            }
        });

        if let Some(val) = adc_reading {
            cx.shared.runtime_config.lock(|rc| {
                rc.target_velocity = val;
            });
        }
    }
}
