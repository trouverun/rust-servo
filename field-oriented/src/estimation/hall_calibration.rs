use core::f32::consts::PI;

enum CalibrationState {
    InitialSettle { waited_s: f32 },
    Sweeping {
        waited_s: f32,
        target_angle_rad: f32,
        first_edge: Option<u8>,
        prev_pattern: u8,
    },
    Done,
}

pub struct HallCalibrator {
    state: CalibrationState,
    iteration_counter: u32,
    initial_settle_time_s: f32,
    step_settle_time_s: f32,
    pub hall_pattern_to_angle: [f32; 6],
}

impl HallCalibrator {
    pub fn new(initial_settle_time_s: f32, step_settle_time_s: f32) -> Self {
        Self {
            state: CalibrationState::InitialSettle { waited_s: 0.0 },
            iteration_counter: 0,
            hall_pattern_to_angle: [0.0; 6],
            initial_settle_time_s,
            step_settle_time_s,
        }
    }

    pub fn start(&mut self) {
        self.state = CalibrationState::InitialSettle { waited_s: 0.0 };
        self.iteration_counter = 0;
        self.hall_pattern_to_angle = [0.0; 6];
    }

    /// Increment the target rotor angle by a small amount
    pub fn calibration_step<const PWM_FREQ: u32>(
        &mut self, hall_pattern: u8, rotation_speed_rad_s: f32
    ) -> f32 {
        // Step by 100 FOC ISRs at a time (10-20 kHz to 100-200 Hz)
        if self.iteration_counter >= 100 {
            let time_passed_s: f32 = 100.0 / PWM_FREQ as f32;
            match &mut self.state {
                CalibrationState::InitialSettle { waited_s } => {
                    *waited_s += time_passed_s;
                    if *waited_s >= self.initial_settle_time_s {
                        self.state = CalibrationState::Sweeping {
                            waited_s: 0.0,
                            target_angle_rad: 0.0,
                            first_edge: None,
                            prev_pattern: hall_pattern,
                        };
                    }
                }
                CalibrationState::Sweeping { waited_s, target_angle_rad, first_edge, prev_pattern } => {
                    if *waited_s >= self.step_settle_time_s {
                        *target_angle_rad += rotation_speed_rad_s * time_passed_s;
                        if *target_angle_rad >= 2.0*PI {
                            *target_angle_rad -= 2.0*PI;
                        }
                        if *prev_pattern != hall_pattern {
                            if let Some(pattern) = first_edge {
                                if *pattern == hall_pattern {
                                    self.state = CalibrationState::Done;
                                    return 0.0;
                                }
                            } else {
                                *first_edge = Some(hall_pattern);
                            }
                            let idx = (hall_pattern.clamp(1, 6) - 1) as usize;
                            self.hall_pattern_to_angle[idx] = *target_angle_rad;
                        }
                        *waited_s = 0.0;
                        *prev_pattern = hall_pattern;
                    } else {
                        *waited_s += time_passed_s;
                    }
                }
                CalibrationState::Done => {}
            }
            self.iteration_counter = 0;
        } else {
            self.iteration_counter += 1;
        }

        match &self.state {
            CalibrationState::Sweeping { target_angle_rad, .. } => *target_angle_rad,
            _ => 0.0,
        }
    }

    pub fn check_calibration_done(&self) -> bool {
        matches!(self.state, CalibrationState::Done)
    }
}

#[cfg(test)]
mod test {
    use core::f32::consts::PI;

    use super::{HallCalibrator};
    use crate::{
        ClarkParkValue, FOC, FocConfig, FocInput, MotorParams, ConstantMotorParameters, MotorParamsEstimate,
        PMSMConfig, PMSMSim, HallEncoder, FocInputType, AngleType, DummyAccelerator, plot_simulation, SimRecord
    };

    #[test]
    fn hall_calibration_works() {
        let pwm_freq_hz = 20_000.0;
        let dt = 1.0 / pwm_freq_hz;
        let mut sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt, sim_cfg)
            .with_hall_encoder(HallEncoder::ideal());

        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.0
        };
        let mut foc = FOC::new(foc_cfg, pwm_freq_hz);
        let mut accelerator = DummyAccelerator;
        let motor_params = MotorParamsEstimate::from_nominal(
            MotorParams {
                num_pole_pairs: sim_cfg.num_pole_pairs as u8,
                stator_resistance: sim_cfg.stator_resistance,
                d_inductance: sim_cfg.inductance,
                q_inductance: sim_cfg.inductance,
                pm_flux_linkage: sim_cfg.pm_flux_linkage
            }
        );
        let mut calibrator = HallCalibrator::new(5.0, 0.01);

        let mut state = sim.state();
        let mut t = 0.0;
        let record_interval = (0.1 / dt).round() as u64;
        let mut step: u64 = 0;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        while !calibrator.check_calibration_done() {
            let pattern = state.hall_pattern.unwrap();
            let angle = calibrator.calibration_step::<20_000>(pattern, 1.0);

            let foc_input = FocInput {
                dc_bus_voltage: 24.0,
                command: FocInputType::TargetCurrents(ClarkParkValue {
                    d: 0.5, q: 0.0
                }),
                theta: angle,
                angle_type: AngleType::Electrical,
                omega: 0.0,
                phase_currents: state.currents
            };

            let foc_result = foc.compute(foc_input, motor_params, &mut accelerator);
            state = sim.step(foc_result.unwrap());
            if step % record_interval == 0 {
                records.push(SimRecord {
                    input: foc_input,
                    result: foc_result.unwrap(),
                    sim: state,
                });
            }

            step += 1;
            t += dt;
            if (t > 30.0) {
                plot_simulation("hall_calibration.html", dt * record_interval as f32, &records);
                assert!(false, "Timeout reached")
            }
        }

        if let Some(encoder) = sim.hall_encoder {
            let tolerance = 0.02;
            for (i, &angle) in calibrator.hall_pattern_to_angle.iter().enumerate() {
                let pattern = (i + 1) as u8;
                let expected: f32 = encoder.edge_angle(pattern).unwrap();
                let d = angle - expected;
                let error = d.sin().atan2(d.cos()).abs();
                assert!(error < tolerance, "pattern {pattern}: got {angle:.3}, expected {expected:.3}");
            }
        } else {
            assert!(false)
        }

        plot_simulation("hall_calibration.html", dt * record_interval as f32, &records);
    }
}
