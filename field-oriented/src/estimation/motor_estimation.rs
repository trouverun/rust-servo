use crate::{
    ClarkParkValue, MotorParams, MotorParamEstimator, FocIterationData,
};
use super::utils::Lse;

enum StepResult {
    Transition(OfflineEstimatorState),
    EstimateR { resistance: f32, next: OfflineEstimatorState },
    EstimateL { d_inductance: f32, next: OfflineEstimatorState },
    EstimateF { pm_flux_linkage: f32 },
}

enum OfflineEstimatorState {
    Off,
    /// Commanding d-axis current, waiting for rotor to align
    RotorLockWait { waited_s: f32 },
    /// Steady-state d-axis current: 
    /// R = v_d / i_d (di/dt ≈ 0)
    EstR {
        waited_s: f32,
        /// Solves R from the LSE problem:
        /// v_d = R * i_d
        lse: Lse,
    },
    /// Square wave d-axis voltage, R term cancels over symmetric excitation:
    /// L = |v| / |di/dt|
    EstL {
        waited_s: f32,
        voltage_sign: f32,
        settle_count: u32,
        prev_i_d: f32,
        /// Solves L from |v| = L * |di/dt|
        /// x = -sign * ΔI / dt (sign was toggled, so negate to recover applied sign)
        lse: Lse,
    },
    /// Constant-speed rotation using hall feedback:
    /// pm_flux = (v_q - R*i_q) / omega_e
    EstF {
        waited_s: f32,
        /// Solves pm_flux from the LSE problem:
        /// v_q - R*i_q = pm_flux * omega_e
        lse: Lse,
    },
    Done,
}

impl OfflineEstimatorState {
    fn step(
        &mut self,
        data: &FocIterationData,
        config: &OfflineEstimatorConfig,
        r_est: f32,
        omega_e: f32,
    ) -> Option<StepResult> {
        let dt = config.dt;
        match self {
            Self::Off | Self::Done => None,
            Self::RotorLockWait { waited_s } => {
                *waited_s += dt;
                if *waited_s >= config.settle_time_s {
                    Some(StepResult::Transition(Self::EstR {
                        waited_s: 0.0,
                        lse: Lse::new(),
                    }))
                } else {
                    None
                }
            }
            Self::EstR { waited_s, lse } => {
                // v_d = R * i_d
                lse.accumulate(data.measured_i_dq.d, data.u_dq.d);
                *waited_s += dt;
                if *waited_s >= config.test_time_s {
                    let resistance = lse.solve().unwrap_or(0.0);
                    Some(StepResult::EstimateR {
                        resistance,
                        next: Self::EstL {
                            waited_s: 0.0,
                            voltage_sign: 1.0,
                            settle_count: 100,
                            prev_i_d: data.measured_i_dq.d,
                            lse: Lse::new(),
                        },
                    })
                } else {
                    None
                }
            }
            Self::EstL {
                waited_s, voltage_sign, settle_count, prev_i_d, lse,
            } => {
                if *settle_count > 0 {
                    *settle_count -= 1;
                    *prev_i_d = data.measured_i_dq.d;
                    *voltage_sign *= -1.0;
                    return None;
                }

                // |v| = L * |di/dt|
                // use sign(v)*di/dt instead of abs(di/dt) to cancel out noise around low values
                // (and invert sign(v) since there is a one cycle delay from command to measurement = "wrong" sign)
                let x = -*voltage_sign * (data.measured_i_dq.d - *prev_i_d) / dt;
                let y = data.u_dq.d.abs();
                *prev_i_d = data.measured_i_dq.d;
                lse.accumulate(x, y);

                *voltage_sign *= -1.0;
                *waited_s += dt;
                if *waited_s >= config.test_time_s {
                    let d_inductance = lse.solve().unwrap_or(0.0);
                    Some(StepResult::EstimateL {
                        d_inductance,
                        next: Self::EstF {
                            waited_s: 0.0,
                            lse: Lse::new(),
                        },
                    })
                } else {
                    None
                }
            }
            Self::EstF {
                waited_s, lse,
            } => {
                // v_q - R*i_q = pm_flux * omega_e
                let x = omega_e;
                let y = data.u_dq.q - r_est * data.measured_i_dq.q;
                lse.accumulate(x, y);
                *waited_s += dt;

                if *waited_s >= config.test_time_s {
                    let pm_flux_linkage = lse.solve().unwrap_or(0.0);
                    Some(StepResult::EstimateF { pm_flux_linkage })
                } else {
                    None
                }
            }
        }
    }
}

pub enum OfflineEstimatorOutput {
    Current(ClarkParkValue),
    Voltage(ClarkParkValue),
}

pub struct OfflineEstimatorCommand {
    pub output: OfflineEstimatorOutput,
    pub rotor_angle: f32,
}

pub struct OfflineEstimatorConfig {
    pub settle_time_s: f32,
    pub test_time_s: f32,
    pub dt: f32,
}

pub struct OfflineMotorEstimator {
    state: OfflineEstimatorState,
    pub params: MotorParams,
    config: OfflineEstimatorConfig,
}

impl OfflineMotorEstimator {
    pub fn new(config: OfflineEstimatorConfig) -> Self {
        Self {
            state: OfflineEstimatorState::Off,
            params: MotorParams {
                num_pole_pairs: 0,
                stator_resistance: 0.0,
                d_inductance: 0.0,
                q_inductance: 0.0,
                pm_flux_linkage: 0.0,
            },
            config,
        }
    }

    pub fn start(&mut self) {
        self.state = OfflineEstimatorState::RotorLockWait { waited_s: 0.0 };
    }

    /// Returns the command for the current state.
    pub fn step(&self, max_current: f32, max_voltage: f32, rotor_angle_rad: f32) -> OfflineEstimatorCommand {
        match &self.state {
            OfflineEstimatorState::RotorLockWait { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: max_current, q: 0.0 }),
                rotor_angle: 0.0,
            },
            OfflineEstimatorState::EstR { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: max_current, q: 0.0 }),
                rotor_angle: 0.0,
            },
            OfflineEstimatorState::EstL { voltage_sign, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Voltage(ClarkParkValue { d: *voltage_sign * max_voltage, q: 0.0 }),
                rotor_angle: 0.0,
            },
            OfflineEstimatorState::EstF { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: 0.0, q: max_current }),
                rotor_angle: rotor_angle_rad,
            },
            _ => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: 0.0, q: 0.0 }),
                rotor_angle: 0.0,
            },
        }
    }

    pub fn check_estimation_done(&self) -> bool {
        matches!(self.state, OfflineEstimatorState::Done)
    }
}

impl MotorParamEstimator for OfflineMotorEstimator {
    fn after_foc_iteration(&mut self, data: FocIterationData) {
        let r_est = self.params.stator_resistance;
        if let Some(result) = self.state.step(&data, &self.config, r_est, data.omega_e) {
            match result {
                StepResult::Transition(next) => {
                    self.state = next;
                }
                StepResult::EstimateR { resistance, next } => {
                    self.params.stator_resistance = resistance;
                    self.state = next;
                }
                StepResult::EstimateL { d_inductance, next } => {
                    self.params.d_inductance = d_inductance;
                    self.params.q_inductance = d_inductance;
                    self.state = next;
                }
                StepResult::EstimateF { pm_flux_linkage } => {
                    self.params.pm_flux_linkage = pm_flux_linkage;
                    self.state = OfflineEstimatorState::Done;
                }
            }
        }
    }
    fn get_params(&self) -> MotorParams {
        self.params
    }
    fn parameters_valid(&self) -> bool {
        matches!(self.state, OfflineEstimatorState::Done)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{
        FOC, FocConfig, FocInput, FocInputType, AngleType,
        PMSMConfig, PMSMSim,
        DummyAccelerator, plot_simulation, SimRecord,
    };

    #[test]
    fn motor_param_estimation() {
        let pwm_freq_hz = 20_000.0;
        let dt = 1.0 / pwm_freq_hz;
        let sim_cfg = PMSMConfig::default();
        let mut sim = PMSMSim::new(dt, sim_cfg);

        let foc_cfg = FocConfig { saturation_d_ratio: 0.0 };
        let mut foc = FOC::new(foc_cfg, pwm_freq_hz);

        let mut accelerator = DummyAccelerator;

        let max_current = 1.0;
        let max_voltage = 1.0;
        let est_config = OfflineEstimatorConfig {
            settle_time_s: 0.5,
            test_time_s: 0.5,
            dt,
        };
        let mut estimator = OfflineMotorEstimator::new(est_config);
        estimator.start();

        let mut state = sim.state();
        let mut t = 0.0;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        let timeout = 3.0;

        while !estimator.check_estimation_done() {
            let rotor_angle = state.theta * sim_cfg.num_pole_pairs as f32;
            let rotor_velocity = state.omega * sim_cfg.num_pole_pairs as f32;
            let cmd = estimator.step(max_current, max_voltage, rotor_angle);

            let command = match cmd.output {
                OfflineEstimatorOutput::Current(i_dq) => FocInputType::TargetCurrents(i_dq),
                OfflineEstimatorOutput::Voltage(u_dq) => FocInputType::TargetVoltage(u_dq),
            };
            let foc_input = FocInput {
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                command,
                rotor_angle_rad: cmd.rotor_angle,
                angle_type: AngleType::Electrical,
                rotor_angular_velocity_rad_s: rotor_velocity,
                phase_currents: state.currents,
            };

            let foc_result = foc.compute(foc_input, &mut accelerator, &mut estimator);

            state = sim.step(foc_result);
            records.push(SimRecord {
                input: foc_input,
                result: foc_result,
                sim: state,
            });

            t += dt;
            if t > timeout {
                plot_simulation("motor_estimation.html", dt as f32, &records);
                panic!("Estimation did not complete within {timeout}s");
            }
        }

        plot_simulation("motor_estimation.html", dt as f32, &records);

        let est = estimator.params;
        let r_err = (est.stator_resistance - sim_cfg.stator_resistance).abs() / sim_cfg.stator_resistance;
        let l_err = (est.d_inductance - sim_cfg.inductance).abs() / sim_cfg.inductance;
        let f_err = (est.pm_flux_linkage - sim_cfg.pm_flux_linkage).abs() / sim_cfg.pm_flux_linkage;

        assert!(r_err < 0.10, "R estimate error {:.1}%: got {}, expected {}",
            r_err * 100.0, est.stator_resistance, sim_cfg.stator_resistance);
        assert!(l_err < 0.10, "L estimate error {:.1}%: got {}, expected {}",
            l_err * 100.0, est.d_inductance, sim_cfg.inductance);
        assert!(f_err < 0.10, "F estimate error {:.1}%: got {}, expected {}",
            l_err * 100.0, est.pm_flux_linkage, sim_cfg.pm_flux_linkage);
    }
}
