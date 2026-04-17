use crate::{
    ClarkParkValue, FocResult, MotorParamEstimator, MotorParamsEstimate
};
use super::utils::Lse;

#[derive(Clone, Copy, defmt::Format)]
pub enum EstimationStepFault {
    MissingParameter,
    Overflow,
    InsufficientSamples,
    DegenSolution,
    ParameterOutOfBounds
}

enum StepResult {
    Transition(OfflineEstimatorState),
    EstimateR { resistance: f32, next: OfflineEstimatorState },
    EstimateL { d_inductance: f32, next: OfflineEstimatorState },
    EstimateF { next: OfflineEstimatorState },
    RampDown { pm_flux_linkage: Option<f32> }
}

pub struct OfflineEstimatorConfig {
    pub settle_time_s: f32,
    pub test_time_s: f32,
    pub spin_time_s: f32,
    pub min_spin_omega: f32,
    pub dt: f32,
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
        resistance: f32,
        voltage_sign: f32,
        settle_count: u32,
        prev_i_d: f32,
        /// Solves L from the LSE problem:
        /// |v| = L * |di/dt|
        lse: Lse,
    },
    /// Constant-speed rotation using rotor angle feedback:
    /// pm_flux = (v_q - R*i_q) / omega_e
    EstF {
        waited_s: f32,
        resistance: f32,
        /// Solves pm_flux from the LSE problem:
        /// v_q - R*i_q = pm_flux * omega_e
        lse: Lse,
    },
    /// Slowly ramp down the current after EstF to avoid abrupt stop
    RampDown {
        pm_flux_linkage: Option<f32>,
        pending_fault: Option<EstimationStepFault>,
        waited_s: f32
    },
    Failure{fault: EstimationStepFault},
    Done
}

impl OfflineEstimatorState {
    fn step(
        &mut self,
        data: &FocResult,
        config: &OfflineEstimatorConfig,
        omega_e: f32,
        num_pole_pairs: u8
    ) -> Result<Option<StepResult>, EstimationStepFault> {
        let dt = config.dt;
        match self {
            Self::Off | Self::Done | Self::Failure { .. } => Ok(None),
            Self::RotorLockWait { waited_s } => {
                *waited_s += dt;
                if *waited_s >= config.settle_time_s {
                    let result = Some(StepResult::Transition(
                        Self::EstR {
                            waited_s: 0.0,
                            lse: Lse::new(),
                        }
                    ));
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::EstR { waited_s, lse } => {
                // v_d = R * i_d
                lse.accumulate(data.measured_i_dq.d, data.u_dq.d);
                *waited_s += dt;
                if *waited_s >= config.test_time_s {
                    let resistance = lse.solve(100)?;
                    let result = Some(StepResult::EstimateR {
                        resistance,
                        next: Self::EstL {
                            waited_s: 0.0,
                            resistance,
                            voltage_sign: 1.0,
                            settle_count: 100,
                            prev_i_d: data.measured_i_dq.d,
                            lse: Lse::new(),
                        }
                    });
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::EstL {
                waited_s, resistance, voltage_sign, settle_count, prev_i_d, lse,
            } => {
                // Wait for the current to settle from previous stage:
                if *settle_count > 0 {
                    *settle_count -= 1;
                    *prev_i_d = data.measured_i_dq.d;
                    *voltage_sign *= -1.0;
                    return Ok(None);
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
                    let d_inductance = lse.solve(100)?;
                    let result = Some(StepResult::EstimateL {
                        d_inductance,
                        next: Self::EstF {
                            waited_s: 0.0,
                            resistance: *resistance,
                            lse: Lse::new(),
                        },
                    });
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::EstF {
                waited_s, resistance, lse,
            } => {
                // v_q - R*i_q = pm_flux * omega_e
                let x = omega_e;
                let y = data.u_dq.q - *resistance * data.measured_i_dq.q;

                if omega_e.abs() / num_pole_pairs as f32 >= config.min_spin_omega {
                    lse.accumulate(x, y);
                }
                
                *waited_s += dt;
                if *waited_s >= config.spin_time_s {
                    let pm_flux_linkage = lse.solve(100);
                    let result = match pm_flux_linkage {
                        Ok(pmf) => {
                            Some(StepResult::EstimateF { 
                                next: Self::RampDown { pm_flux_linkage: Some(pmf), waited_s: 0.0, pending_fault: None }
                            })
                        }
                        // Delegate the fault to the rampdown, so it still gets to execute:
                        Err(e) => {
                            Some(StepResult::EstimateF { 
                                next: Self::RampDown { pm_flux_linkage: None, waited_s: 0.0, pending_fault: Some(e)}
                            })
                        }
                    };
                    Ok(result)
                } else {
                    Ok(None)
                }
            }
            Self::RampDown { pm_flux_linkage, pending_fault, waited_s } => {
                *waited_s += dt;
                if *waited_s >= config.spin_time_s {
                    if let Some(fault) = *pending_fault {
                        Err(fault)
                    } else {
                        Ok(Some(StepResult::RampDown { pm_flux_linkage: *pm_flux_linkage }))
                    }
                } else {
                    Ok(None)
                }
            }
        }
    }
}

pub enum OfflineEstimatorOutput {
    Current(ClarkParkValue),
    Voltage(ClarkParkValue),
}

pub struct OfflineEstimatorInput {
    pub target_voltage: f32,
    pub target_current: f32,
    pub theta: f32
}

pub struct OfflineEstimatorCommand {
    pub output: OfflineEstimatorOutput,
    pub theta: f32,
}

pub struct OfflineMotorEstimator {
    state: OfflineEstimatorState,
    pub params: MotorParamsEstimate,
    config: OfflineEstimatorConfig,
}

impl OfflineMotorEstimator {
    pub fn new(config: OfflineEstimatorConfig) -> Self {
        Self {
            state: OfflineEstimatorState::Off,
            params: MotorParamsEstimate::new_empty(),
            config,
        }
    }

    pub fn reset(&mut self) {
        self.state = OfflineEstimatorState::Off;
        self.params = MotorParamsEstimate::new_empty();
    }

    pub fn start(&mut self, num_pole_pairs: u8) {
        self.params.num_pole_pairs = Some(num_pole_pairs);
        self.state = OfflineEstimatorState::RotorLockWait { waited_s: 0.0 };
    }

    /// Returns the command for the current state.
    pub fn get_command(&self, input: OfflineEstimatorInput) -> OfflineEstimatorCommand {
        match &self.state {
            OfflineEstimatorState::RotorLockWait { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: input.target_current, q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstR { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: input.target_current, q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstL { voltage_sign, .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Voltage(ClarkParkValue { d: *voltage_sign * input.target_voltage, q: 0.0 }),
                theta: 0.0,
            },
            OfflineEstimatorState::EstF { .. } => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: 0.0, q: input.target_current}),
                theta: input.theta,
            },
            OfflineEstimatorState::RampDown { waited_s, .. } => {
                let factor = (1.0 - waited_s / self.config.spin_time_s).clamp(0.0, 1.0);
                OfflineEstimatorCommand { 
                    output: OfflineEstimatorOutput::Current(ClarkParkValue { d: 0.0, q: factor*input.target_current }), 
                    theta: input.theta
                }
            }
            _ => OfflineEstimatorCommand {
                output: OfflineEstimatorOutput::Current(ClarkParkValue { d: 0.0, q: 0.0 }),
                theta: 0.0,
            },
        }
    }

    pub fn estimation_done(&self) -> bool {
        matches!(self.state, OfflineEstimatorState::Done)
    }

    pub fn estimation_failed(&self) -> bool {
        matches!(self.state, OfflineEstimatorState::Failure { .. })
    }

    pub fn fault(&self) -> Option<EstimationStepFault> {
        match self.state {
            OfflineEstimatorState::Failure { fault } => Some(fault),
            _ => None
        }
    }

}

impl MotorParamEstimator for OfflineMotorEstimator {
    fn after_foc_iteration(&mut self, data: FocResult) {
        if let Some(num_pole_pairs) = self.params.num_pole_pairs {
            match self.state.step(&data, &self.config, data.omega_e, num_pole_pairs) {
                Err(fault) => {
                    self.state = OfflineEstimatorState::Failure { fault }
                }
                Ok(Some(result)) => {
                    match result {
                        StepResult::Transition(next) => {
                            self.state = next;
                        }
                        StepResult::EstimateR { resistance, next } => {
                            self.params.stator_resistance = Some(resistance);
                            self.state = next;
                        }
                        StepResult::EstimateL { d_inductance, next } => {
                            self.params.d_inductance = Some(d_inductance);
                            self.params.q_inductance = Some(d_inductance);
                            self.state = next;
                        }
                        StepResult::EstimateF { next } => {
                            self.state = next;
                        }
                        StepResult::RampDown { pm_flux_linkage } => { 
                            self.params.pm_flux_linkage = pm_flux_linkage;
                            self.state = OfflineEstimatorState::Done; 
                        }
                    }
                }
                Ok(None) => {}
            }
        } else {
            self.state = OfflineEstimatorState::Failure { fault: EstimationStepFault::MissingParameter }
        }
    }
    
    fn get_estimate(&self) -> MotorParamsEstimate {
        self.params
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
        let max_voltage = 6.0;
        let est_config = OfflineEstimatorConfig {
            settle_time_s: 0.5,
            test_time_s: 0.5,
            spin_time_s: 5.0,
            min_spin_omega: 50.0,
            dt,
        };
        let mut estimator = OfflineMotorEstimator::new(est_config);
        estimator.start(sim_cfg.num_pole_pairs as u8);

        let mut state = sim.state();
        let mut t = 0.0;
        let mut records: std::vec::Vec<SimRecord> = std::vec::Vec::new();
        let timeout = 20.0;

        while !estimator.estimation_done() {
            let theta_e = state.theta * sim_cfg.num_pole_pairs as f32;
            let omega_e = state.omega * sim_cfg.num_pole_pairs as f32;
            let step_in = OfflineEstimatorInput {
                target_voltage: max_voltage,
                target_current: max_current,
                theta: theta_e
            };
            let cmd = estimator.get_command(step_in);

            let command = match cmd.output {
                OfflineEstimatorOutput::Current(i_dq) => FocInputType::TargetCurrents(i_dq),
                OfflineEstimatorOutput::Voltage(u_dq) => FocInputType::TargetVoltage(u_dq),
            };
            let foc_input = FocInput {
                dc_bus_voltage: sim_cfg.dc_bus_voltage,
                command,
                theta: cmd.theta,
                angle_type: AngleType::Electrical,
                omega: omega_e,
                phase_currents: state.currents,
            };

            let foc_result = foc.compute(foc_input, estimator.get_estimate(), &mut accelerator);
            estimator.after_foc_iteration(foc_result.unwrap());

            state = sim.step(foc_result.unwrap());
            records.push(SimRecord {
                input: foc_input,
                result: foc_result.unwrap(),
                sim: state,
            });

            t += dt;
            if t > timeout {
                plot_simulation("motor_estimation.html", dt as f32, &records);
                panic!("Estimation did not complete within {timeout}s");
            }

            if estimator.estimation_failed() {
                plot_simulation("motor_estimation.html", dt as f32, &records);
                panic!("Estimation failed!")
            }
        }

        plot_simulation("motor_estimation.html", dt as f32, &records);

        let est = estimator.params;
        let r_err = (est.stator_resistance.unwrap() - sim_cfg.stator_resistance).abs() / sim_cfg.stator_resistance;
        let l_err = (est.d_inductance.unwrap() - sim_cfg.inductance).abs() / sim_cfg.inductance;
        let f_err = (est.pm_flux_linkage.unwrap() - sim_cfg.pm_flux_linkage).abs() / sim_cfg.pm_flux_linkage;

        assert!(r_err < 0.10, "R estimate error {:.1}%: got {}, expected {}",
            r_err * 100.0, est.stator_resistance.unwrap(), sim_cfg.stator_resistance);
        assert!(l_err < 0.10, "L estimate error {:.1}%: got {}, expected {}",
            l_err * 100.0, est.d_inductance.unwrap(), sim_cfg.inductance);
        assert!(f_err < 0.10, "F estimate error {:.1}%: got {}, expected {}",
            l_err * 100.0, est.pm_flux_linkage.unwrap(), sim_cfg.pm_flux_linkage);
    }
}
