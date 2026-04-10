#![no_std]

#[cfg(test)]
extern crate std;

mod types;
mod math;
mod pi_control;
mod estimation;
#[cfg(test)]
mod sim;
#[cfg(test)]
mod test_utils;

pub use crate::types::*;
use crate::{math::*, pi_control::PIGains};
pub use crate::pi_control::{PIController, compute_current_pi_controller_gains};
pub use crate::estimation::*;
#[cfg(test)]
pub use crate::sim::*;
#[cfg(test)]
pub use crate::test_utils::*;

pub struct ControllerParameters {
    pub d_pi: PIGains,
    pub q_pi: PIGains
}

pub struct FOC {
    config: FocConfig,
    d_pi: PIController,
    q_pi: PIController,
    u_dq_saturation: ClarkParkValue
}

impl FOC {
    pub fn new(config: FocConfig, pwm_freq_hz: f32) -> Self {
        let zero_gains = PIGains { kr: 0.0, kp: 0.0, ki: 0.0, kt: 0.0 };
        let d_pi = PIController::new(zero_gains, pwm_freq_hz);
        let q_pi = PIController::new(zero_gains, pwm_freq_hz);

        Self {
            config,
            d_pi, q_pi,
            u_dq_saturation: ClarkParkValue { d: 0.0, q: 0.0 }
        }
    }

    pub fn compute<A, E>(&mut self, 
        input: FocInput, 
        accelerator: &mut A, 
        estimator: &mut E
    ) -> FocResult where A: DoesFocMath, E: MotorParamEstimator {
        let motor_params = estimator.get_params();
        let theta_e = motor_params.num_pole_pairs as f32 * input.rotor_angle_rad;
        let omega_e = motor_params.num_pole_pairs as f32 * input.rotor_angular_velocity_rad_s;
        let sc_e = accelerator.sin_cos(theta_e);
        let measured_i_dq = forward_clark_park(input.phase_currents, sc_e);

        let u_dq = match input.command {
            FocInputType::TargetTorque(target_torque) => {
                // Derive target q,d-axis currents:
                let k_tau = if motor_params.num_pole_pairs != 0 && motor_params.pm_flux_linkage != 0.0 {
                    0.666667 / (motor_params.num_pole_pairs as f32 * motor_params.pm_flux_linkage)
                } else {
                    0.0
                };
                let target_i_d = 0.0;
                let target_i_q = k_tau * target_torque;
                // q,d-axis current PI controllers:
                let u_d = self.d_pi.compute(target_i_d, measured_i_dq.d, self.u_dq_saturation.d);
                let u_q = self.q_pi.compute(target_i_q, measured_i_dq.q, self.u_dq_saturation.q);
                // Cross-coupling compensation feedforward:
                let d_ff = motor_params.q_inductance*measured_i_dq.q*omega_e;
                let q_ff = -omega_e*(motor_params.pm_flux_linkage + motor_params.d_inductance*measured_i_dq.d);
                ClarkParkValue { d: u_d + d_ff, q: u_q + q_ff }
            }
            FocInputType::RawVoltage(u) => u
        };

        // When outside of the linear modulation region clamp in a way which 
        // prioritizes direct axis getting at least a desired fraction of max voltage
        // (which can then be used for field weakening)
        const SQRT3_RECIPROCAL: f32 = 1.0/1.73205080757;
        let u_max = input.dc_bus_voltage * SQRT3_RECIPROCAL;
        let u_d_limit = self.config.saturation_d_ratio * u_max;
        let u_q_limit = u_max * accelerator.sqrt(1.0 - self.config.saturation_d_ratio*self.config.saturation_d_ratio);
        let u_d_sat = u_dq.d.clamp(-u_d_limit, u_d_limit);
        let u_q_sat = u_dq.q.clamp(-u_q_limit, u_q_limit);

        // Update saturation error for next PI iteration anti-windup:
        self.u_dq_saturation = ClarkParkValue { 
            d: u_d_sat - u_dq.d, 
            q: u_q_sat - u_dq.q
        };

        let u_dq = ClarkParkValue {d: u_d_sat, q: u_q_sat};
        let u_ab = inverse_park(u_dq, sc_e);
        let voltage_hexagon_sector = voltage_sector(&u_ab);

        // Space vector modulation:
        let v_tgt = inverse_clarke(u_ab);
        let v0_tgt = -0.5*(min3(v_tgt.u, v_tgt.v, v_tgt.w) + max3(v_tgt.u, v_tgt.v, v_tgt.w));
        let bus_reciprocal = if input.dc_bus_voltage != 0.0 {
            1.0 / input.dc_bus_voltage
        } else {
            0.0
        };
        let duty_cycles = PhaseValues {
            u: (0.5 + bus_reciprocal*(v_tgt.u + v0_tgt)).clamp(0.0, 1.0),
            v: (0.5 + bus_reciprocal*(v_tgt.v + v0_tgt)).clamp(0.0, 1.0),
            w: (0.5 + bus_reciprocal*(v_tgt.w + v0_tgt)).clamp(0.0, 1.0)
        };

        // Update the parameter estimator:
        let data = FocIterationData {

        };
        estimator.after_foc_iteration(data);

        FocResult {
            duty_cycles,
            voltage_hexagon_sector
        }
    }

    pub fn set_pi_gains(&mut self, gains: ControllerParameters) {
        self.d_pi.set_gains(gains.d_pi);
        self.q_pi.set_gains(gains.q_pi);
    }

    pub fn get_pi_gains(&self) -> ControllerParameters {
        ControllerParameters {
            d_pi: self.d_pi.get_gains(),
            q_pi: self.q_pi.get_gains()
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------


#[cfg(test)]
mod tests {
    use libm::sinf;
    use super::*;

    #[test]
    fn pmsm_known_params_velocity_tracking() {
        let dt = 1e-5;
        let sim_cfg = PMSMConfig::default();
        let sim = PMSMSim::new(dt, sim_cfg);

        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.0
        };
        let foc = FOC::new(foc_cfg, 20_000.0);

        let accelerator = DummyAccelerator;
        let estimator = ConstantMotorParameters {
            params: MotorParams { 
                num_pole_pairs: sim_cfg.num_pole_pairs as u8,
                stator_resistance: sim_cfg.stator_resistance, 
                d_inductance: sim_cfg.inductance, 
                q_inductance: sim_cfg.inductance,
                pm_flux_linkage: sim_cfg.pm_flux_linkage
            }
        };

        /*
        let mut time_s = 0.0_f32;
        while time_s < 2.5 {
            let foc_input = FocInput {
                command: FocInputType::TargetTorque()
            }

            time_s += dt;
        }*/
    }
}