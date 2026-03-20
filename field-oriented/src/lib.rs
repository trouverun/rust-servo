#![no_std]

mod types;
mod math;
mod pi_control;
mod estimation;
#[cfg(test)]
mod sim;

pub use crate::types::*;
use crate::{math::*, pi_control::PIGains};
pub use crate::pi_control::PIController;
pub use crate::estimation::*;

pub struct FocConfig {
    pub num_poles: u32,
    pub saturation_d_ratio: f32,
}

pub struct FOC {
    config: FocConfig,
    d_pi: PIController,
    q_pi: PIController,
    u_dq_saturation: ClarkParkResult
}

impl FOC {
    pub fn new(config: FocConfig) -> Self {
        let zero_gains = PIGains { kr: 0.0, kp: 0.0, ki: 0.0 };
        let d_pi = PIController::new(zero_gains);
        let q_pi = PIController::new(zero_gains);

        Self {
            config,
            d_pi, q_pi,
            u_dq_saturation: ClarkParkResult { d: 0.0, q: 0.0 }
        }
    }

    pub fn compute<A, E>(&mut self, 
        input: FocInput, 
        accelerator: &mut A, 
        estimator: &mut E
    ) -> FocResult where A: DoesFocMath, E: MotorParamEstimator {
        let sc = accelerator.sin_cos(self.config.num_poles as f32 * input.rotor_angle_rad);
        let measured_i_dq = forward_clark_park(input.phase_currents, sc);

        let motor_params = estimator.get_params();
        let target_i_d = 0.0;
        let target_i_q = 1.5 * self.config.num_poles as f32 * motor_params.pm_flux_linkage * input.target_torque;
        
        let u_d= self.d_pi.compute(target_i_d, measured_i_dq.d, self.u_dq_saturation.d);
        let u_q= self.q_pi.compute(target_i_q, measured_i_dq.q, self.u_dq_saturation.q);
        
        // When outside of the linear modulation region clamp in a way which 
        // prioritizes direct axis getting at least a desired fraction of max voltage
        // (which can then be used for field weakening)
        const SQRT3_RECIPROCAL: f32 = 1.0/1.73205080757;
        let u_max = input.bus_voltage * SQRT3_RECIPROCAL;
        let u_d_limit = self.config.saturation_d_ratio * u_max;
        let u_q_limit = u_max * accelerator.sqrt(1.0 - self.config.saturation_d_ratio*self.config.saturation_d_ratio);
        let u_d_sat = u_d.clamp(-u_d_limit, u_d_limit);
        let u_q_sat = u_q.clamp(-u_q_limit, u_q_limit);
        self.u_dq_saturation = ClarkParkResult { 
            d: u_d_sat - u_d, 
            q: u_q_sat - u_q
        };

        let u_dq = ClarkParkResult {d: u_d_sat, q: u_q_sat};
        let u_ab = inverse_park(u_dq, sc);
        let hexagon_sector = voltage_sector(&u_ab);

        let v_tgt = inverse_clarke(u_ab);
        let v0_tgt = -0.5*(min3(v_tgt.u, v_tgt.v, v_tgt.w) + max3(v_tgt.u, v_tgt.v, v_tgt.w));
        let bus_reciprocal = 1.0 / input.bus_voltage;
        let duty_cycles = PhaseValues {
            u: (0.5 + bus_reciprocal*(v_tgt.u + v0_tgt)).clamp(0.0, 1.0),
            v: (0.5 + bus_reciprocal*(v_tgt.v + v0_tgt)).clamp(0.0, 1.0),
            w: (0.5 + bus_reciprocal*(v_tgt.w + v0_tgt)).clamp(0.0, 1.0)
        };

        FocResult {
            duty_cycles,
            hexagon_sector
        }
    }

    pub fn set_pi_gains(&mut self) {

    }
}

// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn velocity_tracking() {
        // let cfg = FocConfig {};
        // let foc = FOC::new(cfg);
    }
}