use core::f32::consts::TAU;
use libm::{cosf as cosf32, sinf as sinf32};
use crate::ClarkParkValue;

#[derive(Clone, Copy)]
pub struct SimOutput {
    /// Rotor angle in radians
    pub theta: f32,
    /// Rotor angular velocity in rad/s
    pub omega: f32,
    /// Phase winding currents in UVW coordinates
    pub currents: crate::PhaseValues,
    /// Produced rotor torque in Nm
    pub torque: f32
}

#[derive(Clone, Copy)]
struct PMSMState {
    /// Direct axis current
    i_d: f32,
    /// Quadrature axis current
    i_q: f32,
    /// Rotor angular velocity in rad/s
    omega: f32,
    /// Rotor angle in radians
    theta: f32,
}

#[derive(Clone, Copy)]
pub struct PMSMConfig {
    pub dc_bus_voltage: f32,
    pub num_pole_pairs: f32,
    pub stator_resistance: f32,
    pub inductance: f32,
    pub pm_flux_linkage: f32,
    pub rotor_inertia: f32,
}

impl Default for PMSMConfig {
    fn default() -> Self {
        PMSMConfig {
            dc_bus_voltage: 24.0,
            num_pole_pairs: 2.0,
            stator_resistance: 0.66, 
            inductance: 0.00184, 
            pm_flux_linkage: 0.0167,
            rotor_inertia: 6.7e-6, 
        }
    }
}

pub struct PMSMSim {
    /// Simulation timestep in seconds
    dt: f32,
    state: PMSMState,
    config: PMSMConfig,
}

impl PMSMSim {
    pub fn new(dt: f32, config: PMSMConfig) -> Self {
        Self {
            dt,
            state: PMSMState { i_d: 0.0, i_q: 0.0, omega: 0.0, theta: 0.0 },
            config,
        }
    }

    pub fn step(&mut self, input: crate::FocResult) -> SimOutput {
        let cfg = &self.config;
        let PMSMState { i_d, i_q, omega, theta } = self.state;

        // Electrical angles:
        let omega_e = cfg.num_pole_pairs * omega;
        let theta_e = cfg.num_pole_pairs * theta;

        let sc = crate::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
        let voltages = crate::PhaseValues {
            u: cfg.dc_bus_voltage * input.duty_cycles.u,
            v: cfg.dc_bus_voltage * input.duty_cycles.v,
            w: cfg.dc_bus_voltage * input.duty_cycles.w,
        };
        let v = crate::forward_clark_park(voltages, sc);

        // Euler integration of dq current dynamics:
        //   di_d/dt = v_d/L - R*i_d/L + omega_e*i_q
        //   di_q/dt = v_q/L - R*i_q/L - omega_e*i_d - omega_e*pm_flux_linkage/L
        let di_d = (v.d - cfg.stator_resistance * i_d + cfg.inductance * omega_e * i_q) / cfg.inductance;
        let di_q = (v.q - cfg.stator_resistance * i_q - cfg.inductance * omega_e * i_d - omega_e * cfg.pm_flux_linkage) / cfg.inductance;
        let i_d = i_d + self.dt * di_d;
        let i_q = i_q + self.dt * di_q;

        let torque = 1.5 * cfg.num_pole_pairs * cfg.pm_flux_linkage * i_q;
        let theta = (theta + self.dt * omega).rem_euclid(TAU);
        let omega = omega + self.dt * torque / cfg.rotor_inertia;

        self.state = PMSMState { i_d, i_q, omega, theta };
        let tmp =  ClarkParkValue { d: i_d, q: i_q };

        SimOutput {
            theta,
            omega,
            currents: crate::inverse_clark_park(tmp, sc),
            torque
        }
    }

    pub fn state(&self) -> SimOutput {
        let cfg = &self.config;
        let PMSMState { i_d, i_q, omega, theta } = self.state;
        let theta_e = cfg.num_pole_pairs * theta;
        let sc = crate::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
        SimOutput {
            theta,
            omega,
            currents: crate::inverse_clark_park(
                crate::ClarkParkValue { d: i_d, q: i_q }, sc
            ),
            torque: 0.0
        }
    }
}