use core::f32::consts::TAU;
use libm::{cosf as cosf32, sinf as sinf32};
use crate::ClarkParkResult;

pub struct SimOutput {
    pub theta: f32,
    pub omega: f32,
    pub currents: super::PhaseValues,
}

#[derive(Clone, Copy)]
struct PMSMState {
    i_d: f32,
    i_q: f32,
    omega: f32,
    theta: f32,
}

#[derive(Clone, Copy)]
pub struct PMSMConfig {
    pub bus_voltage: f32,
    pub stator_resistance: f32,
    pub pm_flux_linkage: f32,
    pub stator_inductance: f32,
    pub rotor_inertia: f32,
    pub pole_pairs: f32,
}

pub struct PMSMSim {
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

    pub fn step(&mut self, input: super::FocResult) -> SimOutput {
        let cfg = &self.config;
        let PMSMState { i_d, i_q, omega, theta } = self.state;

        // Electrical angles:
        let omega_e = cfg.pole_pairs * omega;
        let theta_e = cfg.pole_pairs * theta;

        let sc = super::SinCosResult { sin: sinf32(theta_e), cos: cosf32(theta_e) };
        let voltages = super::PhaseValues {
            u: cfg.bus_voltage * input.duty_cycles.u,
            v: cfg.bus_voltage * input.duty_cycles.v,
            w: cfg.bus_voltage * input.duty_cycles.w,
        };
        let v = super::forward_clark_park(voltages, sc);

        // Euler integration of dq current dynamics:
        //   L*di_d/dt = v_d - R*i_d + L*omega_e*i_q
        //   L*di_q/dt = v_q - R*i_q - L*omega_e*i_d - omega_e*pm_flux_linkage
        let di_d = (v.d - cfg.stator_resistance * i_d + cfg.stator_inductance * omega_e * i_q) / cfg.stator_inductance;
        let di_q = (v.q - cfg.stator_resistance * i_q - cfg.stator_inductance * omega_e * i_d + omega_e * cfg.pm_flux_linkage) / cfg.stator_inductance;
        let i_d = i_d + self.dt * di_d;
        let i_q = i_q + self.dt * di_q;

        let torque = cfg.pole_pairs * cfg.pm_flux_linkage * i_q;
        let theta = (theta + self.dt * omega).rem_euclid(TAU);
        let omega = omega + self.dt * torque / cfg.rotor_inertia;

        self.state = PMSMState { i_d, i_q, omega, theta };
        let tmp =  ClarkParkResult { d: i_d, q: i_q };

        SimOutput {
            theta,
            omega,
            currents: super::inverse_clark_park(tmp, sc),
        }
    }
}