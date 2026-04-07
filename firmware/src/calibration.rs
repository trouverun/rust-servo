use core::f32::consts::PI;
use crate::{boards::PWM_FREQ};

use field_oriented::{};
use field_oriented::{
    ClarkParkValue, MotorParams, MotorParamEstimator, FocIterationData,
    FocInputType, ControllerParameters, compute_current_pi_controller_gains
};

pub struct HallCalibrator {
    target_angle_rad: f32,
    iteration_counter: u32,
    settling_time_waited_s: f32,
    prev_pattern: Option<u8>,
    pub hall_pattern_to_angle: [f32; 6],
}

impl HallCalibrator {
    /// Increment the target rotor angle by a small amount
    pub fn calibration_step(&mut self, hall_pattern: u8, rotation_speed_rad_s: f32) -> f32 {
        if self.prev_pattern == None {
            self.prev_pattern = Some(hall_pattern);
        }

        let angle = self.target_angle_rad;
        // Step by 100 FOC ISRs at a time (around 10 kHz to 100 Hz)
        if self.iteration_counter >= 100 {
            const TIME_PASSED_S: f32 = 100.0 / PWM_FREQ.0 as f32;
            if self.settling_time_waited_s >= 0.1 {
                self.target_angle_rad += rotation_speed_rad_s * TIME_PASSED_S;
                if let Some(prev_pattern) = self.prev_pattern {
                    if prev_pattern != hall_pattern {
                        // Hall pattern changed = Hall edge at this angle
                        let idx = (hall_pattern.clamp(1, 6) - 1) as usize;
                        self.hall_pattern_to_angle[idx] = self.target_angle_rad;
                    }
                }
            } else {
                self.settling_time_waited_s += TIME_PASSED_S;
            }
            self.iteration_counter = 0;
        } else {
            self.iteration_counter += 1;
        }
        self.prev_pattern = Some(hall_pattern);
        
        angle
    }
    
    pub fn check_calibration_done(&self) -> bool {
        self.target_angle_rad > PI
    }
}

impl HallCalibrator {
    pub fn new() -> Self {
        Self {
            target_angle_rad: -PI,
            iteration_counter: 0,
            settling_time_waited_s: 0.0,
            prev_pattern: None,
            hall_pattern_to_angle: [0.0; 6],
        }
    }
}

#[cfg(test)]
mod hall_test {
    use core::f32::consts::PI;
    use field_oriented::{ClarkParkValue, FOC, FocConfig, FocInput, PMSMConfig, PMSMSim, SimOutput};

    use crate::calibration::HallCalibrator;

    fn hall_pattern_coder(rotor_angle: f32) -> u8 {
        if rotor_angle >= 0.0 && rotor_angle < 1/3*PI {
            return 0b110
        } else if rotor_angle >= 1/3*PI && rotor_angle < 2/3*PI {
            return 0b010
        } else if rotor_angle >= 2/3*PI && rotor_angle < PI {
            return 0b011
        } else if rotor_angle >= PI && rotor_angle < 4/3*PI {
            return 0b001
        } else if rotor_angle >= 4/3*PI && rotor_angle < 5/3*PI {
            return 0b101
        } else if rotor_angle >= 5/3*PI && rotor_angle < 2*PI {
            return 0b100
        }
        0b000
    }

    #[test]
    fn hall_calibration_works() { 
        let sim_cfg = PMSMConfig::default();
        let sim = PMSMSim::new(0.01, sim_cfg);

        let foc_cfg = FocConfig {
            saturation_d_ratio: 0.0
        };
        let foc = FOC::new(foc_cfg);

        let calibrator = HallCalibrator::new();

        let state = sim.state();
        while !calibrator.check_calibration_done() {
            let pattern = hall_pattern_coder(state.theta);
            let angle = calibrator.calibration_step(pattern, 0.25);

            let foc_input = FocInput {
                dc_bus_voltage: 24.0,
                command: field_oriented::FocInputType::RawVoltage(ClarkParkValue {
                    d: 0.0, q: 5.0
                }),
                rotor_angle_rad: angle,
                phase_currents: state.currents
            };

            let foc_result = foc.compute(foc_input, accelerator, estimator);
            state = sim.step(input);
        }
    }
}


// --------------------------------------------------------------------------------------------------------------------

enum OfflineMotorEstimatorState {
    Off,
    RotorLockWait,
    // R = v_d/i_d (Rotor locked, steady state)
    EstR,
    // L  = (v_d - R*i_d) / di_d/dt (Rotor locked)
    EstL,
    // pm_flux_linkage = (v_q - R*i_q) / omega_e (Steady state)
    EstF,
    Done
}

pub struct OfflineMotorEstimator {
    state: OfflineMotorEstimatorState,
    pub active: bool,
    pub params: MotorParams,
}

impl OfflineMotorEstimator {
    pub fn new() -> Self {
        let params = MotorParams { 
            num_pole_pairs: 1,
            pm_flux_linkage: 0.0, 
            stator_inductance: 0.0, 
            stator_resistance: 0.0, 
            rotor_inductance: 0.0 
        };

        Self {
            state: OfflineMotorEstimatorState::Off,
            active: false,
            params  
        }
    }

    pub fn get_target_voltages(&mut self, target_voltage: f32) -> ClarkParkValue {
        if self.active {
            match self.state {
                OfflineMotorEstimatorState::Off => { self.state = OfflineMotorEstimatorState::RotorLockWait }
                OfflineMotorEstimatorState::RotorLockWait => {

                }
                OfflineMotorEstimatorState::EstR => {
                    return ClarkParkValue { d: 0.0, q: 0.0 }
                }
                OfflineMotorEstimatorState::EstL => {
                    return ClarkParkValue { d: 0.0, q: 0.0 }
                }
                OfflineMotorEstimatorState::EstF => {
                    return ClarkParkValue { d: 0.0, q: 0.0 }
                }
                OfflineMotorEstimatorState::Done => {}
            }
        }
        return ClarkParkValue { d: 0.0, q: 0.0 }
    }

    pub fn should_rotor_lock(&self) -> bool {
        self.active && matches!(self.state, OfflineMotorEstimatorState::RotorLockWait 
                                            | OfflineMotorEstimatorState::EstR 
                                            | OfflineMotorEstimatorState::EstL)
    }

    pub fn check_estimation_done(&self) -> bool {
        matches!(self.state, OfflineMotorEstimatorState::Done)
    }
}

impl MotorParamEstimator for OfflineMotorEstimator {
    fn initialize_params(&mut self, params: MotorParams) {}
    fn after_foc_iteration(&mut self, data: FocIterationData) {}
    fn get_params(&self) -> MotorParams {
        self.params
    }
}

// --------------------------------------------------------------------------------------------------------------------

pub struct CalibrationInputs {
    pub hall_angle: f32,
    pub hall_pattern: u8,
    pub target_voltage: f32,
    pub target_speed: f32,
}

pub struct CalibrationOutput {
    pub rotor_angle_rad: f32,
    pub foc_command: FocInputType,
    pub rotor_lock_prompt: bool,
}

pub enum StageResult {
    HallCalibration {
        angle_table: [f32; 6],
        pole_count: u8,
    },
    MotorParameters {
        params: MotorParams,
        pi_gains: ControllerParameters,
    },
}

enum CalibrationPhase {
    HallCalibration,
    MotorEstimation,
    Done,
}

pub struct CalibrationRunner {
    hall_calibrator: HallCalibrator,
    pub motor_estimator: OfflineMotorEstimator,
    phase: CalibrationPhase,
}

impl CalibrationRunner {
    pub fn new(hall_calibrator: HallCalibrator, motor_estimator: OfflineMotorEstimator) -> Self {
        Self {
            hall_calibrator,
            motor_estimator,
            phase: CalibrationPhase::HallCalibration,
        }
    }

    pub fn tick(
        &mut self,
        inputs: CalibrationInputs,
    ) -> (CalibrationOutput, Option<StageResult>) {
        match self.phase {
            CalibrationPhase::HallCalibration => {
                if self.hall_calibrator.check_calibration_done() {
                    let result = StageResult::HallCalibration {
                        angle_table: self.hall_calibrator.hall_pattern_to_angle,
                        pole_count: self.hall_calibrator.detected_pole_count(),
                    };
                    self.phase = CalibrationPhase::MotorEstimation;
                    self.motor_estimator.active = true;
                    let output = Self::motor_estimation_output(motor_estimator, inputs);
                    (output, Some(result))
                } else {
                    let angle = self.hall_calibrator.calibration_step(inputs.hall_pattern, inputs.target_speed);
                    let output = CalibrationOutput {
                        rotor_angle_rad: angle,
                        foc_command: FocInputType::RawVoltage(ClarkParkValue { q: inputs.target_voltage, d: 0.0 }),
                        rotor_lock_prompt: false,
                    };
                    (output, None)
                }
            }
            CalibrationPhase::MotorEstimation => {
                if self.motor_estimator.check_estimation_done() {
                    self.motor_estimator.active = false;
                    self.phase = CalibrationPhase::Done;
                    let params = self.motor_estimator.params;
                    let pi_gains = compute_current_pi_controller_gains(params);
                    let result = StageResult::MotorParameters { params, pi_gains };
                    let output = Self::motor_estimation_output(self.motor_estimator, inputs);
                    (output, Some(result))
                } else {
                    let voltages = self.motor_estimator.get_target_voltages(inputs.target_voltage);
                    let output = CalibrationOutput {
                        rotor_angle_rad: inputs.hall_angle,
                        foc_command: FocInputType::RawVoltage(voltages),
                        rotor_lock_prompt: self.motor_estimator.should_rotor_lock(),
                    };
                    (output, None)
                }
            }
            CalibrationPhase::Done => {
                let output = CalibrationOutput {
                    rotor_angle_rad: inputs.hall_angle,
                    foc_command: FocInputType::RawVoltage(ClarkParkValue { d: 0.0, q: 0.0 }),
                    rotor_lock_prompt: false,
                };
                (output, None)
            }
        }
    }

    pub fn all_done(&self) -> bool {
        matches!(self.phase, CalibrationPhase::Done)
    }
}