use crate::{boards::PWM_FREQ};
use field_oriented::{
    ClarkParkValue, MotorParams, MotorParamEstimator, HallCalibrator, OfflineMotorEstimator,
    FocInputType, ControllerParameters, compute_current_pi_controller_gains
};

pub struct CalibrationInputs {
    pub hall_angle: f32,
    pub hall_pattern: u8,
    pub target_voltage: f32,
    pub target_speed: f32,
}

/// Outputs needed regardless of stage
pub struct CalibrationOutput {
    pub rotor_angle_rad: f32,
    pub foc_command: FocInputType,
}

/// Stage specific outputs / results
pub enum StageResult {
    HallCalibration {
        angle_table: [f32; 6],
        pole_count: u8,
    },
    MotorParameters {
        motor_params: MotorParams,
        pi_gains: ControllerParameters,
    },
    Failure
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
                    // Start the motor estimation:
                    self.phase = CalibrationPhase::MotorEstimation;
                    self.motor_estimator.active = true;
                    let output = self.motor_estimation_output(inputs);
                    (output, Some(result))
                } else {
                    let angle = self.hall_calibrator.calibration_step(inputs.hall_pattern, inputs.target_speed);
                    let output = CalibrationOutput {
                        rotor_angle_rad: angle,
                        foc_command: FocInputType::RawVoltage(ClarkParkValue { q: inputs.target_voltage, d: 0.0 }),
                    };
                    (output, None)
                }
            }
            CalibrationPhase::MotorEstimation => {
                if self.motor_estimator.check_estimation_done() {
                    self.motor_estimator.active = false;
                    self.phase = CalibrationPhase::Done;
                    let output = self.idle_output(inputs);
                    // Try to tune the PI controller:
                    let motor_params = self.motor_estimator.get_params();
                    let pi_gains_opt = compute_current_pi_controller_gains::<50>(
                        motor_params, PWM_FREQ.0 as f32
                    );
                    if let Some(pi_gains) = pi_gains_opt {
                        let result = StageResult::MotorParameters { motor_params, pi_gains };
                        return (output, Some(result))
                    } else {
                        return (output, Some(StageResult::Failure))
                    }
                } else {
                    let output = self.motor_estimation_output(inputs);
                    (output, None)
                }
            }
            CalibrationPhase::Done => {
                let output = self.idle_output(inputs);
                (output, None)
            }
        }
    }
    
    fn motor_estimation_output(&mut self, inputs: CalibrationInputs) -> CalibrationOutput {
        let angle = if matches!(self.motor_estimator.)
        let voltages = self.motor_estimator.get_target_voltages(inputs.target_voltage);
        CalibrationOutput {
            rotor_angle_rad: ,
            foc_command: FocInputType::RawVoltage(voltages),
        }
    }

    fn idle_output(&self, inputs: CalibrationInputs) -> CalibrationOutput {
        CalibrationOutput {
            rotor_angle_rad: inputs.hall_angle,
            foc_command: FocInputType::RawVoltage(ClarkParkValue { d: 0.0, q: 0.0 }),
        }
    }

    pub fn all_done(&self) -> bool {
        matches!(self.phase, CalibrationPhase::Done)
    }
}