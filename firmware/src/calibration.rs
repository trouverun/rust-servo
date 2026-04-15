use crate::{boards::PWM_FREQ};
use field_oriented::{
    ClarkParkValue, MotorParamsEstimate, MotorParamEstimator, HallCalibrator, OfflineMotorEstimator,
    OfflineEstimatorConfig, FocInputType, ControllerParameters, OfflineEstimatorOutput, 
    OfflineEstimatorInput, compute_current_pi_controller_gains
};

pub struct CalibrationInputs {
    pub theta: f32,
    pub hall_pattern: u8,
    pub num_pole_pairs: Option<u8>,
    pub target_voltage: f32,
    pub target_current: f32,
    pub target_omega: f32,
}

/// Outputs needed regardless of stage
pub struct CalibrationOutput {
    pub theta: f32,
    pub foc_command: FocInputType,
}

pub enum FailureCause {
    MissingParameter,
    MotorParameterEstimation,
    PiTuning
}

/// Stage specific outputs / results
pub enum StageResult {
    HallCalibration {
        angle_table: [f32; 6]
    },
    MotorParameters {
        motor_params: MotorParamsEstimate
    },
    /// Separate stage with idle outputs,
    /// so the PI tuning can safely consume multiple FOC iterations
    Tuning {
        pi_gains: ControllerParameters,
    },
    Failure {
        cause: FailureCause
    }
}

enum CalibrationPhase {
    HallCalibration,
    MotorEstimation,
    Tuning,
    Done,
}

pub struct CalibrationRunner {
    hall_calibrator: HallCalibrator,
    motor_estimator: OfflineMotorEstimator,
    phase: CalibrationPhase,
}

impl CalibrationRunner {
    pub fn new() -> Self {
        let hall_calibrator = HallCalibrator::new(1.0, 0.01);
        let cfg = OfflineEstimatorConfig {
            dt: 1.0 / PWM_FREQ.0 as f32,
            settle_time_s: 1.0,
            test_time_s: 1.0
        };
        let motor_estimator = OfflineMotorEstimator::new(cfg);
        Self {
            hall_calibrator,
            motor_estimator,
            phase: CalibrationPhase::HallCalibration,
        }
    }

    pub fn start(&mut self) {
        self.phase = CalibrationPhase::HallCalibration;
        self.hall_calibrator.start();
    }

    pub fn tick(&mut self, 
        inputs: CalibrationInputs,
    ) -> (CalibrationOutput, Option<StageResult>) {
        match self.phase {
            CalibrationPhase::HallCalibration => {
                if self.hall_calibrator.check_calibration_done() {
                    let result = StageResult::HallCalibration {
                        angle_table: self.hall_calibrator.hall_pattern_to_theta,
                    };
                    // Number of pole pairs should have been configured already:
                    if let Some(num_pole_pairs) = inputs.num_pole_pairs {
                        self.phase = CalibrationPhase::MotorEstimation;
                        self.motor_estimator.start(num_pole_pairs);
                        let output = self.motor_estimation_output(inputs);
                        (output, Some(result))
                    } else {
                        self.phase = CalibrationPhase::Done;
                        let output = self.idle_output(inputs);
                        (output, Some(StageResult::Failure { cause: FailureCause::MissingParameter }))
                    }
                } else {
                    const PWM_FREQ_HZ : u32 = PWM_FREQ.0;
                    let angle = self.hall_calibrator.calibration_step::<PWM_FREQ_HZ>(
                        inputs.hall_pattern, inputs.target_omega
                    );
                    let output = CalibrationOutput {
                        theta: angle,
                        foc_command: FocInputType::TargetCurrents(ClarkParkValue {
                            d: inputs.target_current, q: 0.0
                        })
                    };
                    (output, None)
                }
            }
            CalibrationPhase::MotorEstimation => {
                if self.motor_estimator.estimation_done() {
                    self.phase = CalibrationPhase::Tuning;
                    let output = self.idle_output(inputs);
                    let result = Some(StageResult::MotorParameters { 
                        motor_params: self.motor_estimator.get_estimate()
                    });
                    (output, result)
                } else if self.motor_estimator.estimation_failed() {
                    self.phase = CalibrationPhase::Done;
                    let output = self.idle_output(inputs);
                    (output, Some(StageResult::Failure { cause: FailureCause::MotorParameterEstimation }))
                } else {
                    let output = self.motor_estimation_output(inputs);
                    (output, None)
                }
            }
            CalibrationPhase::Tuning => {
                self.phase = CalibrationPhase::Done;
                let output = self.idle_output(inputs);
                if let Some(motor_params) = self.motor_estimator.get_estimate().to_params() {
                    let pi_gains_opt = compute_current_pi_controller_gains::<50>(
                        motor_params, PWM_FREQ.0 as f32
                    );
                    if let Some(pi_gains) = pi_gains_opt {
                        let result = StageResult::Tuning { pi_gains };
                        return (output, Some(result))
                    }
                }
                (output, Some(StageResult::Failure { cause: FailureCause::PiTuning }))
            }
            CalibrationPhase::Done => {
                let output = self.idle_output(inputs);
                (output, None)
            }
        }
    }
    
    fn motor_estimation_output(&mut self, inputs: CalibrationInputs) -> CalibrationOutput {
        let step_input = OfflineEstimatorInput {
            target_voltage: inputs.target_voltage,
            target_current: inputs.target_current,
            theta: inputs.theta
        };
        let estimator_command = self.motor_estimator.step(step_input);
        let foc_command = match estimator_command.output {
            OfflineEstimatorOutput::Current(i_dq) => FocInputType::TargetCurrents(i_dq),
            OfflineEstimatorOutput::Voltage(u_dq) => FocInputType::TargetVoltage(u_dq),
        };
        CalibrationOutput {
            theta: estimator_command.theta,
            foc_command
        }
    }

    fn idle_output(&self, inputs: CalibrationInputs) -> CalibrationOutput {
        CalibrationOutput {
            theta: inputs.theta,
            foc_command: FocInputType::TargetVoltage(ClarkParkValue { d: 0.0, q: 0.0 }),
        }
    }

    pub fn get_estimator(&mut self) -> &mut OfflineMotorEstimator {
        &mut self.motor_estimator
    }

    pub fn all_done(&self) -> bool {
        matches!(self.phase, CalibrationPhase::Done)
    }
}