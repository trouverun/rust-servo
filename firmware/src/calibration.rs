use crate::boards::PWM_FREQ;
use crate::types::{CalibrationPhase, CalibrationRunner};
use field_oriented::{
    ClarkParkValue, EstimationStepFault, FocInputType, HallCalibrator, MotorParamEstimator,
    MotorParamsEstimate, OfflineEstimatorConfig, OfflineEstimatorInput, OfflineEstimatorOutput,
    OfflineMotorEstimator,
};

pub struct CalibrationInputs {
    pub dc_bus_voltage: f32,
    pub theta: f32,
    pub hall_pattern: u8,
    pub target_voltage: f32,
    pub target_current: f32,
    pub target_omega: f32,
}

/// Outputs needed regardless of stage
pub struct CalibrationOutput {
    pub theta: f32,
    pub foc_command: FocInputType,
}

#[derive(defmt::Format)]
pub enum CalibrationFailureCause {
    MissingParameter,
    MotorParameterEstimation { fault: EstimationStepFault },
}

/// Stage specific outputs / results
pub enum StageResult {
    HallCalibration { angle_table: [f32; 6] },
    ResetRequest,
    TuningRequest { params_estimate: MotorParamsEstimate },
    MotorParameters { motor_params: MotorParamsEstimate },
    Failure { cause: CalibrationFailureCause },
}

impl CalibrationRunner {
    pub fn new(num_pole_pairs: u8) -> Self {
        let hall_calibrator = HallCalibrator::new(3.0);
        let cfg = OfflineEstimatorConfig {
            dt: 1.0 / PWM_FREQ.0 as f32,
            settle_time_s: 3.0,
            test_time_s: 5.0,
            max_spin_time_s: 15.0,
            min_spin_omega: 250.0,
        };
        let motor_estimator = OfflineMotorEstimator::new(cfg, num_pole_pairs);
        Self {
            num_pole_pairs,
            hall_calibrator,
            motor_estimator,
            phase: CalibrationPhase::HallCalibration,
        }
    }

    pub fn start(&mut self) {
        self.phase = CalibrationPhase::HallCalibration;
        self.hall_calibrator.start();
    }

    pub fn step(&mut self, inputs: CalibrationInputs) -> (CalibrationOutput, Option<StageResult>) {
        match self.phase {
            CalibrationPhase::HallCalibration => {
                if self.hall_calibrator.check_calibration_done() {
                    let result = StageResult::HallCalibration {
                        angle_table: self.hall_calibrator.hall_pattern_to_theta,
                    };
                    self.phase = CalibrationPhase::MotorEstimation;
                    self.motor_estimator.start(self.num_pole_pairs);
                    let output = self.motor_estimation_output(inputs);
                    (output, Some(result))
                } else {
                    const PWM_FREQ_HZ: u32 = PWM_FREQ.0;
                    let theta = self
                        .hall_calibrator
                        .calibration_step::<PWM_FREQ_HZ>(inputs.hall_pattern, inputs.target_omega);
                    let output = CalibrationOutput {
                        theta,
                        foc_command: FocInputType::CalibrationCurrents(ClarkParkValue {
                            d: inputs.target_current,
                            q: 0.0,
                        }),
                    };
                    (output, None)
                }
            }
            CalibrationPhase::MotorEstimation => {
                if self.motor_estimator.should_reset_controller() {
                    let output = self.idle_output(inputs);
                    (output, Some(StageResult::ResetRequest))
                } else if self.motor_estimator.should_tune_controller() {
                    let output = self.idle_output(inputs);
                    (output, Some(StageResult::TuningRequest { params_estimate: self.motor_estimator.get_estimate() } ))
                } else if self.motor_estimator.estimation_done() {
                    self.phase = CalibrationPhase::Done;
                    let output = self.idle_output(inputs);
                    let result = Some(StageResult::MotorParameters {
                        motor_params: self.motor_estimator.get_estimate(),
                    });
                    (output, result)
                } else if let Some(fault) = self.motor_estimator.get_fault() {
                    self.phase = CalibrationPhase::Done;
                    let output = self.idle_output(inputs);
                    let result = StageResult::Failure {
                        cause: CalibrationFailureCause::MotorParameterEstimation { fault },
                    };
                    (output, Some(result))
                } else {
                    let output = self.motor_estimation_output(inputs);
                    (output, None)
                }
            }
            _ => {
                let output = self.idle_output(inputs);
                (output, None)
            }
        }
    }

    fn motor_estimation_output(&mut self, inputs: CalibrationInputs) -> CalibrationOutput {
        let step_input = OfflineEstimatorInput {
            dc_bus_voltage: inputs.dc_bus_voltage,
            target_voltage: inputs.target_voltage,
            target_current: inputs.target_current,
            theta: inputs.theta,
        };
        let estimator_command = self.motor_estimator.get_command(step_input);
        let foc_command = match estimator_command.output {
            OfflineEstimatorOutput::CalibrationCurrent(i_dq) => {
                FocInputType::CalibrationCurrents(i_dq)
            }
            OfflineEstimatorOutput::CalibrationVoltage(u_dq) => {
                FocInputType::CalibrationVoltage(u_dq)
            }
            OfflineEstimatorOutput::Current(i_dq) => FocInputType::TargetCurrents(i_dq),
        };
        CalibrationOutput {
            theta: estimator_command.theta,
            foc_command,
        }
    }

    fn idle_output(&self, inputs: CalibrationInputs) -> CalibrationOutput {
        CalibrationOutput {
            theta: inputs.theta,
            foc_command: FocInputType::CalibrationVoltage(ClarkParkValue { d: 0.0, q: 0.0 }),
        }
    }

    pub fn get_estimator(&mut self) -> &mut OfflineMotorEstimator {
        &mut self.motor_estimator
    }

    pub fn tuning_completed(&mut self) {
        self.phase = CalibrationPhase::MotorEstimation;
    }
}
