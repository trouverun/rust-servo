use crate::calibration::CalibrationFailureCause;
use field_oriented::{EstimationStepFault, FocFault, HallCalibrator, OfflineMotorEstimator};
use defmt::{Format, Formatter, write, info};

#[derive(Clone, Copy, defmt::Format)]
pub enum FaultCause {
    Empty,
    Overcurrent,
    Break1,
    Break2,
    CalibrationTimeout,
    EstimationFail,
    ControllerTuningFail,
    MissingMotorParams,
}

impl From<FocFault> for FaultCause {
    fn from(f: FocFault) -> Self {
        match f {
            FocFault::MissingMotorParams => FaultCause::MissingMotorParams,
        }
    }
}

impl From<EstimationStepFault> for FaultCause {
    fn from(f: EstimationStepFault) -> Self {
        match f {
            EstimationStepFault::MissingParameter => FaultCause::MissingMotorParams,
            EstimationStepFault::Overflow
            | EstimationStepFault::InsufficientSamples
            | EstimationStepFault::DegenSolution
            | EstimationStepFault::ParameterOutOfBounds => FaultCause::EstimationFail,
        }
    }
}

impl From<CalibrationFailureCause> for FaultCause {
    fn from(f: CalibrationFailureCause) -> Self {
        match f {
            CalibrationFailureCause::Timeout => FaultCause::CalibrationTimeout,
            CalibrationFailureCause::MissingParameter => FaultCause::MissingMotorParams,
            CalibrationFailureCause::MotorParameterEstimation { fault } => fault.into(),
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub enum Command {
    Idle,
    StartCalibration { num_pole_pairs: u8 },
    FinishTuning,
    FinishCalibration,
    EnableTorqueControl,
    EnableVelocityControl,
    AssertFault { cause: FaultCause },
}

#[derive(Clone, Copy, defmt::Format)]
pub enum CalibrationPhase {
    EncoderZeroing { duration_waited_s: f32, reset_sent: bool },
    HallCalibration,
    MotorEstimation,
    MotorTuning,
    Done,
}

pub struct CalibrationRunner {
    pub num_pole_pairs: u8,
    pub hall_calibrator: HallCalibrator,
    pub motor_estimator: OfflineMotorEstimator,
    pub phase: CalibrationPhase,
}

pub enum OperatingMode {
    Idle,
    Calibration { calibrator: CalibrationRunner },
    TorqueControl,
    VelocityControl,
    Fault {
        write_index: usize,
        trace: [FaultCause; 16],
    },
}

impl Format for OperatingMode {
    fn format(&self, f: Formatter<'_>) {
        match self {
            OperatingMode::Idle => {
                write!(f, "Idle")
            }
            OperatingMode::Calibration { calibrator, .. } => {
                write!(f, "Calibration {{ phase: {} }}", calibrator.phase)
            }
            OperatingMode::TorqueControl => {
                write!(f, "TorqueControl")
            }
            OperatingMode::VelocityControl => {
                write!(f, "VelocityControl")
            }
            OperatingMode::Fault { write_index, trace } => {
                write!(f, "Fault {{ write_index: {}, trace: {} }}", write_index, trace)
            }
        }
    }
}

impl OperatingMode {
    pub fn on_command(&mut self, command: Command) {
        info!("On command {}, state {}", command, &*self);
        let new_state = match (&mut *self, command) {
            (OperatingMode::Fault { write_index, trace }, Command::AssertFault { cause }) => {
                if *write_index < trace.len() {
                    trace[*write_index] = cause;
                    *write_index += 1;
                }
                return;
            }
            (_, Command::AssertFault { cause }) => {
                let mut trace = [FaultCause::Empty; 16];
                trace[0] = cause;
                OperatingMode::Fault { write_index: 1, trace }
            }
            (OperatingMode::Idle, Command::StartCalibration { num_pole_pairs } ) => {
                OperatingMode::Calibration { calibrator: CalibrationRunner::new(num_pole_pairs) }
            }
            (OperatingMode::Idle, Command::EnableTorqueControl) => OperatingMode::TorqueControl,
            (OperatingMode::Calibration { calibrator }, Command::FinishTuning) => {
                calibrator.tuning_completed();
                return;
            }
            (OperatingMode::Calibration { .. }, Command::FinishCalibration) => OperatingMode::Idle,
            (OperatingMode::TorqueControl, Command::Idle) => OperatingMode::Idle,
            (_, _) => return,
        };
        *self = new_state;
    }
}

pub struct BoardStatus {
    pub dc_bus_voltage: Option<f32>,
    pub temperature: Option<f32>,
}

pub struct FirmwareConfig {
    pub calibration_voltage: f32,
    pub calibration_current: f32,
    pub calibration_omega: f32,
    pub current_limit: f32,
}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {
            calibration_voltage: 18.0,
            calibration_current: 1.5,
            calibration_omega: 1.0,
            current_limit: 0.0,
        }
    }
}

pub struct RuntimeValues {
    pub target_omega: f32,
    pub target_torque: f32,
}

impl Default for RuntimeValues {
    fn default() -> Self {
        Self {
            target_omega: 0.0,
            target_torque: 0.0,
        }
    }
}

#[derive(Clone, Copy)]
pub enum EdgeType {
    Rising,
    Falling,
}

#[derive(Clone, Copy)]
pub enum ButtonState {
    Waiting,
    LongPress,
    ShortPress,
    DoublePress,
}

impl ButtonState {
    pub fn on_edge(&self, edge: EdgeType) -> Self {
        match (self, edge) {
            (ButtonState::Waiting, EdgeType::Rising) => ButtonState::LongPress,
            (ButtonState::LongPress, EdgeType::Falling) => ButtonState::ShortPress,
            (ButtonState::ShortPress, EdgeType::Rising) => ButtonState::DoublePress,
            (_, _) => *self,
        }
    }
}
