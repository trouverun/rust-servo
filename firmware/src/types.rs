use field_oriented::{FocFault, EstimationStepFault};
use crate::calibration::CalibrationFailureCause;

#[derive(Clone, Copy, defmt::Format)]
pub enum FaultCause {
    Empty,
    Overcurrent,
    CalibrationFail,
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
            CalibrationFailureCause::MissingParameter => FaultCause::MissingMotorParams,
            CalibrationFailureCause::MotorParameterEstimation { fault } => fault.into()
        }
    }
}

pub enum Command {
    Idle,
    StartCalibration,
    FinishCalibration,
    StartTuning,
    FinishTuning,
    EnableTorqueControl,
    EnableVelocityControl,
    AssertFault { cause: FaultCause },
}

#[derive(Clone, Copy, defmt::Format)]
pub enum OperatingMode {
    Idle,
    Calibration,
    ControllerTuning,
    TorqueControl,
    VelocityControl,
    Fault { 
        write_index: usize, 
        trace: [FaultCause; 16]
    }
}

impl OperatingMode {
    pub fn on_command(&mut self, command: Command) -> Self {
        match (*self, command) {
            // Transition from fault to fault, append to fault trace:
            (OperatingMode::Fault { mut write_index, mut trace }, Command::AssertFault { cause }) => {
                if write_index < trace.len() {
                    trace[write_index] = cause;
                    write_index += 1;
                }
                OperatingMode::Fault { write_index, trace }
            },
            // Transition from any other mode to fault, start from scratch:
            (_, Command::AssertFault { cause }) => {
                let mut trace = [FaultCause::Empty; 16];
                trace[0] = cause;
                OperatingMode::Fault { write_index: 1, trace }
            },
            (OperatingMode::Idle, Command::StartCalibration) => OperatingMode::Calibration,
            (OperatingMode::Idle, Command::EnableTorqueControl) => OperatingMode::TorqueControl,
            (OperatingMode::Calibration, Command::StartTuning) => OperatingMode::ControllerTuning,
            (OperatingMode::Calibration, Command::FinishCalibration) => OperatingMode::Idle,
            (OperatingMode::ControllerTuning, Command::FinishTuning) => OperatingMode::Calibration,
            (OperatingMode::TorqueControl, Command::Idle) => OperatingMode::Idle,
            (_, _) => *self
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
    DoublePress
}

impl ButtonState {
    pub fn on_edge(&self, edge: EdgeType) -> Self {
        match (self, edge) {
            (ButtonState::Waiting, EdgeType::Rising) => ButtonState::LongPress,
            (ButtonState::LongPress, EdgeType::Falling) => ButtonState::ShortPress,
            (ButtonState::ShortPress, EdgeType::Rising) => ButtonState::DoublePress,
            (_, _) => *self
        }
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
            current_limit: 0.0
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
            target_omega: 0.0, target_torque: 0.0
        }
    }
}