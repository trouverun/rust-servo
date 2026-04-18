pub enum Command {
    Idle,
    StartCalibration,
    FinishCalibration,
    StartTuning,
    FinishTuning,
    EnableTorqueControl,
    EnableVelocityControl,
    Fault,
}

#[derive(Clone, Copy, defmt::Format)]
pub enum OperatingMode {
    Idle,
    Calibration,
    Tuning,
    TorqueControl,
    VelocityControl,
    Fault
}

impl OperatingMode {
    pub fn on_command(&self, command: Command) -> Self {
        match (self, command) {
            (_, Command::Fault) => OperatingMode::Fault,
            (OperatingMode::Idle, Command::StartCalibration) => OperatingMode::Calibration,
            (OperatingMode::Idle, Command::EnableTorqueControl) => OperatingMode::TorqueControl,
            (OperatingMode::Calibration, Command::StartTuning) => OperatingMode::Tuning,
            (OperatingMode::Calibration, Command::FinishCalibration) => OperatingMode::Idle,
            (OperatingMode::Tuning, Command::FinishTuning) => OperatingMode::Calibration,
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
    pub temperature: f32,
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
            calibration_current: 2.0,
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