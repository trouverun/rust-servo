pub enum Command {
    StartCalibration,
    FinishCalibration,
    EnableTorqueControl,
    EnableVelocityControl
}

#[derive(Clone, Copy)]
pub enum OperatingMode {
    Idle,
    Calibration,
    TorqueControl,
    VelocityControl
}

impl OperatingMode {
    pub fn on_command(&self, command: Command) -> Self {
        match (self, command) {
            (OperatingMode::Idle, Command::StartCalibration) => OperatingMode::Calibration,
            (OperatingMode::Calibration, Command::FinishCalibration) => OperatingMode::Idle,
            (OperatingMode::Idle, Command::EnableTorqueControl) => OperatingMode::TorqueControl,
            (_, _) => *self
        }
    }
}

#[derive(Clone, Copy)]
pub enum ButtonState {
    Waiting,
    LongPress,
    ShortPress,
    DoublePress
}   

impl ButtonState {
    pub fn on_edge(&self, rising: bool) -> Self {
        match (self, rising) {
            (ButtonState::Waiting, true) => ButtonState::LongPress,
            (ButtonState::LongPress, false) => ButtonState::ShortPress,
            (ButtonState::ShortPress, true) => ButtonState::DoublePress,
            (_, _) => *self
        }
    }
}

pub struct BoardStatus {
    pub dc_bus_voltage: f32,
    pub temperature: f32,
}

pub struct FirmwareConfig {
    pub calibration_voltage: f32,
    pub calibration_current: f32,
    pub calibration_omega: f32,
}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {
            calibration_voltage: 6.0,
            calibration_current: 0.33,
            calibration_omega: 33.0,
        }
    }
}

pub struct RuntimeValues {
    pub target_velocity: f32,
    pub target_torque: f32,
}

impl Default for RuntimeValues {
    fn default() -> Self {
        Self {
            target_velocity: 0.0, target_torque: 0.0
        }
    }
}