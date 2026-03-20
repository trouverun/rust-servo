use core::f32::consts::PI;
use crate::{boards::PWM_FREQ};

pub enum UserCommand {
    StartCalibration,
    FinishCalibration,
    EnableControl,
}

#[derive(Clone, Copy)]
pub enum ApplicationState {
    Startup, 
    Idle,
    Calibration,
    Running,
}

impl ApplicationState {
    pub fn on_command(&self, command: UserCommand) -> Self {
        match (self, command) {
            (ApplicationState::Startup, UserCommand::StartCalibration) => ApplicationState::Calibration,
            (ApplicationState::Idle, UserCommand::EnableControl) => ApplicationState::Running,
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

pub enum ControlMode {
    Velocity,
    Torque
}

pub struct RuntimeConfig {
    pub application_state: ApplicationState,
    pub control_mode: ControlMode,
    pub target_velocity: f32,
    pub target_torque: f32,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            application_state: ApplicationState::Startup,
            control_mode: ControlMode::Velocity,
            target_velocity: 0.0, target_torque: 0.0
        }
    }
}

pub struct CalibrationState {
    target_angle_rad: f32,
    iteration_counter: u32,
    settling_time_waited_s: f32,
    prev_pattern: u8,
    pub hall_pattern_to_angle: [f32; 4]
}

impl CalibrationState {
    pub fn step(&mut self, hall_pattern: u8) -> f32 {
        let angle = self.target_angle_rad;
        if self.iteration_counter >= 1000 {
            const TIME_PASSED: f32 = 1000.0 / PWM_FREQ.0 as f32;
            if self.settling_time_waited_s >= 1.0 {
                self.target_angle_rad += 0.62 * TIME_PASSED;
                if self.prev_pattern != hall_pattern {
                    self.hall_pattern_to_angle[hall_pattern.clamp(1, 4) as usize] = self.target_angle_rad;
                }
            } else {
                self.settling_time_waited_s += TIME_PASSED;
                self.prev_pattern = hall_pattern;
            }
            self.iteration_counter = 0;
        } else {
            self.iteration_counter += 1;
        }
        angle
    }
    pub fn done(&self) -> bool {
        self.target_angle_rad > PI
    }
}

impl Default for CalibrationState {
    fn default() -> Self {
        Self {
            target_angle_rad: -PI,
            iteration_counter: 0,
            settling_time_waited_s: 0.0,
            prev_pattern: 0,
            hall_pattern_to_angle: [0.0, 0.0, 0.0, 0.0]
        }
    }
}