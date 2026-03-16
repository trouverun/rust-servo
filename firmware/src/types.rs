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