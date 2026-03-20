pub struct RotorFeedback {
    pub angle: f32,
    pub speed: f32
}

pub trait HasRotorFeedback {
    fn read(&self) -> RotorFeedback;
}

#[derive(Clone, Copy)]
pub struct SinCosResult {
    pub sin: f32,
    pub cos: f32
}

pub trait DoesFocMath {
    fn sin_cos(&mut self, angle_rad: f32) -> SinCosResult;
    fn sqrt(&mut self, val: f32) -> f32;
}

pub struct AlphaBeta {
    pub alpha: f32,
    pub beta: f32
}

#[derive(Clone, Copy)]
pub struct ClarkParkResult {
    pub d: f32,
    pub q: f32
}

pub struct PhaseValues {
    pub u: f32,
    pub v: f32,
    pub w: f32
}

pub struct FocInput {
    pub bus_voltage: f32,
    pub rotor_angle_rad: f32,
    pub phase_currents: PhaseValues,
    pub target_torque: f32,
}

pub struct FocResult {
    pub duty_cycles: PhaseValues,
    pub hexagon_sector: u8
}