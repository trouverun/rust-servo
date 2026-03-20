

#[derive(Clone, Copy)]
pub struct PIGains {
    pub kr: f32,
    pub kp: f32,
    pub ki: f32,
}

pub struct PIController {
    gains: PIGains,
    integral_term: f32,
}

impl PIController {
    pub fn new(gains: PIGains) -> Self {
        Self {
            gains,
            integral_term: 0.0
        }
    }

    pub fn compute(&mut self, reference: f32, measurement: f32, saturation_error: f32) -> f32 {
        let proportional = self.gains.kp * (self.gains.kr * reference - measurement);
        self.integral_term += self.gains.ki * (reference - measurement + saturation_error / (self.gains.kp * self.gains.kr));
        proportional + self.integral_term
    }

    pub fn set_gains(&mut self, gains: PIGains) {
        self.gains = gains;
    }
}