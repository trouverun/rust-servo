use core::f32::consts::TAU;
use field_oriented::PhaseValues;
use crate::boards::PWM_FREQ;

pub fn iir_cutoff_to_alpha(cutoff_hz: f32) -> f32 {
    libm::expf(-TAU * cutoff_hz / PWM_FREQ.0 as f32)
}

pub struct PhaseCurrentFilter {
    alpha: f32,
    filtered: PhaseValues,
    prev_measurement: PhaseValues,
    overcurrent_limit: f32,
}

impl PhaseCurrentFilter {
    pub fn new(lowpass_cutoff_hz: f32, overcurrent_limit: f32) -> Self {
        Self {
            alpha: iir_cutoff_to_alpha(lowpass_cutoff_hz),
            filtered: PhaseValues { u: 0.0, v: 0.0, w: 0.0 },
            prev_measurement: PhaseValues { u: 0.0, v: 0.0, w: 0.0 },
            overcurrent_limit,
        }
    }

    /// Update the filter with a new measurement.
    pub fn update(&mut self, measurement: PhaseValues) {
        let a = self.alpha;
        let b = 1.0 - a;
        self.filtered.u = a * self.filtered.u + b * self.prev_measurement.u;
        self.filtered.v = a * self.filtered.v + b * self.prev_measurement.v;
        self.filtered.w = a * self.filtered.w + b * self.prev_measurement.w;
        self.prev_measurement = measurement;
    }

    pub fn overcurrent(&self) -> bool {
        self.filtered.u.abs() > self.overcurrent_limit
            || self.filtered.v.abs() > self.overcurrent_limit
            || self.filtered.w.abs() > self.overcurrent_limit
    }

    pub fn set_overcurrent_limit(&mut self, limit: f32) {
        self.overcurrent_limit = limit;
    }

    pub fn filtered(&self) -> PhaseValues {
        self.filtered
    }
}
