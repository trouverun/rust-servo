use core::f32::consts::TAU;
use field_oriented::PhaseValues;
use crate::boards::PWM_FREQ;

pub fn iir_cutoff_to_alpha(cutoff_hz: f32) -> f32 {
    libm::expf(-TAU * cutoff_hz / PWM_FREQ.0 as f32)
}

pub struct LowPassFilter {
    alpha: f32,
    prev_filtered_value: f32,
    prev_measurement: f32
}

impl LowPassFilter {
    pub fn new(cutoff_hz: f32) -> Self {
        Self {
            alpha: iir_cutoff_to_alpha(cutoff_hz),
            prev_filtered_value: 0.0,
            prev_measurement: 0.0
        }
    }

    pub fn update(&mut self, measurement: f32) -> f32 {
        self.prev_filtered_value = self.alpha * self.prev_filtered_value + (1.0-self.alpha) * self.prev_measurement;
        self.prev_measurement = measurement;
        self.prev_filtered_value
    }

    pub fn filtered(&self) -> f32 {
        self.prev_filtered_value
    }
}

pub struct FilteredPhases {
    u: LowPassFilter,
    v: LowPassFilter,
    w: LowPassFilter
}

pub struct PhaseCurrentFilter {
    filters: FilteredPhases,
    overcurrent_limit: f32,
}

impl PhaseCurrentFilter {
    pub fn new(lowpass_cutoff_hz: f32, overcurrent_limit: f32) -> Self {
        let filters = FilteredPhases {
            u: LowPassFilter::new(lowpass_cutoff_hz),
            v: LowPassFilter::new(lowpass_cutoff_hz),
            w: LowPassFilter::new(lowpass_cutoff_hz)
        };
        Self {
            filters,
            overcurrent_limit,
        }
    }

    /// Update the filter with a new measurement.
    pub fn update(&mut self, measurement: PhaseValues) {
        self.filters.u.update(measurement.u);
        self.filters.v.update(measurement.v);
        self.filters.w.update(measurement.w);
    }

    pub fn check_overcurrent(&self) -> bool {
        self.filters.u.filtered().abs() > self.overcurrent_limit
            || self.filters.v.filtered() > self.overcurrent_limit
            || self.filters.w.filtered() > self.overcurrent_limit
    }

    pub fn set_overcurrent_limit(&mut self, limit: f32) {
        self.overcurrent_limit = limit;
    }

    pub fn filtered(&self) -> PhaseValues {
        PhaseValues { 
            u: self.filters.u.filtered(),
            v: self.filters.v.filtered(),
            w: self.filters.w.filtered() 
        }
    }
}
