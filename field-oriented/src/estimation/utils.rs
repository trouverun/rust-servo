/// First-order IIR low-pass filter: y[k] = (1 - α) * y[k-1] + α * x[k-1]
pub struct LowPassFilter {
    alpha: f32,
    prev_input: f32,
    output: f32,
}

impl LowPassFilter {
    pub fn new(cutoff_hz: f32, dt: f32) -> Self {
        let omega_c = 2.0 * core::f32::consts::PI * cutoff_hz;
        Self {
            alpha: (omega_c * dt) / (1.0 + omega_c * dt),
            prev_input: 0.0,
            output: 0.0,
        }
    }

    pub fn update(&mut self, input: f32) -> f32 {
        self.output = (1.0 - self.alpha) * self.output + self.alpha * self.prev_input;
        self.prev_input = input;
        self.output
    }
}

/// Accumulator for solving y = a*x via least-squares: a = Σ(x·y) / Σ(x²)
pub struct Lse {
    xy_sum: f32,
    xx_sum: f32,
}

impl Lse {
    pub fn new() -> Self {
        Self { xy_sum: 0.0, xx_sum: 0.0 }
    }

    pub fn accumulate(&mut self, x: f32, y: f32) {
        self.xy_sum += x * y;
        self.xx_sum += x * x;
    }

    pub fn solve(&self) -> Option<f32> {
        if self.xx_sum > 1e-12 {
            Some(self.xy_sum / self.xx_sum)
        } else {
            None
        }
    }
}
