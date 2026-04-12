/// Accumulator for solving y = a*x via least-squares: a = sum(x*y) / sum(x^2)
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
