/// Accumulator for solving y = a*x via least-squares: a = sum(x*y) / sum(x^2)
pub struct Lse {
    xy_sum: f32,
    xx_sum: f32,
    num_data: u32,
    overflow: bool
}

impl Lse {
    pub fn new() -> Self {
        Self { xy_sum: 0.0, xx_sum: 0.0, num_data: 0, overflow: false }
    }

    pub fn accumulate(&mut self, x: f32, y: f32) {
        let xy = x * y;
        let xx = x * x;
        let xy_ok = f32::MIN < self.xy_sum + xy && self.xy_sum + xy < f32::MAX;
        let xx_ok = f32::MIN < self.xx_sum + xx && self.xx_sum + xx < f32::MAX;
        if  xy_ok && xx_ok {
            self.xy_sum += xy;
            self.xx_sum += xx;
            self.num_data += 1;
        } else {
            self.overflow = true;
        }
    }

    pub fn solve(&self, min_data: u32) -> Option<f32> {
        if !self.overflow && self.num_data > min_data && self.xx_sum > 1e-12 {
            Some(self.xy_sum / self.xx_sum)
        } else {
            None
        }
    }
}
