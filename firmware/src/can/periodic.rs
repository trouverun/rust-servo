use rtic_monotonics::{Monotonic, stm32::{ExtU64, Tim2 as Mono}};

use super::messages::*;

include!(concat!(env!("OUT_DIR"), "/periodic.rs"));

pub struct Slot {
    pub kind: Periodic,
    pub next_due: <Mono as Monotonic>::Instant,
    pub period: <Mono as Monotonic>::Duration,
}

impl Slot {
    pub fn new(kind: Periodic, now: <Mono as Monotonic>::Instant) -> Self {
        let period = (kind.period_us() as u64).micros();
        Self { kind, next_due: now + period, period }
    }
}
