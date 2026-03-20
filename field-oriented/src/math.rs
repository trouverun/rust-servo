use super::types::*;

/// = 0.5
const A: f32 = 0.5;
/// = sqrt(3)/2
const B: f32 = 0.86602540378; 
/// = 2/3
const C: f32 = 0.66666666666;

pub(crate) fn forward_clark_park(vals: PhaseValues, sc: SinCosResult) -> ClarkParkResult {
    let d = C * (sc.cos * vals.u + (-A*sc.cos + B*sc.sin) * vals.v + (-A*sc.cos - B*sc.sin) * vals.w);
    let q = C * (-sc.sin * vals.u + (A*sc.sin + B*sc.cos) * vals.v + (A*sc.sin - B*sc.cos) * vals.w);
    ClarkParkResult { d, q }
}

pub(crate) fn inverse_clark_park(vals: ClarkParkResult, sc: SinCosResult) -> PhaseValues {
    let u = sc.cos * vals.d - sc.sin * vals.q;
    let v = (-A*sc.cos + B*sc.sin) * vals.d + (A*sc.sin + B*sc.cos) * vals.q;
    let w = (-A*sc.cos - B*sc.sin) * vals.d + (A*sc.sin - B*sc.cos) * vals.q;
    PhaseValues { u, v, w }
}

pub(crate) fn inverse_park(vals: ClarkParkResult, sc: SinCosResult) -> AlphaBeta {
    AlphaBeta {
        alpha: sc.cos * vals.d - sc.sin * vals.q,
        beta:  sc.sin * vals.d + sc.cos * vals.q,
    }
}

pub(crate) fn inverse_clarke(ab: AlphaBeta) -> PhaseValues {
    PhaseValues {
        u: ab.alpha,
        v: -A * ab.alpha + B * ab.beta,
        w: -A * ab.alpha - B * ab.beta,
    }
}

const SQRT3: f32 = 1.73205080757;
pub(crate) fn voltage_sector(ab: &AlphaBeta) -> u8 {
    let a = ab.beta > 0.0;
    let b = ab.beta >  SQRT3 * ab.alpha;
    let c = ab.beta > -SQRT3 * ab.alpha;
    match (a as u8) | ((b as u8) << 1) | ((c as u8) << 2) {
        0b101 => 0,
        0b111 => 1,
        0b011 => 2,
        0b010 => 3,
        0b000 => 4,
        0b100 => 5,
        _ => 0,
    }
}

pub(crate) fn min3(a: f32, b: f32, c: f32) -> f32 {
    a.min(b).min(c)
}

pub(crate) fn max3(a: f32, b: f32, c: f32) -> f32 {
    a.max(b).max(c)
}