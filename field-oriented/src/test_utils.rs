extern crate std;
use std::vec::Vec;
use std::string::String;

use crate::{DoesFocMath, FocInput, FocInputType, FocResult};
use crate::sim::SimOutput;

pub struct DummyAccelerator;
impl DoesFocMath for DummyAccelerator {
    fn sin_cos(&mut self, angle_rad: f32) -> crate::SinCosResult {
        crate::SinCosResult {
            cos: angle_rad.cos(), sin: angle_rad.sin()
        }
    }

    fn sqrt(&mut self, val: f32) -> f32 {
        val.sqrt()
    }
}

