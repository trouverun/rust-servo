use embassy_stm32::can::frame::Frame;

use super::messages::*;

pub trait IntoFrame {
    fn into_frame(&self) -> Frame;
}

include!(concat!(env!("OUT_DIR"), "/frames.rs"));
