use embassy_stm32::can::frame::Frame;
use super::messages::MotorCurrents;

pub trait IntoFrame {
    fn into_frame(&self) -> Frame;
}

macro_rules! impl_into_frame {
    ($($msg:ty),* $(,)?) => {
        $(impl IntoFrame for $msg {
            fn into_frame(&self) -> Frame {
                Frame::new_standard(<$msg>::MESSAGE_ID as u16, self.raw()).unwrap()
            }
        })*
    };
}

impl_into_frame!(MotorCurrents);
