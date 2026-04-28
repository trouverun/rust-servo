use embassy_stm32::adc::AnyAdcChannel;
use embassy_stm32::comp::Comp;
use embassy_stm32::dac::{Dac, DacChannel};
#[cfg(feature = "spi-encoder")]
use embassy_stm32::gpio::Output;
#[cfg(feature = "rs485-encoder")]
use embassy_stm32::mode::Async;
use embassy_stm32::mode::Blocking;
#[cfg(feature = "mcu-opamps")]
use embassy_stm32::opamp::OpAmpOutput;
use embassy_stm32::peripherals::{CORDIC, FLASH};
#[cfg(feature = "spi-encoder")]
use embassy_stm32::spi::{mode::Master, Spi};
use embassy_stm32::time::Hertz;
#[cfg(feature = "hall-feedback")]
use embassy_stm32::timer::hall::HallSensor;
use embassy_stm32::timer::pwm::{NotRunning, PwmDeadtime, PWM};
use embassy_stm32::timer::{trigger_output::BasicTrgoOutput, CountingMode};
#[cfg(feature = "rs485-encoder")]
use embassy_stm32::usart::Uart;
use embassy_stm32::Peri;

#[cfg(feature = "board-zest1")]
mod zest1;
#[cfg(feature = "board-zest1")]
pub use zest1::*;

pub const PWM_FREQ: Hertz = Hertz(20_000);
pub const COUNTING_MODE: CountingMode = CountingMode::CenterAlignedBothInterrupts;

pub struct TherimistorLinearScale {
    pub slope: f32,
    pub bias: f32,
}

pub struct BoardInfo {
    pub shunt_resistance_mohm: f32,
    pub opamp_gain: f32,
    pub vbus_divide_factor: f32,
    pub thermistor_scaling: TherimistorLinearScale,
}

#[cfg(feature = "mcu-opamps")]
pub struct ShuntOpAmps {
    u: OpAmpOutput<'static, OpAmpU>,
    v: OpAmpOutput<'static, OpAmpV>,
    w: OpAmpOutput<'static, OpAmpW>,
}

pub struct AdcFeedbackMappings {
    #[cfg(feature = "mcu-opamps")]
    pub opamps: ShuntOpAmps,
    pub adc_a: Peri<'static, FeedbackAdcA>,
    pub adc_b: Peri<'static, FeedbackAdcB>,
    pub u_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    pub v_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    pub w_channel: AnyAdcChannel<'static, FeedbackAdcB>,
    pub vbus_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    pub tboard_channel: AnyAdcChannel<'static, FeedbackAdcB>,
    pub sample_trigger: BasicTrgoOutput<'static, AdcFeedbackTimer>,
}

#[cfg(feature = "hall-feedback")]
pub struct HallFeedbackMappings {
    pub sensor: HallSensor<'static, HallFeedbackTimer>,
}

#[cfg(feature = "spi-encoder")]
pub struct SPIEncoderMappings {
    pub spi: Spi<'static, Blocking, Master>,
    pub cs: Output<'static>,
}

#[cfg(feature = "overcurrent-comparators")]
pub struct CurrentComparators {
    pub dac_single: DacChannel<'static, ComparatorDacSingle, ComparatorDacChannel, Blocking>,
    pub dac_dual: Dac<'static, ComparatorDacDual, Blocking>,
    pub comp_u: Comp<'static, CompU>,
    pub comp_v: Comp<'static, CompV>,
    pub comp_w: Comp<'static, CompW>,
}

pub struct PwmOutputMappings {
    #[cfg(feature = "overcurrent-comparators")]
    pub comparators: CurrentComparators,
    pub pwm: PWM<'static, PwmTimer, NotRunning>,
    pub deadtime: PwmDeadtime,
}

pub struct AccelerationMappings {
    pub cordic: Peri<'static, CORDIC>,
}

pub struct MemoryMappings {
    pub flash: Peri<'static, FLASH>,
}
