use embassy_stm32::pac::Interrupt;
use embassy_stm32::comp::*;
use embassy_stm32::dac::{Ch1};
use embassy_stm32::peripherals::{
    OPAMP3, OPAMP4, OPAMP5,
    ADC3, ADC4,
    TIM3, TIM8,
    COMP4, COMP6, COMP7,
    DAC1, DAC4,
    ADC1, TIM6, PD15, TIM2
};
use embassy_stm32::opamp::{OpAmp, OpAmpSpeed};
use embassy_stm32::adc::{
    Adc345InjectedTrigger, AdcChannel, AdcConfig, AnyAdcChannel, 
    ExternalTriggeredADC, NotQueued, Running, SampleTime, Exten,
    Adc, Resolution, Adc12RegularTrigger, JeosInterruptEnabled, EocInterruptEnabled
};
use embassy_stm32::timer::low_level::{RoundTo, Timer};
use embassy_stm32::{time::Hertz};
use embassy_stm32::timer::{pwm::{PWM, PwmDeadtime}, low_level::FilterValue, trigger_output::BasicTrgoOutput};
use embassy_stm32::pac::timer::vals::{Bkinp, Bkp};
use embassy_stm32::timer::hall::{Config as HallConfig, HallSensor};
use embassy_stm32::dac::{Dac, DacCh1};
use embassy_stm32::exti::{ExtiInput, TriggerEdge};
use embassy_stm32::{mode::Blocking};
use embassy_stm32::gpio::{Level, Output, Speed, Pull};


// Current feedback:
pub type OpAmpU = OPAMP3;
pub type OpAmpV = OPAMP4;
pub type OpAmpW = OPAMP5;
pub type FeedbackAdcA = ADC3;
pub type FeedbackAdcB = ADC4;
pub const FEEDBACK_TRIGGER_A: Adc345InjectedTrigger = Adc345InjectedTrigger::Tim8Trgo2;
pub const FEEDBACK_TRIGGER_B: Adc345InjectedTrigger = Adc345InjectedTrigger::Tim8Trgo2;

// Hall feedback:
pub type HallFeedbackTimer = TIM3;

// PWM output:
pub type CompU = COMP4;
pub type CompV = COMP6;
pub type CompW = COMP7;
pub type ComparatorDacSingle = DAC1;
pub type ComparatorDacChannel = Ch1;
pub type ComparatorDacDual = DAC4; 
pub type PwmTimer = TIM8;

pub const BOARD: super::BoardInfo = super::BoardInfo {
    shunt_resistance_mohm: 15.0,
    opamp_gain: 15.0
};

pub struct DebugMappings {
    pub basic_trigger: BasicTrgoOutput<'static, TIM6>,
    pub pot_channel: AnyAdcChannel<'static, ADC1>,
    pub pot_adc: ExternalTriggeredADC<'static, ADC1, Running, NotQueued>,
    pub user_btn: ExtiInput<'static, Blocking>,
    pub la_pin: Output<'static>,
    pub btn_timer: Timer<'static, TIM2>,
}

pub fn map_peripherals(p: embassy_stm32::Peripherals) -> 
    (
        super::CurrentFeedbackMappings,
        super::HallFeedbackMappings,   
        super::PwmOutputMappings,
        super::AccelerationMappings,
        super::MemoryMappings,
        DebugMappings
    ) {

    let current_feedback = super::CurrentFeedbackMappings {
        opamps: super::ShuntOpAmps {
            u: OpAmp::new(p.OPAMP3, OpAmpSpeed::HighSpeed).standalone_ext(p.PB0, p.PB2, p.PB1),
            v: OpAmp::new(p.OPAMP4, OpAmpSpeed::HighSpeed).standalone_ext(p.PB11, p.PB10, p.PB12),
            w: OpAmp::new(p.OPAMP5, OpAmpSpeed::HighSpeed).standalone_ext(p.PB14, p.PB15, p.PA8),     
        },
        adc_a: p.ADC3,
        adc_b: p.ADC4,
        u_channel: AdcChannel::<FeedbackAdcA>::degrade_adc(p.PD11),
        v_channel: AdcChannel::<FeedbackAdcA>::degrade_adc(p.PD12),
        w_channel: AdcChannel::<FeedbackAdcB>::degrade_adc(p.PD14),
    };

    let hall_feedback = super::HallFeedbackMappings {
        sensor: HallSensor::new(
            p.TIM3, p.PE2, p.PE3, p.PE4, 
            HallConfig::default()
        )
    };

    let pwm = super::PwmOutputMappings {
        comparators: super::CurrentComparators {
            dac_single: DacCh1::new_internal_blocking(p.DAC1, 3.3), 
            dac_dual: Dac::new_internal_blocking(p.DAC4, 3.3),
            comp_u: Comp::new(p.COMP4, Comp4InpSel::PB0, Comp4InmSel::Dac1Ch1, Comp4BlankSel::None), 
            comp_v: Comp::new(p.COMP6, Comp6InpSel::PB11, Comp6InmSel::Dac4Ch2, Comp6BlankSel::None), 
            comp_w: Comp::new(p.COMP7, Comp7InpSel::PB14, Comp7InmSel::Dac4Ch1, Comp7BlankSel::None)
        },
        pwm: PWM::new(p.TIM8, super::PWM_FREQ, super::COUNTING_MODE)
                .with_ch1(p.PC6).with_ch1n(p.PC10)
                .with_ch2(p.PC7).with_ch2n(p.PC11)
                .with_ch3(p.PC8).with_ch3n(p.PC12)
                .with_break2_pin(p.PD1, Bkinp::INVERTED, Bkp::ACTIVE_HIGH, FilterValue::FCK_INT_N4),
        deadtime: PwmDeadtime::Nanosecods(200)
    };

    let acceleration = super::AccelerationMappings {
        cordic: p.CORDIC,
    };

    let storage = super::MemoryMappings {
        flash: p.FLASH,
    };
    
    // Board specific debug assignments:
    let pot_channel = AdcChannel::degrade_adc(p.PA3);
    let mut pot_adc_config = AdcConfig::default();
    pot_adc_config.resolution = Some(Resolution::BITS12);
    let pot_adc = Adc::new(p.ADC1, pot_adc_config)
            .to_external_triggered()
            .using_sampletimes(&[(pot_channel.get_hw_channel(), SampleTime::CYCLES12_5)])
            .with_sequence(&[pot_channel.get_hw_channel()], Adc12RegularTrigger::Tim6Trgo, Exten::RISING_EDGE)
            .start(EocInterruptEnabled::ENABLED, JeosInterruptEnabled::DISABLED);
    
    let mut user_btn = ExtiInput::new_blocking(p.PC13, p.EXTI13, Pull::Up, TriggerEdge::Any);
    user_btn.enable_interrupt();
        
    let btn_timer = Timer::new(p.TIM2);
    btn_timer.set_period_secs(1, RoundTo::Slower);
    btn_timer.enable_update_interrupt(true);

    let debug = DebugMappings {
        basic_trigger: BasicTrgoOutput::new(p.TIM6, Hertz(100)),
        pot_channel,
        pot_adc,
        user_btn,
        la_pin: Output::new(p.PD15, Level::Low, Speed::Low),
        btn_timer
    };        

    (current_feedback, hall_feedback, pwm, acceleration, storage, debug)
}
