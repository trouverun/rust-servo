
use core::f32::consts::PI;

use embassy_stm32::{peripherals::CORDIC, timer::{low_level::FilterValue, pwm::{PWM, Running as PwmRunning}}};
use embassy_stm32::{Config as RccConfig};
use embassy_stm32::rcc::*;
use embassy_stm32::adc::{
    ExternalTriggeredADC, EocInterruptEnabled, JeosInterruptEnabled, 
    Running as AdcRunning, Queued, AnyAdcChannel, AdcConfig, SampleTime, StartMode,
    Adc, Exten, Dual, Adc345InjectedTrigger
};
use embassy_stm32::cordic::{Cordic, Config, Function, Precision, Scale};
use embassy_stm32::pac::timer::{vals::{Bkp}};
use embassy_stm32::timer::hall::{HallSensor};
#[cfg(feature = "hall-feedback")]
use field_oriented::HasRotorFeedback;
use crate::boards::*;
use field_oriented::{RotorFeedback, SinCos};

const ADC_VOLTAGE: f32 = 3.3;
const OPAMP_GAIN_RECIPROCAL: f32 = 1.0 / BOARD.opamp_gain;
const SHUNT_CONDUCTANCE_SIEMENS: f32 = 1000.0 / BOARD.shunt_resistance_mohm;
const ADC_RESULT_SCALER: f32 = 0.5 / ((1 << 12) - 1) as f32 * ADC_VOLTAGE * OPAMP_GAIN_RECIPROCAL * SHUNT_CONDUCTANCE_SIEMENS;

pub struct PhaseCurrents {
    pub u: f32,
    pub v: f32,
    pub w: f32
}

pub fn init() -> embassy_stm32::Peripherals {
    // Configure sysclk (to 170MHz)
    let mut rcc_config = RccConfig::default();
    rcc_config.rcc.hsi = true;
    rcc_config.rcc.pll = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL85,
        divr: Some(PllRDiv::DIV2),
        divq: None,
        divp: Some(PllPDiv::DIV8),
    });
    rcc_config.rcc.sys = Sysclk::PLL1_R;
    rcc_config.rcc.ahb_pre = AHBPrescaler::DIV1;
    rcc_config.rcc.apb1_pre = APBPrescaler::DIV1;
    rcc_config.rcc.apb2_pre = APBPrescaler::DIV1;
    rcc_config.rcc.mux.adc12sel = mux::Adcsel::PLL1_P;
    rcc_config.rcc.mux.adc345sel = mux::Adcsel::PLL1_P;
    embassy_stm32::init(rcc_config)
}

pub struct CurrentFeedback {
    #[cfg(feature = "mcu-opamps")]
    _opamps: ShuntOpAmps,
    u_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    v_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    w_channel: AnyAdcChannel<'static, FeedbackAdcB>,
    adc_a: ExternalTriggeredADC<'static, FeedbackAdcA, AdcRunning, Queued>,
    adc_b: ExternalTriggeredADC<'static, FeedbackAdcB, AdcRunning, Queued>,
    sampled_sector: u8,
}

impl CurrentFeedback {
    pub fn new(mappings: CurrentFeedbackMappings) -> Self {
        let CurrentFeedbackMappings { 
            #[cfg(feature = "mcu-opamps")]
            opamps,
            adc_a, adc_b, 
            u_channel, v_channel, w_channel,
        } = mappings;
        
        let mut adc_a_config: AdcConfig = AdcConfig::default();
        adc_a_config.dual_mode = Some(Dual::INDEPENDENT);
        adc_a_config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
        let adc_a = Adc::new(adc_a, adc_a_config)
            .to_external_triggered_queued()
            .using_sampletimes(&[
                (u_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (v_channel.get_hw_channel(), SampleTime::CYCLES6_5)])
            .start(EocInterruptEnabled::DISABLED, JeosInterruptEnabled::ENABLED, StartMode::EMPTY);

        let mut adc_b_config = AdcConfig::default();
        adc_b_config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
        let adc_b = Adc::new(adc_b, adc_b_config)
            .to_external_triggered_queued()
            .using_sampletimes(&[
                (v_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (w_channel.get_hw_channel(), SampleTime::CYCLES6_5)])
            .start(EocInterruptEnabled::DISABLED, JeosInterruptEnabled::ENABLED, StartMode::EMPTY);

        Self {
            _opamps: opamps,
            u_channel, v_channel, w_channel,
            adc_a, adc_b,
            sampled_sector: 0
        }.calibrate_adc_offset()
    }

    fn calibrate_adc_offset(self) -> Self {
        let mut val_u = 0i32;
        let mut val_va = 0i32;
        let mut val_vb = 0i32;
        let mut val_w = 0i32;
        self.adc_a.insert_injected_context(&[self.u_channel.get_hw_channel()], FEEDBACK_TRIGGER_A, Exten::RISING_EDGE);
        self.adc_b.insert_injected_context(&[self.w_channel.get_hw_channel()], FEEDBACK_TRIGGER_B, Exten::RISING_EDGE);
        for i in 0..200 {
            if i < 100 {
                val_u += self.adc_a.read_injected::<1>()[0] as i32;
                val_vb += self.adc_b.read_injected::<1>()[0] as i32;
                self.adc_a.insert_injected_context(
                    &[self.u_channel.get_hw_channel()], FEEDBACK_TRIGGER_A, Exten::RISING_EDGE);
                self.adc_b.insert_injected_context(
                    &[self.v_channel.get_hw_channel()], FEEDBACK_TRIGGER_B, Exten::RISING_EDGE);
            } else {
                val_va += self.adc_a.read_injected::<1>()[0] as i32;
                val_w += self.adc_b.read_injected::<1>()[0] as i32;
                self.adc_a.insert_injected_context(
                    &[self.v_channel.get_hw_channel()], FEEDBACK_TRIGGER_A, Exten::RISING_EDGE);
                self.adc_b.insert_injected_context(
                    &[self.w_channel.get_hw_channel()], FEEDBACK_TRIGGER_B, Exten::RISING_EDGE);
            }
        }
        let offset_u = -(val_u / 100) as i16;
        let offset_va = -(val_va / 100) as i16;
        let offset_vb = -(val_vb / 100) as i16;
        let offset_w = -(val_w / 100) as i16;
        let tmp_a = self.adc_a.stop()
            .using_offsets(&[(self.u_channel.get_hw_channel(), offset_u), (self.v_channel.get_hw_channel(), offset_va)])
            .start(EocInterruptEnabled::DISABLED, JeosInterruptEnabled::ENABLED, StartMode::EMPTY);
        let tmp_b = self.adc_b.stop()
            .using_offsets(&[(self.v_channel.get_hw_channel(), offset_vb), (self.w_channel.get_hw_channel(), offset_w)])
            .start(EocInterruptEnabled::DISABLED, JeosInterruptEnabled::ENABLED, StartMode::EMPTY);
        
        Self {
            _opamps: self._opamps,
            u_channel: self.u_channel, v_channel: self.v_channel, w_channel: self.w_channel,
            adc_a: tmp_a, adc_b: tmp_b,
            sampled_sector: self.sampled_sector
        }
    }

    pub fn read_currents(&self) -> Option<PhaseCurrents>{
        if self.adc_a.check_jeos() {
            // U or V:
            let result_a= self.adc_a.read_injected::<1>()[0];
            // V or W:
            let result_b= self.adc_b.read_injected::<1>()[0];
            let amps_a = result_a as f32 * ADC_RESULT_SCALER;
            let amps_b = result_b as f32 * ADC_RESULT_SCALER;
            let amps_c = -(amps_a + amps_b);
            match self.sampled_sector {
                // 1 0 0, sampled VW:
                0 => return Some(PhaseCurrents{u: amps_c, v: amps_a, w: amps_b}), 
                // 1 1 0, sampled UW
                1 => return Some(PhaseCurrents{u: amps_a, v: amps_b, w: amps_c}),
                // 0 1 0, sampled UW 
                2 => return Some(PhaseCurrents{u: amps_a, v: amps_b, w: amps_c}),
                // 0 1 1, sampled UV
                3 => return Some(PhaseCurrents{u: amps_a, v: amps_b, w: amps_c}),
                // 0 0 1, sampled UV
                4 => return Some(PhaseCurrents{u: amps_a, v: amps_b, w: amps_c}),
                // 1 0 1, sampled VW
                5 => return Some(PhaseCurrents{u: amps_c, v: amps_a, w: amps_b}),
                _ => return None
            }
        } else {
            return None
        }
    }

    pub fn sample_sector(&mut self, sector: u8) {
        let (source_a, source_b) = match self.sampled_sector {
            // 1 0 0, sampled VW:
            0 => (self.v_channel.get_hw_channel(), self.w_channel.get_hw_channel()), 
            // 1 1 0, sampled UW
            1 => (self.u_channel.get_hw_channel(), self.w_channel.get_hw_channel()), 
            // 0 1 0, sampled UW 
            2 => (self.u_channel.get_hw_channel(), self.w_channel.get_hw_channel()), 
            // 0 1 1, sampled UV
            3 =>(self.u_channel.get_hw_channel(), self.v_channel.get_hw_channel()), 
            // 0 0 1, sampled UV
            4 => (self.u_channel.get_hw_channel(), self.v_channel.get_hw_channel()), 
            // 1 0 1, sampled VW
            5 => (self.v_channel.get_hw_channel(), self.w_channel.get_hw_channel()), 
            _ => return ()
        };
        self.adc_a.insert_injected_context(&[source_a], Adc345InjectedTrigger::Tim8Trgo2, Exten::RISING_EDGE);
        self.adc_b.insert_injected_context(&[source_b], Adc345InjectedTrigger::Tim8Trgo2, Exten::RISING_EDGE);
        self.sampled_sector = sector;
    }
}

#[cfg(feature = "hall-feedback")]
pub struct HallFeedback {
    pub sensor: HallSensor<'static, HallFeedbackTimer>,
    hall_pattern_to_angle: [f32; 4],
}

#[cfg(feature = "hall-feedback")]
impl HallFeedback {
    pub fn new(mappings: HallFeedbackMappings) -> Self {
        

        Self {
            sensor: mappings.sensor,
            hall_pattern_to_angle: [0.0, 0.0, 0.0, 0.0]
        }
    }

    pub fn on_interrupt(&mut self) {
        self.sensor.on_interrupt();
    }

    pub fn get_pattern(&self) -> u8 {
        self.sensor.read_hall_pattern()
    }
}

#[cfg(feature = "hall-feedback")]
impl HasRotorFeedback for HallFeedback {
    fn read(&self) -> RotorFeedback {
        let raw_state = self.sensor.read_state();
        RotorFeedback {
            angle: 0.0,
            speed: 0.0
            
        }
    }
}

pub struct PwmOutput {
    #[cfg(feature = "overcurrent-comparators")]
    comparators: CurrentComparators,
    pwm: PWM<'static, PwmTimer, PwmRunning>
}

impl PwmOutput {
    pub fn new(mappings: PwmOutputMappings) -> Self {        
        let mut tmp = mappings.pwm
            .with_peak_trgo2_from_ch4()
            .with_deadtime(mappings.deadtime);

        #[cfg(feature = "overcurrent-comparators")]
        {
            mappings.comparators.dac_single.set_voltage(0.2);
            mappings.comparators.dac_dual.set_voltage(0.2, 0.2);
            tmp = tmp
                .with_break1_comp(&mappings.comparators.comp_u, Bkp::ACTIVE_HIGH, FilterValue::FCK_INT_N4)
                .with_break1_comp(&mappings.comparators.comp_v, Bkp::ACTIVE_HIGH, FilterValue::FCK_INT_N4)
                .with_break1_comp(&mappings.comparators.comp_w, Bkp::ACTIVE_HIGH, FilterValue::FCK_INT_N4);
        }

        Self {
            comparators: mappings.comparators,
            pwm: tmp.start()
        }
    }

    pub fn wait_break2_ready(&self) {
        for _ in 0..10000 {
            let brake_set = self.pwm.acknowledge_break2();
            if !brake_set {
                break
            }
        }
        self.pwm.clear_fault();
    }
}

pub struct Acceleration {
    sin_cordic: Cordic<'static, CORDIC>, 
}

impl Acceleration {
    pub fn new(mappings: AccelerationMappings) -> Self {
        let cordic_config = Config::new(Function::Sin, Precision::Iters12, Scale::Arg1Res1);
        let Ok(config) = cordic_config else {
            panic!("Invalid CORDIC config!");
        };
        let sin_cordic = Cordic::new(mappings.cordic, config);
        Self {
            sin_cordic
        }
    }
    
}

impl SinCos for Acceleration {
    fn sin_cos(&mut self, angle_rad: f32) -> (f32, f32) {
        const PI_RECIPROCAL: f32 = 1.0 / PI;
        let tmp = (angle_rad * PI_RECIPROCAL).clamp(-1.0, 1.0);
        let angle_q1_31 = (tmp * i32::MAX as f32) as i32;

        let mut result = [0u32; 2];
        self.sin_cordic.blocking_calc_32bit(&[angle_q1_31 as u32], &mut result, true, false).unwrap();

        const Q1_31_RECIP: f32 = 1.0 / i32::MAX as f32;
        (result[0] as i32 as f32 * Q1_31_RECIP, result[1] as i32 as f32 * Q1_31_RECIP)
    }
}