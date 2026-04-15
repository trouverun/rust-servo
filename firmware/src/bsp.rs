
use core::f32::consts::{PI, TAU};
use num_traits::Float;

use embassy_stm32::{flash::{Blocking as BlockingFlash, Flash}, peripherals::CORDIC, timer::{low_level::FilterValue, pwm::{PWM, Running as PwmRunning}, trigger_output::BasicTrgoOutput}};
use embassy_stm32::{Config as RccConfig};
use embassy_stm32::rcc::*;
use embassy_stm32::adc::{
    ExternalTriggeredADC, EocInterruptEnabled, JeosInterruptEnabled, 
    Running as AdcRunning, Queued, AnyAdcChannel, AdcConfig, SampleTime, StartMode,
    Adc, Exten, Dual, Adc345InjectedTrigger
};
use embassy_stm32::cordic::{Cordic, Precision, NoScale, SqrtScale, Sin, Sqrt, Q15};
use embassy_stm32::pac::timer::{vals::{Bkp}};
use embassy_stm32::timer::hall::{HallSensor};
use embassy_stm32::cordic::utils::{f32_to_q1_15, q1_15_to_f32};
use field_oriented::{ControllerParameters, HasRotorFeedback, MotorParamsEstimate};
use crate::{boards::*, types::FirmwareConfig};
use field_oriented::{DoesFocMath, RotorFeedback, SinCosResult, PhaseValues};

const ADC_VOLTAGE: f32 = 3.3;
const OPAMP_GAIN_RECIPROCAL: f32 = 1.0 / BOARD.opamp_gain;
const SHUNT_CONDUCTANCE_SIEMENS: f32 = 1000.0 / BOARD.shunt_resistance_mohm;
const ADC_SCALER: f32 = ((1 << 12) - 1) as f32 * ADC_VOLTAGE;
const CURRENT_READING_SCALER: f32 = 0.5 * ADC_SCALER * OPAMP_GAIN_RECIPROCAL * SHUNT_CONDUCTANCE_SIEMENS;
const VBUS_READING_SCLAER: f32 = ADC_SCALER * BOARD.vbus_divide_factor;
const TBOARD_READING_SCLAER: f32 = ADC_SCALER * BOARD.thermistor_scaling.slope;

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

pub type BusVoltage = f32;
pub type BoardTemperature = f32;

pub struct AdcFeedback {
    #[cfg(feature = "mcu-opamps")]
    _opamps: ShuntOpAmps,
    u_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    v_channel: AnyAdcChannel<'static, FeedbackAdcA>,
    w_channel: AnyAdcChannel<'static, FeedbackAdcB>,
    adc_a: ExternalTriggeredADC<'static, FeedbackAdcA, AdcRunning, Queued>,
    adc_b: ExternalTriggeredADC<'static, FeedbackAdcB, AdcRunning, Queued>,
    sample_trigger: BasicTrgoOutput<'static, AdcFeedbackTimer>,
    sampled_sector: u8,
}

impl AdcFeedback {
    pub fn new(mappings: AdcFeedbackMappings) -> Self {
        let AdcFeedbackMappings { 
            #[cfg(feature = "mcu-opamps")]
            opamps,
            adc_a, adc_b, 
            u_channel, v_channel, w_channel,
            vbus_channel, tboard_channel,
            sample_trigger
        } = mappings;
        
        let mut adc_a_config: AdcConfig = AdcConfig::default();
        adc_a_config.dual_mode = Some(Dual::INDEPENDENT);
        adc_a_config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
        let adc_a = Adc::new(adc_a, adc_a_config)
            .to_external_triggered_queued()
            .with_sequence(
                &[vbus_channel.get_hw_channel()],
                BOARD_FEEDBACK_TRIGGER,
                Exten::RISING_EDGE
            )
            .using_sampletimes(&[
                (vbus_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (u_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (v_channel.get_hw_channel(), SampleTime::CYCLES6_5)])
            .start(EocInterruptEnabled::DISABLED, JeosInterruptEnabled::ENABLED, StartMode::EMPTY);

        let mut adc_b_config = AdcConfig::default();
        adc_b_config.resolution = Some(embassy_stm32::adc::Resolution::BITS12);
        let adc_b = Adc::new(adc_b, adc_b_config)
            .to_external_triggered_queued()
            .with_sequence(
                &[tboard_channel.get_hw_channel()],
                BOARD_FEEDBACK_TRIGGER,
                Exten::RISING_EDGE
            )
            .using_sampletimes(&[
                (vbus_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (v_channel.get_hw_channel(), SampleTime::CYCLES6_5),
                (w_channel.get_hw_channel(), SampleTime::CYCLES6_5)])
            .start(EocInterruptEnabled::DISABLED, JeosInterruptEnabled::ENABLED, StartMode::EMPTY);

        Self {
            _opamps: opamps,
            u_channel, v_channel, w_channel,
            adc_a, adc_b,
            sample_trigger,
            sampled_sector: 0,
        }.calibrate_opamp_offset()
    }

    #[cfg(not(feature = "mcu-opamps"))]
    fn calibrate_opamp_offset(self) -> Self {
        self
    }

    /// Finds the opamp offsets from N samples for each motor phase, and configures the ADC(s) to negate them
    #[cfg(feature = "mcu-opamps")]
    fn calibrate_opamp_offset(self) -> Self {
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
            sample_trigger: self.sample_trigger,
            sampled_sector: self.sampled_sector
        }
    }

    pub fn read_currents(&self) -> Option<PhaseValues>{
        if self.adc_a.check_jeos() {
            // U or V:
            let result_a= self.adc_a.read_injected::<1>()[0];
            // V or W:
            let result_b= self.adc_b.read_injected::<1>()[0];
            let amps_a = result_a as f32 * CURRENT_READING_SCALER;
            let amps_b = result_b as f32 * CURRENT_READING_SCALER;
            let amps_c = -(amps_a + amps_b);
            match self.sampled_sector {
                // 1 0 0, sampled VW:
                0 => return Some(PhaseValues{u: amps_c, v: amps_a, w: amps_b}), 
                // 1 1 0, sampled UW
                1 => return Some(PhaseValues{u: amps_a, v: amps_b, w: amps_c}),
                // 0 1 0, sampled UW 
                2 => return Some(PhaseValues{u: amps_a, v: amps_b, w: amps_c}),
                // 0 1 1, sampled UV
                3 => return Some(PhaseValues{u: amps_a, v: amps_b, w: amps_c}),
                // 0 0 1, sampled UV
                4 => return Some(PhaseValues{u: amps_a, v: amps_b, w: amps_c}),
                // 1 0 1, sampled VW
                5 => return Some(PhaseValues{u: amps_c, v: amps_a, w: amps_b}),
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

    pub fn read_board_info(&self) -> Option<(BusVoltage, BoardTemperature)> {
        if self.adc_a.check_eoc() {
            let vbus = self.adc_a.read() as f32 * VBUS_READING_SCLAER;
            let tboard = self.adc_b.read() as f32 * TBOARD_READING_SCLAER + BOARD.thermistor_scaling.bias;
            return Some((vbus, tboard))
        } else {
            return None
        }
    }
}

#[cfg(feature = "hall-feedback")]
pub struct HallFeedback {
    pub sensor: HallSensor<'static, HallFeedbackTimer>,
    hall_pattern_to_angle: [f32; 6],
    /// Unsigned angular span of each hall sector, indexed by pattern - 1.
    /// Computed from the calibration table: distance from this edge to the next edge in the forward direction.
    sector_span: [f32; 6],
    /// Forward hall sequence: sector_span[pattern] leads to forward_next[pattern].
    /// Used to determine rotation direction from pattern transitions.
    forward_next: [u8; 6],
    filtered_velocity: f32,
    prev_raw_velocity: f32,
}

#[cfg(feature = "hall-feedback")]
impl HallFeedback {
    const LOWPASS_CUTOFF_HZ: f32 = 1_000.0;
    const LOWPASS_ALPHA: f32 = {
        let x = -TAU * Self::LOWPASS_CUTOFF_HZ / PWM_FREQ.0 as f32;
        // Pade approximation of exp(x) (f32::exp() is not const fn):
        let x2 = x * x;
        let x3 = x2 * x;
        let num = 1.0 + x / 2.0 + x2 / 10.0 + x3 / 120.0;
        let den = 1.0 - x / 2.0 + x2 / 10.0 - x3 / 120.0;
        num / den
    };

    pub fn new(mappings: HallFeedbackMappings) -> Self {
        Self {
            sensor: mappings.sensor,
            hall_pattern_to_angle: [0.0; 6],
            sector_span: [0.0; 6],
            forward_next: [0; 6],
            filtered_velocity: 0.0,
            prev_raw_velocity: 0.0,
        }
    }

    pub fn get_pattern(&self) -> u8 {
        self.sensor.read_hall_pattern()
    }

    pub fn set_calibration(&mut self, calibrations: [f32; 6]) {
        self.hall_pattern_to_angle = calibrations;

        // Build (angle, pattern) pairs and sort by angle.
        // This gives the forward (increasing angle) sequence.
        // Hall patterns are 1..=6, stored at index pattern-1.
        let mut by_angle: [(f32, u8); 6] = core::array::from_fn(|i| (calibrations[i], (i + 1) as u8));
        for i in 1..6 {
            let mut j = i;
            while j > 0 && by_angle[j].0 < by_angle[j - 1].0 {
                by_angle.swap(j, j - 1);
                j -= 1;
            }
        }

        // Walk the sorted sequence. For each pattern, record:
        //  - which pattern comes next in the forward direction
        //  - the angular distance to that next edge (wrapping around at 2pi)
        for i in 0..6 {
            let (angle, pattern) = by_angle[i];
            let (next_angle, next_pattern) = by_angle[(i + 1) % 6];
            let idx = (pattern - 1) as usize;

            self.forward_next[idx] = next_pattern;

            let mut span = next_angle - angle;
            if span < 0.0 {
                span += core::f32::consts::TAU;
            }
            self.sector_span[idx] = span;
        }
    }

    pub fn on_interrupt(&mut self) {
        self.sensor.on_interrupt();
    }

    /// Determine rotation direction from a pattern transition.
    /// Returns 1 for forward, -1 for reverse, 0 if patterns are equal.
    fn direction(&self, prev_pattern: u8, pattern: u8) -> i8 {
        if prev_pattern == pattern {
            return 0;
        }
        let prev_idx = prev_pattern.wrapping_sub(1) as usize;
        if prev_idx < 6 && self.forward_next[prev_idx] == pattern {
            1
        } else {
            -1
        }
    }
}

#[cfg(feature = "hall-feedback")]
impl HasRotorFeedback for HallFeedback {
    fn read(&mut self) -> RotorFeedback {
        // Todo: return Result so caller can deal with invalid patterns 000 and 111
        let raw_state = self.sensor.read_state();
        let idx = (raw_state.pattern.wrapping_sub(1) as usize).min(5);
        let dir = self.direction(raw_state.prev_pattern, raw_state.pattern);
        let span = self.sector_span[idx];

        // Size of the current Hall sector in electrical angle radians:
        let signed_span = dir as f32 * span;
        // Fraction of current sector elapsed (dimensionless: counter/period, ticks cancel)
        let fraction = raw_state.counter as f32 * raw_state.hall_period_reciprocal_cycles;

        let angle = self.hall_pattern_to_angle[idx] + signed_span * fraction;
        let raw_velocity = signed_span * raw_state.hall_period_reciprocal_cycles * self.sensor.timer_frequency_reciprocal_s;

        // IIR low-pass: y[n] = alpha*y[n-1] + (1-alpha)*x[n-1]
        let alpha = Self::LOWPASS_ALPHA;
        self.filtered_velocity = alpha * self.filtered_velocity + (1.0 - alpha) * self.prev_raw_velocity;
        self.prev_raw_velocity = raw_velocity;

        RotorFeedback { theta: angle, omega: self.filtered_velocity }
    }
}

pub struct PwmOutput {
    #[cfg(feature = "overcurrent-comparators")]
    _comparators: CurrentComparators,
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
            _comparators: mappings.comparators,
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

    pub fn set_duty_cycles(&self, duty_cycles: PhaseValues) {
        
    }
}

pub struct Acceleration {
    pub cordic: Cordic<'static, CORDIC>,
}

impl Acceleration {
    pub fn new(mappings: AccelerationMappings) -> Self {
        let cordic = Cordic::new(mappings.cordic);
        Self { cordic }
    }
}

impl DoesFocMath for Acceleration {
    fn sin_cos(&mut self, angle_rad: f32) -> SinCosResult {
        const INV_TAU: f32 = 1.0 / TAU;
        const INV_PI: f32 = 1.0 / PI;
        let reduced = angle_rad - TAU * (angle_rad * INV_TAU).round();
        let angle_normalized = reduced * INV_PI;
        let angle_normalized = angle_normalized.clamp(-1.0, 1.0);

        let angle_q15 = f32_to_q1_15(angle_normalized).unwrap();
        let mut sin_cfg = self.cordic.configure::<Sin, Q15>(Precision::Iters12, NoScale);
        let started = sin_cfg.start1(angle_q15);
        let (sin_raw, cos_raw) = started.result2();

        SinCosResult {
            sin: q1_15_to_f32(sin_raw),
            cos: q1_15_to_f32(cos_raw),
        }
    }

    fn sqrt(&mut self, val: f32) -> f32 {
        // Pre-scale by 4^k to bring val into CORDIC range [0.027, 2.34]
        let mut x = val;
        let mut k: u32 = 0;
        while x >= 2.341 {
            x *= 0.25;
            k += 1;
        }

        // Pick hardware scale n whose valid x range contains pre-scaled value
        let (scale, n, arg_mul) = if x < 0.75 {
            (SqrtScale::N0, 0u32, 1.0)
        } else if x < 1.75 {
            (SqrtScale::N1, 1, 0.5)
        } else {
            (SqrtScale::N2, 2, 0.25)
        };
        let x_q15 = f32_to_q1_15(x * arg_mul).unwrap();

        let mut cfg = self.cordic.configure::<Sqrt, Q15>(Precision::Iters12, scale);
        let raw = cfg.start1(x_q15).result();

        // Unscale and convert to float:
        q1_15_to_f32(raw) * (1 << (n + k)) as f32
    }
}

pub struct Memory {
    flash: Flash<'static, BlockingFlash>
}

impl Memory {
    pub fn new(mappings: MemoryMappings) -> Self {
        let flash = Flash::new_blocking(mappings.flash);
        Self {
            flash
        }
    }

    pub fn read_firmware_config(&self) -> Option<FirmwareConfig> {
        None
    }

    pub fn write_firmware_config(&self, config: FirmwareConfig) {

    }

    pub fn read_hall_calibrations(&self) -> Option<[f32; 6]> {
        None
    }

    pub fn write_hall_calibrations(&self, calibrations: [f32; 6]) {

    }

    pub fn read_motor_parameters(&self) -> Option<MotorParamsEstimate> {
        None
    }

    pub fn write_motor_parameters(&self, params: MotorParamsEstimate) {

    }

    pub fn read_controller_parameters(&self) -> Option<ControllerParameters> {
        None
    }

    pub fn write_controller_parameters(&self, params: ControllerParameters) {

    }
    
}