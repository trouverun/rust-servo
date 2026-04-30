use embassy_stm32::dma::{Channel as DmaChannel, ChannelInstance};
use embassy_stm32::{Peri};
use embassy_stm32::timer::{Dma, GeneralInstance4Channel, Ch1, Ch2, Ch3, Ch4};
use embassy_stm32::spi::{Config as SpiConfig, DmaDrivenSpi, Instance, MODE_0, MisoPin, MosiPin, SckPin, RxDma};
use embassy_stm32::timer::low_level::{Timer};
use embassy_stm32::time::Hertz;
use embassy_stm32::gpio::{Pin, Level, Output, Speed};

use crate::boards::SPIEncoderMappings;

pub fn encoder_mappings<EncSpi, SpiSck, SpiMosi, SpiMiso, CsPin, DmaTimer, CsLowDma, Tx1Dma, Tx2Dma, RxDmaCh, CsHighDma>(
    spi: Peri<'static, EncSpi>,
    sck: Peri<'static, SpiSck>,
    mosi: Peri<'static, SpiMosi>,
    miso: Peri<'static, SpiMiso>,
    cs: Peri<'static, CsPin>,
    timer: Peri<'static, DmaTimer>,
    cs_low_channel: Peri<'static, CsLowDma>,
    dma_tx1_channel: Peri<'static, Tx1Dma>,
    dma_tx2_channel: Peri<'static, Tx2Dma>,
    dma_rx_channel: Peri<'static, RxDmaCh>,
    cs_high_channel: Peri<'static, CsHighDma>,
) -> SPIEncoderMappings<EncSpi, DmaTimer>
    where
    EncSpi: Instance,
    SpiSck: SckPin<EncSpi>,
    SpiMosi: MosiPin<EncSpi>,
    SpiMiso: MisoPin<EncSpi>,
    CsPin: Pin,
    DmaTimer: GeneralInstance4Channel,
    CsLowDma: ChannelInstance + Dma<DmaTimer, Ch1>,
    Tx1Dma: ChannelInstance + Dma<DmaTimer, Ch2>,
    Tx2Dma: ChannelInstance + Dma<DmaTimer, Ch3>,
    RxDmaCh: ChannelInstance + RxDma<EncSpi>,
    CsHighDma: ChannelInstance + Dma<DmaTimer, Ch4>,
{
    let mut amt22_spi_config = SpiConfig::default();
    amt22_spi_config.mode = MODE_0;
    amt22_spi_config.frequency = Hertz(2_000_000);
    amt22_spi_config.gpio_speed = Speed::VeryHigh;
    let spi =  DmaDrivenSpi::new(spi, sck, mosi, miso, amt22_spi_config);

    let dma_timer = Timer::new(timer);
    let cs_low_trigger = dma_timer.cc1_request(&cs_low_channel);
    let tx1_trigger = dma_timer.cc2_request(&dma_tx1_channel);
    let tx2_trigger = dma_timer.cc3_request(&dma_tx2_channel);
    let cs_high_trigger = dma_timer.cc4_request(&cs_high_channel);
    let rx_trigger = DmaDrivenSpi::rx_request(&dma_rx_channel);

    let cs_low_dma = unsafe { DmaChannel::new_no_irq(cs_low_channel) };
    let tx1_dma = unsafe { DmaChannel::new_no_irq(dma_tx1_channel) };
    let tx2_dma = unsafe { DmaChannel::new_no_irq(dma_tx2_channel) };
    let rx_dma = unsafe { DmaChannel::new_no_irq(dma_rx_channel) };
    let cs_high_dma = unsafe { DmaChannel::new_no_irq(cs_high_channel) };

    SPIEncoderMappings {
        spi,
        cs: Output::new(cs, Level::High, Speed::VeryHigh),
        dma_timer,
        cs_low_trigger,
        tx1_trigger,
        tx2_trigger,
        rx_trigger,
        cs_high_trigger,
        cs_low_dma,
        tx1_dma,
        tx2_dma,
        rx_dma,
        cs_high_dma
    }
}