use core::cell::RefCell;

use dw3000_ng::{
    self,
    configs::{StsLen, StsMode},
    hl::ConfigGPIOs,
};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Timer};
use embedded_hal::delay::DelayNs;
use esp_hal::macros::ram;
use esp_hal::{
    dma::ChannelCreator1,
    gpio::{AnyInput, AnyOutput},
    peripherals::SPI3,
    spi::{master::Spi, FullDuplexMode},
};

use crate::utils::nonblocking_wait;

/// Task for the UWB Tag
///
/// This task runs the Tag side state machine
///
/// Basic operation is as follows:
///
/// # As Initiator
///
/// 1. Send the Poll packet
/// 2. Wait for the Response packet
/// 3. Send the Final packet
///
/// # As Responder
///
/// 1. Wait for the Poll packet
/// 2. Send the Response packet
/// 3. Wait for the Final packet
#[embassy_executor::task(pool_size = 1)]
#[ram]
pub async fn symmetric_twr_tag_task(
    bus: Spi<'static, SPI3, FullDuplexMode>,
    cs_gpio: AnyOutput<'static>,
    mut rst_gpio: AnyOutput<'static>,
    mut int_gpio: AnyInput<'static>,
    dma_channel: ChannelCreator1,
) -> ! {
    defmt::info!("Starting TWR Tag Task!");

    let bus = NoopMutex::new(RefCell::new(bus));
    let spidev = SpiDevice::new(&bus, cs_gpio);

    let mut dwm_config = dw3000_ng::Config::default();
    dwm_config.bitrate = dw3000_ng::configs::BitRate::Kbps6800;
    dwm_config.sts_len = StsLen::StsLen128;
    dwm_config.sts_mode = StsMode::StsMode1;
    dwm_config.pdoa_mode = dw3000_ng::configs::PdoaMode::Mode3;

    // Reset
    rst_gpio.set_low();

    Timer::after(Duration::from_millis(10)).await;

    rst_gpio.set_high();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

    let mut dw3000 = dw3000_ng::DW3000::new(spidev)
        .init()
        .expect("Failed init.")
        .config_async(dwm_config, |d: u32| Timer::after_micros(d as u64))
        // .config(dwm_config, |d: u32| embassy_time::Delay.delay_us(d))
        .await
        .expect("Failed config.");

    dw3000.gpio_config(ConfigGPIOs::enable_led()).unwrap();
    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    // Enable Super Deterministic Code (SDC)
    dw3000.ll().sys_cfg().modify(|_, w| w.cp_sdc(0x1)).unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_interrupts().unwrap();
    dw3000.enable_tx_interrupts().unwrap();
    dw3000.enable_rx_interrupts().unwrap();

    // Read DW3000 Device ID
    let dev_id = dw3000.ll().dev_id().read().unwrap();

    if dev_id.model() != 0x03 {
        defmt::error!("Invalid DW3000 model: {:#x}", dev_id.model());
        panic!();
    }

    dw3000.ll().rx_fwto().write(|w| w.value(1000000)).unwrap();
    dw3000.ll().sys_cfg().modify(|_, w| w.rxwtoe(1)).unwrap();
    dw3000.ll().sys_status().modify(|_, w| w.rxfto(1)).unwrap();

    loop {
        let mut rxing = dw3000.receive(dwm_config).unwrap();

        // wait on the RX
        let result = nonblocking_wait(
            || -> Result<(), nb::Error<_>> {
                defmt::trace!("Waiting for packet...");
                let mut buffer = [0u8; 256];
                let status = rxing.r_wait_buf(&mut buffer);

                if status.is_err() {
                    return status.map(|_| ());
                }

                Ok(())
            },
            &mut int_gpio,
        )
        .await;

        if let Ok(()) = result {
            defmt::info!("Received packet!");
        } else {
            defmt::error!("Failed to receive packet, reason: {}", result.unwrap_err());
        }

        dw3000 = rxing.finish_receiving().unwrap();

        Timer::after_secs(1).await;
    }
}
