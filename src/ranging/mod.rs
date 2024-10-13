use core::cell::{OnceCell, RefCell};

use alloc::sync::Arc;
use dw3000_ng::{
    self,
    configs::{StsLen, StsMode},
    hl::ConfigGPIOs,
};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{
    blocking_mutex::{
        raw::{CriticalSectionRawMutex, NoopRawMutex},
        NoopMutex,
    },
    mutex::Mutex,
};
use embassy_time::{Duration, Timer};
use embedded_hal::delay::DelayNs;
use esp_hal::{
    dma::ChannelCreator,
    gpio::{AnyPin, Input, Output},
    peripherals::SPI3,
    spi::{master::Spi, FullDuplexMode},
};
use esp_hal::{
    dma::{DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    macros::ram,
};

use crate::{
    configuration::ConfigurationStore,
    utils::{nonblocking_wait, nonblocking_wait_async},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(C)]
pub struct UwbConfig {
    /// 16-bit short address
    pub short_address: u16,
    /// UWB MAC address
    pub long_address: [u8; 6],
    /// PAN ID
    pub pan_id: u16,
    /// Channel
    pub channel: u8,
}

impl Default for UwbConfig {
    fn default() -> Self {
        Self {
            short_address: 0,
            long_address: [0; 6],
            pan_id: 0,
            channel: 0,
        }
    }
}

/// Task for the UWB Driver
///
/// This task receives from the TX/RX queue and communicates with the DW3000
#[embassy_executor::task(pool_size = 1)]
#[ram]
pub async fn uwb_driver_task(
    config_store: Arc<Mutex<CriticalSectionRawMutex, ConfigurationStore>>,
    bus: Spi<'static, SPI3, FullDuplexMode>,
    cs_gpio: Output<'static>,
    mut rst_gpio: Output<'static>,
    mut int_gpio: Input<'static>,
    dma_channel: ChannelCreator<1>,
) -> ! {
    defmt::info!("Starting UWB Driver Task!");

    config_store
        .lock()
        .await
        .registry
        .register::<UwbConfig>(b"UWB_CONF")
        .unwrap();

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(1024);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let bus = bus
        .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
        .with_buffers(dma_rx_buf, dma_tx_buf);

    let spi_bus = OnceCell::<Mutex<NoopRawMutex, _>>::new();
    let _ = spi_bus.set(Mutex::new(bus));

    let spidev = SpiDevice::new(&spi_bus.get().unwrap(), cs_gpio);

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

    let dw3000 = dw3000_ng::DW3000::new(spidev).init().await;
    if let Err(e) = dw3000 {
        defmt::error!("DW3000 failed init: {}", e);
        core::future::pending::<()>().await;
        loop {}
    }

    let mut dw3000 = dw3000
        .unwrap()
        .config(dwm_config, embassy_time::Delay)
        // .config(dwm_config, |d: u32| embassy_time::Delay.delay_us(d))
        .await
        .expect("Failed config.");

    dw3000.gpio_config(ConfigGPIOs::enable_led()).await.unwrap();
    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .await
        .unwrap();

    // Enable Super Deterministic Code (SDC)
    dw3000
        .ll()
        .sys_cfg()
        .modify(|_, w| w.cp_sdc(0x1))
        .await
        .unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_interrupts().await.unwrap();
    dw3000.enable_tx_interrupts().await.unwrap();
    dw3000.enable_rx_interrupts().await.unwrap();

    // Read DW3000 Device ID
    let dev_id = dw3000.ll().dev_id().read().await.unwrap();

    if dev_id.model() != 0x03 {
        defmt::error!("Invalid DW3000 model: {:#x}", dev_id.model());
        panic!();
    }

    dw3000
        .ll()
        .rx_fwto()
        .write(|w| w.value(1000000))
        .await
        .unwrap();
    dw3000
        .ll()
        .sys_cfg()
        .modify(|_, w| w.rxwtoe(1))
        .await
        .unwrap();
    dw3000
        .ll()
        .sys_status()
        .modify(|_, w| w.rxfto(1))
        .await
        .unwrap();

    loop {
        let mut rxing = dw3000.receive(dwm_config).await.unwrap();

        // wait on the RX
        let result = nonblocking_wait_async(
            async || -> Result<(), nb::Error<_>> {
                defmt::trace!("Waiting for packet...");
                let mut buffer = [0u8; 256];
                let status = rxing.r_wait_buf(&mut buffer).await;

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

        dw3000 = rxing.finish_receiving().await.unwrap();

        Timer::after_secs(1).await;
    }
}
