use core::cell::OnceCell;

use alloc::sync::Arc;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::Publisher;
use embassy_time::{Delay, Duration, Ticker, Timer};
use esp_hal::dma::{DmaPriority, DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::Output;
use esp_hal::peripherals::SPI2;
use esp_hal::spi::master::Spi;
use esp_hal::spi::FullDuplexMode;
use esp_hal::{dma, dma_buffers};

use crate::configuration::ConfigurationStore;

#[embassy_executor::task]
pub async fn imu_task(
    config_store: Arc<Mutex<CriticalSectionRawMutex, ConfigurationStore>>,
    mut cs_output: Output<'static>,
    dma_channel: dma::ChannelCreator<0>,
    spi: Spi<'static, SPI2, FullDuplexMode>,
    imu_pub: Publisher<'static, CriticalSectionRawMutex, icm426xx::fifo::FifoPacket4, 3, 1, 1>,
) {
    defmt::info!("Starting IMU task");

    config_store
        .lock()
        .await
        .registry
        .register::<u32>(b"IMU_RATE")
        .unwrap();

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(256);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = spi
        .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
        .with_buffers(dma_rx_buf, dma_tx_buf);

    cs_output.set_low();
    Timer::after_millis(1).await;
    cs_output.set_high();

    let spi_bus = OnceCell::<Mutex<NoopRawMutex, _>>::new();
    let _ = spi_bus.set(Mutex::new(spi));

    let spidev = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(
        spi_bus.get().unwrap(),
        cs_output,
    );

    let icm = icm426xx::ICM42688::new(spidev);

    Timer::after_secs(1).await;

    let mut icm = icm.initialize(Delay).await.unwrap();

    let mut bank = icm.ll().bank::<{ icm426xx::register_bank::BANK0 }>();

    // print WHO_AM_I register
    let who_am_i = bank.who_am_i().async_read().await;

    defmt::info!("WHO_AM_I: {:x}", who_am_i.unwrap().value());

    let afsr = icm.ll().bank::<0>().intf_config1().async_read().await;

    defmt::info!("AFSR: {:b}", afsr.unwrap().afsr());

    let mut ticker = Ticker::every(Duration::from_hz(1000));
    loop {
        // No need to check FIFO count since we are reading 3 packets at a time
        // and the sample rate is 1kHz, we will only get 1 packet per read anyway

        // let fifo_count = time_operation(icm.read_fifo_count()).await;
        // defmt::trace!("FIFO count: {}", fifo_count);

        // 16 * 4 = 64 bytes, 4 bytes header + 3 * 20 bytes data
        let mut fifo_buffer = [0u32; 16];
        let buffered_num = icm.read_fifo(&mut fifo_buffer).await.unwrap();

        defmt::trace!("In-FIFO: {} packets", buffered_num);

        let num_read = if buffered_num > 3 {
            defmt::error!("FIFO is not being read fast enough");
            3
        } else {
            buffered_num
        };

        let packets = bytemuck::cast_slice::<u32, icm426xx::fifo::FifoPacket4>(
            &fifo_buffer[1..(1 + (num_read * 5))],
        );

        for packet in packets {
            let header = packet.fifo_header;
            let parsed_header = icm426xx::fifo::FifoHeader::from(header);

            defmt::trace!("Packet: {:?}", parsed_header);

            defmt::trace!(
                "[{}], Accel: {}, {}, {}, Gyro: {}, {}, {}, T={}",
                packet.timestamp(),
                packet.accel_data_x(),
                packet.accel_data_y(),
                packet.accel_data_z(),
                packet.gyro_data_x(),
                packet.gyro_data_y(),
                packet.gyro_data_z(),
                packet.temperature_raw(),
            );

            imu_pub.publish_immediate(*packet);
        }

        ticker.next().await;
    }
}
