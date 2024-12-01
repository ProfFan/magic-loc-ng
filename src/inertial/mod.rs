use core::cell::{OnceCell, RefCell};

use crate::configuration::ConfigurationStore;
use alloc::sync::Arc;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use esp_hal::gpio::{Input, Output};
use esp_hal::macros::ram;
use esp_hal::peripherals::SPI2;
use esp_hal::spi::master::Spi;
use esp_hal::sync::RawMutex as EspRawMutex;
use esp_hal::Blocking;

#[embassy_executor::task]
#[ram]
pub async fn imu_task(
    config_store: Arc<Mutex<EspRawMutex, ConfigurationStore>>,
    mut cs_output: Output<'static>,
    mut int_input: Input<'static>,
    // _dma_channel: esp_hal::dma::AnyGdmaChannel,
    spi: Spi<'static, Blocking, SPI2>,
    imu_pubsub: &'static embassy_sync::pubsub::PubSubChannel<
        esp_hal::sync::RawMutex,
        icm426xx::fifo::FifoPacket4,
        3,
        2,
        1,
    >,
) {
    defmt::info!("Starting IMU task");

    config_store
        .lock()
        .await
        .registry
        .register::<u32>(b"IMU_RATE")
        .unwrap();

    // let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(256);
    // let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    // let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // let spi = spi
    //     .with_dma(dma_channel.configure(false, DmaPriority::Priority0))
    //     .with_buffers(dma_rx_buf, dma_tx_buf)
    //     .into_async();

    let imu_pub = imu_pubsub.publisher().unwrap();

    cs_output.set_low();
    Timer::after_millis(1).await;
    cs_output.set_high();

    let spi_bus = OnceCell::<embassy_sync::blocking_mutex::Mutex<NoopRawMutex, _>>::new();
    let _ = spi_bus.set(embassy_sync::blocking_mutex::Mutex::new(RefCell::new(spi)));

    let spidev = embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(
        spi_bus.get().unwrap(),
        cs_output,
    );

    let icm = icm426xx::ICM42688::new(spidev);

    Timer::after_secs(1).await;

    defmt::info!("Initializing ICM");

    let mut icm = icm.initialize(Delay).unwrap();

    let mut bank = icm.ll().bank::<{ icm426xx::register_bank::BANK0 }>();

    // print WHO_AM_I register
    let who_am_i = bank.who_am_i().read();

    defmt::info!("WHO_AM_I: {:x}", who_am_i.unwrap().value());

    let afsr = icm.ll().bank::<0>().intf_config1().read();

    defmt::info!("AFSR: {:b}", afsr.unwrap().afsr());

    // Switch to bank 2 from bank 0
    icm.ll()
        .bank::<0>()
        .reg_bank_sel()
        .write(|r| r.bank_sel(2))
        .unwrap();
    icm.ll().set_bank(2);

    // Set Accelerometer Anti-Aliasing Filter
    icm.ll()
        .bank::<2>()
        .accel_config_static2()
        .modify(|_, w| w.accel_aaf_delt(7)) // 303 Hz 3dB Bandwidth
        .unwrap();
    icm.ll()
        .bank::<2>()
        .accel_config_static3()
        .modify(|_, w| w.accel_aaf_deltsqr_7_0(49)) // 303 Hz 3dB Bandwidth
        .unwrap();
    icm.ll()
        .bank::<2>()
        .accel_config_static4()
        .modify(|_, w| w.accel_aaf_deltsqr_11_8(0).accel_aaf_bitshift(9)) // 303 Hz 3dB Bandwidth
        .unwrap();

    // Switch to bank 1
    icm.ll()
        .bank::<2>()
        .reg_bank_sel()
        .write(|r| r.bank_sel(1))
        .unwrap();
    icm.ll().set_bank(1);

    // Set Pin 9 of the ICM to CLKIN
    icm.ll()
        .bank::<1>()
        .intf_config5()
        .modify(|_, w| w.pin9_function(0b10))
        .unwrap();

    // Switch to bank 0
    icm.ll()
        .bank::<1>()
        .reg_bank_sel()
        .write(|r| r.bank_sel(0))
        .unwrap();
    icm.ll().set_bank(0);

    // Set RTC_MODE to 1
    icm.ll()
        .bank::<0>()
        .intf_config1()
        .modify(|_, w| w.rtc_mode(1))
        .unwrap();

    // Set FIFO watermark to 1 packet
    icm.ll()
        .bank::<0>()
        .fifo_config2()
        .modify(|_, w| w.fifo_wm_7_0(1))
        .unwrap();

    // Enable interrupt
    icm.ll()
        .bank::<0>()
        .int_source0()
        .modify(|_, w| w.fifo_ths_int1_en(1))
        .unwrap();

    // let mut ticker = Ticker::every(Duration::from_hz(1000));
    loop {
        int_input.wait_for_low().await;

        // No need to check FIFO count since we are reading 3 packets at a time
        // and the sample rate is 1kHz, we will only get 1 packet per read anyway

        // let fifo_count = time_operation(icm.read_fifo_count()).await;
        // defmt::trace!("FIFO count: {}", fifo_count);

        // 16 * 4 = 64 bytes, 4 bytes header + 3 * 20 bytes data
        let mut fifo_buffer = [0u32; 16];
        let buffered_num = icm.read_fifo(&mut fifo_buffer).unwrap();

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
    }
}
