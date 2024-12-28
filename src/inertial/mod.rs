use crate::configuration::ConfigurationStore;
use crate::multipin_spi::MultipinSpiDevice;
use alloc::sync::Arc;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};
use embedded_hal::spi::Operation;
use embedded_hal_async::spi::SpiDevice;
use esp_hal::gpio::{Input, Output};
use esp_hal::macros::ram;
use esp_hal::spi::master::SpiDmaBus;
use esp_hal::Async;

#[embassy_executor::task]
#[ram]
pub async fn imu_task(
    config_store: Arc<Mutex<CriticalSectionRawMutex, ConfigurationStore>>,
    cs_output: Output<'static>,
    mut int_input: Input<'static>,
    // _dma_channel: esp_hal::dma::AnyGdmaChannel,
    spi_mutex: &'static Mutex<
        CriticalSectionRawMutex,
        SpiDmaBus<'static, Async, esp_hal::peripherals::SPI2>,
    >,
    imu_pubsub: &'static embassy_sync::pubsub::PubSubChannel<
        CriticalSectionRawMutex,
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

    let imu_pub = imu_pubsub.publisher().unwrap();

    let peripherals = unsafe { esp_hal::peripherals::Peripherals::steal() };
    let mut spidev = MultipinSpiDevice::new(
        spi_mutex,
        cs_output,
        esp_hal::gpio::InputSignal::FSPIQ,
        peripherals.GPIO47,
    );

    spidev
        .transaction(&mut [Operation::DelayNs(1000)])
        .await
        .unwrap(); // Assert and deassert CS

    let icm = icm426xx::ICM42688::new(spidev);

    Timer::after_secs(1).await;

    defmt::info!("Initializing ICM");

    let mut icm = icm.initialize(Delay).await.unwrap();

    let mut bank = icm.ll().bank::<{ icm426xx::register_bank::BANK0 }>();

    // print WHO_AM_I register
    let who_am_i = bank.who_am_i().async_read().await;

    defmt::info!("WHO_AM_I: {:x}", who_am_i.unwrap().value());

    let afsr = icm.ll().bank::<0>().intf_config1().async_read().await;

    defmt::info!("AFSR: {:b}", afsr.unwrap().afsr());

    // Turn off gyro and accelerometer before changing the configuration
    // Refer to Section 12.9 of the datasheet
    icm.ll()
        .bank::<0>()
        .pwr_mgmt0()
        .async_modify(|_, w| w.accel_mode(0).gyro_mode(0))
        .await
        .unwrap();

    // Switch to bank 2 from bank 0
    icm.ll()
        .bank::<0>()
        .reg_bank_sel()
        .async_write(|r| r.bank_sel(2))
        .await
        .unwrap();
    icm.ll().set_bank(2);

    // Set Accelerometer Anti-Aliasing Filter
    icm.ll()
        .bank::<2>()
        .accel_config_static2()
        .async_modify(|_, w| w.accel_aaf_delt(5).accel_aaf_dis(0)) // 213 Hz 3dB Bandwidth
        .await
        .unwrap();
    icm.ll()
        .bank::<2>()
        .accel_config_static3()
        .async_modify(|_, w| w.accel_aaf_deltsqr_7_0(25)) // 213 Hz 3dB Bandwidth
        .await
        .unwrap();
    icm.ll()
        .bank::<2>()
        .accel_config_static4()
        .async_modify(|_, w| w.accel_aaf_deltsqr_11_8(0).accel_aaf_bitshift(10)) // 213 Hz 3dB Bandwidth
        .await
        .unwrap();

    // Switch to bank 1
    icm.ll()
        .bank::<2>()
        .reg_bank_sel()
        .async_write(|r| r.bank_sel(1))
        .await
        .unwrap();
    icm.ll().set_bank(1);

    icm.ll()
        .bank::<1>()
        .gyro_config_static2()
        .async_modify(|_, w| w.gyro_aaf_dis(0)) // 258 Hz 3dB Bandwidth
        .await
        .unwrap();

    icm.ll()
        .bank::<1>()
        .gyro_config_static3()
        .async_modify(|_, w| w.gyro_aaf_delt(6)) // 258 Hz 3dB Bandwidth
        .await
        .unwrap();

    icm.ll()
        .bank::<1>()
        .gyro_config_static4()
        .async_modify(|_, w| w.gyro_aaf_deltsqr_7_0(36)) // 258 Hz 3dB Bandwidth
        .await
        .unwrap();

    icm.ll()
        .bank::<1>()
        .gyro_config_static5()
        .async_modify(|_, w| w.gyro_aaf_deltsqr_11_8(0).gyro_aaf_bitshift(10)) // 258 Hz 3dB Bandwidth
        .await
        .unwrap();

    // Set Pin 9 of the ICM to CLKIN
    icm.ll()
        .bank::<1>()
        .intf_config5()
        .async_modify(|_, w| w.pin9_function(0b10))
        .await
        .unwrap();

    // Switch to bank 0
    icm.ll()
        .bank::<1>()
        .reg_bank_sel()
        .async_write(|r| r.bank_sel(0))
        .await
        .unwrap();
    icm.ll().set_bank(0);

    // Set RTC_MODE to 1
    icm.ll()
        .bank::<0>()
        .intf_config1()
        .async_modify(|_, w| w.rtc_mode(1))
        .await
        .unwrap();

    // Set FIFO watermark to 1 packet
    icm.ll()
        .bank::<0>()
        .fifo_config2()
        .async_modify(|_, w| w.fifo_wm_7_0(1))
        .await
        .unwrap();

    // Enable interrupt
    icm.ll()
        .bank::<0>()
        .int_source0()
        .async_modify(|_, w| w.fifo_ths_int1_en(1))
        .await
        .unwrap();

    // Turn on gyro and accelerometer
    icm.ll()
        .bank::<0>()
        .pwr_mgmt0()
        .async_modify(|_, w| w.accel_mode(0b11).gyro_mode(0b11))
        .await
        .unwrap();

    // Wait 300us for the gyro and accelerometer to be ready
    Timer::after_micros(300).await;

    // let mut ticker = Ticker::every(Duration::from_hz(1000));
    loop {
        int_input.wait_for_low().await;

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
    }
}
