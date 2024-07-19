#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod inertial;
mod serial_comm;
mod utils;

use defmt as _;
use embassy_executor::{task, Spawner};
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::*,
    gpio::{GpioPin, Io, Level, Output},
    peripherals::{Peripherals, SPI2},
    prelude::*,
    rng::Rng,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    system::SystemControl,
    timer::{timg::TimerGroup, OneShotTimer, PeriodicTimer},
};
use esp_wifi::{self, wifi::WifiApDevice, EspWifiInitFor};
use static_cell::make_static;

/// Wrap the `imu_task` function in a non-generic task
///
/// This is necessary because the `#[task]` macro does not support generics
#[task]
pub async fn imu_task_spi2_dma(
    cs: GpioPin<34>,
    dma_channel: esp_hal::dma::ChannelCreator<0>,
    spi_bus: Spi<'static, SPI2, FullDuplexMode>,
) {
    inertial::imu_task::<_>(cs, dma_channel, spi_bus).await;
}

#[main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timers = make_static!([OneShotTimer::new(timg0.timer0.into())]);
    esp_hal_embassy::init(&clocks, timers);

    // Start the serial comm as early as possible
    // Since the `esp-println` impl will block if buffer becomes full
    spawner.spawn(serial_comm::serial_comm_task()).unwrap();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio33;
    let mosi = io.pins.gpio40;
    let miso = io.pins.gpio47;
    let cs = io.pins.gpio34;
    let baro_cs = io.pins.gpio11;

    // Prevent the barometer from operating in I2C mode
    let mut baro_cs = Output::new(baro_cs, Level::High);
    baro_cs.set_low();
    Timer::after_millis(1).await;
    baro_cs.set_high();

    // --- WIFI ---
    let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None);
    let wifi_timer = PeriodicTimer::new(timg1.timer0.into());

    let wifi = peripherals.WIFI;
    let init = esp_wifi::initialize(
        EspWifiInitFor::Wifi,
        wifi_timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let (wifi_interface, mut controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiApDevice).unwrap();

    controller.start().await.unwrap();

    // --- IMU ---

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let spi = Spi::new(peripherals.SPI2, 24.MHz(), SpiMode::Mode0, &clocks)
        .with_sck(sclk)
        .with_miso(miso)
        .with_mosi(mosi);

    spawner
        .spawn(imu_task_spi2_dma(cs, dma_channel, spi))
        .unwrap();
}
