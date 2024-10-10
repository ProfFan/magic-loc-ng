#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
#![feature(pointer_is_aligned_to)]
#![feature(impl_trait_in_assoc_type)]
#![feature(async_closure)]
#![feature(let_chains)]

mod configuration;
mod console;
mod display;
mod indicator;
mod inertial;
mod network;
mod ranging;
mod utils;

use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use esp_fast_serial;
use esp_hal_embassy::InterruptExecutor;
use static_cell::StaticCell;

extern crate alloc;

use defmt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    cpu_control::{CpuControl, Stack},
    dma::*,
    gpio::{Input, Io, Level, Output, Pull},
    interrupt::{self, software::SoftwareInterruptControl},
    prelude::*,
    rng::Rng,
    spi::{master::Spi, SpiMode},
    timer::{timg::TimerGroup, AnyTimer, OneShotTimer},
    Async,
};
use esp_wifi::{self, EspWifiInitFor};

// Stack for the second core
static mut APP_CORE_STACK: Stack<65536> = Stack::new();

#[main]
async fn main(spawner: Spawner) {
    let mut hal_config = esp_hal::Config::default();
    hal_config.cpu_clock = esp_hal::clock::CpuClock::Clock240MHz;

    let peripherals = esp_hal::init(hal_config);

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    let timer1: AnyTimer = timg0.timer1.into();
    static TIMERS_STATIC: StaticCell<[OneShotTimer<'static, AnyTimer>; 2]> = StaticCell::new();
    let timers = TIMERS_STATIC.init([OneShotTimer::new(timer0), OneShotTimer::new(timer1)]);
    esp_hal_embassy::init(timers);

    esp_alloc::heap_allocator!(96 * 1024);

    let config_store = alloc::sync::Arc::new(embassy_sync::mutex::Mutex::<
        CriticalSectionRawMutex,
        _,
    >::new(
        configuration::ConfigurationStore::new().unwrap()
    ));

    config_store
        .lock()
        .await
        .registry
        .register::<u8>(b"MVERSION")
        .unwrap();

    let mut buff = [0u8; 256];
    let ver = config_store
        .lock()
        .await
        .get::<u8>(&mut buff, b"MVERSION")
        .await
        .unwrap();

    defmt::info!("Configuration version: {}", ver);

    // Spawners

    // Start the serial comm as early as possible
    // Since the `esp-println` impl will block if buffer becomes full
    spawner
        .spawn(esp_fast_serial::serial_comm_task(peripherals.USB_DEVICE))
        .unwrap();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio33;
    let mosi = io.pins.gpio40;
    let miso = io.pins.gpio47;
    let imu_cs = io.pins.gpio34;
    let baro_cs = io.pins.gpio11;

    // Prevent the barometer from operating in I2C mode
    let mut baro_cs = Output::new(baro_cs, Level::High);
    baro_cs.set_low();
    Timer::after_millis(1).await;
    baro_cs.set_high();

    Timer::after_secs(2).await;

    // --- WIFI ---
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    // let timg1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None);
    // let wifi_timer = PeriodicTimer::new(timg0.timer0.into());

    let wifi = peripherals.WIFI;
    let init = esp_wifi::init(
        EspWifiInitFor::Wifi,
        timg1.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    spawner.spawn(network::wifi_test_task(init, wifi)).unwrap();

    Timer::after_secs(1).await;

    // --- BMS ---
    let bms_sda = io.pins.gpio1;
    let bms_scl = io.pins.gpio2;

    let mut bms_i2c = esp_hal::i2c::I2c::new_async(peripherals.I2C0, bms_sda, bms_scl, 100.kHz());
    let mut reg07 = [0u8; 1];

    // Register 0x07, 1 byte
    bms_i2c.write_read(0x6B, &[0x07], &mut reg07).await.ok();
    bms_i2c
        .write(0x6B, &[0x07, reg07[0] | 0b00100000u8])
        // .write(0x6B, &[0x07, reg07[0] & 0b11011111u8])
        .await
        .ok();

    // --- Display ---
    let display_sda = io.pins.gpio6;
    let display_scl = io.pins.gpio7;

    static I2C1: StaticCell<
        Mutex<NoopRawMutex, esp_hal::i2c::I2c<'static, esp_hal::peripherals::I2C1, Async>>,
    > = StaticCell::new();
    let i2c1 = I2C1.init(Mutex::<NoopRawMutex, _>::new(esp_hal::i2c::I2c::new_async(
        peripherals.I2C1,
        display_sda,
        display_scl,
        400.kHz(),
    )));

    // Scan I2C bus
    let display_i2c = I2cDevice::new(i2c1);

    spawner.spawn(display::display_task(display_i2c)).unwrap();

    // --- IMU ---
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let spi = Spi::new(peripherals.SPI2, 24.MHz(), SpiMode::Mode0)
        .with_sck(sclk)
        .with_miso(miso)
        .with_mosi(mosi);

    spawner
        .spawn(inertial::imu_task(
            config_store.clone(),
            Output::new(imu_cs, Level::High),
            dma_channel,
            spi,
        ))
        .unwrap();

    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let config_store_ = config_store.clone();
    let cpu1_fnctn = move || {
        static EXECUTOR_CORE1: StaticCell<InterruptExecutor<2>> = StaticCell::new();
        let executor_core1 =
            EXECUTOR_CORE1.init(InterruptExecutor::new(sw_ints.software_interrupt2));
        let spawner = executor_core1.start(interrupt::Priority::Priority2);

        let dw_cs = io.pins.gpio8;
        let dw_rst = io.pins.gpio9;
        let dw_irq = io.pins.gpio15;

        let dw_sclk = io.pins.gpio36;
        let dw_mosi = io.pins.gpio35;
        let dw_miso = io.pins.gpio37;

        let spi = Spi::new(peripherals.SPI3, 24.MHz(), SpiMode::Mode0)
            .with_sck(dw_sclk)
            .with_miso(dw_miso)
            .with_mosi(dw_mosi);

        let dma_channel = dma.channel1;

        // Start the ranging task
        spawner
            .spawn(ranging::symmetric_twr_tag_task(
                config_store_,
                spi,
                Output::new(dw_cs, Level::High),
                Output::new(dw_rst, Level::High),
                Input::new(dw_irq, Pull::Up),
                dma_channel,
            ))
            .unwrap();
        loop {}
    };

    defmt::info!("Starting core 1");

    let _guard = cpu_control
        .start_app_core(
            unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) },
            cpu1_fnctn,
        )
        .unwrap();

    let led = io.pins.gpio5;
    spawner
        .spawn(indicator::control_led(Output::new(led, Level::Low)))
        .ok();

    // Start the interactive console
    spawner
        .spawn(console::console(config_store.clone()))
        .unwrap();

    // A never-ending heartbeat
    loop {
        Timer::after_secs(5).await;
    }
}
