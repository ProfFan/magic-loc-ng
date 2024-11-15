use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use esp_hal::{gpio::Output, macros::ram, spi::master::SpiDmaBus, Async};

// pub mod uwb_driver;

// #[embassy_executor::task]
// #[ram]
// pub async fn uwb_driver_task(
//     mut runner: uwb_driver::UwbRunner<
//         'static,
//         127,
//         SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'static>>,
//     >,
// ) {
//     runner.run().await;
// }
