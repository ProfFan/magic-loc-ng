use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;
use esp_hal::{i2c::I2C, peripherals::I2C1, Async};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::Rectangle,
    text::{Baseline, Text},
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306Async};

#[embassy_executor::task]
pub async fn display_task(
    mut i2c_dev: I2cDevice<'static, NoopRawMutex, I2C<'static, I2C1, Async>>,
) {
    if i2c_dev.read(0x3C, &mut [0u8; 1]).await.is_err() {
        defmt::error!("Failed to read from I2C device, display not connected?");

        return;
    }

    let di = display_interface_i2c::I2CInterface::new(
        i2c_dev, // I2C
        0x3C,    // I2C Address
        0x40,    // Databyte
    );

    let mut disp = Ssd1306Async::new(di, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    disp.init().await.unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut disp)
        .unwrap();

    disp.flush().await.unwrap();

    loop {
        // Do nothing
        Timer::after_millis(1000).await;
    }
}
