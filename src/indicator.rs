use embassy_time::Duration;
use esp_hal::{self, gpio::Level};

/// Waits for a message that contains a duration, then flashes a led for that
/// duration of time.
#[embassy_executor::task]
pub async fn control_led(mut led: esp_hal::gpio::Output<'static>) {
    defmt::info!(
        "Starting control_led() on core {}",
        esp_hal::Cpu::current() as usize
    );

    let duration = Duration::from_hz(1);
    let mut led_state = Level::Low;
    loop {
        led_state = match led_state {
            Level::Low => Level::High,
            Level::High => Level::Low,
        };

        // Flash the led
        led.set_level(led_state);

        // Wait for the duration
        embassy_time::Timer::after(duration).await;
    }
}
