use embassy_time::Timer;
use esp_wifi::{self, EspWifiInitialization};

use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, blocking_mutex::Mutex};
use static_cell::make_static;

#[task]
pub async fn wifi_test_task(
    wifi_init: EspWifiInitialization,
    wifi_dev: esp_hal::peripherals::WIFI,
) {
    let esp_now = esp_wifi::esp_now::EspNow::new(&wifi_init, wifi_dev).unwrap();
    let (manager, sender, receiver) = esp_now.split();
    let manager = make_static!(manager);
    let sender = make_static!(Mutex::<NoopRawMutex, _>::new(sender));

    loop {
        Timer::after_secs(1).await;
    }
}
