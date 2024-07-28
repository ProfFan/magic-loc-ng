use embassy_time::Duration;

use embassy_futures::select::{select, Either};
use embassy_time::{Instant, Timer};
use esp_wifi::{self, esp_now::EspNowSender, EspWifiInitialization};

use embassy_executor::task;
use static_cell::make_static;

#[task]
pub async fn wifi_test_task(
    wifi_init: EspWifiInitialization,
    wifi_dev: esp_hal::peripherals::WIFI,
) {
    let esp_now = esp_wifi::esp_now::EspNow::new(&wifi_init, wifi_dev).unwrap();

    esp_now.set_channel(7).unwrap();
    esp_now
        .set_rate(esp_wifi::esp_now::WifiPhyRate::Rate54m)
        .unwrap();

    let (manager, sender, receiver) = esp_now.split();
    let manager = make_static!(manager);
    let sender = make_static!(sender);

    manager.set_pmk(&[0x00; 16]).unwrap();

    let buffer = [1u8; 220];
    let addr = [0xFFu8; 6];
    let mut sent_count = 0;

    let timeout_time = Instant::now() + Duration::from_secs(10);
    loop {
        let timeout_fut = Timer::at(timeout_time);
        let send_fut = sender.send_async(&addr, &buffer);

        let race = select(timeout_fut, send_fut).await;

        match race {
            Either::First(_) => {
                defmt::info!("Sent {} packets", sent_count);
                break;
            }

            Either::Second(Ok(_)) => {
                // defmt::debug!("{}", result);
                sent_count += 1;
            }

            Either::Second(Err(err)) => {
                defmt::error!("Error: {:?}", err);
            }
        }
    }

    loop {
        Timer::after_secs(10).await;
    }
}
