use core::sync::atomic::{AtomicI32, Ordering};

use alloc::sync::Arc;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Instant, Timer};
use esp_hal::macros::ram;
use esp_wifi::{self, wifi::Protocol, EspWifiInitialization};

use embassy_executor::task;
use ieee80211::{
    common::{FrameControlField, FrameType},
    data_frame::builder::DataFrameBuilder,
    match_frames,
};
use scroll::{Pread, Pwrite};

use crate::configuration::ConfigurationStore;

extern crate alloc;

#[task]
#[ram]
pub async fn wifi_test_task(
    config_store: Arc<Mutex<CriticalSectionRawMutex, ConfigurationStore>>,
    wifi_init: EspWifiInitialization,
    wifi_dev: esp_hal::peripherals::WIFI,
) {
    let (_wifi_device, mut wifi_ctl) =
        esp_wifi::wifi::new_with_mode(&wifi_init, wifi_dev, esp_wifi::wifi::WifiApDevice).unwrap();

    let ap_config =
        esp_wifi::wifi::Configuration::AccessPoint(esp_wifi::wifi::AccessPointConfiguration {
            ssid: "esp-wifi-1".try_into().unwrap(),
            ssid_hidden: true,
            channel: 7,
            protocols: Protocol::P802D11BGN.into(),
            ..Default::default()
        });

    wifi_ctl.set_configuration(&ap_config).unwrap();
    wifi_ctl.start().await.unwrap();

    if unsafe {
        esp_wifi_sys::include::esp_wifi_config_80211_tx_rate(
            esp_wifi_sys::include::wifi_interface_t_WIFI_IF_AP,
            esp_wifi_sys::include::wifi_phy_rate_t_WIFI_PHY_RATE_MCS4_LGI,
        )
    } != esp_wifi_sys::include::ESP_OK as i32
    {
        defmt::error!("Failed to set tx rate");
    } else {
        defmt::info!("Set tx rate to MCS4 LGI"); // MCS4 Long GI with 40MHz = 81 Mbps
    }

    // Set bandwidth
    if unsafe {
        esp_wifi_sys::include::esp_wifi_set_bandwidth(
            esp_wifi_sys::include::wifi_interface_t_WIFI_IF_AP,
            esp_wifi_sys::include::wifi_bandwidth_t_WIFI_BW_HT40,
        )
    } != esp_wifi_sys::include::ESP_OK as i32
    {
        defmt::error!("Failed to set bandwidth");
    } else {
        defmt::info!("Set bandwidth to HT40");
    }

    let mut sniffer = wifi_ctl.take_sniffer().unwrap();

    // Register config in store
    config_store
        .lock()
        .await
        .registry
        .register::<u64>(b"W_TX_DLY")
        .unwrap();

    let mut tx_delay_us = config_store
        .lock()
        .await
        .get::<u64>(&mut [0b0; 20], b"W_TX_DLY")
        .await
        .unwrap()
        .unwrap_or(30000);

    if tx_delay_us < 800 {
        defmt::warn!("TX delay is less than 800 us, setting to 800 us");
        tx_delay_us = 800;
    }

    static RX_COUNT: AtomicI32 = AtomicI32::new(0);

    sniffer.set_promiscuous_mode(true).unwrap();
    sniffer.set_receive_cb(|packet| {
        if let Ok(fcf) = packet.data.pread(0).map(FrameControlField::from_bits) {
            if let FrameType::Data(data_subtype) = fcf.frame_type() {
                defmt::debug!("Received data frame with subtype: {:?}", data_subtype);
            }
        }

        let _ = match_frames! {
            packet.data,
            data = ieee80211::data_frame::DataFrame  => {
                defmt::debug!("Frame ctrl: {:?}", packet.rx_cntl);
                defmt::debug!("Received data frame: TS = {}, MCS={}, {:?}", packet.rx_cntl.timestamp, packet.rx_cntl.mcs, &data);

                RX_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        };
    });

    let chip_mac = {
        let mut mac = [0u8; 6];
        esp_wifi::wifi::get_sta_mac(&mut mac);
        mac
    };

    // e8:65:d4:cb:74:19
    let transmitter_addr = mac_parser::MACAddress::new(chip_mac);
    let my_mac = mac_parser::MACAddress::new(chip_mac);

    let super_long_payload = [0xFEu8; 400];
    let frame = DataFrameBuilder::new()
        .to_and_from_ds()
        .category_data()
        // .payload(b"Hello, world!".as_slice())
        .payload(super_long_payload.as_slice())
        .destination_address(mac_parser::BROADCAST)
        .source_address(my_mac)
        .transmitter_address(transmitter_addr)
        .receiver_address(mac_parser::BROADCAST)
        .build();

    let mut frame_buf = [0u8; 1000];

    let length = frame_buf.pwrite(frame, 0).unwrap();
    let frame_buf = &frame_buf[..length];

    defmt::info!("Frame buf: {:x}", frame_buf);

    let mut tx_count = 0;

    loop {
        let current_time = Instant::now();
        if sniffer.send_raw_frame(false, frame_buf, true).is_ok() {
            tx_count += 1;
        }
        let elapsed = current_time.elapsed();

        if tx_count % 500 == 0 {
            defmt::info!("TX time: {:?} us", elapsed.as_micros());
            defmt::info!(
                "TX count: {}, RX count: {}",
                tx_count,
                RX_COUNT.load(Ordering::Relaxed)
            );
        }
        if tx_count % 10000 == 0 {
            RX_COUNT.store(0, Ordering::Relaxed);
            tx_count = 0;
        }
        Timer::after_micros(tx_delay_us).await;
    }
}
