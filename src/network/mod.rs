use embassy_time::Timer;
use esp_wifi::{self, wifi::Protocol, EspWifiInitialization};
use esp_wifi_sys;

use embassy_executor::task;
use ieee80211::{
    common::{FrameControlField, FrameType},
    data_frame::builder::DataFrameBuilder,
    match_frames,
};
use scroll::{Pread, Pwrite};

extern crate alloc;

#[task]
pub async fn wifi_test_task(
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
            esp_wifi_sys::include::wifi_phy_rate_t_WIFI_PHY_RATE_MCS7_SGI,
        )
    } != esp_wifi_sys::include::ESP_OK as i32
    {
        defmt::error!("Failed to set tx rate");
    } else {
        defmt::info!("Set tx rate to MCS7 SGI");
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
                defmt::debug!("Received data frame: TS = {}, MCS={}, {:?}", packet.rx_cntl.timestamp, packet.rx_cntl.mcs, &data);
            }
        };
    });

    // e8:65:d4:cb:74:19
    let transmitter_addr = mac_parser::MACAddress::new([0xe8, 0x65, 0xd4, 0xcb, 0x74, 0x19]);
    let my_mac = mac_parser::MACAddress::new([0x74, 0x19, 0xff, 0xfc, 0xff, 0xff]);

    // let super_long_payload = [0xFEu8; 20];
    let frame = DataFrameBuilder::new()
        .to_and_from_ds()
        .category_data()
        .payload(b"Hello, world!".as_slice())
        .destination_address(mac_parser::BROADCAST)
        .source_address(my_mac)
        .transmitter_address(transmitter_addr)
        .receiver_address(mac_parser::BROADCAST)
        .build();

    // frame.header.subtype = DataFrameSubtype::Null;
    // frame.header.fcf_flags.set_to_ds(true);
    // frame.header.sequence_control.set_sequence_number(2374);

    defmt::info!("Frame: {:?}", frame);

    let mut frame_buf = [0u8; 1000];

    let length = frame_buf.pwrite(frame, 0).unwrap();
    let frame_buf = &frame_buf[..length];

    defmt::info!("Frame buf: {:x}", frame_buf);

    loop {
        sniffer.send_raw_frame(false, &frame_buf, true).unwrap();

        Timer::after_secs(3).await;
    }
}
