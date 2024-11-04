use alloc::sync::Arc;
use embassy_futures::select::{select, Either};
use embassy_net::driver::LinkState;
use embassy_net_driver_channel as ch;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    once_lock::OnceLock,
    zerocopy_channel::{self},
};
use embassy_time::{Duration, Timer};
use esp_hal::macros::ram;
use esp_wifi::{self, wifi::Protocol, EspWifiInitialization};

use embassy_executor::{task, Spawner};
use esp_wifi_sys::include::{
    wifi_ap_config_t, wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
    wifi_cipher_type_t_WIFI_CIPHER_TYPE_CCMP, wifi_config_t, wifi_pmf_config_t,
};
use ieee80211::{
    common::DataFrameSubtype,
    data_frame::{builder::DataFrameBuilder, DataFrameReadPayload},
    match_frames,
};
use scroll::Pwrite;
use static_cell::StaticCell;

use crate::configuration::ConfigurationStore;

pub struct RawWifiDriver<const MTU: usize, const N_RX: usize, const N_TX: usize> {
    ch_state: ch::State<MTU, N_RX, N_TX>,
}

impl<const MTU: usize, const N_RX: usize, const N_TX: usize> RawWifiDriver<MTU, N_RX, N_TX> {
    pub fn new() -> Self {
        Self {
            ch_state: ch::State::new(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RawPacketBuffer<const SIZE: usize> {
    len: usize,
    buf: [u8; SIZE],
}

impl<const SIZE: usize> RawPacketBuffer<SIZE> {
    pub const fn new() -> Self {
        Self {
            len: 0,
            buf: [0; SIZE],
        }
    }
}

impl<const SIZE: usize> Default for RawPacketBuffer<SIZE> {
    fn default() -> Self {
        Self {
            len: 0,
            buf: [0; SIZE],
        }
    }
}

pub const APP_MTU: usize = 1500;
pub const RAW_SIZE: usize = 1536;

/// Background runner for the wifi driver
///
/// A background task has to be spawned with the `.run()` method
pub struct Runner<'d, const MTU: usize> {
    pub sniffer: esp_wifi::wifi::Sniffer,
    pub mac_addr: mac_parser::MACAddress,

    ch: ch::Runner<'d, APP_MTU>,
}

static BUF: StaticCell<[RawPacketBuffer<RAW_SIZE>; 5]> = StaticCell::new();
static RX_CHAN: StaticCell<
    zerocopy_channel::Channel<'static, CriticalSectionRawMutex, RawPacketBuffer<RAW_SIZE>>,
> = StaticCell::new();
static RX_CHAN_SEND: OnceLock<
    Mutex<
        NoopRawMutex,
        zerocopy_channel::Sender<'static, CriticalSectionRawMutex, RawPacketBuffer<RAW_SIZE>>,
    >,
> = OnceLock::new();

impl<'d, const MTU: usize> Runner<'d, MTU> {
    pub async fn run(mut self) {
        let (state_chan, mut rx_chan, mut tx_chan) = self.ch.split();

        let buf = BUF.init([RawPacketBuffer::new(); 5]);

        let chan = RX_CHAN.init(zerocopy_channel::Channel::new(buf));

        let (rx_send, mut rx_recv) = chan.split();

        RX_CHAN_SEND.get_or_init(|| Mutex::new(rx_send));

        self.sniffer.set_receive_cb(|packet| {
            let _ = match_frames! {
                packet.data,
                data = ieee80211::data_frame::DataFrame  => {
                    defmt::trace!("Frame ctrl: {:?}", packet.rx_cntl);
                    // Make sure it's a data frame
                    if data.header.subtype != DataFrameSubtype::Data || packet.rx_cntl.mcs != 4 {
                        return;
                    }

                    defmt::trace!("Received data frame: TS = {}, MCS={}, {:?}", packet.rx_cntl.timestamp, packet.rx_cntl.mcs, &data);

                    let rx_chan_send = RX_CHAN_SEND.try_get().unwrap();
                    let mut rx_chan = rx_chan_send.try_lock().unwrap();

                    let payload = data.payload.unwrap();

                    if let DataFrameReadPayload::Single(payload) = payload {
                        // Copy the data into the channel
                        if let Some(packet_buf) = rx_chan.try_send() {
                            packet_buf.buf[..payload.len()].copy_from_slice(payload);
                            packet_buf.len = payload.len();
                            rx_chan.send_done();
                        }
                    }
                }
            };
        });

        let rx_fut = async move {
            state_chan.set_link_state(LinkState::Up);

            loop {
                let p = rx_chan.rx_buf().await;
                let rx_buf = rx_recv.receive().await;

                // Post the packet to the rx_chan
                if p.len() >= rx_buf.len {
                    p[..rx_buf.len].copy_from_slice(&rx_buf.buf[..rx_buf.len]);
                } else {
                    defmt::error!("Packet buffer too small: {} < {}", p.len(), rx_buf.len);
                }
                rx_chan.rx_done(rx_buf.len);
                rx_recv.receive_done();
            }
        };

        let tx_fut = async move {
            loop {
                let p = tx_chan.tx_buf().await;

                // Inside the buffer is an IP packet, we need to wrap it in an 802.11 frame
                // and send it out
                let frame = DataFrameBuilder::new()
                    .to_and_from_ds()
                    .category_data()
                    // .payload(b"Hello, world!".as_slice())
                    .payload(&p[..])
                    .destination_address(mac_parser::BROADCAST)
                    .source_address(self.mac_addr)
                    .transmitter_address(self.mac_addr)
                    .receiver_address(mac_parser::BROADCAST)
                    .build();

                let mut send_buf = [0u8; RAW_SIZE];
                let len_written = send_buf.pwrite(frame, 0).unwrap();

                match self
                    .sniffer
                    .send_raw_frame(false, &send_buf[..len_written], false)
                {
                    Ok(_) => defmt::debug!("Sent raw frame"),
                    Err(e) => {
                        defmt::error!("Failed to send raw frame: {:?}", e)
                    }
                }

                tx_chan.tx_done();
            }
        };

        match select(rx_fut, tx_fut).await {
            Either::First(rx) => rx,
            Either::Second(tx) => tx,
        }
    }
}

#[task]
#[ram]
pub async fn net_task(mut runner: embassy_net::Runner<'static, ch::Device<'static, APP_MTU>>) {
    runner.run().await;
}

#[task]
#[ram]
pub async fn embassy_net_task(runner: Runner<'static, RAW_SIZE>) {
    runner.run().await;
}

pub static WIFI_STACK: OnceLock<embassy_net::Stack<'static>> = OnceLock::new();

#[task]
#[ram]
pub async fn wifi_driver_task(
    config_store: Arc<Mutex<CriticalSectionRawMutex, ConfigurationStore>>,
    wifi_init: EspWifiInitialization,
    wifi_dev: esp_hal::peripherals::WIFI,
    spawner: Spawner,
) {
    let (_wifi_device, mut wifi_ctl) =
        esp_wifi::wifi::new_with_mode(&wifi_init, wifi_dev, esp_wifi::wifi::WifiApDevice).unwrap();

    let ap_config =
        esp_wifi::wifi::Configuration::AccessPoint(esp_wifi::wifi::AccessPointConfiguration {
            ssid: "esp-wifi-1".try_into().unwrap(),
            ssid_hidden: true,
            channel: 7,
            protocols: Protocol::P802D11BGN.into(),
            auth_method: esp_wifi::wifi::AuthMethod::WPA2Personal,
            password: "do-not-conn3ct-to-me".try_into().unwrap(),
            ..Default::default()
        });

    wifi_ctl.set_configuration(&ap_config).unwrap();

    let ap_conf_inner = ap_config.as_ap_conf_ref().unwrap();
    let mut cfg = wifi_config_t {
        ap: wifi_ap_config_t {
            ssid: [0; 32],
            password: [0; 64],
            ssid_len: 0,
            channel: ap_conf_inner.channel,
            authmode: wifi_auth_mode_t_WIFI_AUTH_WPA2_PSK,
            ssid_hidden: if ap_conf_inner.ssid_hidden { 1 } else { 0 },
            max_connection: ap_conf_inner.max_connections as u8,
            beacon_interval: 5000,
            pairwise_cipher: wifi_cipher_type_t_WIFI_CIPHER_TYPE_CCMP,
            ftm_responder: false,
            pmf_cfg: wifi_pmf_config_t {
                capable: true,
                required: true,
            },
            sae_pwe_h2e: 0,
            csa_count: 3,
            dtim_period: 2,
        },
    };
    unsafe {
        cfg.ap.ssid[0..(ap_conf_inner.ssid.len())].copy_from_slice(ap_conf_inner.ssid.as_bytes());
        cfg.ap.ssid_len = ap_conf_inner.ssid.len() as u8;
        cfg.ap.password[0..(ap_conf_inner.password.len())]
            .copy_from_slice(ap_conf_inner.password.as_bytes());
    }

    if unsafe {
        esp_wifi_sys::include::esp_wifi_set_config(
            esp_wifi_sys::include::wifi_interface_t_WIFI_IF_AP,
            &mut cfg,
        )
    } != esp_wifi_sys::include::ESP_OK as i32
    {
        defmt::error!("Failed to set raw config");
    }

    // Disable 802.11b
    if unsafe {
        esp_wifi_sys::include::esp_wifi_config_11b_rate(
            esp_wifi_sys::include::wifi_interface_t_WIFI_IF_AP,
            true,
        )
    } != esp_wifi_sys::include::ESP_OK as i32
    {
        defmt::error!("Failed to set 11b rate");
    }

    if unsafe {
        esp_wifi_sys::include::esp_wifi_set_protocol(
            esp_wifi_sys::include::wifi_interface_t_WIFI_IF_AP,
            esp_wifi_sys::include::WIFI_PROTOCOL_11G as u8
                | esp_wifi_sys::include::WIFI_PROTOCOL_11N as u8,
        )
    } != esp_wifi_sys::include::ESP_OK as i32
    {
        defmt::error!("Failed to set protocol to 802.11g/n");
    }

    wifi_ctl.start().await.unwrap();

    if unsafe {
        esp_wifi_sys::include::esp_wifi_config_80211_tx_rate(
            esp_wifi_sys::include::wifi_interface_t_WIFI_IF_AP,
            esp_wifi_sys::include::wifi_phy_rate_t_WIFI_PHY_RATE_MCS4_SGI,
        )
    } != esp_wifi_sys::include::ESP_OK as i32
    {
        defmt::error!("Failed to set tx rate");
    } else {
        defmt::info!("Set tx rate to MCS4 SGI"); // MCS4 Short GI with 40MHz = 90 Mbps
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

    let sniffer = wifi_ctl.take_sniffer().unwrap();

    sniffer.set_promiscuous_mode(true).unwrap();

    let chip_mac = {
        let mut mac = [0u8; 6];
        esp_wifi::wifi::get_sta_mac(&mut mac);
        mac
    };

    // Register IP in the config store
    config_store
        .lock()
        .await
        .registry
        .register::<[u8; 4]>(b"IP_ADDR4")
        .unwrap();
    config_store
        .lock()
        .await
        .registry
        .register::<u8>(b"IP_PREF4")
        .unwrap();

    let mut conf_tmp_buf = [0u8; 20];
    let ip_addr = config_store
        .lock()
        .await
        .get::<[u8; 4]>(&mut conf_tmp_buf, b"IP_ADDR4")
        .await
        .unwrap();
    let ip_prefix_len = config_store
        .lock()
        .await
        .get::<u8>(&mut conf_tmp_buf, b"IP_PREF4")
        .await
        .unwrap();

    if ip_addr.is_none() || ip_prefix_len.is_none() {
        defmt::error!("Failed to get IP address from config store");
        return;
    }

    let ip_addr = ip_addr.unwrap();
    let ip_prefix_len = ip_prefix_len.unwrap();

    let ip_addr = smoltcp::wire::Ipv4Cidr::new(
        smoltcp::wire::Ipv4Address::new(ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]),
        ip_prefix_len,
    );

    static DRIVER: StaticCell<RawWifiDriver<1500, 5, 5>> = StaticCell::new();
    let driver = DRIVER.init(RawWifiDriver::<1500, 5, 5>::new());

    let (runner, device) = ch::new(
        &mut driver.ch_state,
        ch::driver::HardwareAddress::Ethernet(chip_mac),
    );

    let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: ip_addr,
        gateway: None,
        dns_servers: heapless::Vec::new(),
    });
    let seed = 0;
    static RESOURCES: StaticCell<embassy_net::StackResources<3>> = StaticCell::new();

    let (stack, net_runner) = embassy_net::new(
        device,
        config,
        RESOURCES.init(embassy_net::StackResources::new()),
        seed,
    );
    let stack = WIFI_STACK.get_or_init(|| stack);

    spawner
        .spawn(embassy_net_task(Runner {
            sniffer,
            mac_addr: mac_parser::MACAddress::new(chip_mac),
            ch: runner,
        }))
        .unwrap();
    spawner.spawn(net_task(net_runner)).unwrap();

    stack.wait_config_up().await;

    defmt::info!("Network up");

    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
