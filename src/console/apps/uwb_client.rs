use core::sync::atomic::AtomicBool;

use dw3000_ng::{
    configs::{StsLen, StsMode},
    hl::{ConfigGPIOs, RxQuality, SendTime},
    time::{Duration as DwDuration, Instant as DwInstant},
};
use embassy_executor::{SendSpawner, Spawner};

use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, once_lock::OnceLock, signal::Signal,
};
use embassy_time::{Duration, Timer};
use esp_hal::macros::ram;
use smoltcp::wire::{Ieee802154Address, Ieee802154Frame, Ieee802154Repr};

use crate::{
    console::{
        apps::uwb_master::{UwbClientResponse, UwbMasterPoll, UwbRxTimeReportSlot},
        Token,
    },
    utils::nonblocking_wait_async,
};

use super::uwb_master::UwbClientReport;

/// UWB client report streamer
///
/// Similar to the master streamer, but for the client
///
/// This task will send client's own RX times as well as all eavesdropped RX times to the host.
///
/// Runs on core 0
#[embassy_executor::task]
#[ram]
pub async fn uwb_client_streamer_task(
    stop_signal: &'static embassy_sync::signal::Signal<CriticalSectionRawMutex, bool>,
    stopped_signal: &'static core::sync::atomic::AtomicBool,
    report_channel: &'static embassy_sync::channel::Channel<
        CriticalSectionRawMutex,
        UwbClientReport,
        1,
    >,
) {
    stopped_signal.store(false, core::sync::atomic::Ordering::Release);

    let stack = *crate::network::WIFI_STACK.get().await;

    let source_endpoint =
        embassy_net::IpEndpoint::new(stack.config_v4().unwrap().address.address().into(), 40002);
    let broadcast_ep =
        embassy_net::IpEndpoint::new(embassy_net::IpAddress::v4(255, 255, 255, 255), 50002);

    if !stack.is_link_up() {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"WiFi not initialized\n");
        return;
    }

    let mut rx_metadata_buffer = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut rx_payload_buffer = [0; 1024];
    let mut tx_metadata_buffer = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut tx_payload_buffer = [0; 1024];

    // UDP socket
    let mut socket = embassy_net::udp::UdpSocket::new(
        stack,
        &mut rx_metadata_buffer,
        &mut rx_payload_buffer,
        &mut tx_metadata_buffer,
        &mut tx_payload_buffer,
    );

    socket.bind(source_endpoint).unwrap();

    loop {
        if stop_signal.signaled() {
            stop_signal.reset(); // IMPORTANT: reset the signal so we can re-enter the loop
            break;
        }

        let report = match select(report_channel.receive(), stop_signal.wait()).await {
            Either::First(report) => report,
            Either::Second(_) => {
                stop_signal.reset();
                break;
            }
        };

        defmt::debug!("Received report: {:?}", report);

        let report_bytes = bytemuck::bytes_of(&report);

        socket.send_to(report_bytes, broadcast_ep).await.unwrap();
    }

    stopped_signal.store(true, core::sync::atomic::Ordering::Release);
}

/// UWB client application
///
/// The UWB client application is responsible for:
/// - Receiving ranging responses from the UWB master
/// - Sending timeslice requests to the UWB master
/// - If allocated, sending response packets to the master
#[embassy_executor::task]
#[ram]
pub async fn uwb_client_task(
    stop_signal: &'static embassy_sync::signal::Signal<CriticalSectionRawMutex, bool>,
    stopped_signal: &'static core::sync::atomic::AtomicBool,
    report_channel: &'static embassy_sync::channel::Channel<
        CriticalSectionRawMutex,
        UwbClientReport,
        1,
    >,
) {
    stopped_signal.store(false, core::sync::atomic::Ordering::Release);

    let mut uwb_device = crate::DW3000.get().await.lock().await;

    let (spi, rst, irq) = uwb_device.split_borrow();

    let stack = crate::network::WIFI_STACK.get().await;
    let address = stack.config_v4().unwrap().address;
    let address = address.address().octets()[3]; // Just use the last octet as the node ID

    // Reset
    rst.set_low();
    Timer::after(Duration::from_millis(10)).await;
    rst.set_high();

    defmt::debug!("DW3000 Reset!");

    Timer::after(Duration::from_millis(100)).await;

    let dw3000 = dw3000_ng::DW3000::new(spi);

    let dw3000 = dw3000.init().unwrap();

    let dwm_config = dw3000_ng::Config {
        bitrate: dw3000_ng::configs::BitRate::Kbps6800,
        sts_len: StsLen::StsLen128,
        sts_mode: StsMode::StsMode1,
        pdoa_mode: dw3000_ng::configs::PdoaMode::Mode3,
        ..Default::default()
    };

    let mut dw3000 = dw3000.config(dwm_config, embassy_time::Delay).unwrap();

    defmt::debug!("DW3000 Initialized!");

    dw3000.gpio_config(ConfigGPIOs::enable_led()).unwrap();
    dw3000
        .ll()
        .led_ctrl()
        .modify(|_, w| w.blink_tim(0x2))
        .unwrap();

    // Enable Super Deterministic Code (SDC)
    dw3000.ll().sys_cfg().modify(|_, w| w.cp_sdc(0x1)).unwrap();

    Timer::after(Duration::from_millis(200)).await;

    // Disable SPIRDY interrupt
    dw3000.disable_interrupts().unwrap();
    dw3000.enable_tx_interrupts().unwrap();
    dw3000.enable_rx_interrupts().unwrap();

    // Read DW3000 Device ID
    let dev_id = dw3000.ll().dev_id().read().unwrap();

    if dev_id.model() != 0x03 {
        defmt::error!("Invalid DW3000 model: {:#x}", dev_id.model());
        panic!();
    }

    defmt::debug!("DW3000 Ready!");

    loop {
        if stop_signal.signaled() {
            stop_signal.reset(); // IMPORTANT: reset the signal so we can re-enter the loop
            break;
        }

        // Wait for the **master** to send the poll packet

        let mut receiving = dw3000.receive(dwm_config).unwrap();

        let mut buffer = [0; 128];
        let (len, rxts_poll, quality): (usize, DwInstant, RxQuality) =
            match nonblocking_wait_async(async || receiving.r_wait_buf(&mut buffer), irq).await {
                Ok(r) => r,
                _ => {
                    defmt::error!("Error receiving packet");
                    dw3000 = receiving.finish_receiving().unwrap();
                    continue;
                }
            };
        let rx_timestamp_cpu = embassy_time::Instant::now();

        dw3000 = receiving.finish_receiving().unwrap();

        // Parse the poll packet
        let frame = match Ieee802154Frame::new_checked(&buffer[..len]) {
            Ok(packet) => packet,
            Err(e) => {
                defmt::error!("Error parsing poll packet: {:?}", e);
                continue;
            }
        };

        defmt::debug!(
            "Received poll packet: len: {}, rxts_poll: {}, quality: {:?}",
            len,
            rxts_poll,
            quality
        );

        let (payload, _crc) = frame
            .payload()
            .unwrap()
            .split_last_chunk::<2>()
            .unwrap_or((&[], &[0, 0]));

        if payload[0] != b'P' {
            defmt::debug!("Received non-poll packet");

            if payload[0] != b'R' {
                defmt::debug!("Received invalid header: {:?}", payload[0]);
                continue;
            }

            let response_packet = match bytemuck::try_from_bytes::<UwbClientResponse>(payload) {
                Ok(response_packet) => response_packet,
                Err(_e) => {
                    defmt::debug!("Not a response packet: {:?}", payload);
                    continue;
                }
            };

            defmt::debug!(
                "Received response packet from {}: {:?}",
                frame.src_addr(),
                response_packet
            );
        }

        let poll_packet: &UwbMasterPoll = match bytemuck::try_from_bytes(payload) {
            Ok(poll_packet) => poll_packet,
            Err(_e) => {
                defmt::debug!("Received invalid payload: {:?}", payload);
                continue;
            }
        };

        defmt::debug!("Received poll packet: {:?}", poll_packet);

        let mut report = UwbClientReport {
            poll_rx_time: *rxts_poll.value().to_le_bytes().first_chunk::<5>().unwrap(),
            poll_sequence_number: frame.sequence_number().unwrap_or(0),
            cpu_rx_time: rx_timestamp_cpu.as_micros(),
            address: [0x00, address],
            ..Default::default()
        };

        for (i, slot_addr) in poll_packet.slots.iter().enumerate() {
            // Each slot is 3 milliseconds
            let slot_time_timeout = rx_timestamp_cpu + Duration::from_millis(4 + 2 + 3 * i as u64);

            if *slot_addr == address {
                // We are allocated to this slot, send a response
                // Prepare the response packet
                let response_txtime = rxts_poll
                    + DwDuration::from_nanos(
                        Duration::from_micros(5500 + (i as u64) * 3000).as_micros() as u32 * 1000,
                    );
                let response_txtime = DwInstant::new(response_txtime.value() >> 9 << 9).unwrap();

                let response_repr: Ieee802154Repr = Ieee802154Repr {
                    frame_type: smoltcp::wire::Ieee802154FrameType::Data,
                    frame_version: smoltcp::wire::Ieee802154FrameVersion::Ieee802154_2006,
                    security_enabled: false,
                    sequence_number: frame.sequence_number(),
                    frame_pending: false,
                    ack_request: false,
                    pan_id_compression: true,
                    dst_addr: Some(Ieee802154Address::BROADCAST),
                    src_addr: Some(Ieee802154Address::Short([0x00, address])),
                    src_pan_id: Some(smoltcp::wire::Ieee802154Pan(0xDEAD)),
                    dst_pan_id: None,
                };

                let mut tx_buffer = [0; 127];
                let mut tx_frame = Ieee802154Frame::new_unchecked(&mut tx_buffer);
                response_repr.emit(&mut tx_frame);

                tx_frame.payload_mut().unwrap()[..core::mem::size_of::<UwbClientResponse>()]
                    .copy_from_slice(bytemuck::bytes_of(&UwbClientResponse {
                        header: b'R',
                        rx_time_poll: *rxts_poll.value().to_le_bytes().first_chunk::<5>().unwrap(),
                        tx_time_response: *response_txtime
                            .value()
                            .to_le_bytes()
                            .first_chunk::<5>()
                            .unwrap(),
                    }));
                let len = response_repr.buffer_len() + core::mem::size_of::<UwbClientResponse>();

                let mut sending = dw3000
                    .send_raw(
                        &tx_buffer[..len],
                        SendTime::Delayed(response_txtime),
                        &dwm_config,
                    )
                    .unwrap();

                let send_result: Result<DwInstant, dw3000_ng::Error<_>> =
                    nonblocking_wait_async(async || sending.s_wait(), irq).await;

                if let Err(e) = send_result {
                    defmt::error!("Error sending response packet: {:?}", e);
                    dw3000 = sending.finish_sending().unwrap();
                    continue;
                }

                dw3000 = sending.finish_sending().unwrap();

                defmt::debug!("Sent response packet to slot {}", i);
            } else {
                // Listen for a response from other nodes
                let mut receiving = dw3000.receive(dwm_config).unwrap();
                let timeout = Timer::at(slot_time_timeout);

                let mut buffer = [0; 128];
                let rx_fut = async {
                    let result: Option<(usize, DwInstant, RxQuality)> =
                        match nonblocking_wait_async(
                            async || receiving.r_wait_buf(&mut buffer),
                            irq,
                        )
                        .await
                        {
                            Ok(r) => Some(r),
                            _ => None,
                        };

                    result
                };

                let (len, rx_time, _rx_quality) = match select(rx_fut, timeout).await {
                    Either::First(Some(rx_result)) => rx_result,
                    Either::First(None) => {
                        defmt::debug!("No response from slot {}", i);
                        dw3000 = receiving.finish_receiving().unwrap();
                        continue;
                    }
                    Either::Second(_) => {
                        defmt::debug!("Timeout waiting for response from slot {}", i);
                        dw3000 = receiving.finish_receiving().unwrap();
                        continue;
                    }
                };

                dw3000 = receiving.finish_receiving().unwrap();

                // Parse the response packet
                let frame = match Ieee802154Frame::new_checked(&buffer[..len]) {
                    Ok(packet) => packet,
                    Err(e) => {
                        defmt::error!("Error parsing response packet: {:?}", e);
                        continue;
                    }
                };

                let (payload, _crc) = frame
                    .payload()
                    .unwrap_or(&[0, 0])
                    .split_last_chunk::<2>()
                    .unwrap_or((&[], &[0, 0]));

                let response_packet = match bytemuck::try_from_bytes::<UwbClientResponse>(payload) {
                    Ok(response_packet) => response_packet,
                    Err(_e) => {
                        defmt::debug!("Not a response packet: {:?}", payload);
                        continue;
                    }
                };

                defmt::debug!(
                    "Received response packet from {}: {:?}",
                    frame.src_addr(),
                    response_packet
                );

                report.response_rx_time[i] = UwbRxTimeReportSlot {
                    rx_time: *rx_time.value().to_le_bytes().first_chunk::<5>().unwrap(),
                    address: [0x00, *slot_addr],
                };
            }
        }

        report_channel.send(report).await;
    }

    defmt::debug!("UWB client stopped");

    stopped_signal.store(true, core::sync::atomic::Ordering::Release);
}

static CLIENT_REPORT_CHANNEL: OnceLock<
    embassy_sync::channel::Channel<CriticalSectionRawMutex, UwbClientReport, 1>,
> = OnceLock::new();

static STOP_SIGNAL: OnceLock<Signal<CriticalSectionRawMutex, bool>> = OnceLock::new();
static STOP_SIGNAL_STREAMER: OnceLock<Signal<CriticalSectionRawMutex, bool>> = OnceLock::new();
static STOPPED_SIGNAL: AtomicBool = AtomicBool::new(true);

pub async fn uwb_client_start(
    spawner_core0: Spawner,
    spawner_core1: SendSpawner,
) -> Result<(), ()> {
    if !STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB client already running\n");
        return Err(());
    }

    let stop_signal = STOP_SIGNAL.get_or_init(Signal::new);
    let stop_signal_streamer = STOP_SIGNAL_STREAMER.get_or_init(Signal::new);
    let report_channel = CLIENT_REPORT_CHANNEL.get_or_init(embassy_sync::channel::Channel::new);

    spawner_core1
        .spawn(uwb_client_task(
            stop_signal,
            &STOPPED_SIGNAL,
            report_channel,
        ))
        .unwrap();

    spawner_core0
        .spawn(uwb_client_streamer_task(
            stop_signal_streamer,
            &STOPPED_SIGNAL, // TODO: use a different signal?
            report_channel,
        ))
        .unwrap();

    Ok(())
}

pub async fn uwb_client_stop() -> Result<(), ()> {
    if STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB client not running\n");
        return Err(());
    }

    STOP_SIGNAL.get_or_init(Signal::new).signal(true);
    STOP_SIGNAL_STREAMER.get_or_init(Signal::new).signal(true);
    Ok(())
}

/// Controller for the UWB client application
pub async fn uwb_client<'a>(
    spawner_core0: Spawner,
    spawner_core1: SendSpawner,
    args: &[Token<'a>],
) -> Result<(), ()> {
    if args.len() < 2 {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: uwb_client <start|stop>\n");
        return Err(());
    }

    let command = &args[1];

    if let Token::String(command) = command
        && *command == "start"
    {
        // make sure we don't already have a task running
        if !STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB client already running\n");
            return Err(());
        }

        let result = uwb_client_start(spawner_core0, spawner_core1).await;

        if let Err(()) = result {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Failed to start UWB client\n");
            return Err(());
        }

        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB client started\n");
    } else if let Token::String(command) = command
        && *command == "stop"
    {
        let result = uwb_client_stop().await;

        if let Err(()) = result {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Failed to stop UWB client\n");
            return Err(());
        }
    } else {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: uwb_client <start|stop>\n");
        return Err(());
    }

    Ok(())
}
