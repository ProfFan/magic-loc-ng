use core::sync::atomic::AtomicBool;

use dw3000_ng::{
    configs::{StsLen, StsMode},
    hl::{ConfigGPIOs, RxQuality, SendTime},
    time::{Duration as DwDuration, Instant as DwInstant},
};
use embassy_executor::SendSpawner;

use bytemuck::{AnyBitPattern, NoUninit};
use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    once_lock::OnceLock,
    signal::Signal,
};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::macros::ram;
use smoltcp::wire::{Ieee802154Address, Ieee802154Frame, Ieee802154Repr};

use crate::{
    console::{
        apps::uwb_master::{UwbClientResponse, UwbMasterPoll},
        Token,
    },
    utils::nonblocking_wait_async,
};

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

    defmt::info!("DW3000 Reset!");

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

    defmt::info!("DW3000 Initialized!");

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

    defmt::info!("DW3000 Ready!");

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

        dw3000 = receiving.finish_receiving().unwrap();

        // Parse the poll packet
        let frame = match Ieee802154Frame::new_checked(&buffer[..len]) {
            Ok(packet) => packet,
            Err(e) => {
                defmt::error!("Error parsing poll packet: {:?}", e);
                continue;
            }
        };

        defmt::info!(
            "Received poll packet: len: {}, rxts_poll: {}, quality: {:?}",
            len,
            rxts_poll,
            quality
        );

        let (payload, crc) = frame
            .payload()
            .unwrap()
            .split_last_chunk::<2>()
            .unwrap_or((&[], &[0, 0]));

        let poll_packet: &UwbMasterPoll = match bytemuck::try_from_bytes(payload) {
            Ok(poll_packet) => poll_packet,
            Err(e) => {
                defmt::error!("Received invalid payload: {:?}", payload);
                continue;
            }
        };

        defmt::info!("Received poll packet: {:?}", poll_packet);

        let my_tx_slot = poll_packet.slots.iter().position(|s| *s == address);

        if my_tx_slot.is_none() {
            defmt::info!("Not allocated to any slots");
            continue;
        }

        let my_tx_slot = my_tx_slot.unwrap();

        // Prepare the response packet
        let response_txtime = rxts_poll
            + DwDuration::from_nanos(
                Duration::from_millis(4 + (my_tx_slot as u64) * 2).as_micros() as u32 * 1000,
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
    }

    defmt::info!("UWB client stopped");

    stopped_signal.store(true, core::sync::atomic::Ordering::Release);
}

/// Controller for the UWB client application
pub async fn uwb_client<'a>(spawner_core1: SendSpawner, args: &[Token<'a>]) -> Result<(), ()> {
    if args.len() < 2 {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: uwb_master <start|stop>\n");
        return Err(());
    }

    static STOP_SIGNAL: OnceLock<Signal<CriticalSectionRawMutex, bool>> = OnceLock::new();
    let stop_signal = STOP_SIGNAL.get_or_init(Signal::new);
    static STOPPED_SIGNAL: AtomicBool = AtomicBool::new(true);

    let command = &args[1];

    if let Token::String(command) = command
        && *command == "start"
    {
        // make sure we don't already have a task running
        if !STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB client already running\n");
            return Err(());
        }

        spawner_core1
            .spawn(uwb_client_task(stop_signal, &STOPPED_SIGNAL))
            .unwrap();

        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB client started\n");
    } else if let Token::String(command) = command
        && *command == "stop"
    {
        if STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB client not running\n");
            return Err(());
        }

        stop_signal.signal(true);
    } else {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: uwb_master <start|stop>\n");
        return Err(());
    }

    Ok(())
}
