use core::sync::atomic::AtomicBool;

use dw3000_ng::{
    configs::{StsLen, StsMode},
    hl::{ConfigGPIOs, RxQuality, SendTime},
    time::{Duration as DwDuration, Instant as DwInstant},
};
use embassy_executor::SendSpawner;

use bytemuck::{AnyBitPattern, NoUninit, Pod, Zeroable};
use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    once_lock::OnceLock,
    signal::Signal,
};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::macros::ram;
use smoltcp::wire::{Ieee802154Address, Ieee802154Frame, Ieee802154Repr};

use crate::{console::Token, utils::nonblocking_wait_async};

#[derive(Debug, Clone, Copy, AnyBitPattern, NoUninit, defmt::Format)]
#[repr(C)]
pub struct UwbMasterPoll {
    /// Header
    pub header: u8,

    /// 8 TX slots
    pub slots: [u8; 8],

    /// 40-bit DW3000 time on master, wrap-around in about 17.2 seconds
    pub tx_time: [u8; 5],
}

#[derive(Debug, Clone, Copy, AnyBitPattern, NoUninit, defmt::Format)]
#[repr(C)]
pub struct UwbClientResponse {
    /// Header
    pub header: u8,

    /// 40-bit DW3000 timestamp when the client received the poll
    pub rx_time_poll: [u8; 5],

    /// 40-bit DW3000 timestamp when the client sent the response
    pub tx_time_response: [u8; 5],
}

#[derive(Debug, Clone, Copy, Zeroable, Pod, defmt::Format)]
#[repr(C)]
pub struct UwbRxTimeReportSlot {
    /// Address of the client
    pub address: [u8; 2],
    /// RX time
    pub rx_time: [u8; 5],
}

#[derive(Debug, Clone, Copy, Zeroable, Pod, defmt::Format)]
#[repr(C)]
pub struct UwbTxTimeReportSlot {
    /// Address of the client
    pub address: [u8; 2],
    /// TX time
    pub tx_time: [u8; 5],
}

#[derive(Debug, Clone, Copy, AnyBitPattern, NoUninit, defmt::Format)]
#[repr(C)]
pub struct UwbMasterReport {
    /// TX time in the CPU clock
    pub cpu_tx_completion_time: u64,

    /// TX time of the current poll
    pub poll_tx_time: [u8; 5],

    /// Sequence number of the current poll
    pub poll_sequence_number: u8,

    /// Number of active slots in the current round
    pub num_slots: u8,

    /// Number of responses in the current round
    pub num_responses: u8,

    /// RX times for each of the 8 slots in the current round
    pub response_rx_time: [UwbRxTimeReportSlot; 8],

    /// TX times for each of the 8 slots in the current round
    pub response_tx_time: [UwbTxTimeReportSlot; 8],
}

#[derive(Debug, Clone, Copy, AnyBitPattern, NoUninit, defmt::Format)]
#[repr(C)]
pub struct UwbClientReport {
    /// RX time of the poll in CPU clock
    pub cpu_rx_time: u64,

    /// RX time of the poll
    pub poll_rx_time: [u8; 5],

    /// Sequence number of the poll
    pub poll_sequence_number: u8,

    /// Address of the client
    pub address: [u8; 2],

    /// Number of responses from other clients
    pub num_responses: u8,

    /// Padding
    pub padding: [u8; 7],

    /// RX time of the responses from other clients
    pub response_rx_time: [UwbRxTimeReportSlot; 8],
}

/// UWB master application
///
/// The UWB master application is responsible for initiating ranging polls and processing the responses.
/// It also handles the airtime allocation for the network.
///
/// The master application runs on core 1.
#[embassy_executor::task]
#[ram]
pub async fn uwb_master_task(
    stop_signal: &'static embassy_sync::signal::Signal<CriticalSectionRawMutex, bool>,
    stopped_signal: &'static core::sync::atomic::AtomicBool,
) {
    stopped_signal.store(false, core::sync::atomic::Ordering::Release);

    let mut uwb_device = crate::DW3000.get().await.lock().await;

    let (spi, rst, irq) = uwb_device.split_borrow();

    // Reset
    rst.set_low();
    Timer::after(Duration::from_millis(10)).await;
    rst.set_high();

    defmt::info!("DW3000 Reset!");

    Timer::after(Duration::from_millis(200)).await;

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

    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(100));
    let mut sequence_number = 0;
    loop {
        ticker.next().await;

        if stop_signal.signaled() {
            stop_signal.reset(); // IMPORTANT: reset the signal so we can re-enter the loop
            break;
        }

        let timeout = Instant::now() + Duration::from_millis(90);

        // The master is always with address 0x0000 and pan id 0xDEAD
        const FRAME_REPR: Ieee802154Repr = Ieee802154Repr {
            frame_type: smoltcp::wire::Ieee802154FrameType::Data,
            frame_version: smoltcp::wire::Ieee802154FrameVersion::Ieee802154_2006,
            security_enabled: false,
            sequence_number: Some(0),
            frame_pending: false,
            ack_request: false,
            pan_id_compression: true,
            dst_addr: Some(Ieee802154Address::BROADCAST),
            src_addr: Some(Ieee802154Address::Short([0x00, 0x00])),
            src_pan_id: Some(smoltcp::wire::Ieee802154Pan(0xDEAD)),
            dst_pan_id: None,
        };

        let mut buffer = [0u8; 127];
        let mut frame = Ieee802154Frame::new_unchecked(&mut buffer);
        FRAME_REPR.emit(&mut frame);

        frame.set_sequence_number(sequence_number);
        sequence_number += 1;

        let current_dw_time: DwInstant =
            DwInstant::new((dw3000.sys_time().unwrap() as u64) << 8).unwrap();
        let send_time = current_dw_time + DwDuration::from_nanos(1500000); // 1.5ms
        let send_time = DwInstant::new(send_time.value() >> 9 << 9).unwrap();

        frame.payload_mut().unwrap()[..core::mem::size_of::<UwbMasterPoll>()].copy_from_slice(
            bytemuck::bytes_of(&UwbMasterPoll {
                header: b'P',
                slots: [0x07, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00],
                tx_time: *send_time.value().to_le_bytes().first_chunk::<5>().unwrap(),
            }),
        );
        let len = FRAME_REPR.buffer_len() + core::mem::size_of::<UwbMasterPoll>();

        let mut sending = dw3000
            .send_raw(&buffer[..len], SendTime::Delayed(send_time), &dwm_config)
            .unwrap();

        let send_result: Result<DwInstant, dw3000_ng::Error<_>> =
            nonblocking_wait_async(async || sending.s_wait(), irq).await;

        if let Err(e) = send_result {
            defmt::error!("Error waiting for send: {:?}", e);
            dw3000 = sending.finish_sending().unwrap();
            continue;
        }

        defmt::info!("Sent poll with time {:?}", send_result.unwrap());

        dw3000 = sending.finish_sending().unwrap();

        // Wait for responses for up to 90ms
        loop {
            let mut receiving = dw3000.receive(dwm_config).unwrap();

            let mut buffer = [0u8; 127];
            let receive_fut =
                nonblocking_wait_async(async || receiving.r_wait_buf(&mut buffer), irq);

            let race = select(receive_fut, Timer::at(timeout)).await;

            let result: Result<(usize, DwInstant, RxQuality), _> = match race {
                Either::First(receive_result) => receive_result,
                Either::Second(_) => {
                    defmt::info!("Timeout waiting for response");
                    dw3000 = receiving.finish_receiving().unwrap();
                    break;
                }
            };
            dw3000 = receiving.finish_receiving().unwrap();

            let (len, rx_time, quality) = match result {
                Ok(result) => result,
                Err(e) => {
                    defmt::error!("Error receiving: {:?}", e);
                    continue;
                }
            };
            defmt::info!("Received response at {}, quality {:?}", rx_time, quality);

            let frame = match Ieee802154Frame::new_checked(&buffer[..len]) {
                Ok(frame) => frame,
                Err(e) => {
                    defmt::error!("Received invalid frame: {:?}", e);
                    continue;
                }
            };

            let (payload, _fcs) = frame
                .payload()
                .unwrap()
                .split_last_chunk::<2>()
                .unwrap_or((&[], &[0, 0]));

            let uwb_client_response: &UwbClientResponse = match bytemuck::try_from_bytes(payload) {
                Ok(uwb_client_response) => uwb_client_response,
                Err(_e) => {
                    defmt::error!("Received invalid payload: {:?}", payload);
                    continue;
                }
            };

            defmt::info!(
                "Received response from {:?} at {:?}, response {:?}",
                frame.src_addr(),
                rx_time,
                uwb_client_response
            );
        }
    }

    stopped_signal.store(true, core::sync::atomic::Ordering::Release);
}

/// Controller for the UWB master application
pub async fn uwb_master<'a>(spawner_core1: SendSpawner, args: &[Token<'a>]) -> Result<(), ()> {
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
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB master already running\n");
            return Err(());
        }

        spawner_core1
            .spawn(uwb_master_task(stop_signal, &STOPPED_SIGNAL))
            .unwrap();

        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB master started\n");
    } else if let Token::String(command) = command
        && *command == "stop"
    {
        if STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB master not running\n");
            return Err(());
        }

        stop_signal.signal(true);
    } else {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: uwb_master <start|stop>\n");
        return Err(());
    }

    Ok(())
}
