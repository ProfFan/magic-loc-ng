use core::sync::atomic::AtomicBool;

use crate::hist_buffer::HistoryBuffer;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex, once_lock::OnceLock, pubsub::WaitResult, signal::Signal,
};
use esp_hal::macros::ram;

use super::Token;

const IMU_PACKET_HISTORY_SIZE: usize = 30;

#[derive(Debug)]
#[repr(C)]
pub struct IMUPacket {
    /// Protocol header
    pub header: [u8; 4],
    /// IPv4 address of the sender
    pub origin: [u8; 4],
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Sample number
    pub sample_num: u64,
    /// FIFO packets
    pub packets: HistoryBuffer<icm426xx::fifo::FifoPacket4, IMU_PACKET_HISTORY_SIZE>,
}

/// `defmt` formatter for `IMUPacket`
impl defmt::Format for IMUPacket {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "IMUPacket {{ timestamp: {}, packets: {:?} }}",
            self.timestamp,
            self.packets.iter()
        )
    }
}

/// IMU streaming application
#[embassy_executor::task]
#[ram]
pub async fn imu_stream_task(
    stop_signal: &'static Signal<NoopRawMutex, bool>,
    stopped_signal: &'static AtomicBool,
) {
    stopped_signal.store(false, core::sync::atomic::Ordering::Release);

    let mut imu_sub = crate::IMU_PUBSUB.get().await.subscriber().unwrap();

    let mut wire_packet = IMUPacket {
        header: *b"MIMU",
        origin: [0; 4],
        timestamp: 0,
        sample_num: 0,
        packets: HistoryBuffer::new(),
    };

    let stack = crate::network::WIFI_STACK.get().await;
    let address = stack.config_v4().unwrap().address;
    wire_packet.origin = address.address().octets();

    let source_endpoint = embassy_net::IpEndpoint::new(address.address().into(), 50000);
    let broadcast_ep =
        embassy_net::IpEndpoint::new(embassy_net::IpAddress::v4(255, 255, 255, 255), 50001);

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
        *stack,
        &mut rx_metadata_buffer,
        &mut rx_payload_buffer,
        &mut tx_metadata_buffer,
        &mut tx_payload_buffer,
    );

    socket.bind(source_endpoint).unwrap();

    let mut records_written: u64 = 0;
    let mut sample_idx: u64 = 0;

    loop {
        if stop_signal.signaled() {
            stop_signal.reset();
            break;
        }

        let imu_packet = match imu_sub.next_message().await {
            WaitResult::Message(imu_packet) => imu_packet,
            WaitResult::Lagged(_) => {
                defmt::warn!("IMU stream lagged!!!");
                continue;
            }
        };

        wire_packet.timestamp = embassy_time::Instant::now().as_micros();
        wire_packet.packets.write(imu_packet);
        records_written += 1;
        sample_idx += 1;
        wire_packet.sample_num = sample_idx;

        if records_written >= (IMU_PACKET_HISTORY_SIZE / 3) as u64 {
            let wire_packet_bytes = unsafe {
                core::mem::transmute::<&IMUPacket, &[u8; core::mem::size_of::<IMUPacket>()]>(
                    &wire_packet,
                )
            };

            socket
                .send_to(wire_packet_bytes, broadcast_ep)
                .await
                .unwrap();
            records_written = 0;
        }
    }

    socket.flush().await;

    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"IMU stream stopped\n");

    stopped_signal.store(true, core::sync::atomic::Ordering::Release);
}

static STOP_SIGNAL: OnceLock<Signal<NoopRawMutex, bool>> = OnceLock::new();
static STOPPED_SIGNAL: AtomicBool = AtomicBool::new(true);

pub async fn imu_streamer_start(spawner: Spawner) -> Result<(), ()> {
    let stop_signal = STOP_SIGNAL.get_or_init(Signal::new);

    // make sure we don't already have a task running
    if !STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
        return Err(());
    }

    spawner
        .spawn(imu_stream_task(stop_signal, &STOPPED_SIGNAL))
        .unwrap();

    Ok(())
}

pub async fn imu_streamer_stop() -> Result<(), ()> {
    if STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
        return Err(()); // already stopped
    }

    STOP_SIGNAL.get_or_init(Signal::new).signal(true);
    Ok(())
}

pub async fn imu_stream<'a>(spawner: Spawner, args: &[Token<'a>]) -> Result<(), ()> {
    if args.len() < 2 {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: imu_stream <start|stop>\n");
        return Err(());
    }

    let command = &args[1];

    if let Token::String(command) = command
        && *command == "start"
    {
        let result = imu_streamer_start(spawner).await;

        if let Err(()) = result {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Failed to start IMU stream\n");
            return Err(());
        }

        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"IMU stream started\n");
    } else if let Token::String(command) = command
        && *command == "stop"
    {
        let result = imu_streamer_stop().await;

        if let Err(()) = result {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Failed to stop IMU stream\n");
            return Err(());
        }
    } else {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: imu_stream <start|stop>\n");
        return Err(());
    }

    Ok(())
}
