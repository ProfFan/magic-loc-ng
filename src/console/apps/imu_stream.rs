use core::sync::atomic::AtomicBool;

use crate::hist_buffer::HistoryBuffer;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, once_lock::OnceLock, signal::Signal};

use super::Token;

use crate::IMU_PUBSUB;

const IMU_PACKET_HISTORY_SIZE: usize = 10;

#[derive(Debug)]
#[repr(C)]
pub struct IMUPacket {
    /// Protocol header
    pub header: [u8; 4],
    /// IPv4 address of the sender
    pub origin: [u8; 4],
    /// Timestamp in microseconds
    pub timestamp: u64,
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
pub async fn imu_stream_task(
    stop_signal: &'static Signal<NoopRawMutex, bool>,
    stopped_signal: &'static AtomicBool,
) {
    stopped_signal.store(false, core::sync::atomic::Ordering::Release);

    let mut imu_sub = IMU_PUBSUB.get().await.subscriber().unwrap();

    let mut wire_packet = IMUPacket {
        header: *b"MIMU",
        origin: [0; 4],
        timestamp: 0,
        packets: HistoryBuffer::new(),
    };

    let stack = crate::network::WIFI_STACK.get().await;
    let address = stack.config_v4().unwrap().address;
    wire_packet.origin = address.address().octets();

    let source_endpoint = embassy_net::IpEndpoint::new(address.address().into(), 50000);
    let endpoint =
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

    let mut records_written = 0;

    loop {
        let imu_packet = match select(imu_sub.next_message_pure(), stop_signal.wait()).await {
            Either::First(p) => p,
            Either::Second(_) => break,
        };

        wire_packet.timestamp = embassy_time::Instant::now().as_micros();
        wire_packet.packets.write(imu_packet);
        records_written += 1;

        if records_written >= IMU_PACKET_HISTORY_SIZE / 2 {
            let wire_packet_bytes = unsafe {
                core::mem::transmute::<&IMUPacket, &[u8; core::mem::size_of::<IMUPacket>()]>(
                    &wire_packet,
                )
            };

            socket.send_to(wire_packet_bytes, endpoint).await.unwrap();
            records_written = 0;
        }
    }

    socket.flush().await;

    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"IMU stream stopped\n");

    stopped_signal.store(true, core::sync::atomic::Ordering::Release);
}

pub async fn imu_stream<'a>(spawner: Spawner, args: &[Token<'a>]) -> Result<(), ()> {
    if args.len() < 2 {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: imu_stream <start|stop>\n");
        return Err(());
    }

    static STOP_SIGNAL: OnceLock<Signal<NoopRawMutex, bool>> = OnceLock::new();
    let stop_signal = STOP_SIGNAL.get_or_init(Signal::new);
    static STOPPED_SIGNAL: AtomicBool = AtomicBool::new(true);

    let command = &args[1];

    if let Token::String(command) = command
        && *command == "start"
    {
        // make sure we don't already have a task running
        if !STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"IMU stream already running\n");
            return Err(());
        }

        spawner
            .spawn(imu_stream_task(stop_signal, &STOPPED_SIGNAL))
            .unwrap();

        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"IMU stream started\n");
    } else if let Token::String(command) = command
        && *command == "stop"
    {
        if STOPPED_SIGNAL.load(core::sync::atomic::Ordering::Acquire) {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"IMU stream not running\n");
            return Err(());
        }

        stop_signal.signal(true);
    } else {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Usage: imu_stream <start|stop>\n");
        return Err(());
    }

    Ok(())
}
