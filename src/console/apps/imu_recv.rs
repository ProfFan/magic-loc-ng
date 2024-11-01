use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_net::IpEndpoint;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, once_lock::OnceLock, signal::Signal};

use super::Token;

/// IMU receiver application
#[embassy_executor::task]
pub async fn imu_recv_app(
    endpoint: IpEndpoint,
    stop_token: &'static Signal<NoopRawMutex, bool>,
    stopped: &'static AtomicBool,
) {
    stopped.store(false, Ordering::Release);

    // UDP socket
    let stack = crate::network::WIFI_STACK.get().await;

    let mut rx_metadata_buffer = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut rx_payload_buffer = [0; 1024];
    let mut tx_metadata_buffer = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut tx_payload_buffer = [0; 1024];

    let mut socket = embassy_net::udp::UdpSocket::new(
        *stack,
        &mut rx_metadata_buffer,
        &mut rx_payload_buffer,
        &mut tx_metadata_buffer,
        &mut tx_payload_buffer,
    );

    match socket.bind(endpoint) {
        Ok(_) => {}
        Err(e) => {
            defmt::error!("Failed to bind UDP socket: {:?}", e);
            stopped.store(true, Ordering::Release);
            return;
        }
    }

    loop {
        let mut packet_buffer = [0; 1024];
        let (len_recv, _metadata) =
            match select(socket.recv_from(&mut packet_buffer), stop_token.wait()).await {
                Either::First(recv) => recv.unwrap(),
                Either::Second(_) => break,
            };

        defmt::trace!("Received {} bytes", len_recv);

        const IMU_PACKET_SIZE: usize = core::mem::size_of::<super::imu_stream::IMUPacket>();
        if len_recv == IMU_PACKET_SIZE {
            let packet_received: &[u8; IMU_PACKET_SIZE] =
                &packet_buffer[..IMU_PACKET_SIZE].try_into().unwrap();
            let imu_packet = unsafe {
                core::mem::transmute::<&[u8; IMU_PACKET_SIZE], &super::imu_stream::IMUPacket>(
                    packet_received,
                )
            };

            if let Some(last_sample) = imu_packet.packets.last() {
                // fixed width integer formatting
                let _ = esp_fast_serial::write_to_usb_serial_buffer(alloc::format!(
                    "IMU packet from {:?}: T_HOST: {:09}, ACC: ({:06}, {:06}, {:06}), GYR: ({:06}, {:06}, {:06}), TEMP: {:04}\n",
                    imu_packet.origin,
                    imu_packet.timestamp,
                    last_sample.accel_data_x(),
                    last_sample.accel_data_y(),
                    last_sample.accel_data_z(),
                    last_sample.gyro_data_x(),
                    last_sample.gyro_data_y(),
                    last_sample.gyro_data_z(),
                    last_sample.temperature_raw()
                ).as_bytes());
            }
        }
    }

    stopped.store(true, Ordering::Release);
}

/// IMU receiver control command
pub async fn imu_recv(spawner: Spawner, args: &[Token<'_>]) -> Result<(), ()> {
    if args.len() != 3 {
        let _ =
            esp_fast_serial::write_to_usb_serial_buffer(b"Usage: imu_recv <port> <start|stop>\n");

        return Err(());
    }

    let port_str = match args[1] {
        Token::String(s) => s,
        _ => return Err(()),
    };

    let port = port_str.parse::<u16>().map_err(|_| ())?;

    let command = match args[2] {
        Token::String(s) => s,
        _ => return Err(()),
    };

    static STOP_SIGNAL: OnceLock<Signal<NoopRawMutex, bool>> = OnceLock::new();
    let stop_signal = STOP_SIGNAL.get_or_init(Signal::new);
    static STOPPED_FLAG: AtomicBool = AtomicBool::new(true);

    match command {
        "start" => {
            if !STOPPED_FLAG.load(core::sync::atomic::Ordering::Acquire) {
                let _ =
                    esp_fast_serial::write_to_usb_serial_buffer(b"IMU receiver already running\n");
                return Err(());
            }

            spawner
                .spawn(imu_recv_app(
                    IpEndpoint::new(embassy_net::IpAddress::v4(0, 0, 0, 0), port),
                    stop_signal,
                    &STOPPED_FLAG,
                ))
                .unwrap();
        }
        "stop" => {
            if STOPPED_FLAG.load(core::sync::atomic::Ordering::Acquire) {
                let _ = esp_fast_serial::write_to_usb_serial_buffer(b"IMU receiver not running\n");
                return Err(());
            }

            stop_signal.signal(true);
        }
        _ => return Err(()),
    }

    Ok(())
}
