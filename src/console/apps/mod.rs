use super::Token;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Timer};

extern crate alloc;

pub mod conf;
pub use conf::conf;

pub mod iperf;
pub use iperf::iperf;

pub mod imu_stream;
pub use imu_stream::imu_stream;

pub mod imu_recv;
pub use imu_recv::imu_recv;

pub async fn free<'a>(_: &[Token<'a>]) -> Result<(), ()> {
    let free_mem = esp_alloc::HEAP.free();
    let _ = esp_fast_serial::write_to_usb_serial_buffer(
        alloc::format!("Free memory: {} bytes\n", free_mem).as_bytes(),
    );
    Ok(())
}

pub async fn ping<'a>(args: &[Token<'a>]) -> Result<(), ()> {
    if args.len() < 2 {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Invalid number of arguments\n");
        return Err(());
    }

    let ip_addr = if let Token::String(ip_addr) = args[1] {
        ip_addr
            .parse::<smoltcp::wire::Ipv4Address>()
            .map_err(|_| ())?
    } else {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Invalid IP address\n");
        return Err(());
    };

    let stack = crate::network::WIFI_STACK.get().await;

    if !stack.is_link_up() {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"WiFi not initialized\n");
        return Err(());
    }

    let mut rx_metadata_buffer = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut rx_payload_buffer = [0; 1024];
    let mut tx_metadata_buffer = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut tx_payload_buffer = [0; 1024];

    // UDP socket
    let mut socket = embassy_net::udp::UdpSocket::new(
        stack.clone(),
        &mut rx_metadata_buffer,
        &mut rx_payload_buffer,
        &mut tx_metadata_buffer,
        &mut tx_payload_buffer,
    );

    socket.bind(19888).map_err(|_| {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Failed to bind UDP socket\n");
    })?;

    for _ in 0..10 {
        let packet = b"Hello, world!";

        let send_timeout = Timer::after(Duration::from_millis(100));
        let send_result = select(
            async {
                socket
                    .send_to(
                        packet,
                        embassy_net::IpEndpoint::new(embassy_net::IpAddress::Ipv4(ip_addr), 19888),
                    )
                    .await
            },
            send_timeout,
        )
        .await;

        if let Either::Second(_) = send_result {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(
                b"Failed to send packet (host unreachable?)\n",
            );
            return Err(());
        }

        let send_time = Instant::now();

        if let Either::First(r) = send_result {
            if r.is_err() {
                let _ = esp_fast_serial::write_to_usb_serial_buffer(
                    alloc::format!("Failed to send packet: {:?}\n", r).as_bytes(),
                );
                return Err(());
            }
        }

        // Wait for a reply
        let timeout = Timer::after(Duration::from_millis(100));

        let mut recv_buffer = [0; 1024];
        let res = select(socket.recv_from(&mut recv_buffer), timeout).await;

        match res {
            Either::First(_) => {
                let _ = esp_fast_serial::write_to_usb_serial_buffer(
                    alloc::format!(
                        "Received reply, time: {}ms\n",
                        send_time.elapsed().as_millis()
                    )
                    .as_bytes(),
                );
            }
            Either::Second(_) => {
                let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Timeout waiting for reply\n");
            }
        }

        Timer::after(Duration::from_millis(1000)).await;
    }
    Ok(())
}

pub async fn pong<'a>(_args: &[Token<'a>]) -> Result<(), ()> {
    let stack = crate::network::WIFI_STACK.get().await;

    if !stack.is_link_up() {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"WiFi not initialized\n");
        return Err(());
    }

    let mut rx_metadata_buffer = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut rx_payload_buffer = [0; 1024];
    let mut tx_metadata_buffer = [embassy_net::udp::PacketMetadata::EMPTY; 1];
    let mut tx_payload_buffer = [0; 1024];

    // UDP socket
    let mut socket = embassy_net::udp::UdpSocket::new(
        stack.clone(),
        &mut rx_metadata_buffer,
        &mut rx_payload_buffer,
        &mut tx_metadata_buffer,
        &mut tx_payload_buffer,
    );

    socket.bind(19888).map_err(|_| {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Failed to bind UDP socket\n");
    })?;

    let mut pong_count = 0;

    loop {
        if pong_count > 20 {
            break;
        }

        let mut recv_buffer = [0; 1024];

        let timeout = Timer::after(Duration::from_millis(10000)); // 10 second timeout

        let res = select(socket.recv_from(&mut recv_buffer), timeout).await;

        match res {
            Either::First(result) => {
                if let Ok((_size, endpoint)) = result {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(
                        alloc::format!("Received ping from {:?}\n", endpoint).as_bytes(),
                    );
                    let packet = b"Pong!";
                    socket.send_to(packet, endpoint).await.map_err(|_| {
                        let _ =
                            esp_fast_serial::write_to_usb_serial_buffer(b"Failed to send packet\n");
                    })?;

                    socket.flush().await; // Ensure the response packet is sent

                    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Sent pong\n");
                    pong_count += 1;
                }
            }
            Either::Second(_) => {
                let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Timeout\n");

                return Err(());
            }
        }
    }

    Ok(())
}

pub async fn reset<'a>(_args: &[Token<'a>]) -> Result<(), ()> {
    esp_hal::reset::software_reset();
    Ok(())
}
