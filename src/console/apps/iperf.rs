use super::Token;

pub async fn iperf_server() -> Result<(), ()> {
    Ok(())
}

pub async fn iperf_client(_ip_addr: smoltcp::wire::Ipv4Address) -> Result<(), ()> {
    Ok(())
}

/// UDP iperf application
/// iperf -s
/// iperf <ip>
pub async fn iperf<'a>(args: &[Token<'a>]) -> Result<(), ()> {
    if args.len() < 2 {
        let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Invalid number of arguments\n");
        return Err(());
    }

    let is_server = if let Token::String(arg) = args[1] {
        arg == "-s"
    } else {
        false
    };

    if !is_server {
        let ip_addr = if let Token::String(ip_addr) = args[2] {
            ip_addr
        } else {
            let _ = esp_fast_serial::write_to_usb_serial_buffer(b"Invalid IP address\n");
            return Err(());
        };

        let ip_addr = ip_addr.parse::<smoltcp::wire::Ipv4Address>().unwrap();

        iperf_client(ip_addr).await?;
    } else {
        iperf_server().await?;
    }

    Ok(())
}
