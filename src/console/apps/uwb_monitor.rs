// UWB monitor app

use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Reader};

use crate::ranging::uwb_driver::RxTiming;

use super::Token;

pub async fn uwb_monitor<'a>(
    reader: &Reader<'static, CriticalSectionRawMutex, 256>,
    _args: &[Token<'a>],
) -> Result<(), ()> {
    let uwb_device = crate::UWB_DEVICE.get().await;

    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB Monitor started\n");

    loop {
        let mut uwb_device = uwb_device.lock().await;
        uwb_device.send_rx_request(RxTiming::Now).await;

        let keyboard_event = async {
            let mut key_buffer = [0u8; 1];
            loop {
                let len = reader.read(&mut key_buffer).await;
                if len > 0 && key_buffer[0] == b'q' {
                    break;
                }
            }
        };
        let packet = match select(uwb_device.rx.receive(), keyboard_event).await {
            Either::First(packet) => packet,
            Either::Second(_) => return Ok(()),
        };

        defmt::trace!("UWB Packet: {:?}", packet);

        alloc::format!("Received: {:?}\n", packet)
            .as_bytes()
            .chunks(64)
            .for_each(|c| {
                let _ = esp_fast_serial::write_to_usb_serial_buffer(c);
            });

        // Try decode with 802.15.4
        let decoded = smoltcp::wire::Ieee802154Frame::new_checked(packet.as_slice());
        defmt::trace!("Decoded: {:?}", decoded);

        if let Ok(decoded) = decoded {
            alloc::format!("Decoded: {}, Payload: {:?}\n", decoded, decoded.payload())
                .as_bytes()
                .chunks(64)
                .for_each(|c| {
                    let _ = esp_fast_serial::write_to_usb_serial_buffer(c);
                });
        }

        uwb_device.rx.receive_done();
    }
}
