// UWB monitor app

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Reader};

use crate::ranging::uwb_driver::RxTiming;

use super::Token;

pub async fn uwb_monitor<'a>(
    reader: &Reader<'static, CriticalSectionRawMutex, 256>,
    args: &[Token<'a>],
) -> Result<(), ()> {
    let uwb_device = crate::UWB_DEVICE.get().await;

    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB Monitor started\n");

    loop {
        let mut uwb_device = uwb_device.lock().await;
        uwb_device.send_rx_request(RxTiming::Now).await;

        let packet = uwb_device.rx.receive().await;

        defmt::debug!("UWB Packet: {:?}", packet);

        if packet.rx_meta.success {
            break;
        }

        uwb_device.rx.receive_done();
    }

    Ok(())
}
