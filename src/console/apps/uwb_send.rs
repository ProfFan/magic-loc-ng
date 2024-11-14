// Send UWB test packets

use core::mem::MaybeUninit;

use alloc::sync::Arc;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, once_lock::OnceLock, pipe::Reader,
};

use crate::ranging::uwb_driver::{TxTiming, MTU_802154};

use smoltcp::wire::{Ieee802154Address, Ieee802154Frame, Ieee802154Repr};

use super::Token;

pub async fn uwb_send<'a>(
    _reader: &Reader<'static, CriticalSectionRawMutex, 256>,
    _args: &[Token<'a>],
) -> Result<(), ()> {
    let uwb_device = crate::UWB_DEVICE.get().await;

    let _ = esp_fast_serial::write_to_usb_serial_buffer(b"UWB Send started\n");

    let mut uwb_device = uwb_device.lock().await;

    let tx_result = Arc::new(OnceLock::new());

    uwb_device
        .send_uwb_tx_request(|txr| {
            txr.tx_time = TxTiming::Now;

            txr.tx_result = Some(tx_result.clone());

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

            // SAFETY: we will not read from the `[u8]` created from the `[MaybeUninit<u8>]`
            let mut frame = unsafe {
                Ieee802154Frame::new_unchecked(core::mem::transmute::<
                    &mut [MaybeUninit<u8>; MTU_802154],
                    &mut [u8; MTU_802154],
                >(&mut txr.buf))
            };

            FRAME_REPR.emit(&mut frame);
            frame.payload_mut().unwrap()[0..13].copy_from_slice(b"Hello, world!");

            txr.len = FRAME_REPR.buffer_len() + 13;
        })
        .await;

    let tx_result = tx_result.get().await;

    defmt::info!("TX result: {:?}", tx_result);

    Ok(())
}
