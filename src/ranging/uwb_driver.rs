use core::{cell::OnceCell, mem::MaybeUninit};

use alloc::sync::Arc;
use dw3000_ng::{
    self,
    configs::{StsLen, StsMode},
    hl::ConfigGPIOs,
};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    zerocopy_channel,
};
use embassy_time::{Duration, Timer};
use esp_hal::{
    dma::ChannelCreator,
    gpio::{Input, Output},
    peripherals::SPI3,
    spi::{master::Spi, FullDuplexMode},
};
use esp_hal::{
    dma::{DmaPriority, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    macros::ram,
};

use crate::{configuration::ConfigurationStore, utils::nonblocking_wait_async};

/// 127 bytes according to the 802.15.4 Standard
pub const MTU_802154: usize = 127;

/// Send timing for TX
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum TxTiming {
    /// Send as fast as possible
    Now,
    /// Send after some time
    Delayed(dw3000_ng::time::Instant),
}

/// Rx timing for RX
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum RxTiming {
    /// Receive as fast as possible
    Now,
    /// Receive after some time (at the given DW3000 timestamp)
    Delayed(dw3000_ng::time::Instant),
}

/// Represents a packet of size MTU that is ready to be sent.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct UwbPacketTxBuf<const MTU: usize> {
    tx_time: TxTiming,
    len: usize,
    buf: [u8; MTU],
}

impl<const MTU: usize> UwbPacketTxBuf<MTU> {
    pub const fn new() -> Self {
        Self {
            tx_time: TxTiming::Now,
            len: 0,
            buf: [0; MTU],
        }
    }
}

/// Request to receive a packet
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct UwbPacketRxRequest {
    rx_time: RxTiming,
}

#[derive(Debug, Clone, Copy, defmt::Format, Default)]
pub enum UwbRequest<const MTU: usize> {
    #[default]
    None,
    Tx(UwbPacketTxBuf<MTU>),
    Rx(UwbPacketRxRequest),
}

/// Rx packet buffer
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct UwbPacketRxBuf<const MTU: usize> {
    rx_time: dw3000_ng::time::Instant,
    len: usize,
    buf: [u8; MTU],
}

impl<const MTU: usize> Default for UwbPacketRxBuf<MTU> {
    fn default() -> Self {
        Self {
            rx_time: dw3000_ng::time::Instant::new(0).unwrap(),
            len: 0,
            buf: [0; MTU],
        }
    }
}

pub struct State<const MTU: usize, const N_RX: usize, const N_TX: usize> {
    tx: [UwbRequest<MTU>; N_TX],
    rx: [UwbPacketRxBuf<MTU>; N_RX],
    inner: MaybeUninit<StateInner<'static, MTU>>,
}

impl<const MTU: usize, const N_RX: usize, const N_TX: usize> State<MTU, N_RX, N_TX> {
    pub fn new() -> Self {
        Self {
            tx: [UwbRequest::<MTU>::default(); N_TX],
            rx: [UwbPacketRxBuf::<MTU>::default(); N_RX],
            inner: MaybeUninit::uninit(),
        }
    }
}

pub struct StateInner<'d, const MTU: usize> {
    tx: zerocopy_channel::Channel<'d, NoopRawMutex, UwbRequest<MTU>>,
    rx: zerocopy_channel::Channel<'d, NoopRawMutex, UwbPacketRxBuf<MTU>>,
}

pub struct UwbRunner<'d, const MTU: usize> {
    tx: zerocopy_channel::Receiver<'d, NoopRawMutex, UwbRequest<MTU>>,
    rx: zerocopy_channel::Sender<'d, NoopRawMutex, UwbPacketRxBuf<MTU>>,
}

pub struct Device<'d, const MTU: usize> {
    tx: zerocopy_channel::Sender<'d, NoopRawMutex, UwbRequest<MTU>>,
    rx: zerocopy_channel::Receiver<'d, NoopRawMutex, UwbPacketRxBuf<MTU>>,
}

pub fn new<'d, const MTU: usize, const N_RX: usize, const N_TX: usize>(
    state: &'d mut State<MTU, N_RX, N_TX>,
) -> (UwbRunner<'d, MTU>, Device<'d, MTU>) {
    let state_uninit: *mut MaybeUninit<StateInner<'d, MTU>> =
        (&mut state.inner as *mut MaybeUninit<StateInner<'static, MTU>>).cast();

    // SAFETY: this is a self-referential struct, however:
    // - it can't move while the `'d` borrow is active.
    // - when the borrow ends, the dangling references inside the MaybeUninit will never be used again.
    let state = unsafe { &mut *state_uninit }.write(StateInner {
        tx: zerocopy_channel::Channel::new(&mut state.tx[..]),
        rx: zerocopy_channel::Channel::new(&mut state.rx[..]),
    });

    let (tx_sender, tx_receiver) = state.tx.split();
    let (rx_sender, rx_receiver) = state.rx.split();

    (
        UwbRunner {
            tx: tx_receiver,
            rx: rx_sender,
        },
        Device {
            tx: tx_sender,
            rx: rx_receiver,
        },
    )
}
