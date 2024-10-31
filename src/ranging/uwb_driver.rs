use core::{
    cell::{OnceCell, RefCell},
    mem::MaybeUninit,
    task::Waker,
};

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
    waitqueue::WakerRegistration,
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
#[derive(Debug, Clone)]
pub struct UwbPacketTxRequest<const MTU: usize> {
    tx_time: TxTiming,
    len: usize,
    buf: [u8; MTU],
    waker: Option<Waker>,
}

impl<const MTU: usize> UwbPacketTxRequest<MTU> {
    pub const fn new() -> Self {
        Self {
            tx_time: TxTiming::Now,
            len: 0,
            buf: [0; MTU],
            waker: None,
        }
    }
}

/// Request to receive a packet
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct UwbPacketRxRequest {
    rx_time: RxTiming,
}

#[derive(Debug, Clone, defmt::Format, Default)]
pub enum UwbRequest<const MTU: usize> {
    #[default]
    None,
    Tx(UwbPacketTxRequest<MTU>),
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
            tx: core::array::from_fn(|_| UwbRequest::<MTU>::default()),
            rx: core::array::from_fn(|_| UwbPacketRxBuf::<MTU>::default()),
            inner: MaybeUninit::uninit(),
        }
    }
}

pub struct Shared {
    waker: WakerRegistration,
}

pub struct StateInner<'d, const MTU: usize> {
    tx: zerocopy_channel::Channel<'d, NoopRawMutex, UwbRequest<MTU>>,
    rx: zerocopy_channel::Channel<'d, NoopRawMutex, UwbPacketRxBuf<MTU>>,
    shared: Mutex<CriticalSectionRawMutex, RefCell<Shared>>,
}

pub struct UwbRunner<'d, const MTU: usize, SPI: embedded_hal_async::spi::SpiDevice> {
    tx: zerocopy_channel::Receiver<'d, NoopRawMutex, UwbRequest<MTU>>,
    rx: zerocopy_channel::Sender<'d, NoopRawMutex, UwbPacketRxBuf<MTU>>,

    /// SPI device for the DW3000
    spi: &'d mut SPI,
    /// RST GPIO for the DW3000
    rst: Output<'d>,
    /// IRQ GPIO for the DW3000
    irq: Input<'d>,
}

impl<'d, const MTU: usize, SPI: embedded_hal_async::spi::SpiDevice> UwbRunner<'d, MTU, SPI>
where
    <SPI as embedded_hal_async::spi::ErrorType>::Error: defmt::Format,
{
    pub async fn run(&mut self) {
        let dwm_config = dw3000_ng::Config {
            bitrate: dw3000_ng::configs::BitRate::Kbps6800,
            sts_len: StsLen::StsLen128,
            sts_mode: StsMode::StsMode1,
            pdoa_mode: dw3000_ng::configs::PdoaMode::Mode3,
            ..Default::default()
        };

        // Reset
        self.rst.set_low();

        Timer::after(Duration::from_millis(10)).await;

        self.rst.set_high();

        defmt::info!("DW3000 Reset!");

        Timer::after(Duration::from_millis(200)).await;

        let dw3000 = dw3000_ng::DW3000::new(&mut self.spi).init().await;
        if let Err(e) = dw3000 {
            defmt::error!("DW3000 failed init: {}", e);
            core::future::pending::<()>().await;
            unreachable!()
        }

        let mut dw3000 = dw3000
            .unwrap()
            .config(dwm_config, embassy_time::Delay)
            // .config(dwm_config, |d: u32| embassy_time::Delay.delay_us(d))
            .await
            .expect("Failed config.");

        dw3000.gpio_config(ConfigGPIOs::enable_led()).await.unwrap();
        dw3000
            .ll()
            .led_ctrl()
            .modify(|_, w| w.blink_tim(0x2))
            .await
            .unwrap();

        // Enable Super Deterministic Code (SDC)
        dw3000
            .ll()
            .sys_cfg()
            .modify(|_, w| w.cp_sdc(0x1))
            .await
            .unwrap();

        Timer::after(Duration::from_millis(200)).await;

        // Disable SPIRDY interrupt
        dw3000.disable_interrupts().await.unwrap();
        dw3000.enable_tx_interrupts().await.unwrap();
        dw3000.enable_rx_interrupts().await.unwrap();

        // Read DW3000 Device ID
        let dev_id = dw3000.ll().dev_id().read().await.unwrap();

        if dev_id.model() != 0x03 {
            defmt::error!("Invalid DW3000 model: {:#x}", dev_id.model());
            panic!();
        }

        dw3000
            .ll()
            .rx_fwto()
            .write(|w| w.value(1000000))
            .await
            .unwrap();
        dw3000
            .ll()
            .sys_cfg()
            .modify(|_, w| w.rxwtoe(1))
            .await
            .unwrap();
        dw3000
            .ll()
            .sys_status()
            .modify(|_, w| w.rxfto(1))
            .await
            .unwrap();

        loop {
            // First get the current request
            let request = self.tx.receive().await;

            match request {
                UwbRequest::Tx(tx_request) => {
                    defmt::debug!("Tx request");

                    let tx_time = tx_request.tx_time;
                    let len = tx_request.len;
                    let buf = tx_request.buf;

                    let tx_time_dw = match tx_time {
                        TxTiming::Now => dw3000_ng::hl::SendTime::Now,
                        TxTiming::Delayed(t) => dw3000_ng::hl::SendTime::Delayed(t),
                    };

                    let mut sending = dw3000
                        .send_raw(&buf[..len], tx_time_dw, &dwm_config)
                        .await
                        .unwrap();

                    let result = nonblocking_wait_async(
                        async || -> Result<(), nb::Error<_>> {
                            let status = sending.s_wait().await;

                            if status.is_err() {
                                return status.map(|_| ());
                            }

                            Ok(())
                        },
                        &mut self.irq,
                    )
                    .await;

                    dw3000 = sending.finish_sending().await.unwrap();

                    if let Some(waker) = tx_request.waker.take() {
                        waker.wake();
                    }

                    self.tx.receive_done();
                }
                UwbRequest::Rx(rx_request) => {
                    defmt::debug!("Rx request");
                }
                UwbRequest::None => {
                    defmt::error!("Invalid request!");
                }
            }
        }
    }
}

pub struct Device<'d, const MTU: usize> {
    tx: zerocopy_channel::Sender<'d, NoopRawMutex, UwbRequest<MTU>>,
    rx: zerocopy_channel::Receiver<'d, NoopRawMutex, UwbPacketRxBuf<MTU>>,
    shared: &'d Mutex<CriticalSectionRawMutex, RefCell<Shared>>,
}

impl<'d, const MTU: usize> Device<'d, MTU> {
    /// Borrow the device
    pub fn borrow(&mut self) -> Device<'_, MTU> {
        Device {
            tx: self.tx.borrow(),
            rx: self.rx.borrow(),
            shared: self.shared,
        }
    }
}

pub fn new<
    'd,
    const MTU: usize,
    const N_RX: usize,
    const N_TX: usize,
    SPI: embedded_hal_async::spi::SpiDevice,
>(
    state: &'d mut State<MTU, N_RX, N_TX>,
    spi: &'d mut SPI,
    rst: Output<'d>,
    irq: Input<'d>,
) -> (UwbRunner<'d, MTU, SPI>, Device<'d, MTU>) {
    let state_uninit: *mut MaybeUninit<StateInner<'d, MTU>> =
        (&mut state.inner as *mut MaybeUninit<StateInner<'static, MTU>>).cast();

    // SAFETY: this is a self-referential struct, however:
    // - it can't move while the `'d` borrow is active.
    // - when the borrow ends, the dangling references inside the MaybeUninit will never be used again.
    let state = unsafe { &mut *state_uninit }.write(StateInner {
        tx: zerocopy_channel::Channel::new(&mut state.tx[..]),
        rx: zerocopy_channel::Channel::new(&mut state.rx[..]),
        shared: Mutex::new(RefCell::new(Shared {
            waker: WakerRegistration::new(),
        })),
    });

    let (tx_sender, tx_receiver) = state.tx.split();
    let (rx_sender, rx_receiver) = state.rx.split();

    (
        UwbRunner {
            tx: tx_receiver,
            rx: rx_sender,
            spi,
            rst,
            irq,
        },
        Device {
            tx: tx_sender,
            rx: rx_receiver,
            shared: &state.shared,
        },
    )
}
