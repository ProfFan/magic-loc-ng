use core::{cell::RefCell, mem::MaybeUninit};

use alloc::sync::Arc;
use dw3000_ng::{
    self,
    configs::{StsLen, StsMode},
    hl::ConfigGPIOs,
};
use embassy_futures::select::{select, Either};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, once_lock::OnceLock,
    waitqueue::WakerRegistration, zerocopy_channel,
};
use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Input, Output};

use crate::utils::nonblocking_wait_async;

/// 127 bytes according to the 802.15.4 Standard
pub const MTU_802154: usize = 127;

/// Send timing for TX
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum TxTiming {
    /// Send as fast as possible
    Now,
    /// Send after some time
    #[allow(dead_code)]
    Delayed(dw3000_ng::time::Instant),
}

/// Rx timing for RX
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum RxTiming {
    /// Receive as fast as possible
    Now,
    /// Receive after some time (at the given DW3000 timestamp)
    #[allow(dead_code)]
    Delayed(dw3000_ng::time::Instant),
}

#[derive(Clone, defmt::Format)]
pub struct TxResult {
    pub success: bool,
}

/// Represents a packet of size MTU that is ready to be sent.
#[derive(Clone)]
pub struct UwbPacketTxRequest<const MTU: usize> {
    pub tx_time: TxTiming,
    pub len: usize,
    pub buf: [MaybeUninit<u8>; MTU],
    pub tx_result: Option<Arc<OnceLock<TxResult>>>,
}

impl<const MTU: usize> UwbPacketTxRequest<MTU> {
    pub const fn new() -> Self {
        Self {
            tx_time: TxTiming::Now,
            len: 0,
            buf: [const { MaybeUninit::uninit() }; MTU],
            tx_result: None,
        }
    }
}

/// Request to receive a packet
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct UwbPacketRxRequest {
    rx_time: RxTiming,
}

#[derive(Clone, defmt::Format, Default)]
pub enum UwbRequest<const MTU: usize> {
    #[default]
    None,
    Tx(UwbPacketTxRequest<MTU>),
    Rx(UwbPacketRxRequest),
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum RxError {
    Spi,
    Phy,
    Timeout,
}

/// Metadata for a received packet
#[derive(Debug, Clone, Copy, defmt::Format, Default)]
pub struct RxMetadata {
    /// Whether the RX was successful
    pub success: bool,
    /// RSSI of the received packet
    pub rssi: i8,
    /// Error
    pub error: Option<RxError>,
}

/// Rx packet buffer
#[derive(Clone, Copy, defmt::Format)]
pub struct UwbPacketRxBuf<const MTU: usize> {
    pub rx_meta: RxMetadata,
    /// Time at which the packet was received
    pub rx_time: dw3000_ng::time::Instant,
    /// Length of the packet
    pub len: usize,
    /// Buffer containing the packet
    pub buf: [MaybeUninit<u8>; MTU],
}

impl<const MTU: usize> UwbPacketRxBuf<MTU> {
    pub fn as_slice(&self) -> &[u8] {
        unsafe { core::mem::transmute::<&[MaybeUninit<u8>], &[u8]>(&self.buf[..self.len]) }
    }
}

impl<const MTU: usize> core::fmt::Debug for UwbPacketRxBuf<MTU> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("UwbPacketRxBuf")
            .field("rx_meta", &self.rx_meta)
            .field("rx_time", &self.rx_time)
            .field("len", &self.len)
            .field("buf", &unsafe {
                core::mem::transmute::<&[MaybeUninit<u8>], &[u8]>(&self.buf[..self.len])
            })
            .finish()
    }
}

impl<const MTU: usize> Default for UwbPacketRxBuf<MTU> {
    fn default() -> Self {
        Self {
            rx_meta: RxMetadata::default(),
            rx_time: dw3000_ng::time::Instant::new(0).unwrap(),
            len: 0,
            buf: [const { MaybeUninit::uninit() }; MTU],
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
            inner: const { MaybeUninit::uninit() },
        }
    }
}

pub struct Shared {
    #[allow(dead_code)]
    waker: WakerRegistration,
}

pub struct StateInner<'d, const MTU: usize> {
    ioctl: zerocopy_channel::Channel<'d, CriticalSectionRawMutex, UwbRequest<MTU>>,
    rx: zerocopy_channel::Channel<'d, CriticalSectionRawMutex, UwbPacketRxBuf<MTU>>,
    shared: Mutex<CriticalSectionRawMutex, RefCell<Shared>>,
}

pub struct UwbRunner<'d, const MTU: usize, SPI: embedded_hal_async::spi::SpiDevice> {
    ioctl: zerocopy_channel::Receiver<'d, CriticalSectionRawMutex, UwbRequest<MTU>>,
    rx: zerocopy_channel::Sender<'d, CriticalSectionRawMutex, UwbPacketRxBuf<MTU>>,

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
            let request = self.ioctl.receive().await;

            match request {
                UwbRequest::Tx(tx_request) => {
                    defmt::debug!("Tx request");

                    let tx_time = tx_request.tx_time;
                    let len = tx_request.len;
                    if len > tx_request.buf.len() {
                        defmt::error!("Invalid TX length: {}", len);
                        if let Some(tx_result) = tx_request.tx_result.as_ref() {
                            tx_result.init(TxResult { success: false }).unwrap_or(());
                        }
                        self.ioctl.receive_done(); // Mark the request as done
                        continue;
                    }

                    let buf = unsafe {
                        core::mem::transmute::<&[MaybeUninit<u8>], &[u8]>(&tx_request.buf[..len])
                    };

                    let tx_time_dw = match tx_time {
                        TxTiming::Now => dw3000_ng::hl::SendTime::Now,
                        TxTiming::Delayed(t) => dw3000_ng::hl::SendTime::Delayed(t),
                    };

                    let mut sending = dw3000.send_raw(buf, tx_time_dw, &dwm_config).await.unwrap();

                    let _result = nonblocking_wait_async(
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

                    if let Some(tx_result) = tx_request.tx_result.as_ref() {
                        tx_result.init(TxResult { success: true }).unwrap_or(());
                    }

                    self.ioctl.receive_done();
                }
                UwbRequest::Rx(rx_request) => {
                    defmt::debug!("Rx request");

                    let rx_start_time = rx_request.rx_time;
                    let rx_start_time_dw = match rx_start_time {
                        RxTiming::Now => dw3000_ng::hl::ReceiveTime::Now,
                        RxTiming::Delayed(t) => dw3000_ng::hl::ReceiveTime::Delayed(t),
                    };

                    let mut receiving = dw3000
                        .receive_delayed(rx_start_time_dw, dwm_config)
                        .await
                        .unwrap();

                    // Mark the receive request as done as soon as we start receiving
                    self.ioctl.receive_done();

                    let buffer = self.rx.send().await;
                    let result = select(
                        nonblocking_wait_async(
                            async || -> Result<_, nb::Error<_>> {
                                receiving
                                    // SAFETY: `r_wait_buf` will only write to the buffer
                                    .r_wait_buf(unsafe {
                                        core::mem::transmute::<&mut [MaybeUninit<u8>], &mut [u8]>(
                                            &mut buffer.buf,
                                        )
                                    })
                                    .await
                            },
                            &mut self.irq,
                        ),
                        async {
                            let ioctl_next = self.ioctl.receive().await;

                            match ioctl_next {
                                UwbRequest::None => return 0u8,
                                UwbRequest::Rx(_) => return 1u8,
                                UwbRequest::Tx(_) => return 2u8,
                            }
                        },
                    )
                    .await;

                    let result = match result {
                        Either::First(result) => result,
                        Either::Second(_) => {
                            let ioctl_next = self.ioctl.receive().await;

                            match ioctl_next {
                                UwbRequest::None => {
                                    // Cancel the receive
                                    dw3000 = receiving.finish_receiving().await.unwrap();
                                    buffer.rx_meta.success = false;
                                    self.ioctl.receive_done();
                                }
                                UwbRequest::Rx(_) => {
                                    // Rx request while receiving
                                    dw3000 = receiving.finish_receiving().await.unwrap();
                                    buffer.rx_meta.success = false;
                                    self.ioctl.receive_done();
                                }
                                UwbRequest::Tx(tx_request) => {
                                    // Tx request while receiving
                                    dw3000 = receiving.finish_receiving().await.unwrap();
                                    buffer.rx_meta.success = false;
                                    if let Some(tx_result) = tx_request.tx_result.as_ref() {
                                        tx_result.init(TxResult { success: false }).unwrap_or(());
                                    }
                                    self.ioctl.receive_done();
                                }
                            }

                            continue;
                        }
                    };

                    match result {
                        Ok((len, rx_time, _rx_quality)) => {
                            buffer.len = len;
                            buffer.rx_time = rx_time;
                            buffer.rx_meta.success = true;
                            buffer.rx_meta.error = None;
                            self.rx.send_done();
                        }
                        Err(e) => {
                            defmt::debug!("Failed to receive: {}", e);
                            dw3000 = receiving.finish_receiving().await.unwrap();
                            buffer.rx_meta.success = false;

                            match e {
                                dw3000_ng::Error::Spi(_) => {
                                    buffer.rx_meta.error = Some(RxError::Spi)
                                }
                                dw3000_ng::Error::FrameWaitTimeout => {
                                    buffer.rx_meta.error = Some(RxError::Timeout)
                                }
                                _ => buffer.rx_meta.error = Some(RxError::Phy),
                            }

                            self.rx.send_done();
                            continue;
                        }
                    }

                    dw3000 = receiving.finish_receiving().await.unwrap();
                }
                UwbRequest::None => {
                    defmt::error!("Invalid request!");
                }
            }
        }
    }
}

pub struct Device<'d, const MTU: usize> {
    pub tx: zerocopy_channel::Sender<'d, CriticalSectionRawMutex, UwbRequest<MTU>>,
    pub rx: zerocopy_channel::Receiver<'d, CriticalSectionRawMutex, UwbPacketRxBuf<MTU>>,

    #[allow(dead_code)]
    pub shared: &'d Mutex<CriticalSectionRawMutex, RefCell<Shared>>,
}

impl<'d, const MTU: usize> core::fmt::Debug for Device<'d, MTU> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Device").finish()
    }
}

impl<'d, const MTU: usize> Device<'d, MTU> {
    /// Send a RX request
    pub async fn send_rx_request(&mut self, rx_time: RxTiming) {
        let buffer = self.tx.send().await;
        *buffer = UwbRequest::Rx(UwbPacketRxRequest { rx_time });
        self.tx.send_done();
    }

    /// Send a TX request
    pub async fn send_tx_request<F: FnOnce(&mut UwbPacketTxRequest<MTU>)>(&mut self, f: F) {
        let buffer = self.tx.send().await;
        *buffer = UwbRequest::Tx(UwbPacketTxRequest::<MTU>::new());
        let UwbRequest::Tx(ref mut tx_request) = buffer else {
            unreachable!()
        };

        f(tx_request);

        self.tx.send_done();
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
        ioctl: zerocopy_channel::Channel::new(&mut state.tx[..]),
        rx: zerocopy_channel::Channel::new(&mut state.rx[..]),
        shared: Mutex::new(RefCell::new(Shared {
            waker: WakerRegistration::new(),
        })),
    });

    let (tx_sender, tx_receiver) = state.ioctl.split();
    let (rx_sender, rx_receiver) = state.rx.split();

    (
        UwbRunner {
            ioctl: tx_receiver,
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
