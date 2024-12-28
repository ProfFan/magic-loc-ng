//! Asynchronous shared SPI bus with multiple MISO/MOSI pins

use core::marker::PhantomData;

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::Operation;
use embedded_hal_async::spi;

use embassy_embedded_hal::shared_bus::SpiDeviceError;
use embassy_embedded_hal::SetConfig;
use esp_hal::gpio::interconnect::PeripheralInput;
use esp_hal::peripheral::{Peripheral, PeripheralRef};

/// SPI device on a shared bus with multiple MISO/MOSI pins
///
/// Only support ESP32 devices.
pub struct MultipinSpiDevice<'a, M: RawMutex, BUS, CS, P: 'static> {
    bus: &'a Mutex<M, BUS>,
    cs: CS,
    signal: esp_hal::gpio::InputSignal,
    miso_pin: PeripheralRef<'static, P>,
    phantom: PhantomData<P>,
}

impl<'a, M: RawMutex, BUS, CS, P: 'static> MultipinSpiDevice<'a, M, BUS, CS, P> {
    /// Create a new `SpiDevice`.
    pub fn new(
        bus: &'a Mutex<M, BUS>,
        cs: CS,
        signal: esp_hal::gpio::InputSignal,
        miso_pin: impl Peripheral<P = P> + 'static,
    ) -> Self {
        Self {
            bus,
            cs,
            signal,
            miso_pin: miso_pin.into_ref(),
            phantom: PhantomData,
        }
    }
}

impl<'a, M: RawMutex, BUS, CS, P> spi::ErrorType for MultipinSpiDevice<'a, M, BUS, CS, P>
where
    BUS: spi::ErrorType,
    CS: OutputPin,
{
    type Error = SpiDeviceError<BUS::Error, CS::Error>;
}

impl<M, BUS, CS, P, Word> spi::SpiDevice<Word> for MultipinSpiDevice<'_, M, BUS, CS, P>
where
    M: RawMutex,
    BUS: spi::SpiBus<Word>,
    CS: OutputPin,
    P: PeripheralInput + esp_hal::peripheral::Peripheral<P = P>,
    <P as esp_hal::peripheral::Peripheral>::P: esp_hal::gpio::InputPin,
    Word: Copy + 'static,
{
    async fn transaction(
        &mut self,
        operations: &mut [spi::Operation<'_, Word>],
    ) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;
        self.cs.set_low().map_err(SpiDeviceError::Cs)?;
        self.signal.connect_to(self.miso_pin.reborrow());

        let op_res = 'ops: {
            for op in operations {
                let res = match op {
                    Operation::Read(buf) => bus.read(buf).await,
                    Operation::Write(buf) => bus.write(buf).await,
                    Operation::Transfer(read, write) => bus.transfer(read, write).await,
                    Operation::TransferInPlace(buf) => bus.transfer_in_place(buf).await,
                    Operation::DelayNs(ns) => match bus.flush().await {
                        Err(e) => Err(e),
                        Ok(()) => {
                            embassy_time::Timer::after_nanos(*ns as _).await;
                            Ok(())
                        }
                    },
                };
                if let Err(e) = res {
                    break 'ops Err(e);
                }
            }
            Ok(())
        };

        // On failure, it's important to still flush and deassert CS.
        let flush_res = bus.flush().await;
        let cs_res = self.cs.set_high();

        let op_res = op_res.map_err(SpiDeviceError::Spi)?;
        flush_res.map_err(SpiDeviceError::Spi)?;
        cs_res.map_err(SpiDeviceError::Cs)?;

        Ok(op_res)
    }
}

/// SPI device on a shared bus, with its own configuration.
///
/// This is like [`SpiDevice`], with an additional bus configuration that's applied
/// to the bus before each use using [`SetConfig`]. This allows different
/// devices on the same bus to use different communication settings.
pub struct SpiDeviceWithConfig<'a, M: RawMutex, BUS: SetConfig, CS> {
    bus: &'a Mutex<M, BUS>,
    cs: CS,
    config: BUS::Config,
}

impl<'a, M: RawMutex, BUS: SetConfig, CS> SpiDeviceWithConfig<'a, M, BUS, CS> {
    /// Create a new `SpiDeviceWithConfig`.
    pub fn new(bus: &'a Mutex<M, BUS>, cs: CS, config: BUS::Config) -> Self {
        Self { bus, cs, config }
    }

    /// Change the device's config at runtime
    pub fn set_config(&mut self, config: BUS::Config) {
        self.config = config;
    }
}

impl<'a, M, BUS, CS> spi::ErrorType for SpiDeviceWithConfig<'a, M, BUS, CS>
where
    BUS: spi::ErrorType + SetConfig,
    CS: OutputPin,
    M: RawMutex,
{
    type Error = SpiDeviceError<BUS::Error, CS::Error>;
}

impl<M, BUS, CS, Word> spi::SpiDevice<Word> for SpiDeviceWithConfig<'_, M, BUS, CS>
where
    M: RawMutex,
    BUS: spi::SpiBus<Word> + SetConfig,
    CS: OutputPin,
    Word: Copy + 'static,
{
    async fn transaction(
        &mut self,
        operations: &mut [spi::Operation<'_, Word>],
    ) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;
        bus.set_config(&self.config)
            .map_err(|_| SpiDeviceError::Config)?;
        self.cs.set_low().map_err(SpiDeviceError::Cs)?;

        let op_res = 'ops: {
            for op in operations {
                let res = match op {
                    Operation::Read(buf) => bus.read(buf).await,
                    Operation::Write(buf) => bus.write(buf).await,
                    Operation::Transfer(read, write) => bus.transfer(read, write).await,
                    Operation::TransferInPlace(buf) => bus.transfer_in_place(buf).await,
                    Operation::DelayNs(ns) => match bus.flush().await {
                        Err(e) => Err(e),
                        Ok(()) => {
                            embassy_time::Timer::after_nanos(*ns as _).await;
                            Ok(())
                        }
                    },
                };
                if let Err(e) = res {
                    break 'ops Err(e);
                }
            }
            Ok(())
        };

        // On failure, it's important to still flush and deassert CS.
        let flush_res = bus.flush().await;
        let cs_res = self.cs.set_high();

        let op_res = op_res.map_err(SpiDeviceError::Spi)?;
        flush_res.map_err(SpiDeviceError::Spi)?;
        cs_res.map_err(SpiDeviceError::Cs)?;

        Ok(op_res)
    }
}
