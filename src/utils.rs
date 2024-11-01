use core::{future::Future, ops::AsyncFnMut};

use embassy_time::Instant;

#[inline]
#[allow(dead_code)]
pub async fn time_operation<T>(f: impl Future<Output = T>) -> T {
    let start = Instant::now();
    let r = f.await;
    defmt::trace!(
        "Operation took: {:?}us",
        (Instant::now() - start).as_micros()
    );
    r
}

/// Wait on a function that returns `nb::Result` asynchronously that can be cancelled.
///
/// When `f` returns `nb::Error::WouldBlock`, this function will wait for
/// the GPIO output to go high, and then call `f` again.
#[inline]
#[allow(dead_code)]
pub async fn nonblocking_wait<T, E>(
    mut f: impl FnMut() -> Result<T, nb::Error<E>>,
    int_gpio: &mut impl embedded_hal_async::digital::Wait,
) -> Result<T, E> {
    loop {
        let v = f();

        match v {
            Ok(t) => return Ok(t),
            Err(nb::Error::Other(e)) => return Err(e),
            Err(nb::Error::WouldBlock) => {
                defmt::trace!("Waiting for high...");
                int_gpio.wait_for_high().await.unwrap();
                continue;
            }
        }
    }
}

/// Async version of `nonblocking_wait`
pub async fn nonblocking_wait_async<T, E>(
    mut fut: impl AsyncFnMut() -> Result<T, nb::Error<E>>,
    int_gpio: &mut impl embedded_hal_async::digital::Wait,
) -> Result<T, E> {
    loop {
        let v = fut().await;

        match v {
            Ok(t) => return Ok(t),
            Err(nb::Error::Other(e)) => return Err(e),
            Err(nb::Error::WouldBlock) => {
                defmt::trace!("Waiting for high...");
                int_gpio.wait_for_high().await.unwrap();
                continue;
            }
        }
    }
}
