use core::future::Future;

use embassy_time::Instant;

#[inline]
pub async fn time_operation<T>(f: impl Future<Output = T>) -> T {
    let start = Instant::now();
    let r = f.await;
    defmt::info!(
        "Operation took: {:?}us",
        (Instant::now() - start).as_micros()
    );
    r
}
