pub mod touch;

use embassy_time::{Duration, Timer};

use crate::touch::touch::Touch;

#[embassy_executor::task]
pub async fn touch_task(mut touch: Touch) {
    touch.init();

    loop {
        touch.read();

        Timer::after(Duration::from_millis(20)).await;
    }
}
