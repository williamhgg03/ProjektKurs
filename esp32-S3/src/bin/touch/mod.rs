pub mod touch;

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::controller::{self, BUTTON_SIGNAL, DRIVE_SIGNAL};
use crate::touch::touch::Touch;

/// How often the touch chip is read.
const POLL: Duration = Duration::from_millis(20);

#[embassy_executor::task]
pub async fn touch_task(mut touch: Touch) {
    touch.init();

    let mut held = None;
    loop {
        let now = touch.read().and_then(|(x, y)| controller::hit_test(x, y));

        if now != held {
            held = now;
            info!("Button: {:?}", held);
            BUTTON_SIGNAL.signal(held);
            DRIVE_SIGNAL.signal(held);
        }

        Timer::after(POLL).await;
    }
}
