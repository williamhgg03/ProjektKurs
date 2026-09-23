pub mod display;

use embassy_time::{Duration, Timer};

use crate::lcd::display::Lcd;

#[embassy_executor::task]
pub async fn display_task(mut lcd: Lcd) {
    lcd.init();

    loop {
        lcd.draw_test_pattern();

        Timer::after(Duration::from_secs(1)).await;
    }
}
