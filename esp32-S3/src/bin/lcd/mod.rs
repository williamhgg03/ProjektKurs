pub mod display;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

pub static TOUCH_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

#[embassy_executor::task]
pub async fn display_task(mut lcd: display::Lcd) {
    lcd.init();

    lcd.fill_red();

    loop {
        let touching = TOUCH_SIGNAL.wait().await;

        if touching {
            lcd.fill_blue();
        } else {
            lcd.fill_red();
        }
    }
}
