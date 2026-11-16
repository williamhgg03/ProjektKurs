pub mod display;
pub mod ui;

use crate::controller::BUTTON_SIGNAL;

#[embassy_executor::task]
pub async fn display_task(mut lcd: display::Lcd) {
    lcd.init();

    ui::draw_all(&mut lcd, None).expect("Failed to draw the buttons");
    lcd.flush();

    let mut pressed = None;
    loop {
        let next = BUTTON_SIGNAL.wait().await;
        if next == pressed {
            continue;
        }

        // Only the buttons that changed are redrawn, so the flush stays small
        if let Some(direction) = pressed {
            ui::draw_button(&mut lcd, direction, false).expect("Failed to draw a button");
        }
        if let Some(direction) = next {
            ui::draw_button(&mut lcd, direction, true).expect("Failed to draw a button");
        }
        lcd.flush();

        pressed = next;
    }
}
