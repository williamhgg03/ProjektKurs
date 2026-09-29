use esp_hal::timer::timg::TimerGroup;
use esp_hal::peripherals::TIMG0;

pub fn init( timer: TIMG0 ) -> TimerGroup<'static, TIMG0> {
    TimerGroup::new(timer)
}
