use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::peripherals::SW_INTERRUPT;

pub fn init( sw: SW_INTERRUPT ) -> SoftwareInterruptControl {
    SoftwareInterruptControl::new(sw)
}
