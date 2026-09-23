use esp_hal::clock::CpuClock;
use esp_hal::peripherals::Peripherals;

pub fn init() -> Peripherals {
    let config = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::max());

    esp_hal::init(config)
}
