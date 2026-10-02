#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println as _;

extern crate alloc;

mod system;
mod lcd;
mod touch;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32s3 -o unstable-hal -o alloc -o wifi -o ci -o neovim -o defmt -o esp-backtrace -o embassy -o esp

    let m_Peripherals = system::hardware::init();
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    let m_Timg0 = system::timer::init(m_Peripherals.TIMG0);
    let m_SwInterrupt = system::interrupt::init(m_Peripherals.SW_INTERRUPT);
    esp_rtos::start(m_Timg0.timer0, m_SwInterrupt.software_interrupt0);

    info!("Embassy initialized!");

    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(m_Peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let lcd = lcd::display::Lcd::new(
        m_Peripherals.SPI2,
        m_Peripherals.GPIO47,
        m_Peripherals.GPIO38,
        m_Peripherals.GPIO21,
        m_Peripherals.GPIO18,
        m_Peripherals.GPIO17,
        m_Peripherals.GPIO10,
    );

    let touch = touch::touch::Touch::new(
        m_Peripherals.I2C0,
        m_Peripherals.GPIO6,
        m_Peripherals.GPIO7,
        m_Peripherals.GPIO13,
    );

    // TODO: Spawn some tasks
    spawner
        .spawn(lcd::display_task(lcd)
        .expect("Failed to spawn LCD task")
    );

    spawner.spawn(
        touch::touch_task(touch)
            .expect("Failed to create touch task"),
    );

    info!("LCD task spawned");

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
