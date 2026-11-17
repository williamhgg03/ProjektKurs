// CST816S touch controller, read directly over I2C.
//
// The cst816s crate's `read_event` is not used: it reads the registers starting
// at 0x01 into buffer[0] but then indexes the buffer by register number, so
// every field is shifted by one byte. That turned the press/lift flags into 
// phantom presses and produced X/Y values far outside the
// screen (presses landing on the right/bottom buttons).

use defmt::info;

use esp_hal::{
    delay::Delay,
    gpio::{InputPin, Level, Output, OutputConfig, OutputPin},
    i2c::master::{Config as I2cConfig, I2c},
    peripherals::I2C0,
    time::Rate,
};

use crate::controller::SCREEN;

const ADDRESS: u8 = 0x15;

/// First touch-data register (gesture ID). Reading 6 bytes from here gives:
/// [0] gesture, [1] finger count, [2] X high, [3] X low, [4] Y high, [5] Y low.
const REG_TOUCH_DATA: u8 = 0x01;

/// Event flags in the top two bits of the X-high register.
const EVENT_LIFT_UP: u8 = 0b01;

pub struct Touch {
    i2c: I2c<'static, esp_hal::Blocking>,
    reset: Output<'static>,
}

impl Touch {
    pub fn new(
        i2c0: I2C0<'static>,
        sda: impl InputPin + OutputPin + 'static,
        scl: impl InputPin + OutputPin + 'static,
        reset_pin: impl OutputPin + 'static,
    ) -> Self {
        let reset = Output::new(reset_pin, Level::High, OutputConfig::default());

        let i2c = I2c::new(
            i2c0,
            I2cConfig::default().with_frequency(Rate::from_khz(400)),
        )
        .expect("Failed to initialize touch I2C")
        .with_sda(sda)
        .with_scl(scl);

        Self { i2c, reset }
    }

    /// Hardware reset: hold TP_RST low, then give the chip time to boot.
    pub fn init(&mut self) {
        let delay = Delay::new();
        self.reset.set_low();
        delay.delay_millis(20);
        self.reset.set_high();
        delay.delay_millis(50);

        info!("CST816S initialized!");
    }

    /// Where the finger is, `None` when nothing touches the screen. The chip
    /// goes to sleep when untouched and then doesn't answer on I2C, so a failed
    /// read also means no touch.
    pub fn read(&mut self) -> Option<(u16, u16)> {
        let mut data = [0u8; 6];
        self.i2c
            .write_read(ADDRESS, &[REG_TOUCH_DATA], &mut data)
            .ok()?;

        let fingers = data[1] & 0x0f;
        let event = data[2] >> 6;
        if fingers == 0 || event == EVENT_LIFT_UP {
            return None;
        }

        let x = (u16::from(data[2] & 0x0f) << 8) | u16::from(data[3]);
        let y = (u16::from(data[4] & 0x0f) << 8) | u16::from(data[5]);

        // Anything off the panel is a bad read, not a touch
        let max = SCREEN as u16;
        if x >= max || y >= max {
            return None;
        }

        // The touch panel is mounted rotated 180 degrees relative to the
        // display, so both axes are mirrored to match the drawn buttons.
        Some((max - 1 - x, max - 1 - y))
    }
}
