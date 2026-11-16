use defmt::info;

use esp_hal::{
    delay::Delay,
    gpio::{InputPin, Level, Output, OutputConfig, OutputPin},
    i2c::master::{Config as I2cConfig, I2c},
    peripherals::I2C0,
    time::Rate,
};

use cst816s::Cst816s;

pub struct Touch {
    driver: Cst816s<I2c<'static, esp_hal::Blocking>, Delay>,
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

        let delay = Delay::new();

        let driver = Cst816s::new(i2c, delay);

        Self { driver, reset }
    }

    pub fn init(&mut self) {
        self.driver
            .reset(&mut self.reset, &mut Delay::new())
            .expect("Failed to reset CST816S");

        info!("CST816S initialized!");
    }

    /// Where the finger is, `None` when nothing touches the screen. The chip
    /// goes to sleep when untouched and then doesn't answer on I2C, so a failed
    /// read also means no touch.
    pub fn read(&mut self) -> Option<(u16, u16)> {
        match self.driver.read_event() {
            Ok(event) if event.finger_number > 0 => Some((event.x, event.y)),
            Ok(_) | Err(_) => None,
        }
    }
}
