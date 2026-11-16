use defmt::info;

use esp_hal::{
    delay::Delay,
    gpio::{InputPin, Level, Output, OutputConfig, OutputPin},
    i2c::master::{Config as I2cConfig, I2c},
    peripherals::I2C0,
    time::Rate,
};

use cst816s::{Cst816s};

use crate::lcd::TOUCH_SIGNAL;

pub struct Touch {
    driver: Cst816s<
        I2c<'static, esp_hal::Blocking>,
        Delay,
    >,
    reset: Output<'static>,
}

impl Touch {
    pub fn new(
        i2c0: I2C0<'static>,
        sda: impl InputPin + OutputPin + 'static,
        scl: impl InputPin + OutputPin + 'static,
        reset_pin: impl OutputPin + 'static,
    ) -> Self {
        let reset = Output::new(
            reset_pin,
            Level::High,
            OutputConfig::default(),
        );

        let i2c = I2c::new(
            i2c0,
            I2cConfig::default()
                .with_frequency(Rate::from_khz(400)),
        )
        .expect("Failed to initialize touch I2C")
        .with_sda(sda)
        .with_scl(scl);

        let delay = Delay::new();

        let driver = Cst816s::new(i2c, delay);

        Self {
            driver,
            reset,
        }
    }

    pub fn init(&mut self) {
        self.driver
            .reset(&mut self.reset, &mut Delay::new())
            .expect("Failed to reset CST816S");

        info!("CST816S initialized!");
    }

    pub fn read(&mut self) {
        match self.driver.read_event() {
            Ok(event) => {
                info!(
                    "Touch: x={} y={}",
                    event.x,
                    event.y
                );

                TOUCH_SIGNAL.signal(true);
            }

            Err(_) => {
                TOUCH_SIGNAL.signal(false);
                info!("CST816S I2C read failed");
            }
        }
    }
}
