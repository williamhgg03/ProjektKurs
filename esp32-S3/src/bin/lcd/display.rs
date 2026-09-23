use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    peripherals::{SPI2, GPIO47, GPIO38, GPIO21, GPIO18, GPIO17, GPIO10},
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    time::Rate,
};

use gc9a01::{
    mode::BasicMode,
    prelude::*,
    Gc9a01,
    SPIDisplayInterface,
};

type SpiBus = Spi<'static, esp_hal::Blocking>;

type SpiDevice =
    ExclusiveDevice<SpiBus, Output<'static>, NoDelay>;

type DisplayInterface =
    SPIInterface<SpiDevice, Output<'static>>;

type Display =
    Gc9a01<
        DisplayInterface,
        DisplayResolution240x240,
        BasicMode,
    >;

pub struct Lcd {
    display: Display,

    reset: Output<'static>,
    backlight: Output<'static>,

    delay: Delay,
}

impl Lcd {
    pub fn new(
        spi2: SPI2<'static>,
        MOSI: GPIO47<'static>,
        SCLK: GPIO38<'static>,
        LCD_CS: GPIO21<'static>,
        LCD_DC: GPIO18<'static>,
        LCD_RST: GPIO17<'static>,
        LCD_BL: GPIO10<'static>,
    ) -> Self {

        let mut backlight = Output::new(
            LCD_BL,
            Level::Low,
            OutputConfig::default(),
        );
        backlight.set_high();

        let reset = Output::new(
            LCD_RST,
            Level::High,
            OutputConfig::default(),
        );

        let dc = Output::new(
            LCD_DC,
            Level::High,
            OutputConfig::default(),
        );

        let cs = Output::new(
            LCD_CS,
            Level::High,
            OutputConfig::default(),
        );

        let spi = Spi::new(
            spi2,
            SpiConfig::default()
                .with_frequency(Rate::from_mhz(40))
                .with_mode(Mode::_0),
        )
        .expect("Failed to initialize LCD SPI")
        .with_sck(SCLK)
        .with_mosi(MOSI);

        let bus = ExclusiveDevice::new(
            spi,
            cs,
            NoDelay,
        )
        .expect("Failed to create LCD bus");

        let interface = SPIDisplayInterface::new(
            bus,
            dc,
        );

        let display = Gc9a01::new(
            interface,
            DisplayResolution240x240,
            DisplayRotation::Rotate0,
        );

        Self {
            display,
            reset,
            backlight,
            delay: Delay::new(),
        }
    }

    pub fn init(&mut self) {
        self.display
            .reset(&mut self.reset, &mut self.delay)
            .expect("LCD reset failed");

        self.display
            .init_with_addr_mode(&mut self.delay)
            .expect("LCD initialization failed");
    }

    pub fn draw_test_pattern(&mut self) {
        self.display
            .set_draw_area((0, 0), (239, 239))
            .expect("Failed to set LCD draw area");

        self.display
            .set_write_mode()
            .expect("Failed to set LCD write mode");

        // RGB565 red
        let buffer = [0xF800u16; 32];

        for _ in 0..1800 {
            self.display
                .draw_buffer(&buffer)
                .expect("Failed to draw to LCD");
        }
    }
}
