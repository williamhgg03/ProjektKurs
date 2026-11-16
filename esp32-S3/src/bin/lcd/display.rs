use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

use esp_hal::{
    delay::Delay,
    gpio::{interconnect::PeripheralOutput, Level, Output, OutputConfig, OutputPin},
    peripherals::SPI2,
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
        mosi: impl PeripheralOutput<'static>,
        sclk: impl PeripheralOutput<'static>,
        lcd_cs: impl OutputPin + 'static,
        lcd_dc: impl OutputPin + 'static,
        lcd_rst: impl OutputPin + 'static,
        lcd_bl: impl OutputPin + 'static,
    ) -> Self {

        let mut backlight = Output::new(
            lcd_bl,
            Level::Low,
            OutputConfig::default(),
        );
        backlight.set_high();

        let reset = Output::new(
            lcd_rst,
            Level::High,
            OutputConfig::default(),
        );

        let dc = Output::new(
            lcd_dc,
            Level::High,
            OutputConfig::default(),
        );

        let cs = Output::new(
            lcd_cs,
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
        .with_sck(sclk)
        .with_mosi(mosi);

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

    pub fn fill_red(&mut self) {
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
                .expect("Failed to draw red screen");
        }
    }

    pub fn fill_blue(&mut self) {
        self.display
            .set_draw_area((0, 0), (239, 239))
            .expect("Failed to set LCD draw area");

        self.display
            .set_write_mode()
            .expect("Failed to set LCD write mode");

        // RGB565 blue
        let buffer = [0x001Fu16; 32];

        for _ in 0..1800 {
            self.display
                .draw_buffer(&buffer)
                .expect("Failed to draw blue screen");
        }
    }
}
