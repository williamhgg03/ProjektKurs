// The GC9A01 display behind an `embedded-graphics` DrawTarget.
//
// Drawing goes into a framebuffer in RAM and `flush` sends only the rectangle
// that changed to the display over SPI. The framebuffer is 240*240*2 = 115 KB,
// far too big for a stack, so it lives in a static.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use static_cell::StaticCell;

use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig, OutputPin, interconnect::PeripheralOutput},
    peripherals::SPI2,
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
};

use gc9a01::{Gc9a01, SPIDisplayInterface, mode::BasicMode, prelude::*};

use crate::controller::SCREEN;

const WIDTH: usize = SCREEN as usize;
const HEIGHT: usize = SCREEN as usize;
const FRAME_LEN: usize = WIDTH * HEIGHT;

static FRAME: StaticCell<[u16; FRAME_LEN]> = StaticCell::new();

type SpiBus = Spi<'static, esp_hal::Blocking>;

type SpiDevice = ExclusiveDevice<SpiBus, Output<'static>, NoDelay>;

type DisplayInterface = SPIInterface<SpiDevice, Output<'static>>;

type Display = Gc9a01<DisplayInterface, DisplayResolution240x240, BasicMode>;

/// Bounding box of the pixels drawn since the last flush, inclusive.
#[derive(Clone, Copy)]
struct Dirty {
    x0: u16,
    y0: u16,
    x1: u16,
    y1: u16,
}

impl Dirty {
    const NONE: Self = Self {
        x0: u16::MAX,
        y0: u16::MAX,
        x1: 0,
        y1: 0,
    };

    fn include(&mut self, x: u16, y: u16) {
        self.x0 = self.x0.min(x);
        self.y0 = self.y0.min(y);
        self.x1 = self.x1.max(x);
        self.y1 = self.y1.max(y);
    }

    fn is_empty(&self) -> bool {
        self.x0 > self.x1 || self.y0 > self.y1
    }
}

pub struct Lcd {
    display: Display,
    frame: &'static mut [u16; FRAME_LEN],
    dirty: Dirty,

    reset: Output<'static>,
    #[allow(dead_code, reason = "kept on for as long as the controller runs")]
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
        let mut backlight = Output::new(lcd_bl, Level::Low, OutputConfig::default());
        backlight.set_high();

        let reset = Output::new(lcd_rst, Level::High, OutputConfig::default());

        let dc = Output::new(lcd_dc, Level::High, OutputConfig::default());

        let cs = Output::new(lcd_cs, Level::High, OutputConfig::default());

        let spi = Spi::new(
            spi2,
            SpiConfig::default()
                .with_frequency(Rate::from_mhz(40))
                .with_mode(Mode::_0),
        )
        .expect("Failed to initialize LCD SPI")
        .with_sck(sclk)
        .with_mosi(mosi);

        let bus = ExclusiveDevice::new(spi, cs, NoDelay).expect("Failed to create LCD bus");

        let interface = SPIDisplayInterface::new(bus, dc);

        let display = Gc9a01::new(
            interface,
            DisplayResolution240x240,
            DisplayRotation::Rotate0,
        );

        let frame = {
            let slot = FRAME.uninit();
            // Zero the static in place. `init([0; FRAME_LEN])` would first
            // build the whole 115 KB array on the stack.
            // SAFETY: the slot is written in full before it is read, and
            // `uninit` panics on a second call so there is only one `&mut`.
            unsafe {
                slot.as_mut_ptr().write_bytes(0, 1);
                slot.assume_init_mut()
            }
        };

        Self {
            display,
            frame,
            dirty: Dirty::NONE,
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

    /// Send everything drawn since the last flush to the display.
    pub fn flush(&mut self) {
        let dirty = self.dirty;
        self.dirty = Dirty::NONE;
        if dirty.is_empty() {
            return;
        }

        self.display
            .set_draw_area((dirty.x0, dirty.y0), (dirty.x1, dirty.y1))
            .expect("Failed to set LCD draw area");
        self.display
            .set_write_mode()
            .expect("Failed to set LCD write mode");

        let (x0, x1) = (usize::from(dirty.x0), usize::from(dirty.x1));
        for y in usize::from(dirty.y0)..=usize::from(dirty.y1) {
            let row = y * WIDTH;
            self.display
                .draw_buffer(&self.frame[row + x0..=row + x1])
                .expect("Failed to write LCD row");
        }
    }
}

impl OriginDimensions for Lcd {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for Lcd {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            let (Ok(x), Ok(y)) = (u16::try_from(point.x), u16::try_from(point.y)) else {
                continue;
            };
            if usize::from(x) >= WIDTH || usize::from(y) >= HEIGHT {
                continue;
            }
            self.frame[usize::from(y) * WIDTH + usize::from(x)] = color.into_storage();
            self.dirty.include(x, y);
        }
        Ok(())
    }
}
