// WS2812B-compatible LED strip driver for ESP32-S3 using the RMT peripheral.
// The RMT hardware generates the waveform, so the bit timings are exact and
// unaffected by interrupts or CPU speed.
//
// Protocol: 24 bits per segment in GRB order, MSB first.
// T0H = 0.3us, T0L = 0.9us, T1H = 0.9us, T1L = 0.3us, reset = low >= 200us.

use core::time::Duration;

use esp_idf_svc::hal::delay::Ets;
use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::rmt::config::{MemoryAccess, TransmitConfig, TxChannelConfig};
use esp_idf_svc::hal::rmt::encoder::{BytesEncoder, BytesEncoderConfig};
use esp_idf_svc::hal::rmt::{PinState, Symbol, TxChannelDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::sys::EspError;

/// 10 MHz -> 1 tick = 0.1us, so all datasheet timings are exact tick counts.
const RESOLUTION: Hertz = Hertz(10_000_000);

const T0H: Duration = Duration::from_nanos(300);
const T0L: Duration = Duration::from_nanos(700);
const T1H: Duration = Duration::from_nanos(700);
const T1L: Duration = Duration::from_nanos(300);

/// Reset (latch) time in microseconds, line held low (>= 200us required).
const T_RESET_US: u32 = 200;

#[derive(Debug, Clone, Copy, Default, serde::Deserialize)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

pub struct LedStrip<'d> {
    tx: TxChannelDriver<'d>,
    encoder: BytesEncoder,
    buf: Vec<u8>,
}

impl<'d> LedStrip<'d> {
    pub fn new(pin: impl OutputPin + 'd) -> Result<Self, EspError> {
        let tx = TxChannelDriver::new(
            pin,
            &TxChannelConfig {
                resolution: RESOLUTION,
                // DMA so WiFi/other interrupts can't starve the RMT memory refill
                memory_access: MemoryAccess::Direct {
                    memory_block_symbols: 1024,
                },
                ..Default::default()
            },
        )?;

        let encoder = BytesEncoder::with_config(&BytesEncoderConfig {
            bit0: Symbol::new_with(RESOLUTION, PinState::High, T0H, PinState::Low, T0L)?,
            bit1: Symbol::new_with(RESOLUTION, PinState::High, T1H, PinState::Low, T1L)?,
            msb_first: true,
            ..Default::default()
        })?;

        // Make sure the first frame starts after a valid reset
        Ets::delay_us(T_RESET_US);

        Ok(Self {
            tx,
            encoder,
            buf: Vec::new(),
        })
    }

    /// Send one frame (one colour per segment) and latch it.
    pub fn write(&mut self, colors: &[Rgb]) -> Result<(), EspError> {
        self.buf.clear();
        for c in colors {
            self.buf.extend_from_slice(&[c.g, c.r, c.b]);
        }

        // Line idles low after the transmission (eot_level = false)
        self.tx
            .send_and_wait(&mut self.encoder, &self.buf, &TransmitConfig::default())?;

        Ets::delay_us(T_RESET_US);

        Ok(())
    }
}
