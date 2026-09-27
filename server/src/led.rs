// LED strip state held by the server. The chip only ever sees `output_frame()`:
// exactly `num_leds` colours with brightness already applied, so the firmware
// can feed it straight into `LedStrip::write`.

use std::fmt;

use serde::{Deserialize, Serialize};

/// Upper bound on strip length, keeps request sizes and chip RAM use sane.
pub const MAX_LEDS: usize = 1024;

/// Same shape as `Rgb` in the firmware's led_driver.rs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    pub const OFF: Rgb = Rgb { r: 0, g: 0, b: 0 };

    fn scaled(self, brightness: u8) -> Rgb {
        let s = |c: u8| (c as u16 * brightness as u16 / 255) as u8;
        Rgb {
            r: s(self.r),
            g: s(self.g),
            b: s(self.b),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct LedConfig {
    pub num_leds: usize,
    pub brightness: u8,
}

impl Default for LedConfig {
    fn default() -> Self {
        Self {
            num_leds: 40,
            brightness: 255,
        }
    }
}

/// Partial config update, omitted fields are left unchanged.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct ConfigUpdate {
    pub num_leds: Option<usize>,
    pub brightness: Option<u8>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum LedError {
    InvalidLength(usize),
    FrameTooLong { got: usize, num_leds: usize },
}

impl fmt::Display for LedError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LedError::InvalidLength(n) => {
                write!(f, "num_leds must be between 1 and {MAX_LEDS}, got {n}")
            }
            LedError::FrameTooLong { got, num_leds } => {
                write!(f, "frame has {got} LEDs but the strip is configured for {num_leds}")
            }
        }
    }
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct LedState {
    pub config: LedConfig,
    /// Raw frame as sent by the user, may be shorter than `num_leds`.
    pub frame: Vec<Rgb>,
    /// Bumped on every change so clients can detect updates.
    pub version: u64,
}

impl LedState {
    pub fn apply_config(&mut self, update: ConfigUpdate) -> Result<(), LedError> {
        if let Some(n) = update.num_leds {
            if n == 0 || n > MAX_LEDS {
                return Err(LedError::InvalidLength(n));
            }
            self.config.num_leds = n;
        }
        if let Some(b) = update.brightness {
            self.config.brightness = b;
        }
        self.version += 1;
        Ok(())
    }

    pub fn set_frame(&mut self, leds: Vec<Rgb>) -> Result<(), LedError> {
        if leds.len() > self.config.num_leds {
            return Err(LedError::FrameTooLong {
                got: leds.len(),
                num_leds: self.config.num_leds,
            });
        }
        self.frame = leds;
        self.version += 1;
        Ok(())
    }

    pub fn fill(&mut self, color: Rgb) {
        self.frame = vec![color; self.config.num_leds];
        self.version += 1;
    }

    /// Exactly `num_leds` colours, padded with black and brightness-scaled.
    pub fn output_frame(&self) -> Vec<Rgb> {
        let n = self.config.num_leds;
        self.frame
            .iter()
            .copied()
            .chain(std::iter::repeat(Rgb::OFF))
            .take(n)
            .map(|c| c.scaled(self.config.brightness))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RED: Rgb = Rgb { r: 255, g: 0, b: 0 };

    fn state(num_leds: usize, brightness: u8) -> LedState {
        LedState {
            config: LedConfig {
                num_leds,
                brightness,
            },
            ..Default::default()
        }
    }

    #[test]
    fn output_pads_with_black() {
        let mut s = state(3, 255);
        s.set_frame(vec![RED]).unwrap();
        assert_eq!(s.output_frame(), vec![RED, Rgb::OFF, Rgb::OFF]);
    }

    #[test]
    fn output_truncates_after_shrinking_strip() {
        let mut s = state(3, 255);
        s.fill(RED);
        s.apply_config(ConfigUpdate {
            num_leds: Some(2),
            ..Default::default()
        })
        .unwrap();
        assert_eq!(s.output_frame(), vec![RED, RED]);
    }

    #[test]
    fn output_applies_brightness() {
        let mut s = state(1, 128);
        s.set_frame(vec![Rgb { r: 255, g: 100, b: 0 }]).unwrap();
        assert_eq!(s.output_frame(), vec![Rgb { r: 128, g: 50, b: 0 }]);
    }

    #[test]
    fn rejects_oversize_frame() {
        let mut s = state(1, 255);
        assert_eq!(
            s.set_frame(vec![RED, RED]),
            Err(LedError::FrameTooLong { got: 2, num_leds: 1 })
        );
        assert_eq!(s.version, 0);
    }

    #[test]
    fn rejects_invalid_length() {
        let mut s = state(1, 255);
        for n in [0, MAX_LEDS + 1] {
            let update = ConfigUpdate {
                num_leds: Some(n),
                ..Default::default()
            };
            assert_eq!(s.apply_config(update), Err(LedError::InvalidLength(n)));
        }
    }
}
