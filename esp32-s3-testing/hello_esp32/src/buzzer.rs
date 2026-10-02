// Passive piezo buzzer driven by the LEDC PWM peripheral. A tone is a 50% duty
// square wave at the note's frequency, silence is 0% duty. The hardware keeps
// the wave going, so the CPU only acts on note changes. The melody itself is
// made by the server, see server/src/melody.rs.

use esp_idf_svc::hal::gpio::OutputPin;
use esp_idf_svc::hal::ledc::config::TimerConfig;
use esp_idf_svc::hal::ledc::{
    LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, LowSpeed, Resolution,
};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::sys::EspError;

/// 10 bits keeps the LEDC clock divider valid from ~76 Hz to ~78 kHz.
const RESOLUTION: Resolution = Resolution::Bits10;

/// Same range the server folds its notes into.
const MIN_FREQ: u32 = 100;
const MAX_FREQ: u32 = 4000;

/// One tone (or rest when `freq == 0`). Same shape as `Note` in the server's melody.rs.
#[derive(Debug, Clone, Copy, serde::Deserialize)]
pub struct Note {
    pub freq: u16,
    pub ms: u32,
}

pub struct Buzzer<'d> {
    timer: LedcTimerDriver<'d, LowSpeed>,
    channel: LedcDriver<'d>,
}

impl<'d> Buzzer<'d> {
    pub fn new<C, T>(channel: C, timer: T, pin: impl OutputPin + 'd) -> Result<Self, EspError>
    where
        C: LedcChannel<SpeedMode = LowSpeed> + 'd,
        T: LedcTimer<SpeedMode = LowSpeed> + 'd,
    {
        let timer = LedcTimerDriver::new(
            timer,
            &TimerConfig::new().frequency(Hertz(1000)).resolution(RESOLUTION),
        )?;
        // The channel only reads the timer's settings here, so the timer stays
        // free for `set_frequency` later
        let mut channel = LedcDriver::new(channel, &timer, pin)?;
        channel.set_duty(0)?;

        Ok(Self { timer, channel })
    }

    /// Start (or switch to) a square wave at `hz`, clamped to the supported range.
    pub fn tone(&mut self, hz: u32) -> Result<(), EspError> {
        self.timer.set_frequency(Hertz(hz.clamp(MIN_FREQ, MAX_FREQ)))?;
        self.channel.set_duty(self.channel.get_max_duty() / 2)
    }

    pub fn silence(&mut self) -> Result<(), EspError> {
        self.channel.set_duty(0)
    }
}
