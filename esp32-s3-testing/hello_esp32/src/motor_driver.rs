// TB6612FNG dual H-bridge motor driver. Each channel has two direction inputs
// (IN1/IN2) and a PWM input whose duty cycle sets the speed. STBY low puts the
// whole chip in standby with both outputs off, so the motors coast.
//
// IN1 IN2 PWM -> output
//  H   L   H  -> forward (CW), PWM low phase short-brakes
//  L   H   H  -> reverse (CCW), PWM low phase short-brakes
//  H   H   x  -> short brake
//  L   L   x  -> stop (coast)
//
// The PWM input accepts up to 100 kHz, 20 kHz is above hearing range.

use esp_idf_svc::hal::gpio::{Output, OutputPin, PinDriver};
use esp_idf_svc::hal::ledc::config::{Resolution, TimerConfig};
use esp_idf_svc::hal::ledc::{LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, LowSpeed};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::sys::EspError;

const PWM_FREQUENCY: Hertz = Hertz(20_000);
/// 80 MHz APB / 20 kHz leaves room for 10 bits (1024 duty steps).
const PWM_RESOLUTION: Resolution = Resolution::Bits10;

/// Speeds are a signed percentage of full duty, negative runs the motor in reverse.
pub const MAX_SPEED: i8 = 100;

/// Same shape as `Motor` in the server's motor.rs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Deserialize)]
pub struct Motor {
    pub speed: i8,
    /// Short brake (IN1 = IN2 = high). Overrides `speed`.
    pub brake: bool,
}

/// One H-bridge channel (A or B).
pub struct Channel<'d> {
    pwm: LedcDriver<'d>,
    in1: PinDriver<'d, Output>,
    in2: PinDriver<'d, Output>,
}

impl<'d> Channel<'d> {
    pub fn new<C: LedcChannel<SpeedMode = LowSpeed> + 'd>(
        channel: C,
        timer: &LedcTimerDriver<'d, LowSpeed>,
        pwm: impl OutputPin + 'd,
        in1: impl OutputPin + 'd,
        in2: impl OutputPin + 'd,
    ) -> Result<Self, EspError> {
        // Pin drivers start low and the PWM at duty 0, so the motor starts coasting
        Ok(Self {
            pwm: LedcDriver::new(channel, timer, pwm)?,
            in1: PinDriver::output(in1)?,
            in2: PinDriver::output(in2)?,
        })
    }

    fn set(&mut self, motor: Motor) -> Result<(), EspError> {
        let (in1, in2) = match (motor.brake, motor.speed.signum()) {
            (true, _) => (true, true),
            (false, 1) => (true, false),
            (false, -1) => (false, true),
            _ => (false, false),
        };
        let percent = if motor.brake {
            0
        } else {
            motor.speed.unsigned_abs().min(MAX_SPEED as u8) as u32
        };

        self.in1.set_level(in1.into())?;
        self.in2.set_level(in2.into())?;
        self.pwm.set_duty(self.pwm.get_max_duty() * percent / MAX_SPEED as u32)
    }
}

pub struct Tb6612<'d> {
    stby: PinDriver<'d, Output>,
    a: Channel<'d>,
    b: Channel<'d>,
    // Both channels run off this timer, dropping it would stop the PWM
    _timer: LedcTimerDriver<'d, LowSpeed>,
}

/// The shared PWM timer, create it first and pass it to both `Channel::new` calls.
pub fn pwm_timer<'d>(
    timer: impl LedcTimer<SpeedMode = LowSpeed> + 'd,
) -> Result<LedcTimerDriver<'d, LowSpeed>, EspError> {
    LedcTimerDriver::new(
        timer,
        &TimerConfig::new()
            .frequency(PWM_FREQUENCY)
            .resolution(PWM_RESOLUTION),
    )
}

impl<'d> Tb6612<'d> {
    /// Starts in standby with both motors stopped.
    pub fn new(
        timer: LedcTimerDriver<'d, LowSpeed>,
        stby: impl OutputPin + 'd,
        a: Channel<'d>,
        b: Channel<'d>,
    ) -> Result<Self, EspError> {
        Ok(Self {
            stby: PinDriver::output(stby)?,
            a,
            b,
            _timer: timer,
        })
    }

    /// Set both channels, then take the chip out of standby if `enabled`.
    pub fn set(&mut self, enabled: bool, a: Motor, b: Motor) -> Result<(), EspError> {
        if !enabled {
            self.stby.set_low()?;
        }
        self.a.set(a)?;
        self.b.set(b)?;
        if enabled {
            self.stby.set_high()?;
        }
        Ok(())
    }

    /// Standby with both motors coasting. Used as the failsafe when the server
    /// can't be reached.
    pub fn stop(&mut self) -> Result<(), EspError> {
        self.set(false, Motor::default(), Motor::default())
    }
}
