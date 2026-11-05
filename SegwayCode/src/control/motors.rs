// TB6612FNG dual H-bridge motor driver, ported from the DC_testing branch. Each channel has two
// direction inputs (IN1/IN2) and a PWM input whose duty cycle sets the speed. STBY low puts the
// whole chip in standby with both outputs off, so the motors coast.
//
// IN1 IN2 PWM -> output
//  H   L   H  -> forward (CW), PWM low phase short-brakes
//  L   H   H  -> reverse (CCW), PWM low phase short-brakes
//  H   H   x  -> short brake
//  L   L   x  -> stop (coast)
//
// Unlike DC_testing (integer percent, 100 steps) the duty is an f32 in -1..1 using all 1024
// PWM steps, which the controller needs for smooth small corrections.

use esp_idf_svc::hal::gpio::{Level, Output, OutputPin, PinDriver};
use esp_idf_svc::hal::ledc::config::{Resolution, TimerConfig};
use esp_idf_svc::hal::ledc::{LedcChannel, LedcDriver, LedcTimer, LedcTimerDriver, LowSpeed};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::sys::EspError;

use crate::config::PWM_FREQUENCY_HZ;

/// 80 MHz APB / 20 kHz leaves room for 10 bits (1024 duty steps).
const PWM_RESOLUTION: Resolution = Resolution::Bits10;

/// The shared PWM timer, create it first and pass it to both `Channel::new` calls.
pub fn pwm_timer<'d>(
    timer: impl LedcTimer<SpeedMode = LowSpeed> + 'd,
) -> Result<LedcTimerDriver<'d, LowSpeed>, EspError> {
    LedcTimerDriver::new(
        timer,
        &TimerConfig::new()
            .frequency(Hertz(PWM_FREQUENCY_HZ))
            .resolution(PWM_RESOLUTION),
    )
}

/// One H-bridge channel (A or B).
pub struct Channel<'d> {
    pwm: LedcDriver<'d>,
    in1: PinDriver<'d, Output>,
    in2: PinDriver<'d, Output>,
    max_duty: u32,
    invert: bool,
}

impl<'d> Channel<'d> {
    /// `invert` flips the direction, for a motor that is mounted mirrored or wired backwards.
    pub fn new<C: LedcChannel<SpeedMode = LowSpeed> + 'd>(
        channel: C,
        timer: &LedcTimerDriver<'d, LowSpeed>,
        pwm: impl OutputPin + 'd,
        in1: impl OutputPin + 'd,
        in2: impl OutputPin + 'd,
        invert: bool,
    ) -> Result<Self, EspError> {
        // Pin drivers start low and the PWM at duty 0, so the motor starts coasting
        let pwm = LedcDriver::new(channel, timer, pwm)?;
        Ok(Self {
            max_duty: pwm.get_max_duty(),
            pwm,
            in1: PinDriver::output(in1)?,
            in2: PinDriver::output(in2)?,
            invert,
        })
    }

    /// `duty` in -1..1: the sign picks the direction, the magnitude the PWM duty. 0 coasts.
    fn set(&mut self, duty: f32) -> Result<(), EspError> {
        let duty = if self.invert { -duty } else { duty };
        // NaN fails both comparisons, so a broken controller output coasts
        let (in1, in2, magnitude) = if duty > 0.0 {
            (Level::High, Level::Low, duty)
        } else if duty < 0.0 {
            (Level::Low, Level::High, -duty)
        } else {
            (Level::Low, Level::Low, 0.0)
        };
        let steps = (magnitude.min(1.0) * self.max_duty as f32 + 0.5) as u32;

        self.in1.set_level(in1)?;
        self.in2.set_level(in2)?;
        self.pwm.set_duty(steps.min(self.max_duty))
    }
}

pub struct Tb6612<'d> {
    stby: PinDriver<'d, Output>,
    left: Channel<'d>,
    right: Channel<'d>,
    // Both channels run off this timer, dropping it would stop the PWM
    _timer: LedcTimerDriver<'d, LowSpeed>,
}

impl<'d> Tb6612<'d> {
    /// Starts in standby with both motors stopped.
    pub fn new(
        timer: LedcTimerDriver<'d, LowSpeed>,
        stby: impl OutputPin + 'd,
        left: Channel<'d>,
        right: Channel<'d>,
    ) -> Result<Self, EspError> {
        let mut stby = PinDriver::output(stby)?;
        stby.set_low()?;
        Ok(Self {
            stby,
            left,
            right,
            _timer: timer,
        })
    }

    /// Drive both motors (duty -1..1, positive = robot forward) and leave standby.
    pub fn drive(&mut self, left: f32, right: f32) -> Result<(), EspError> {
        self.left.set(left)?;
        self.right.set(right)?;
        self.stby.set_high()
    }

    /// Standby with both motors coasting. Standby first, so the outputs are off even if one
    /// of the channel writes fails.
    pub fn stop(&mut self) -> Result<(), EspError> {
        self.stby.set_low()?;
        self.left.set(0.0)?;
        self.right.set(0.0)
    }
}
