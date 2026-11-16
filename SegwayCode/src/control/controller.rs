// Balance control: estimated state in, common-mode motor duty out.
//
// `Controller` is the seam for the planned nonlinear controller: implement the trait and swap it
// in control/mod.rs. Steering (turn) is mixed on top in control/mod.rs, outside the controller.

use crate::config::PID_I_LIMIT;
use crate::shared::{Command, Gains};

use super::State;

pub trait Controller {
    /// A new command arrived from the server, pick up any parameters it carries.
    fn configure(&mut self, cmd: &Command);
    /// Duty for both wheels (-1..1, positive drives forward) to hold `target_pitch` (rad).
    fn update(&mut self, state: &State, target_pitch: f32, dt: f32) -> f32;
    /// Clear internal state. Called every time balancing starts.
    fn reset(&mut self);
}

/// Leaning forward (pitch > target) drives the wheels forward, back under the center of mass.
#[derive(Default)]
pub struct Pid {
    gains: Gains,
    /// The integral term's contribution to the duty (ki is already applied).
    integral: f32,
}

impl Controller for Pid {
    fn configure(&mut self, cmd: &Command) {
        if cmd.gains != self.gains {
            self.gains = cmd.gains;
            self.integral = 0.0;
        }
    }

    fn update(&mut self, state: &State, target_pitch: f32, dt: f32) -> f32 {
        let Gains { kp, ki, kd } = self.gains;
        let error = state.pitch - target_pitch;

        let p = kp * error;
        // Derivative on the measured rate from the gyro instead of differentiating the error:
        // no noise amplification and no kick when the target changes
        let d = kd * state.pitch_rate;

        // Anti-windup: stop integrating while the output is saturated in the error's direction
        let unsaturated = p + self.integral + d;
        if unsaturated.abs() < 1.0 || unsaturated.signum() != error.signum() {
            self.integral = (self.integral + ki * error * dt).clamp(-PID_I_LIMIT, PID_I_LIMIT);
        }

        (p + self.integral + d).clamp(-1.0, 1.0)
    }

    fn reset(&mut self) {
        self.integral = 0.0;
    }
}
