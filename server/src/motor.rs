// Motor state held by the server for a TB6612FNG dual motor driver. The chip
// receives this state as-is and maps it straight onto the driver's pins:
// `enabled` -> STBY, `speed` sign -> IN1/IN2, `speed` magnitude -> PWM duty.

use std::fmt;

use serde::{Deserialize, Serialize};

/// Speeds are a signed percentage of full duty, negative runs the motor in reverse.
pub const MAX_SPEED: i8 = 100;

/// Same shape as `Motor` in the firmware's motor_driver.rs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Motor {
    pub speed: i8,
    /// Short brake (IN1 = IN2 = high). Overrides `speed`.
    pub brake: bool,
}

/// Partial motor update, omitted fields are left unchanged. Setting a speed
/// releases the brake and braking sets the speed to 0, so releasing the brake
/// never jumps back to an old speed.
#[derive(Debug, Clone, Copy, Default, Deserialize)]
pub struct MotorUpdate {
    pub speed: Option<i8>,
    pub brake: Option<bool>,
}

/// Partial state update, omitted fields are left unchanged.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct StateUpdate {
    pub enabled: Option<bool>,
    pub a: Option<MotorUpdate>,
    pub b: Option<MotorUpdate>,
}

#[derive(Debug, PartialEq, Eq)]
pub enum MotorError {
    SpeedOutOfRange(i8),
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MotorError::SpeedOutOfRange(s) => {
                write!(f, "speed must be between -{MAX_SPEED} and {MAX_SPEED}, got {s}")
            }
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct MotorState {
    /// Driver out of standby (STBY high). When false both motors coast.
    pub enabled: bool,
    pub a: Motor,
    pub b: Motor,
    /// Bumped on every change so clients can detect updates.
    pub version: u64,
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            enabled: true,
            a: Motor::default(),
            b: Motor::default(),
            version: 0,
        }
    }
}

impl Motor {
    fn apply(&mut self, update: MotorUpdate) {
        if let Some(s) = update.speed {
            self.speed = s;
            self.brake = false;
        }
        if let Some(b) = update.brake {
            self.brake = b;
            if b {
                self.speed = 0;
            }
        }
    }
}

impl MotorState {
    /// Validates the whole update first so a bad request changes nothing.
    pub fn apply(&mut self, update: StateUpdate) -> Result<(), MotorError> {
        for m in [update.a, update.b].into_iter().flatten() {
            if let Some(s) = m.speed
                && !(-MAX_SPEED..=MAX_SPEED).contains(&s)
            {
                return Err(MotorError::SpeedOutOfRange(s));
            }
        }
        if let Some(e) = update.enabled {
            self.enabled = e;
        }
        if let Some(a) = update.a {
            self.a.apply(a);
        }
        if let Some(b) = update.b {
            self.b.apply(b);
        }
        self.version += 1;
        Ok(())
    }

    /// Both motors to speed 0 and coasting, the driver stays enabled.
    pub fn stop(&mut self) {
        self.a = Motor::default();
        self.b = Motor::default();
        self.version += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn speed(s: i8) -> Option<MotorUpdate> {
        Some(MotorUpdate {
            speed: Some(s),
            brake: None,
        })
    }

    #[test]
    fn partial_update_leaves_other_motor() {
        let mut s = MotorState::default();
        s.apply(StateUpdate { a: speed(50), b: speed(-20), ..Default::default() }).unwrap();
        s.apply(StateUpdate { a: speed(10), ..Default::default() }).unwrap();
        assert_eq!(s.a.speed, 10);
        assert_eq!(s.b.speed, -20);
        assert_eq!(s.version, 2);
    }

    #[test]
    fn speed_releases_brake() {
        let mut s = MotorState::default();
        let brake = Some(MotorUpdate { speed: None, brake: Some(true) });
        s.apply(StateUpdate { a: speed(30), ..Default::default() }).unwrap();
        s.apply(StateUpdate { a: brake, ..Default::default() }).unwrap();
        assert_eq!(s.a, Motor { speed: 0, brake: true });
        s.apply(StateUpdate { a: speed(30), ..Default::default() }).unwrap();
        assert!(!s.a.brake);
    }

    #[test]
    fn rejects_out_of_range_without_changes() {
        let mut s = MotorState::default();
        let update = StateUpdate { a: speed(50), b: speed(101), ..Default::default() };
        assert_eq!(s.apply(update), Err(MotorError::SpeedOutOfRange(101)));
        assert_eq!(s.a.speed, 0);
        assert_eq!(s.version, 0);
    }

    #[test]
    fn stop_keeps_enabled() {
        let mut s = MotorState::default();
        s.apply(StateUpdate { enabled: Some(true), a: speed(80), b: speed(-80) }).unwrap();
        s.stop();
        assert_eq!((s.a, s.b), (Motor::default(), Motor::default()));
        assert!(s.enabled);
    }
}
