// Everything the two cores share, and the only way they talk to each other.
//
//   core 0 (networking)                         core 1 (control)
//   setpoints::run  --SHARED.commands------->   reads the latest Command every tick
//   any HTTP success --SHARED.link---------->   decides if the commands are still fresh
//   telemetry::run  <--SHARED.status---------   once-per-second summary (for logging on core 0)
//   telemetry::run  <--mpsc::sync_channel----   Telemetry samples (created in main.rs)
//
// Rule: nothing here may block the control loop. It only uses try_lock / atomics / try_send.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Mutex, PoisonError, TryLockError};

use serde::{Deserialize, Deserializer, Serialize};

use crate::rtos;

pub static SHARED: Shared = Shared {
    commands: Mailbox::new(Command::OFF),
    status: Mailbox::new(Status::BOOT),
    link: Link::new(),
};

pub struct Shared {
    /// Written by core 0 when the server sends a new command, read by core 1 every tick.
    pub commands: Mailbox<Command>,
    /// Written by core 1 about once per second and on mode changes, read by core 0.
    pub status: Mailbox<Status>,
    /// Core 0 marks every successful exchange with the server.
    pub link: Link,
}

// ---------------------------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------------------------

/// PID gains. They act on radians: kp is duty per rad, ki duty per rad*s, kd duty per rad/s.
/// (1 degree = 0.01745 rad, so kp = 6 means 10% duty at about 1 degree of error.)
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Gains {
    #[serde(deserialize_with = "f32_via_f64")]
    pub kp: f32,
    #[serde(deserialize_with = "f32_via_f64")]
    pub ki: f32,
    #[serde(deserialize_with = "f32_via_f64")]
    pub kd: f32,
}

/// What the server wants the robot to do. Body of `GET /device/segway`.
/// Missing fields default to zero/false, so the server can grow this one field at a time.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Command {
    /// Increases on every change on the server, used for the long-poll.
    pub version: u64,
    /// Motors may run. Turning this off always stops them (the robot will fall).
    pub enabled: bool,
    /// Balance point offset (center of mass not above the axle, IMU not level). Kept when the
    /// link to the server is lost.
    #[serde(deserialize_with = "f32_via_f64")]
    pub trim_deg: f32,
    /// Lean on top of the trim to drive forward (+) or backward (-). Zeroed when the link is lost.
    #[serde(deserialize_with = "f32_via_f64")]
    pub lean_deg: f32,
    /// Differential duty, positive turns left. Zeroed when the link is lost.
    #[serde(deserialize_with = "f32_via_f64")]
    pub turn: f32,
    pub gains: Gains,
}

impl Command {
    pub const OFF: Self = Self {
        version: 0,
        enabled: false,
        trim_deg: 0.0,
        lean_deg: 0.0,
        turn: 0.0,
        gains: Gains {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
        },
    };
}

/// The control loop's safety state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    /// Measuring the gyro bias at startup, keep the robot still.
    Calibrating,
    /// `enabled` is off, motors in standby.
    Disabled,
    /// Enabled, waiting to be lifted to within ENGAGE_DEG of upright.
    Armed,
    Balancing,
    /// Tipped past FALL_DEG, motors off. Lift it upright again to re-engage.
    Fallen,
    /// IMU stopped answering or a motor write failed. Latched until `enabled` is turned off.
    Fault,
}

/// One control tick, sent to the server for plotting and offline analysis.
/// Body of `POST /device/telemetry` is `{"samples": [Telemetry, ...]}`.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Telemetry {
    /// Microseconds since boot, when the IMU sample was read.
    pub t_us: i64,
    pub mode: Mode,
    pub pitch_deg: f32,
    pub pitch_rate_dps: f32,
    pub yaw_rate_dps: f32,
    /// trim + lean that the controller was aiming for.
    pub target_deg: f32,
    /// Duty actually sent to the motors, -1..1.
    pub u_left: f32,
    pub u_right: f32,
    /// Raw sensor-frame readings (g and deg/s, gyro bias not removed), for EKF work later.
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
    /// Time the control code took this tick.
    pub loop_us: u16,
    /// Measured control loop rate (= IMU sample rate) since the previous telemetry sample.
    pub rate_hz: f32,
    /// Telemetry samples dropped just before this one because the uploader was behind.
    pub dropped: u16,
    /// Whether the commands were fresh (see LINK_TIMEOUT_MS).
    pub link_ok: bool,
}

/// Control loop health, logged by core 0 so the control thread never waits on the console.
#[derive(Debug, Clone, Copy)]
pub struct Status {
    pub mode: Mode,
    pub pitch_deg: f32,
    /// Shortest and longest time between IMU samples during the last period.
    pub dt_min_us: u32,
    pub dt_max_us: u32,
    /// Longest time the control code took during the last period.
    pub loop_max_us: u32,
    /// Totals since boot.
    pub imu_errors: u32,
    pub telemetry_dropped: u32,
    pub link_ok: bool,
}

impl Status {
    pub const BOOT: Self = Self {
        mode: Mode::Calibrating,
        pitch_deg: 0.0,
        dt_min_us: 0,
        dt_max_us: 0,
        loop_max_us: 0,
        imu_errors: 0,
        telemetry_dropped: 0,
        link_ok: false,
    };
}

/// serde_json's own f32 parsing crashes the Xtensa LLVM backend of the esp toolchain
/// (rustc 1.97: "Cannot select: XtensaISD::PCREL_WRAPPER"), so floats are parsed as f64.
fn f32_via_f64<'de, D: Deserializer<'de>>(deserializer: D) -> Result<f32, D::Error> {
    f64::deserialize(deserializer).map(|v| v as f32)
}

// ---------------------------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------------------------

/// Holds the latest value of `T`, newer writes replace older ones.
///
/// The lock is held only for the copy, a few microseconds. The real-time side uses the `try_`
/// methods: if the other core holds the lock right now it keeps its previous value (reader) or
/// skips this update (writer) instead of waiting.
pub struct Mailbox<T: Copy> {
    slot: Mutex<T>,
}

impl<T: Copy> Mailbox<T> {
    pub const fn new(value: T) -> Self {
        Self {
            slot: Mutex::new(value),
        }
    }

    /// Blocking write, for the networking side.
    pub fn publish(&self, value: T) {
        *self.slot.lock().unwrap_or_else(PoisonError::into_inner) = value;
    }

    /// Blocking read, for the networking side.
    pub fn read(&self) -> T {
        *self.slot.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Non-blocking write. Returns false if the slot was busy and the value was not stored.
    pub fn try_publish(&self, value: T) -> bool {
        match self.slot.try_lock() {
            Ok(mut slot) => {
                *slot = value;
                true
            }
            Err(TryLockError::Poisoned(p)) => {
                *p.into_inner() = value;
                true
            }
            Err(TryLockError::WouldBlock) => false,
        }
    }

    /// Non-blocking read. None if the slot is busy right now.
    pub fn try_read(&self) -> Option<T> {
        match self.slot.try_lock() {
            Ok(slot) => Some(*slot),
            Err(TryLockError::Poisoned(p)) => Some(*p.into_inner()),
            Err(TryLockError::WouldBlock) => None,
        }
    }
}

/// Heartbeat from the networking side: when it last heard from the server.
pub struct Link {
    /// `rtos::now_ms()` of the last success, 0 = never.
    alive_ms: AtomicU32,
}

impl Link {
    pub const fn new() -> Self {
        Self {
            alive_ms: AtomicU32::new(0),
        }
    }

    pub fn mark_alive(&self) {
        // 0 means never, so a success in the very first millisecond is stored as 1
        self.alive_ms
            .store(rtos::now_ms().max(1), Ordering::Relaxed);
    }

    pub fn is_alive(&self, timeout_ms: u32) -> bool {
        let last = self.alive_ms.load(Ordering::Relaxed);
        last != 0 && rtos::now_ms().wrapping_sub(last) < timeout_ms
    }
}
