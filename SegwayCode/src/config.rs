// Every tuning constant in one place. Pins are not here, they are in the PIN BLOCK in main.rs.
//
// Units: the control code works in SI (radians, rad/s, seconds). Degrees only appear at the
// edges (server commands, telemetry, these constants) because they are easier to read.

use std::time::Duration;

// ---------------------------------------------------------------------------------------------
// Control loop timing
// ---------------------------------------------------------------------------------------------

/// The MPU-6050 data-ready interrupt clocks the loop: rate = 1 kHz / (1 + div) with the DLPF on.
/// 4 -> 200 Hz.
pub const MPU_SAMPLE_DIV: u8 = 4;
pub const LOOP_HZ: f32 = 1000.0 / (1 + MPU_SAMPLE_DIV as u32) as f32;
/// dt used for the very first sample, and the upper clamp after missed samples.
pub const NOMINAL_DT: f32 = 1.0 / LOOP_HZ;
pub const MAX_DT: f32 = 4.0 * NOMINAL_DT;

/// Data-ready fires every 5 ms, not seeing it for this long counts as an IMU error.
pub const INT_TIMEOUT: Duration = Duration::from_millis(20);
/// This many IMU errors in a row (timeouts or I2C failures) -> Fault, motors off.
pub const MAX_IMU_ERRORS: u32 = 20;

// ---------------------------------------------------------------------------------------------
// IMU (MPU-6050)
// ---------------------------------------------------------------------------------------------

/// MPU-6050 maximum I2C clock.
pub const I2C_BAUDRATE_HZ: u32 = 400_000;
/// DLPF_CFG register value. 3 = 44 Hz bandwidth, about 4.9 ms delay (what the IMU tests used).
/// 2 = 94 Hz / 3 ms is worth trying once it balances, less delay helps the D term.
pub const MPU_DLPF_CFG: u8 = 3;

/// How the board is mounted: which sensor axis points forward, left and up on the robot.
/// From these:  pitch (forward lean positive) = atan2(-a_forward, a_up)
///              pitch rate                    = gyro_left
///              yaw rate (turning left pos.)  = gyro_up
/// Verify by tilting the robot forward: pitch_deg in the logs/telemetry must go positive.
pub const IMU_FORWARD: Axis = Axis::PosX;
pub const IMU_LEFT: Axis = Axis::PosY;
pub const IMU_UP: Axis = Axis::PosZ;

/// Complementary filter time constant. Below this the accelerometer wins, above it the gyro.
pub const COMP_FILTER_TAU_S: f32 = 0.5;

/// Gyro bias calibration at startup: average this many samples (2 s) with the robot still...
pub const CAL_SAMPLES: u32 = 400;
/// ...and retry if any axis has a standard deviation above this (someone moved it).
pub const CAL_MAX_STD_DPS: f32 = 1.0;

// ---------------------------------------------------------------------------------------------
// Safety
// ---------------------------------------------------------------------------------------------

/// Start balancing only when enabled and within this angle of the target (lift it upright).
pub const ENGAGE_DEG: f32 = 5.0;
/// Past this the robot has fallen: motors off until it is lifted back into the engage window.
pub const FALL_DEG: f32 = 45.0;
/// No successful exchange with the server for this long -> lean/turn commands are zeroed.
/// The robot keeps balancing, cutting the motors would make it fall over.
pub const LINK_TIMEOUT_MS: u32 = 1000;
/// Clamp on commanded lean (motion) and trim from the server.
pub const MAX_LEAN_DEG: f32 = 10.0;
pub const MAX_TRIM_DEG: f32 = 10.0;
/// Clamp on the commanded turn, as a fraction of full duty added/subtracted per wheel.
pub const MAX_TURN: f32 = 0.3;

// ---------------------------------------------------------------------------------------------
// Controller (PID). Gains come from the server, see shared::Gains.
// ---------------------------------------------------------------------------------------------

/// Limit on the integral term's contribution to the duty (anti-windup).
pub const PID_I_LIMIT: f32 = 0.5;

// ---------------------------------------------------------------------------------------------
// Motors (TB6612FNG). Left motor = channel A, right motor = channel B.
// ---------------------------------------------------------------------------------------------

/// 20 kHz is above hearing range; 80 MHz / 20 kHz leaves room for 10 bits (1024 steps).
pub const PWM_FREQUENCY_HZ: u32 = 20_000;
/// Positive duty must drive the robot forward. Flip these instead of rewiring.
pub const MOTOR_LEFT_INVERT: bool = false;
pub const MOTOR_RIGHT_INVERT: bool = false;
/// Smallest duty that makes the wheels turn (static friction). Any nonzero command is mapped
/// into [deadband, 1]. Measure it with the wheels in the air, 0.0 disables the compensation.
pub const MOTOR_DEADBAND: f32 = 0.0;

// ---------------------------------------------------------------------------------------------
// Threads
// ---------------------------------------------------------------------------------------------

/// Above everything else that could end up on core 1 (FreeRTOS max is 24).
pub const CONTROL_PRIORITY: u8 = 20;
pub const CONTROL_STACK: usize = 8192;
pub const NET_TX_PRIORITY: u8 = 5;
pub const NET_TX_STACK: usize = 8192;

// ---------------------------------------------------------------------------------------------
// Networking / telemetry
// ---------------------------------------------------------------------------------------------

/// Every Nth control tick is sent as telemetry: 200 Hz / 4 = 50 Hz.
pub const TELEMETRY_DECIMATION: u32 = 4;
/// 5 s of telemetry at 50 Hz, covers an upload that takes a while.
pub const TELEMETRY_CHANNEL_CAP: usize = 256;
pub const UPLOAD_INTERVAL: Duration = Duration::from_millis(250);
/// Samples per POST, and how many unsent samples to keep while the server is unreachable.
pub const MAX_BATCH: usize = 100;
pub const MAX_PENDING: usize = 500;
/// How often the networking core logs the control loop's status summary.
pub const STATUS_LOG_INTERVAL: Duration = Duration::from_secs(1);

/// The server holds a long-poll open for up to 10 s, so this must be longer.
pub const LONG_POLL_TIMEOUT: Duration = Duration::from_secs(15);
pub const UPLOAD_TIMEOUT: Duration = Duration::from_secs(5);
pub const RETRY_DELAY: Duration = Duration::from_secs(2);

// ---------------------------------------------------------------------------------------------

/// A sensor axis with a sign, see IMU_FORWARD.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // only the variants picked above are used, the rest are for other mountings
pub enum Axis {
    PosX,
    PosY,
    PosZ,
    NegX,
    NegY,
    NegZ,
}

impl Axis {
    /// The component of a sensor-frame vector along this axis.
    pub fn pick(self, v: [f32; 3]) -> f32 {
        match self {
            Axis::PosX => v[0],
            Axis::PosY => v[1],
            Axis::PosZ => v[2],
            Axis::NegX => -v[0],
            Axis::NegY => -v[1],
            Axis::NegZ => -v[2],
        }
    }
}
