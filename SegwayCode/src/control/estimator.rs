// State estimation: IMU samples in, robot pitch / pitch rate / yaw rate out.
//
// `Estimator` is the seam for the planned EKF: implement the trait and swap it in control/mod.rs.
// The EKF will want the motor duty for its prediction step, which is why `update` receives it.

use std::f32::consts::{PI, TAU};

use crate::config::{COMP_FILTER_TAU_S, IMU_FORWARD, IMU_LEFT, IMU_UP, PITCH_OFFSET_DEG};

/// Bring an angle back into -pi..pi after adding an offset.
fn wrap_angle(a: f32) -> f32 {
    (a + PI).rem_euclid(TAU) - PI
}

use super::imu::ImuSample;
use super::{MotorOutput, State};

pub trait Estimator {
    /// One control tick. `applied` is the duty sent to the motors during the previous tick,
    /// `dt` the time since the previous sample in seconds.
    fn update(&mut self, imu: &ImuSample, applied: MotorOutput, dt: f32) -> State;
}

/// Gyro integration corrected by the accelerometer's gravity direction.
///
/// The gyro is accurate short term but drifts, the accelerometer is right on average but
/// disturbed by every bump and wheel acceleration. Blending them with time constant TAU takes
/// the gyro's view for fast changes and the accelerometer's for slow ones.
pub struct ComplementaryFilter {
    /// deg/s, measured at startup with the robot still.
    gyro_bias: [f32; 3],
    /// rad, None until the first sample.
    pitch: Option<f32>,
}

impl ComplementaryFilter {
    pub fn new(gyro_bias_dps: [f32; 3]) -> Self {
        Self {
            gyro_bias: gyro_bias_dps,
            pitch: None,
        }
    }
}

impl Estimator for ComplementaryFilter {
    fn update(&mut self, imu: &ImuSample, _applied: MotorOutput, dt: f32) -> State {
        let gyro: [f32; 3] =
            core::array::from_fn(|i| (imu.gyro[i] - self.gyro_bias[i]).to_radians());
        let pitch_rate = IMU_LEFT.pick(gyro);
        let yaw_rate = IMU_UP.pick(gyro);

        // Leaning forward tips the sensor's forward axis down, so gravity's reaction shows up
        // as negative forward acceleration
        let raw = (-IMU_FORWARD.pick(imu.accel)).atan2(IMU_UP.pick(imu.accel));
        let accel_pitch = wrap_angle(raw + PITCH_OFFSET_DEG.to_radians());

        let pitch = match self.pitch {
            None => accel_pitch,
            Some(prev) => {
                let alpha = COMP_FILTER_TAU_S / (COMP_FILTER_TAU_S + dt);
                alpha * (prev + pitch_rate * dt) + (1.0 - alpha) * accel_pitch
            }
        };
        self.pitch = Some(pitch);

        State {
            pitch,
            pitch_rate,
            yaw_rate,
        }
    }
}

/// Running mean and spread of gyro readings, for the startup bias calibration.
#[derive(Default)]
pub struct GyroCalibration {
    count: u32,
    sum: [f32; 3],
    sum_sq: [f32; 3],
}

impl GyroCalibration {
    pub fn add(&mut self, gyro_dps: [f32; 3]) {
        self.count += 1;
        for i in 0..3 {
            self.sum[i] += gyro_dps[i];
            self.sum_sq[i] += gyro_dps[i] * gyro_dps[i];
        }
    }

    pub fn count(&self) -> u32 {
        self.count
    }

    pub fn mean(&self) -> [f32; 3] {
        let n = self.count.max(1) as f32;
        self.sum.map(|s| s / n)
    }

    /// Largest standard deviation of the three axes, deg/s. High means the robot moved.
    pub fn max_std(&self) -> f32 {
        let n = self.count.max(1) as f32;
        let mean = self.mean();
        (0..3)
            .map(|i| (self.sum_sq[i] / n - mean[i] * mean[i]).max(0.0).sqrt())
            .fold(0.0, f32::max)
    }
}
