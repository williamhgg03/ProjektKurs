// The control core (core 1). Owns the IMU and the motors and runs the balance loop, clocked by
// the MPU-6050's data-ready interrupt (LOOP_HZ).
//
// Each tick:  wait for INT -> read IMU -> Estimator -> latest Command -> safety FSM
//             -> Controller -> mix steering -> TB6612 -> telemetry (every Nth tick)
//
// Real-time rules for everything called from `ControlLoop::tick`:
// - never block on the other core: only try_lock / atomics / try_send (see shared.rs)
// - no logging: a console line takes milliseconds; core 0 logs the Status summary instead
// - no heap allocation

mod controller;
mod estimator;
mod imu;
mod motors;

use std::num::NonZeroU32;
use std::sync::mpsc::{SyncSender, TrySendError};
use std::thread;
use std::time::Duration;

use esp_idf_svc::hal::cpu::{self, Core};
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::gpio::{
    AnyIOPin, AnyInputPin, AnyOutputPin, Input, InterruptType, PinDriver, Pull,
};
use esp_idf_svc::hal::i2c::{I2C0, I2cConfig, I2cDriver};
use esp_idf_svc::hal::ledc::{CHANNEL0, CHANNEL1, TIMER0};
use esp_idf_svc::hal::task::notification::Notification;
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::sys::{EspError, TickType_t};

use crate::config::*;
use crate::rtos;
use crate::shared::{Command, Mode, SHARED, Status, Telemetry};

use controller::{Controller, Pid};
use estimator::{ComplementaryFilter, Estimator, GyroCalibration};
use imu::{ImuSample, Mpu6050};
use motors::{Channel, Tb6612};

/// The peripherals the control core owns. Filled in by the PIN BLOCK in main.rs and moved into
/// the control thread, which creates the drivers itself so their interrupt handlers are
/// installed on core 1 (ESP-IDF allocates an interrupt on the core that installs it).
pub struct ControlHw {
    pub i2c: I2C0<'static>,
    pub imu_sda: AnyIOPin<'static>,
    pub imu_scl: AnyIOPin<'static>,
    /// MPU-6050 INT (data ready).
    pub imu_int: AnyInputPin<'static>,

    pub pwm_timer: TIMER0<'static>,
    pub left_pwm_channel: CHANNEL0<'static>,
    pub right_pwm_channel: CHANNEL1<'static>,
    /// TB6612 PWMA / AIN1 / AIN2 (left motor).
    pub left_pwm: AnyOutputPin<'static>,
    pub left_in1: AnyOutputPin<'static>,
    pub left_in2: AnyOutputPin<'static>,
    /// TB6612 PWMB / BIN1 / BIN2 (right motor).
    pub right_pwm: AnyOutputPin<'static>,
    pub right_in1: AnyOutputPin<'static>,
    pub right_in2: AnyOutputPin<'static>,
    pub motor_stby: AnyOutputPin<'static>,
}

/// Robot state estimate, SI units.
#[derive(Debug, Clone, Copy, Default)]
pub struct State {
    /// Lean from upright in rad, positive = leaning forward.
    pub pitch: f32,
    /// rad/s, positive = tipping forward.
    pub pitch_rate: f32,
    /// rad/s, positive = turning left.
    pub yaw_rate: f32,
}

/// Duty sent to the motors, -1..1, positive = forward.
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorOutput {
    pub left: f32,
    pub right: f32,
}

/// Start the control thread on core 1. Returns once it is spawned; it runs forever.
pub fn spawn(hw: ControlHw, telemetry: SyncSender<Telemetry>) -> anyhow::Result<()> {
    rtos::spawn_pinned(
        c"control",
        Core::Core1,
        CONTROL_PRIORITY,
        CONTROL_STACK,
        move || {
            log::info!("Control thread running on {:?}", cpu::core());
            // Only returns on a setup error. The motor driver has been dropped by then, which
            // releases STBY and the TB6612's internal pull-down puts it in standby
            if let Err(e) = run(hw, telemetry) {
                log::error!("Control thread stopped, motors off: {e:?}");
                SHARED.status.publish(Status {
                    mode: Mode::Fault,
                    ..SHARED.status.read()
                });
            }
        },
    )?;
    Ok(())
}

fn run(hw: ControlHw, telemetry: SyncSender<Telemetry>) -> anyhow::Result<()> {
    // Motors first, so they are in standby while everything else starts up
    let timer = motors::pwm_timer(hw.pwm_timer)?;
    let left = Channel::new(
        hw.left_pwm_channel,
        &timer,
        hw.left_pwm,
        hw.left_in1,
        hw.left_in2,
        MOTOR_LEFT_INVERT,
    )?;
    let right = Channel::new(
        hw.right_pwm_channel,
        &timer,
        hw.right_pwm,
        hw.right_in1,
        hw.right_in2,
        MOTOR_RIGHT_INVERT,
    )?;
    let motors = Tb6612::new(timer, hw.motor_stby, left, right)?;

    let i2c = I2cDriver::new(
        hw.i2c,
        hw.imu_sda,
        hw.imu_scl,
        &I2cConfig::new().baudrate(Hertz(I2C_BAUDRATE_HZ)),
    )?;
    let mut mpu = Mpu6050::new(i2c);
    let mut attempts = 0u32;
    while let Err(e) = mpu.init() {
        if attempts % 10 == 0 {
            log::error!("MPU-6050 not responding ({e}), check SDA/SCL/power. Retrying...");
        }
        attempts += 1;
        thread::sleep(Duration::from_millis(500));
    }
    log::info!("MPU-6050 ready, sampling at {LOOP_HZ} Hz");

    let mut sampler = Sampler::new(mpu, hw.imu_int)?;

    log::info!("Calibrating gyro, keep the robot still...");
    let bias = calibrate_gyro(&mut sampler);
    log::info!(
        "Gyro bias [{:.2}, {:.2}, {:.2}] deg/s",
        bias[0],
        bias[1],
        bias[2]
    );

    // Swap in the EKF / nonlinear controller here
    let mut control = ControlLoop::new(
        motors,
        ComplementaryFilter::new(bias),
        Pid::default(),
        telemetry,
    );
    loop {
        control.tick(sampler.next());
    }
}

/// Average the gyro while the robot is still. Retries until the readings are steady.
fn calibrate_gyro(sampler: &mut Sampler) -> [f32; 3] {
    let mut errors = 0u32;
    loop {
        let mut cal = GyroCalibration::default();
        while cal.count() < CAL_SAMPLES {
            match sampler.next() {
                Ok(sample) => cal.add(sample.gyro),
                Err(e) => {
                    if errors % 50 == 0 {
                        log::warn!("IMU error during gyro calibration: {e}");
                    }
                    errors += 1;
                }
            }
        }

        let std = cal.max_std();
        if std <= CAL_MAX_STD_DPS {
            return cal.mean();
        }
        log::warn!("Robot moved during gyro calibration (std {std:.2} deg/s), retrying");
    }
}

// ---------------------------------------------------------------------------------------------
// IMU sampling, clocked by the data-ready interrupt
// ---------------------------------------------------------------------------------------------

enum SampleError {
    /// No data-ready pulse within INT_TIMEOUT: INT wire loose, or the sensor reset.
    IntTimeout,
    Bus(EspError),
}

impl std::fmt::Display for SampleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SampleError::IntTimeout => write!(f, "no data-ready interrupt, check the INT wire"),
            SampleError::Bus(e) => write!(f, "I2C read failed: {e}"),
        }
    }
}

struct Sampler {
    mpu: Mpu6050<'static>,
    int: PinDriver<'static, Input>,
    notification: Notification,
    timeout: TickType_t,
}

impl Sampler {
    /// Must be called on the control thread: the notification wakes the thread that created
    /// it, and the GPIO interrupt service is installed on the core that subscribes first.
    fn new(mpu: Mpu6050<'static>, int_pin: AnyInputPin<'static>) -> Result<Self, EspError> {
        let notification = Notification::new();
        let notifier = notification.notifier();

        let mut int = PinDriver::input(int_pin, Pull::Down)?;
        int.set_interrupt_type(InterruptType::PosEdge)?;
        // SAFETY: notifying a task is ISR-safe, and Drop unsubscribes before the notified
        // task (this thread) can end.
        unsafe {
            int.subscribe(move || {
                notifier.notify_and_yield(NonZeroU32::MIN);
            })?;
        }

        Ok(Self {
            mpu,
            int,
            notification,
            timeout: TickType::from(INT_TIMEOUT).ticks(),
        })
    }

    /// Block until the next sample is ready and read it.
    fn next(&mut self) -> Result<ImuSample, SampleError> {
        // The driver disables the interrupt each time it fires
        self.int.enable_interrupt().map_err(SampleError::Bus)?;
        if self.notification.wait(self.timeout).is_none() {
            return Err(SampleError::IntTimeout);
        }
        self.mpu.read().map_err(SampleError::Bus)
    }
}

impl Drop for Sampler {
    fn drop(&mut self) {
        let _ = self.int.unsubscribe();
    }
}

// ---------------------------------------------------------------------------------------------
// The loop
// ---------------------------------------------------------------------------------------------

struct ControlLoop<E: Estimator, C: Controller> {
    motors: Tb6612<'static>,
    estimator: E,
    controller: C,
    telemetry: SyncSender<Telemetry>,

    mode: Mode,
    /// Latest command from core 0, kept when the mailbox is busy.
    cmd: Command,
    state: State,
    link_ok: bool,
    /// What the motors were last set to, fed to the estimator next tick.
    applied: MotorOutput,
    last_t_us: Option<i64>,
    imu_errors_in_row: u32,
    tick: u32,
    /// Telemetry samples dropped since the last one that got through.
    dropped: u16,
    /// Common motor duty, low-passed (AUTO_TRIM_TAU_S). Without encoders this is the best
    /// measure of wheel speed: at a steady speed the duty mostly balances the motors' back-EMF.
    duty_lp: f32,
    /// Learned balance-point offset (rad) on top of the server's trim.
    auto_trim: f32,
    /// IMU time of the previous telemetry sample, sent or dropped, for the measured loop rate.
    last_telemetry_t_us: Option<i64>,
    stats: Stats,
}

/// Collected between two Status publishes.
struct Stats {
    dt_min_us: u32,
    dt_max_us: u32,
    loop_max_us: u32,
    imu_errors: u32,
    telemetry_dropped: u32,
}

impl<E: Estimator, C: Controller> ControlLoop<E, C> {
    fn new(
        motors: Tb6612<'static>,
        estimator: E,
        controller: C,
        telemetry: SyncSender<Telemetry>,
    ) -> Self {
        let mut this = Self {
            motors,
            estimator,
            controller,
            telemetry,
            mode: Mode::Disabled,
            cmd: Command::OFF,
            state: State::default(),
            link_ok: false,
            applied: MotorOutput::default(),
            last_t_us: None,
            imu_errors_in_row: 0,
            tick: 0,
            dropped: 0,
            last_telemetry_t_us: None,
            duty_lp: 0.0,
            auto_trim: 0.0,
            stats: Stats {
                dt_min_us: u32::MAX,
                dt_max_us: 0,
                loop_max_us: 0,
                imu_errors: 0,
                telemetry_dropped: 0,
            },
        };
        this.publish_status();
        this
    }

    fn tick(&mut self, sample: Result<ImuSample, SampleError>) {
        let imu = match sample {
            Ok(sample) => {
                self.imu_errors_in_row = 0;
                sample
            }
            Err(_) => {
                // Skip this tick, the motors keep their last duty. Too many in a row and the
                // estimate can't be trusted any more.
                self.stats.imu_errors += 1;
                self.imu_errors_in_row += 1;
                if self.imu_errors_in_row >= MAX_IMU_ERRORS && self.mode != Mode::Fault {
                    self.set_mode(Mode::Fault);
                    self.stop_motors();
                }
                return;
            }
        };
        let start_us = rtos::now_us();

        let dt = match self.last_t_us {
            Some(last) => {
                let dt_us = (imu.t_us - last).max(0) as u32;
                self.stats.dt_min_us = self.stats.dt_min_us.min(dt_us);
                self.stats.dt_max_us = self.stats.dt_max_us.max(dt_us);
                (dt_us as f32 * 1e-6).min(MAX_DT)
            }
            None => NOMINAL_DT,
        };
        self.last_t_us = Some(imu.t_us);

        self.state = self.estimator.update(&imu, self.applied, dt);
        let state = self.state;

        // Latest command from core 0. If it is being written right now, keep last tick's
        if let Some(cmd) = SHARED.commands.try_read() {
            if cmd != self.cmd {
                if cmd.trim_deg != self.cmd.trim_deg {
                    // A new manual target (e.g. the learned one being saved): start over from it
                    self.auto_trim = 0.0;
                }
                self.controller.configure(&cmd);
                self.cmd = cmd;
            }
        }

        // Lost link: stop moving but keep balancing (cutting the motors drops the robot)
        self.link_ok = SHARED.link.is_alive(LINK_TIMEOUT_MS);
        let trim = self.cmd.trim_deg.clamp(-MAX_TRIM_DEG, MAX_TRIM_DEG);
        let (lean, turn) = if self.link_ok {
            (
                self.cmd.lean_deg.clamp(-MAX_LEAN_DEG, MAX_LEAN_DEG),
                self.cmd.turn.clamp(-MAX_TURN, MAX_TURN),
            )
        } else {
            (0.0, 0.0)
        };
        let target = (trim + lean).to_radians() + self.auto_trim;

        let next = self.next_mode(&state, target);
        if next != self.mode {
            if next == Mode::Balancing {
                self.controller.reset();
                self.duty_lp = 0.0;
            }
            self.set_mode(next);
        }

        if self.mode == Mode::Balancing {
            let u = self.controller.update(&state, target, dt);
            self.learn_trim(u, lean, dt);
            // Positive turn = left: right wheel faster
            let out = MotorOutput {
                left: shape_duty(u - turn),
                right: shape_duty(u + turn),
            };
            match self.motors.drive(out.left, out.right) {
                Ok(()) => self.applied = out,
                Err(_) => {
                    self.set_mode(Mode::Fault);
                    self.stop_motors();
                }
            }
        } else {
            self.stop_motors();
        }

        let loop_us = (rtos::now_us() - start_us).max(0) as u32;
        self.stats.loop_max_us = self.stats.loop_max_us.max(loop_us);

        self.tick = self.tick.wrapping_add(1);
        if self.tick % TELEMETRY_DECIMATION == 0 {
            self.send_telemetry(&imu, target, loop_us);
        }
        if self.tick % LOOP_HZ as u32 == 0 {
            self.publish_status();
        }
    }

    /// Auto-trim, an outer loop much slower than the balance loop. Holding any angle other than
    /// the true balance point takes constant torque, so the robot keeps driving one way. If it
    /// keeps driving forward the target is too far forward: move it back, and vice versa.
    fn learn_trim(&mut self, u: f32, lean: f32, dt: f32) {
        self.duty_lp += (u - self.duty_lp) * dt / (AUTO_TRIM_TAU_S + dt);
        // While driving on purpose the duty is meant to be nonzero, learning would cancel it
        if lean != 0.0 {
            return;
        }
        let max = MAX_AUTO_TRIM_DEG.to_radians();
        let step = self.cmd.gains.ka.to_radians() * self.duty_lp * dt;
        self.auto_trim = (self.auto_trim - step).clamp(-max, max);
    }

    /// The safety state machine, see shared::Mode.
    fn next_mode(&self, state: &State, target: f32) -> Mode {
        if !self.cmd.enabled {
            // Also the only way out of Fault
            return Mode::Disabled;
        }
        let upright = (state.pitch - target).abs() < ENGAGE_DEG.to_radians();
        match self.mode {
            Mode::Fault => Mode::Fault,
            Mode::Balancing if state.pitch.abs() > FALL_DEG.to_radians() => Mode::Fallen,
            Mode::Balancing => Mode::Balancing,
            _ if upright => Mode::Balancing,
            Mode::Fallen => Mode::Fallen,
            Mode::Calibrating | Mode::Disabled | Mode::Armed => Mode::Armed,
        }
    }

    fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
        // Core 0 logs mode changes from the status. If the slot is busy the next periodic
        // publish carries it
        self.publish_status();
    }

    fn stop_motors(&mut self) {
        // A failing stop leaves nothing better to do; STBY is set low first inside stop()
        let _ = self.motors.stop();
        self.applied = MotorOutput::default();
    }

    fn send_telemetry(&mut self, imu: &ImuSample, target: f32, loop_us: u32) {
        // Exactly TELEMETRY_DECIMATION ticks since the previous one, so this averages over them
        let rate_hz = match self.last_telemetry_t_us {
            Some(last) if imu.t_us > last => {
                TELEMETRY_DECIMATION as f32 * 1e6 / (imu.t_us - last) as f32
            }
            _ => LOOP_HZ,
        };
        self.last_telemetry_t_us = Some(imu.t_us);
        let sample = Telemetry {
            t_us: imu.t_us,
            mode: self.mode,
            pitch_deg: self.state.pitch.to_degrees(),
            pitch_rate_dps: self.state.pitch_rate.to_degrees(),
            yaw_rate_dps: self.state.yaw_rate.to_degrees(),
            target_deg: target.to_degrees(),
            u_left: self.applied.left,
            u_right: self.applied.right,
            accel: imu.accel,
            gyro: imu.gyro,
            loop_us: loop_us.min(u16::MAX as u32) as u16,
            rate_hz,
            dropped: self.dropped,
            link_ok: self.link_ok,
        };
        match self.telemetry.try_send(sample) {
            Ok(()) => self.dropped = 0,
            Err(TrySendError::Full(_)) => {
                self.dropped = self.dropped.saturating_add(1);
                self.stats.telemetry_dropped += 1;
            }
            // Uploader thread is gone, nothing to do about it from here
            Err(TrySendError::Disconnected(_)) => {}
        }
    }

    fn publish_status(&mut self) {
        let published = SHARED.status.try_publish(Status {
            mode: self.mode,
            pitch_deg: self.state.pitch.to_degrees(),
            dt_min_us: if self.stats.dt_min_us == u32::MAX {
                0
            } else {
                self.stats.dt_min_us
            },
            dt_max_us: self.stats.dt_max_us,
            loop_max_us: self.stats.loop_max_us,
            imu_errors: self.stats.imu_errors,
            telemetry_dropped: self.stats.telemetry_dropped,
            link_ok: self.link_ok,
        });
        if published {
            // Start a new measuring period, the totals keep counting
            self.stats.dt_min_us = u32::MAX;
            self.stats.dt_max_us = 0;
            self.stats.loop_max_us = 0;
        }
    }
}

/// Clamp to -1..1 and map any nonzero duty into [deadband, 1] so small corrections still move
/// the wheels.
fn shape_duty(u: f32) -> f32 {
    let u = u.clamp(-1.0, 1.0);
    if MOTOR_DEADBAND > 0.0 && u != 0.0 {
        u.signum() * (MOTOR_DEADBAND + (1.0 - MOTOR_DEADBAND) * u.abs())
    } else {
        u
    }
}
