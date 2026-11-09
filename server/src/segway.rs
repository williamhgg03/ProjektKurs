// Segway state held by the server: the command the robot long-polls for (`Command`, same shape
// as in the firmware's shared.rs) and the telemetry it uploads (`Sample`, the firmware's
// `Telemetry`). The most recent samples stay in memory for the web page, and every sample is
// also appended to a CSV file for offline analysis.

use std::collections::VecDeque;
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

// Same limits the firmware clamps to (SegwayCode/src/config.rs), rejected here so the web page
// shows an error instead of the robot silently doing less than asked.
pub const MAX_TRIM_DEG: f32 = 10.0;
pub const MAX_LEAN_DEG: f32 = 10.0;
pub const MAX_TURN: f32 = 0.3;
/// Sanity bound on PID gains, far above anything that balances.
pub const MAX_GAIN: f32 = 1000.0;

// ---------------------------------------------------------------------------------------------
// Command
// ---------------------------------------------------------------------------------------------

/// PID gains, acting on radians (kp: duty per rad, ki: duty per rad*s, kd: duty per rad/s).
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Gains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

/// What the robot should do. Body of `GET /device/segway`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize)]
pub struct Command {
    /// Increases on every change, used for the long-poll.
    pub version: u64,
    /// Motors may run. Always starts off when the server starts.
    pub enabled: bool,
    /// Balance point offset in degrees.
    pub trim_deg: f32,
    /// Driving: lean on top of the trim (+ forward). Reset by the dead-man timeout.
    pub lean_deg: f32,
    /// Driving: differential duty (+ left). Reset by the dead-man timeout.
    pub turn: f32,
    pub gains: Gains,
}

/// The part of the command worth keeping across server restarts.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    pub trim_deg: f32,
    pub gains: Gains,
}

/// Partial update from the web page, omitted fields are left unchanged.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct CommandUpdate {
    pub enabled: Option<bool>,
    pub trim_deg: Option<f32>,
    pub kp: Option<f32>,
    pub ki: Option<f32>,
    pub kd: Option<f32>,
}

/// Driving input from the web page. Has to be repeated while held (see the dead-man timeout).
#[derive(Debug, Clone, Copy, Default, Deserialize)]
pub struct Drive {
    pub lean_deg: f32,
    pub turn: f32,
}

#[derive(Debug, PartialEq)]
pub enum CommandError {
    OutOfRange {
        field: &'static str,
        value: f32,
        min: f32,
        max: f32,
    },
}

impl fmt::Display for CommandError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CommandError::OutOfRange {
                field,
                value,
                min,
                max,
            } => write!(f, "{field} = {value} is outside {min}..={max}"),
        }
    }
}

impl std::error::Error for CommandError {}

fn check(field: &'static str, value: f32, min: f32, max: f32) -> Result<f32, CommandError> {
    // NaN and infinity fail the range check too
    if value >= min && value <= max {
        Ok(value)
    } else {
        Err(CommandError::OutOfRange {
            field,
            value,
            min,
            max,
        })
    }
}

fn check_opt(field: &'static str, value: Option<f32>, min: f32, max: f32) -> Result<Option<f32>, CommandError> {
    value.map(|v| check(field, v, min, max)).transpose()
}

impl Command {
    /// Disabled and standing still, with the saved trim and gains.
    pub fn from_settings(settings: Settings) -> Self {
        Self {
            trim_deg: settings.trim_deg,
            gains: settings.gains,
            ..Default::default()
        }
    }

    pub fn settings(&self) -> Settings {
        Settings {
            trim_deg: self.trim_deg,
            gains: self.gains,
        }
    }

    /// Apply a partial update. Everything is validated before anything changes.
    /// Returns whether the command changed. Does not touch `version`, the caller bumps it.
    pub fn apply(&mut self, u: &CommandUpdate) -> Result<bool, CommandError> {
        let trim = check_opt("trim_deg", u.trim_deg, -MAX_TRIM_DEG, MAX_TRIM_DEG)?;
        let kp = check_opt("kp", u.kp, 0.0, MAX_GAIN)?;
        let ki = check_opt("ki", u.ki, 0.0, MAX_GAIN)?;
        let kd = check_opt("kd", u.kd, 0.0, MAX_GAIN)?;

        let before = *self;
        if let Some(enabled) = u.enabled {
            self.enabled = enabled;
            if !enabled {
                // Don't start driving off with an old command when re-enabled
                self.lean_deg = 0.0;
                self.turn = 0.0;
            }
        }
        if let Some(v) = trim {
            self.trim_deg = v;
        }
        if let Some(v) = kp {
            self.gains.kp = v;
        }
        if let Some(v) = ki {
            self.gains.ki = v;
        }
        if let Some(v) = kd {
            self.gains.kd = v;
        }
        Ok(*self != before)
    }

    pub fn drive(&mut self, d: Drive) -> Result<bool, CommandError> {
        let lean = check("lean_deg", d.lean_deg, -MAX_LEAN_DEG, MAX_LEAN_DEG)?;
        let turn = check("turn", d.turn, -MAX_TURN, MAX_TURN)?;
        let before = *self;
        self.lean_deg = lean;
        self.turn = turn;
        Ok(*self != before)
    }

    /// Stop driving (lean and turn back to zero), still balancing.
    pub fn halt(&mut self) -> bool {
        let before = *self;
        self.lean_deg = 0.0;
        self.turn = 0.0;
        *self != before
    }

    /// Emergency stop: motors off. The robot will fall over.
    pub fn stop(&mut self) -> bool {
        let before = *self;
        self.enabled = false;
        self.lean_deg = 0.0;
        self.turn = 0.0;
        *self != before
    }

    pub fn is_driving(&self) -> bool {
        self.lean_deg != 0.0 || self.turn != 0.0
    }
}

/// Saved settings from `path`. A missing file gives the defaults (all zero).
pub fn load_settings(path: &Path) -> io::Result<Settings> {
    match std::fs::read(path) {
        Ok(bytes) => serde_json::from_slice(&bytes).map_err(io::Error::other),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(Settings::default()),
        Err(e) => Err(e),
    }
}

pub fn save_settings(path: &Path, settings: &Settings) -> io::Result<()> {
    let json = serde_json::to_vec_pretty(settings).map_err(io::Error::other)?;
    std::fs::write(path, json)
}

// ---------------------------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------------------------

/// The firmware's control mode (`Mode` in shared.rs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    Calibrating,
    Disabled,
    Armed,
    Balancing,
    Fallen,
    Fault,
    /// A mode added to the firmware after this server was built.
    #[serde(other)]
    Unknown,
}

impl Mode {
    fn as_str(self) -> &'static str {
        match self {
            Mode::Calibrating => "calibrating",
            Mode::Disabled => "disabled",
            Mode::Armed => "armed",
            Mode::Balancing => "balancing",
            Mode::Fallen => "fallen",
            Mode::Fault => "fault",
            Mode::Unknown => "unknown",
        }
    }
}

/// One control tick, same shape as `Telemetry` in the firmware's shared.rs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Sample {
    /// Microseconds since the chip booted.
    pub t_us: i64,
    pub mode: Mode,
    pub pitch_deg: f32,
    pub pitch_rate_dps: f32,
    pub yaw_rate_dps: f32,
    pub target_deg: f32,
    pub u_left: f32,
    pub u_right: f32,
    /// Raw sensor frame: g and deg/s (gyro bias not removed).
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
    pub loop_us: u16,
    /// Measured control loop rate (= IMU sample rate). Missing from older firmware -> 0.
    #[serde(default)]
    pub rate_hz: f32,
    pub dropped: u16,
    pub link_ok: bool,
}

pub const CSV_HEADER: &str = "t_us,mode,pitch_deg,pitch_rate_dps,yaw_rate_dps,target_deg,u_left,u_right,ax,ay,az,gx,gy,gz,loop_us,rate_hz,dropped,link_ok";

impl Sample {
    fn csv_line(&self) -> String {
        let [ax, ay, az] = self.accel;
        let [gx, gy, gz] = self.gyro;
        format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            self.t_us,
            self.mode.as_str(),
            self.pitch_deg,
            self.pitch_rate_dps,
            self.yaw_rate_dps,
            self.target_deg,
            self.u_left,
            self.u_right,
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            self.loop_us,
            self.rate_hz,
            self.dropped,
            self.link_ok as u8,
        )
    }
}

/// Samples kept in memory, 2 minutes at the robot's 50 Hz.
pub const MAX_RECENT: usize = 6000;

#[derive(Default)]
pub struct TelemetryLog {
    recent: VecDeque<Sample>,
    /// Samples received since the server started. Sample `i` (counting from 0) has
    /// sequence number `i`, so this is also the sequence number of the next one.
    total: u64,
    last_batch: Option<Instant>,
    csv: Option<(PathBuf, File)>,
}

impl TelemetryLog {
    /// Log that also appends to `path`, writing the header only if the file is new or empty.
    pub fn with_csv(path: &Path) -> io::Result<Self> {
        let mut file = OpenOptions::new().create(true).append(true).open(path)?;
        if file.metadata()?.len() == 0 {
            writeln!(file, "{CSV_HEADER}")?;
        }
        Ok(Self {
            csv: Some((path.to_path_buf(), file)),
            ..Default::default()
        })
    }

    /// Store a batch. The samples are kept in memory even if writing the CSV fails.
    pub fn push_batch(&mut self, samples: &[Sample]) -> io::Result<()> {
        self.last_batch = Some(Instant::now());
        self.total += samples.len() as u64;
        for s in samples {
            if self.recent.len() == MAX_RECENT {
                self.recent.pop_front();
            }
            self.recent.push_back(*s);
        }

        if let Some((_, file)) = &mut self.csv {
            let lines: String = samples.iter().map(Sample::csv_line).collect();
            file.write_all(lines.as_bytes())?;
        }
        Ok(())
    }

    pub fn total(&self) -> u64 {
        self.total
    }

    /// Time since the robot last uploaded, `None` if it never has.
    pub fn age(&self) -> Option<Duration> {
        self.last_batch.map(|t| t.elapsed())
    }

    pub fn csv_path(&self) -> Option<&Path> {
        self.csv.as_ref().map(|(p, _)| p.as_path())
    }

    /// The newest (at most `limit`) samples with a sequence number `>= since`.
    /// With no `since`, or one from before a server restart (`since > total`), it
    /// returns the newest `limit` samples.
    pub fn since(&self, since: Option<u64>, limit: usize) -> Vec<Sample> {
        let first_seq = self.total - self.recent.len() as u64;
        let available = match since {
            Some(s) if s <= self.total => {
                let skip = s.saturating_sub(first_seq) as usize;
                self.recent.len() - skip
            }
            _ => self.recent.len(),
        };
        let n = available.min(limit);
        self.recent.range(self.recent.len() - n..).copied().collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(t_us: i64) -> Sample {
        Sample {
            t_us,
            mode: Mode::Balancing,
            pitch_deg: 0.5,
            pitch_rate_dps: -1.0,
            yaw_rate_dps: 0.0,
            target_deg: 1.0,
            u_left: 0.1,
            u_right: 0.1,
            accel: [0.0, 0.0, 1.0],
            gyro: [0.5, -0.25, 0.0],
            loop_us: 150,
            rate_hz: 200.0,
            dropped: 0,
            link_ok: true,
        }
    }

    fn log_with(n: i64) -> TelemetryLog {
        let mut log = TelemetryLog::default();
        let samples: Vec<_> = (0..n).map(sample).collect();
        log.push_batch(&samples).unwrap();
        log
    }

    fn times(samples: &[Sample]) -> Vec<i64> {
        samples.iter().map(|s| s.t_us).collect()
    }

    #[test]
    fn device_json_matches_firmware() {
        let cmd = Command {
            version: 3,
            enabled: true,
            trim_deg: 1.5,
            lean_deg: 0.0,
            turn: 0.0,
            gains: Gains { kp: 6.0, ki: 0.0, kd: 0.25 },
        };
        let json = serde_json::to_value(cmd).unwrap();
        assert_eq!(
            json,
            serde_json::json!({
                "version": 3, "enabled": true, "trim_deg": 1.5, "lean_deg": 0.0, "turn": 0.0,
                "gains": {"kp": 6.0, "ki": 0.0, "kd": 0.25}
            })
        );
    }

    #[test]
    fn parses_firmware_telemetry() {
        let body = r#"{"t_us":12345678,"mode":"balancing","pitch_deg":0.4,"pitch_rate_dps":-2.1,
            "yaw_rate_dps":0.0,"target_deg":1.5,"u_left":0.12,"u_right":0.12,
            "accel":[0.0,0.0,1.0],"gyro":[0.1,-2.0,0.0],"loop_us":180,"rate_hz":199.8,"dropped":0,"link_ok":true}"#;
        let s: Sample = serde_json::from_str(body).unwrap();
        assert_eq!(s.mode, Mode::Balancing);
        assert_eq!(s.accel, [0.0, 0.0, 1.0]);

        let future: Sample = serde_json::from_str(&body.replace("balancing", "dancing")).unwrap();
        assert_eq!(future.mode, Mode::Unknown);
    }

    #[test]
    fn csv_line_matches_header() {
        let line = sample(7).csv_line();
        assert_eq!(line.trim_end().split(',').count(), CSV_HEADER.split(',').count());
        assert!(line.starts_with("7,balancing,"));
    }

    #[test]
    fn update_validates_before_changing_anything() {
        let mut cmd = Command::default();
        let bad = CommandUpdate {
            enabled: Some(true),
            kp: Some(5.0),
            kd: Some(-1.0),
            ..Default::default()
        };
        assert!(matches!(cmd.apply(&bad), Err(CommandError::OutOfRange { field: "kd", .. })));
        assert_eq!(cmd, Command::default());

        let nan = CommandUpdate {
            trim_deg: Some(f32::NAN),
            ..Default::default()
        };
        assert!(cmd.apply(&nan).is_err());
    }

    #[test]
    fn update_reports_changes() {
        let mut cmd = Command::default();
        let u = CommandUpdate {
            trim_deg: Some(1.0),
            kp: Some(6.0),
            ..Default::default()
        };
        assert_eq!(cmd.apply(&u), Ok(true));
        assert_eq!(cmd.apply(&u), Ok(false));
        assert_eq!(cmd.settings(), Settings { trim_deg: 1.0, gains: Gains { kp: 6.0, ki: 0.0, kd: 0.0 } });
    }

    #[test]
    fn disabling_stops_driving() {
        let mut cmd = Command::default();
        cmd.apply(&CommandUpdate { enabled: Some(true), ..Default::default() }).unwrap();
        cmd.drive(Drive { lean_deg: 3.0, turn: 0.1 }).unwrap();
        assert!(cmd.is_driving());
        cmd.apply(&CommandUpdate { enabled: Some(false), ..Default::default() }).unwrap();
        assert!(!cmd.is_driving());
    }

    #[test]
    fn drive_limits() {
        let mut cmd = Command::default();
        assert!(cmd.drive(Drive { lean_deg: MAX_LEAN_DEG + 1.0, turn: 0.0 }).is_err());
        assert!(cmd.drive(Drive { lean_deg: 0.0, turn: -MAX_TURN - 0.1 }).is_err());
        assert_eq!(cmd.drive(Drive { lean_deg: 2.0, turn: -0.1 }), Ok(true));
        assert!(cmd.halt());
        assert!(!cmd.halt());
    }

    #[test]
    fn stop_disables() {
        let mut cmd = Command::default();
        cmd.apply(&CommandUpdate { enabled: Some(true), ..Default::default() }).unwrap();
        assert!(cmd.stop());
        assert!(!cmd.enabled);
        assert!(!cmd.stop());
    }

    #[test]
    fn settings_round_trip() {
        let dir = std::env::temp_dir().join(format!("segway-settings-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("settings.json");

        assert_eq!(load_settings(&path).unwrap(), Settings::default());
        let s = Settings { trim_deg: -0.5, gains: Gains { kp: 8.0, ki: 1.0, kd: 0.3 } };
        save_settings(&path, &s).unwrap();
        assert_eq!(load_settings(&path).unwrap(), s);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn keeps_only_newest_samples() {
        let log = log_with(MAX_RECENT as i64 + 5);
        assert_eq!(log.total(), MAX_RECENT as u64 + 5);
        assert_eq!(log.recent.len(), MAX_RECENT);
        assert_eq!(log.recent.front().unwrap().t_us, 5);
    }

    #[test]
    fn since_returns_only_new_samples() {
        let log = log_with(10);
        assert_eq!(times(&log.since(Some(7), 100)), vec![7, 8, 9]);
        assert!(log.since(Some(10), 100).is_empty());
        assert_eq!(times(&log.since(None, 2)), vec![8, 9]);
        // `since` from before a server restart: start over with the newest
        assert_eq!(times(&log.since(Some(50), 3)), vec![7, 8, 9]);
    }
}
