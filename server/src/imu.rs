// IMU samples uploaded by the chip. The most recent ones stay in memory for the
// web page, and every sample is also appended to a CSV file on disk.

use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

/// Samples kept in memory, 100s at the chip's 100 Hz.
pub const MAX_RECENT: usize = 10_000;

pub const CSV_HEADER: &str = "t_us,ax,ay,az,gx,gy,gz,temp_c";

/// Same shape as `Sample` in the firmware's mpu6050.rs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Sample {
    /// Microseconds since the chip booted.
    pub t_us: i64,
    /// Acceleration in g.
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    /// Angular rate in deg/s.
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub temp_c: f32,
}

impl Sample {
    fn csv_line(&self) -> String {
        format!(
            "{},{},{},{},{},{},{},{}\n",
            self.t_us, self.ax, self.ay, self.az, self.gx, self.gy, self.gz, self.temp_c
        )
    }
}

#[derive(Default)]
pub struct ImuLog {
    recent: VecDeque<Sample>,
    /// Samples received since the server started. Sample `i` (counting from 0) has
    /// sequence number `i`, so this is also the sequence number of the next one.
    total: u64,
    last_batch: Option<Instant>,
    csv: Option<(PathBuf, File)>,
}

impl ImuLog {
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

    /// Time since the chip last uploaded, `None` if it never has.
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
            ax: 0.0,
            ay: 0.0,
            az: 1.0,
            gx: 0.5,
            gy: -0.25,
            gz: 0.0,
            temp_c: 25.0,
        }
    }

    fn log_with(n: i64) -> ImuLog {
        let mut log = ImuLog::default();
        let samples: Vec<_> = (0..n).map(sample).collect();
        log.push_batch(&samples).unwrap();
        log
    }

    fn times(samples: &[Sample]) -> Vec<i64> {
        samples.iter().map(|s| s.t_us).collect()
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
    }

    #[test]
    fn since_without_cursor_returns_newest() {
        let log = log_with(10);
        assert_eq!(times(&log.since(None, 3)), vec![7, 8, 9]);
    }

    #[test]
    fn since_limit_keeps_newest() {
        let log = log_with(10);
        assert_eq!(times(&log.since(Some(0), 2)), vec![8, 9]);
    }

    #[test]
    fn since_older_than_buffer_returns_all_kept() {
        let log = log_with(MAX_RECENT as i64 + 5);
        assert_eq!(log.since(Some(0), usize::MAX).len(), MAX_RECENT);
    }

    #[test]
    fn since_after_server_restart_returns_newest() {
        let log = log_with(10);
        assert_eq!(times(&log.since(Some(500), 2)), vec![8, 9]);
    }

    #[test]
    fn csv_line_format() {
        assert_eq!(sample(42).csv_line(), "42,0,0,1,0.5,-0.25,0,25\n");
    }

    #[test]
    fn csv_header_written_once() {
        let path = std::env::temp_dir().join(format!("imu-test-{}.csv", std::process::id()));
        let _ = std::fs::remove_file(&path);

        ImuLog::with_csv(&path).unwrap().push_batch(&[sample(1)]).unwrap();
        ImuLog::with_csv(&path).unwrap().push_batch(&[sample(2)]).unwrap();

        let contents = std::fs::read_to_string(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(
            contents,
            format!("{CSV_HEADER}\n{}{}", sample(1).csv_line(), sample(2).csv_line())
        );
    }
}
