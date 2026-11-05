// MPU-6050 accelerometer + gyroscope driver over I2C, ported from the accel_And_gyro branch.
//
// The chip is a set of 8-bit registers: write [reg, value] to configure it,
// write [reg] then read N bytes to read from consecutive registers.
// It pulses INT (active high) whenever a new sample is ready; that pulse clocks the control loop.

use std::time::Duration;

use esp_idf_svc::hal::delay::{FreeRtos, TickType};
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::sys::{EspError, TickType_t};

use crate::config::{MPU_DLPF_CFG, MPU_SAMPLE_DIV};
use crate::rtos;

/// AD0 low (breakout default). AD0 high gives 0x69.
pub const ADDR: u8 = 0x68;

const SMPLRT_DIV: u8 = 0x19;
const CONFIG: u8 = 0x1A;
const GYRO_CONFIG: u8 = 0x1B;
const ACCEL_CONFIG: u8 = 0x1C;
const INT_PIN_CFG: u8 = 0x37;
const INT_ENABLE: u8 = 0x38;
const ACCEL_XOUT_H: u8 = 0x3B;
const PWR_MGMT_1: u8 = 0x6B;
const WHO_AM_I: u8 = 0x75;

const WHO_AM_I_MPU6050: u8 = 0x68;
/// FS_SEL 1: +-500 deg/s.
const GYRO_500DPS: u8 = 0x08;
const GYRO_LSB_PER_DPS: f32 = 65.5;
/// AFS_SEL 1: +-4 g.
const ACCEL_4G: u8 = 0x08;
const ACCEL_LSB_PER_G: f32 = 8192.0;
/// INT active high, push-pull, 50us pulse, cleared by any register read.
const INT_RD_CLEAR: u8 = 0x10;
const DATA_RDY_EN: u8 = 0x01;
/// Leave sleep mode and clock from the gyro X PLL (more stable than the internal oscillator).
const CLKSEL_PLL_X: u8 = 0x01;
const DEVICE_RESET: u8 = 0x80;

/// A 14-byte read takes about 0.4 ms at 400 kHz. A stuck bus fails after this instead of
/// stalling the control loop.
const I2C_TIMEOUT: Duration = Duration::from_millis(5);

/// One reading in the sensor's own frame (see config::IMU_FORWARD for the robot frame).
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuSample {
    /// Microseconds since boot, taken right after the read.
    pub t_us: i64,
    /// Acceleration in g.
    pub accel: [f32; 3],
    /// Angular rate in deg/s.
    pub gyro: [f32; 3],
}

pub struct Mpu6050<'d> {
    i2c: I2cDriver<'d>,
    timeout: TickType_t,
}

impl<'d> Mpu6050<'d> {
    /// Does not touch the chip, call `init` next (and again if it fails).
    pub fn new(i2c: I2cDriver<'d>) -> Self {
        Self {
            i2c,
            timeout: TickType::from(I2C_TIMEOUT).ticks(),
        }
    }

    pub fn init(&mut self) -> Result<(), EspError> {
        let id = self.read_reg(WHO_AM_I)?;
        if id != WHO_AM_I_MPU6050 {
            // Clones and the MPU-6500 family answer differently but share this register map
            log::warn!(
                "MPU-6050 WHO_AM_I = {id:#04x}, expected {WHO_AM_I_MPU6050:#04x}; continuing anyway"
            );
        }

        // Reset to known register values, then configure
        self.write_reg(PWR_MGMT_1, DEVICE_RESET)?;
        FreeRtos::delay_ms(100);
        self.write_reg(PWR_MGMT_1, CLKSEL_PLL_X)?;
        self.write_reg(SMPLRT_DIV, MPU_SAMPLE_DIV)?;
        self.write_reg(CONFIG, MPU_DLPF_CFG)?;
        self.write_reg(GYRO_CONFIG, GYRO_500DPS)?;
        self.write_reg(ACCEL_CONFIG, ACCEL_4G)?;
        self.write_reg(INT_PIN_CFG, INT_RD_CLEAR)?;
        self.write_reg(INT_ENABLE, DATA_RDY_EN)?;
        Ok(())
    }

    /// Read one sample: accel, temperature and gyro are 7 consecutive big-endian i16 registers
    /// (temperature, word 3, is skipped).
    /// Reading also clears the INT pulse.
    pub fn read(&mut self) -> Result<ImuSample, EspError> {
        let mut buf = [0u8; 14];
        self.i2c
            .write_read(ADDR, &[ACCEL_XOUT_H], &mut buf, self.timeout)?;
        let t_us = rtos::now_us();

        let word = |i: usize| i16::from_be_bytes([buf[2 * i], buf[2 * i + 1]]) as f32;
        Ok(ImuSample {
            t_us,
            accel: [
                word(0) / ACCEL_LSB_PER_G,
                word(1) / ACCEL_LSB_PER_G,
                word(2) / ACCEL_LSB_PER_G,
            ],
            gyro: [
                word(4) / GYRO_LSB_PER_DPS,
                word(5) / GYRO_LSB_PER_DPS,
                word(6) / GYRO_LSB_PER_DPS,
            ],
        })
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), EspError> {
        self.i2c.write(ADDR, &[reg, value], self.timeout)
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, EspError> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(ADDR, &[reg], &mut buf, self.timeout)?;
        Ok(buf[0])
    }
}
