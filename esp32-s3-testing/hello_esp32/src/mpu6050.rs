// MPU-6050 accelerometer + gyroscope driver over I2C.
//
// The chip is a set of 8-bit registers: write [reg, value] to configure it,
// write [reg] then read N bytes to read from consecutive registers.
// It samples at 100 Hz and pulses INT (active high) whenever a new sample is ready.

use esp_idf_svc::hal::delay::{FreeRtos, BLOCK};
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::sys::EspError;
use serde::Serialize;

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

/// Sample rate = 1 kHz / (1 + div) with the DLPF enabled -> 100 Hz.
const SAMPLE_DIV: u8 = 9;
/// DLPF_CFG 3: about 44 Hz bandwidth for both accel and gyro.
const DLPF_44HZ: u8 = 0x03;
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

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Sample {
    /// Microseconds since boot.
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

pub struct Mpu6050<'d> {
    i2c: I2cDriver<'d>,
}

impl<'d> Mpu6050<'d> {
    pub fn new(i2c: I2cDriver<'d>) -> anyhow::Result<Self> {
        let mut mpu = Self { i2c };

        let id = mpu.read_reg(WHO_AM_I)?;
        log::info!("MPU-6050 WHO_AM_I = {id:#04x}");
        if id != ADDR {
            // Clones and the MPU-6500 family answer differently but share this register map
            log::warn!("unexpected WHO_AM_I, expected {ADDR:#04x}; continuing anyway");
        }

        // Reset to known register values, then configure
        mpu.write_reg(PWR_MGMT_1, DEVICE_RESET)?;
        FreeRtos::delay_ms(100);
        mpu.write_reg(PWR_MGMT_1, CLKSEL_PLL_X)?;
        mpu.write_reg(SMPLRT_DIV, SAMPLE_DIV)?;
        mpu.write_reg(CONFIG, DLPF_44HZ)?;
        mpu.write_reg(GYRO_CONFIG, GYRO_500DPS)?;
        mpu.write_reg(ACCEL_CONFIG, ACCEL_4G)?;
        mpu.write_reg(INT_PIN_CFG, INT_RD_CLEAR)?;
        mpu.write_reg(INT_ENABLE, DATA_RDY_EN)?;

        Ok(mpu)
    }

    /// Read one sample: accel, temperature and gyro are 7 consecutive big-endian i16 registers.
    pub fn read(&mut self) -> Result<Sample, EspError> {
        let mut buf = [0u8; 14];
        self.i2c.write_read(ADDR, &[ACCEL_XOUT_H], &mut buf, BLOCK)?;
        let t_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() };

        let word = |i: usize| i16::from_be_bytes([buf[2 * i], buf[2 * i + 1]]) as f32;
        Ok(Sample {
            t_us,
            ax: word(0) / ACCEL_LSB_PER_G,
            ay: word(1) / ACCEL_LSB_PER_G,
            az: word(2) / ACCEL_LSB_PER_G,
            temp_c: word(3) / 340.0 + 36.53,
            gx: word(4) / GYRO_LSB_PER_DPS,
            gy: word(5) / GYRO_LSB_PER_DPS,
            gz: word(6) / GYRO_LSB_PER_DPS,
        })
    }

    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), EspError> {
        self.i2c.write(ADDR, &[reg, value], BLOCK)
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, EspError> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(ADDR, &[reg], &mut buf, BLOCK)?;
        Ok(buf[0])
    }
}
