//! One burst of data registers and its conversion to SI units.

use crate::config::{AccelFullScale, GyroFullScale};

const G: f32 = 9.80665;
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

/// One sample as the device reports it, in LSB.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawSample {
    /// Accelerometer X, Y, Z.
    pub accel: [i16; 3],
    /// Gyroscope X, Y, Z.
    pub gyro: [i16; 3],
    /// Temperature.
    pub temp: i16,
}

impl RawSample {
    /// Decodes registers 0x00..=0x0D (little-endian words: accel XYZ, gyro XYZ, temperature).
    #[inline]
    pub const fn decode(b: &[u8; 14]) -> Self {
        let [
            ax0,
            ax1,
            ay0,
            ay1,
            az0,
            az1,
            gx0,
            gx1,
            gy0,
            gy1,
            gz0,
            gz1,
            t0,
            t1,
        ] = *b;
        Self {
            accel: [
                i16::from_le_bytes([ax0, ax1]),
                i16::from_le_bytes([ay0, ay1]),
                i16::from_le_bytes([az0, az1]),
            ],
            gyro: [
                i16::from_le_bytes([gx0, gx1]),
                i16::from_le_bytes([gy0, gy1]),
                i16::from_le_bytes([gz0, gz1]),
            ],
            temp: i16::from_le_bytes([t0, t1]),
        }
    }

    /// Acceleration in m/s^2 for the configured full scale.
    pub fn accel_ms2(&self, fs: AccelFullScale) -> [f32; 3] {
        let k = G / fs.lsb_per_g();
        self.accel.map(|v| f32::from(v) * k)
    }

    /// Angular rate in rad/s for the configured full scale.
    pub fn gyro_rads(&self, fs: GyroFullScale) -> [f32; 3] {
        let k = DEG_TO_RAD / fs.lsb_per_dps();
        self.gyro.map(|v| f32::from(v) * k)
    }

    /// Temperature in degrees Celsius (raw / 128 + 25).
    pub fn temperature_c(&self) -> f32 {
        f32::from(self.temp) / 128.0 + 25.0
    }
}
