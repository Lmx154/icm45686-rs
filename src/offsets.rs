//! User offsets: the six values the sensors subtract from their output, as raw signed 14-bit register
//! values, with the pure converters and the register images. No bus and no allocation.
//!
//! Each offset is split over two indirect registers: the low byte (bits 7:0) and a high register
//! whose bits 5:0 hold bits 13:8; bits 7:6 of the high register are reserved and keep their reset
//! value 0 (gyro 20.1-20.6, p189-p190; accel 21.1-21.6, p193-p194). The registers are R/W and reset to
//! 0x00.
//!
//! Format and scale, measured on the board, 2026-11-10: each offset is a 14-bit two's complement
//! value, and a positive value RAISES the output (it is added), for the accelerometer and the
//! gyroscope, on all axes. The scale is the full range over 8192 LSB: 1 g / 8192 = 122.07 micro-g per
//! LSB for the accelerometer and 62.5 dps / 8192 = 7.629 millidegrees per second per LSB for the
//! gyroscope (+8191 gave +1000.3 mg and +62.47 dps; -8192 gave -999.7 mg and -62.49 dps; +800 gave
//! 97.6 to 98.1 mg; +1333 gave 10.16 to 10.18 dps). The field is not clamped at its ends, and the
//! scale does not follow the configured full scale (it was the same at 4 g / 2000 dps and at 16 g /
//! 250 dps). The datasheet's figures, 0.125 mg and 7.5 mdps per LSB (21.1, p193; 20.1, p189), are
//! rounded: this crate uses the measured scale, in integers: micro-g = raw * 15625 / 128 and
//! millidegrees per second = raw * 15625 / 2048, each rounded half away from zero.

use crate::reg;

/// Smallest raw offset: -8192, the most negative 14-bit two's complement value.
pub const RAW_MIN: i16 = -8192;
/// Largest raw offset: 8191.
pub const RAW_MAX: i16 = 8191;
/// 62.5 dps / 8192 LSB = 15625 / 2048 millidegrees per second per LSB (measured on the board,
/// 2026-11-10; the datasheet's 7.5 mdps is rounded).
const GYRO_MDPS_NUM: i32 = 15_625;
const GYRO_MDPS_DEN: i32 = 2_048;
/// 1 g / 8192 LSB = 15625 / 128 micro-g per LSB (measured on the board, 2026-11-10; the datasheet's
/// 0.125 mg is rounded).
const ACCEL_UG_NUM: i32 = 15_625;
const ACCEL_UG_DEN: i32 = 128;

/// The sensor axis an out-of-range offset belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum OffsetAxis {
    /// Gyroscope X.
    GyroX,
    /// Gyroscope Y.
    GyroY,
    /// Gyroscope Z.
    GyroZ,
    /// Accelerometer X.
    AccelX,
    /// Accelerometer Y.
    AccelY,
    /// Accelerometer Z.
    AccelZ,
}

/// Why an offset cannot be used.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum OffsetError {
    /// A value given to a converter or to [`register_image`] is outside -8192..=8191 (after
    /// conversion).
    ValueOutOfRange,
    /// This axis of an [`Offsets`] is outside -8192..=8191.
    AxisOutOfRange(OffsetAxis),
}

/// The six user offsets as raw signed 14-bit register values, X, Y, Z. `Default` is all zero, the
/// reset value.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Offsets {
    /// Gyroscope X, Y, Z: two's complement, 62.5 dps / 8192 = 7.629 mdps per LSB; a positive value
    /// raises the output (measured on the board, 2026-11-10).
    pub gyro: [i16; 3],
    /// Accelerometer X, Y, Z: two's complement, 1 g / 8192 = 122.07 micro-g per LSB; a positive
    /// value raises the output (measured on the board, 2026-11-10).
    pub accel: [i16; 3],
}

impl Offsets {
    /// All six offsets zero.
    pub const fn new() -> Self {
        Self {
            gyro: [0; 3],
            accel: [0; 3],
        }
    }

    /// Sets the gyroscope offsets X, Y, Z (raw, not range checked until used).
    #[must_use]
    pub const fn with_gyro(mut self, gyro: [i16; 3]) -> Self {
        self.gyro = gyro;
        self
    }

    /// Sets the accelerometer offsets X, Y, Z (raw, not range checked until used).
    #[must_use]
    pub const fn with_accel(mut self, accel: [i16; 3]) -> Self {
        self.accel = accel;
        self
    }

    /// Checks that every value is in -8192..=8191; the first axis out of range (gyro X, Y, Z, then
    /// accel X, Y, Z) is reported.
    pub fn validate(&self) -> Result<(), OffsetError> {
        let [gx, gy, gz] = self.gyro;
        let [ax, ay, az] = self.accel;
        for (raw, axis) in [
            (gx, OffsetAxis::GyroX),
            (gy, OffsetAxis::GyroY),
            (gz, OffsetAxis::GyroZ),
            (ax, OffsetAxis::AccelX),
            (ay, OffsetAxis::AccelY),
            (az, OffsetAxis::AccelZ),
        ] {
            if !in_range(raw) {
                return Err(OffsetError::AxisOutOfRange(axis));
            }
        }
        Ok(())
    }

    /// The twelve indirect registers as `(address, value, compared mask)`, in write order: gyro X,
    /// Y, Z then accel X, Y, Z, each the low byte then the high register. The values are meaningful
    /// only for offsets that pass [`validate`](Self::validate).
    pub(crate) fn registers(&self) -> [(u16, u8, u8); 12] {
        let [gx, gy, gz] = self.gyro;
        let [ax, ay, az] = self.accel;
        let [gxl, gxh] = pair(reg::GYRO_X_OFFUSER, gx);
        let [gyl, gyh] = pair(reg::GYRO_Y_OFFUSER, gy);
        let [gzl, gzh] = pair(reg::GYRO_Z_OFFUSER, gz);
        let [axl, axh] = pair(reg::ACCEL_X_OFFUSER, ax);
        let [ayl, ayh] = pair(reg::ACCEL_Y_OFFUSER, ay);
        let [azl, azh] = pair(reg::ACCEL_Z_OFFUSER, az);
        [gxl, gxh, gyl, gyh, gzl, gzh, axl, axh, ayl, ayh, azl, azh]
    }
}

const fn in_range(raw: i16) -> bool {
    raw >= RAW_MIN && raw <= RAW_MAX
}

/// Low byte and high-register byte of `raw`, two's complement in 14 bits; the reserved bits 7:6 of
/// the high byte are 0 (their reset value).
const fn encode(raw: i16) -> [u8; 2] {
    let v = (raw as u16) & 0x3FFF;
    v.to_le_bytes()
}

fn pair(low: u16, raw: i16) -> [(u16, u8, u8); 2] {
    let [lo, hi] = encode(raw);
    [(low, lo, 0xFF), (low + 1, hi, reg::OFFUSER_HIGH_MASK)]
}

/// The register image of a raw offset: `(low byte, high register)`. The high register carries bits
/// 13:8 in bits 5:0 and 0 in the reserved bits 7:6. Two's complement (measured on the board,
/// 2026-11-10). [`OffsetError::ValueOutOfRange`] outside -8192..=8191.
pub const fn register_image(raw: i16) -> Result<(u8, u8), OffsetError> {
    if !in_range(raw) {
        return Err(OffsetError::ValueOutOfRange);
    }
    let [lo, hi] = encode(raw);
    Ok((lo, hi))
}

/// The raw offset held by a register image; bits 7:6 of `high` are ignored (reserved).
pub const fn from_register_image(low: u8, high: u8) -> i16 {
    let v = (((high & reg::OFFUSER_HIGH_MASK) as u16) << 8) | low as u16;
    ((v << 2) as i16) >> 2
}

/// `value * num / den`, rounded to the nearest, halves away from zero. `value * num` must not
/// overflow an `i32`; the callers guarantee it.
const fn scale(value: i32, num: i32, den: i32) -> i32 {
    let x = value * num;
    (x + if x < 0 { -(den / 2) } else { den / 2 }) / den
}

/// Gyroscope raw offset to millidegrees per second: raw * 15625 / 2048 (62.5 dps over 8192 LSB,
/// measured on the board, 2026-11-10; the datasheet's 7.5 mdps per LSB is rounded), rounded to the
/// nearest, halves away from zero. The input is not range checked; the product with 15625 fits an
/// `i32` for every `i16`, and the division is 32-bit.
pub const fn gyro_raw_to_mdps(raw: i16) -> i32 {
    scale(raw as i32, GYRO_MDPS_NUM, GYRO_MDPS_DEN)
}

/// Millidegrees per second to a gyroscope raw offset, rounded to the nearest LSB, halves away from
/// zero. [`OffsetError::ValueOutOfRange`] when the result is outside -8192..=8191. The round trip
/// with [`gyro_raw_to_mdps`] returns the raw value.
pub const fn gyro_mdps_to_raw(mdps: i32) -> Result<i16, OffsetError> {
    match mdps.checked_mul(GYRO_MDPS_DEN) {
        Some(x) => to_raw(round_div(x, GYRO_MDPS_NUM)),
        None => Err(OffsetError::ValueOutOfRange),
    }
}

/// Accelerometer raw offset to micro-g: raw * 15625 / 128 (1 g over 8192 LSB, measured on the board,
/// 2026-11-10; the datasheet's 0.125 mg per LSB is rounded), rounded to the nearest, halves away
/// from zero. The product with 15625 fits an `i32` for every `i16`, and the division is 32-bit.
pub const fn accel_raw_to_ug(raw: i16) -> i32 {
    scale(raw as i32, ACCEL_UG_NUM, ACCEL_UG_DEN)
}

/// Micro-g to an accelerometer raw offset, rounded to the nearest LSB, halves away from zero.
/// [`OffsetError::ValueOutOfRange`] when the result is outside -8192..=8191.
pub const fn accel_ug_to_raw(ug: i32) -> Result<i16, OffsetError> {
    match ug.checked_mul(ACCEL_UG_DEN) {
        Some(x) => to_raw(round_div(x, ACCEL_UG_NUM)),
        None => Err(OffsetError::ValueOutOfRange),
    }
}

/// `x / den` rounded to the nearest, halves away from zero, without overflow for `den` below
/// `i32::MAX / 2`: the half is added with saturation, which only matters far outside the raw range.
const fn round_div(x: i32, den: i32) -> i32 {
    x.saturating_add(if x < 0 { -(den / 2) } else { den / 2 }) / den
}

const fn to_raw(n: i32) -> Result<i16, OffsetError> {
    if n < RAW_MIN as i32 || n > RAW_MAX as i32 {
        Err(OffsetError::ValueOutOfRange)
    } else {
        Ok(n as i16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn images_at_zero_one_minus_one_and_the_extremes() {
        // Two's complement in 14 bits: low byte, then bits 13:8 in bits 5:0 of the high register.
        assert_eq!(register_image(0), Ok((0x00, 0x00)));
        assert_eq!(register_image(1), Ok((0x01, 0x00)));
        assert_eq!(register_image(-1), Ok((0xFF, 0x3F)));
        assert_eq!(register_image(RAW_MAX), Ok((0xFF, 0x1F)));
        assert_eq!(register_image(RAW_MIN), Ok((0x00, 0x20)));
        assert_eq!(register_image(256), Ok((0x00, 0x01)));
        assert_eq!(register_image(-256), Ok((0x00, 0x3F)));
    }

    #[test]
    fn out_of_range_is_a_typed_error() {
        assert_eq!(register_image(8192), Err(OffsetError::ValueOutOfRange));
        assert_eq!(register_image(-8193), Err(OffsetError::ValueOutOfRange));
        assert_eq!(register_image(i16::MAX), Err(OffsetError::ValueOutOfRange));
        assert_eq!(register_image(i16::MIN), Err(OffsetError::ValueOutOfRange));
        // The last value that rounds into the range and the first that does not.
        assert_eq!(gyro_mdps_to_raw(62_496), Ok(RAW_MAX));
        assert_eq!(gyro_mdps_to_raw(62_497), Err(OffsetError::ValueOutOfRange));
        assert_eq!(gyro_mdps_to_raw(-62_503), Ok(RAW_MIN));
        assert_eq!(gyro_mdps_to_raw(-62_504), Err(OffsetError::ValueOutOfRange));
        assert_eq!(
            gyro_mdps_to_raw(i32::MAX),
            Err(OffsetError::ValueOutOfRange)
        );
        assert_eq!(
            gyro_mdps_to_raw(i32::MIN),
            Err(OffsetError::ValueOutOfRange)
        );
        assert_eq!(accel_ug_to_raw(999_938), Ok(RAW_MAX));
        assert_eq!(accel_ug_to_raw(999_939), Err(OffsetError::ValueOutOfRange));
        assert_eq!(accel_ug_to_raw(-1_000_061), Ok(RAW_MIN));
        assert_eq!(
            accel_ug_to_raw(-1_000_062),
            Err(OffsetError::ValueOutOfRange)
        );
        assert_eq!(accel_ug_to_raw(i32::MAX), Err(OffsetError::ValueOutOfRange));
        assert_eq!(accel_ug_to_raw(i32::MIN), Err(OffsetError::ValueOutOfRange));
    }

    #[test]
    fn every_raw_value_round_trips_through_its_image_and_the_converters() {
        for raw in RAW_MIN..=RAW_MAX {
            let (lo, hi) = register_image(raw).unwrap();
            assert_eq!(hi & 0xC0, 0, "reserved bits stay 0");
            assert_eq!(from_register_image(lo, hi), raw);
            assert_eq!(
                from_register_image(lo, hi | 0xC0),
                raw,
                "reserved bits ignored"
            );
            assert_eq!(gyro_mdps_to_raw(gyro_raw_to_mdps(raw)), Ok(raw));
            assert_eq!(accel_ug_to_raw(accel_raw_to_ug(raw)), Ok(raw));
        }
    }

    #[test]
    fn converters_follow_the_measured_scale_and_round_half_away_from_zero() {
        // (raw, micro-g, millidegrees per second): raw * 15625 / 128 and raw * 15625 / 2048, measured
        // on the board, 2026-11-10 (+800 gave 97.6 to 98.1 mg, +1333 gave 10.16 to 10.18 dps).
        let cases = [
            (0, 0, 0),
            (1, 122, 8),
            (-1, -122, -8),
            (800, 97_656, 6_104),
            (-800, -97_656, -6_104),
            (1_333, 162_720, 10_170),
            (-1_333, -162_720, -10_170),
            (8_191, 999_878, 62_492),
            (-8_192, -1_000_000, -62_500),
        ];
        for (raw, ug, mdps) in cases {
            assert_eq!(accel_raw_to_ug(raw), ug, "accel {raw}");
            assert_eq!(gyro_raw_to_mdps(raw), mdps, "gyro {raw}");
            assert_eq!(accel_ug_to_raw(ug), Ok(raw), "accel {ug}");
            assert_eq!(gyro_mdps_to_raw(mdps), Ok(raw), "gyro {mdps}");
        }
        // True halves go away from zero: 64 * 15625 / 128 = 7812.5 and 1024 * 15625 / 2048 = 7812.5.
        assert_eq!(accel_raw_to_ug(64), 7_813);
        assert_eq!(accel_raw_to_ug(-64), -7_813);
        assert_eq!(gyro_raw_to_mdps(1_024), 7_813);
        assert_eq!(gyro_raw_to_mdps(-1_024), -7_813);
        // The inverses round to the nearest LSB: 61 micro-g is 0.4997 LSB, 62 is 0.508.
        assert_eq!(accel_ug_to_raw(61), Ok(0));
        assert_eq!(accel_ug_to_raw(62), Ok(1));
        assert_eq!(accel_ug_to_raw(-62), Ok(-1));
        assert_eq!(gyro_mdps_to_raw(3), Ok(0));
        assert_eq!(gyro_mdps_to_raw(4), Ok(1));
        assert_eq!(gyro_mdps_to_raw(-4), Ok(-1));
    }

    #[test]
    fn validate_names_the_first_axis_out_of_range() {
        assert_eq!(Offsets::new().validate(), Ok(()));
        assert_eq!(Offsets::default(), Offsets::new());
        let ok = Offsets::new()
            .with_gyro([RAW_MIN, RAW_MAX, 0])
            .with_accel([RAW_MAX, RAW_MIN, -1]);
        assert_eq!(ok.validate(), Ok(()));
        assert_eq!(
            ok.with_gyro([0, 0, 8192]).validate(),
            Err(OffsetError::AxisOutOfRange(OffsetAxis::GyroZ))
        );
        assert_eq!(
            ok.with_accel([0, -8193, 0]).validate(),
            Err(OffsetError::AxisOutOfRange(OffsetAxis::AccelY))
        );
    }

    #[test]
    fn registers_are_the_twelve_images_in_write_order() {
        let o = Offsets::new()
            .with_gyro([1, -1, RAW_MIN])
            .with_accel([RAW_MAX, 256, 0]);
        let got = o.registers();
        let want = [
            (0xA42A, 0x01, 0xFF),
            (0xA42B, 0x00, 0x3F),
            (0xA438, 0xFF, 0xFF),
            (0xA439, 0x3F, 0x3F),
            (0xA446, 0x00, 0xFF),
            (0xA447, 0x20, 0x3F),
            (0xA518, 0xFF, 0xFF),
            (0xA519, 0x1F, 0x3F),
            (0xA520, 0x00, 0xFF),
            (0xA521, 0x01, 0x3F),
            (0xA528, 0x00, 0xFF),
            (0xA529, 0x00, 0x3F),
        ];
        assert_eq!(got, want);
    }
}
