//! FIFO frame decoding and the unit conversions of FIFO data. No bus access.
//!
//! [`fifo_frames`] walks a caller buffer in place and yields one [`FifoFrame`] per frame. Every
//! input byte belongs to exactly one item, so nothing is skipped and a stream that is not what was
//! configured comes out as raw variants carrying the bytes. The first header byte fixes the frame
//! kind and so its size (5.1, p39; 5.2, p40), the configuration says which kind is expected, and
//! the iterator consumes the expected size for every item.
//!
//! Multi-byte fields are little endian, the device default for the FIFO (14, p57;
//! SREG_DATA_ENDIAN_SEL must stay 0). The figures of 5.1 (p39) are drawn for big endian. Frames
//! with an extended header (external sensors, compression; 5.2, p40) are out of scope and come out
//! as [`FifoFrame::Invalid`].

use crate::config::{FifoConfig, FifoFormat, TimestampResolution};

const G: f32 = 9.80665;
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

/// Header bit 7, EXT_HEADER (5.2, p40).
const HEADER_EXT: u8 = 0x80;
/// Header bits 6:4, ACCEL_EN, GYRO_EN, HIRES_EN (5.2, p40).
const KIND_SHIFT: u8 = 4;
const KIND_MASK: u8 = 0b111;
const KIND_ACCEL: u8 = 0b100;
const KIND_GYRO: u8 = 0b010;
const KIND_ACCEL_GYRO: u8 = 0b110;
/// ACCEL_EN and GYRO_EN both read 1 when high resolution is on (5.2, p40).
const KIND_HIRES: u8 = 0b111;
/// Header bit 3, TMST_FIELD_EN (5.2, p40).
const HEADER_TMST_FIELD: u8 = 0x08;
/// Header bit 2, FSYNC_TAG_EN (5.2, p40).
const HEADER_FSYNC_TAG: u8 = 0x04;
/// Header bit 1, ACCEL_ODR (5.2, p41).
const HEADER_ACCEL_ODR: u8 = 0x02;
/// Header bit 0, GYRO_ODR (5.2, p41).
const HEADER_GYRO_ODR: u8 = 0x01;

/// Accel-only and gyro-only frame size (5.1, p39).
pub const FRAME_BYTES_SINGLE: usize = 8;
/// Accel + gyro frame size (5.1, p39).
pub const FRAME_BYTES_ACCEL_GYRO: usize = 16;
/// High-resolution frame size (5.1, p39).
pub const FRAME_BYTES_HIRES: usize = 20;

/// Accel scale of the high-resolution format, whatever ACCEL_UI_FS_SEL is (5, p39): 16384 LSB/g.
pub const HIRES_ACCEL_LSB_PER_G: f32 = 16384.0;
/// Gyro scale of the high-resolution format, whatever GYRO_UI_FS_SEL is (5, p39): full scale
/// 4000 dps over 2^19 counts. The datasheet prints it rounded to 131.1 LSB/dps.
pub const HIRES_GYRO_LSB_PER_DPS: f32 = 524_288.0 / 4000.0;

/// Axis value that marks a sample the device could not provide, in the 16-bit formats: the most
/// negative 16-bit value, 0x8000 (vendor: TDK's Linux driver). The decoder still returns the value
/// as the number it is; [`AccelFrame::accel_invalid`] and the like report the marker. The datasheet
/// and the vendor facts define no marker for the 20-bit high-resolution format.
pub const INVALID_SAMPLE_I16: i16 = i16::MIN;

#[inline]
const fn all_invalid(axes: &[i16; 3]) -> bool {
    axes[0] == INVALID_SAMPLE_I16 && axes[1] == INVALID_SAMPLE_I16 && axes[2] == INVALID_SAMPLE_I16
}

/// Header bits that are not part of the frame kind.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoFlags {
    /// TMST_FIELD_EN, bit 3 (5.2, p40): the frame has a timestamp field.
    pub timestamp_field: bool,
    /// FSYNC_TAG_EN, bit 2 (5.2, p40): FSYNC fired and the timestamp field holds the FSYNC-to-ODR
    /// delay instead of the timestamp.
    pub fsync_tag: bool,
    /// ACCEL_ODR, bit 1 (5.2, p41): the accel rate differs from the previous accel packet.
    pub accel_odr_changed: bool,
    /// GYRO_ODR, bit 0 (5.2, p41): the gyro rate differs from the previous gyro packet.
    pub gyro_odr_changed: bool,
}

impl FifoFlags {
    // perf: part of the decode path measured at FifoFrames::next
    #[inline(always)]
    const fn from_header(header: u8) -> Self {
        Self {
            timestamp_field: header & HEADER_TMST_FIELD != 0,
            fsync_tag: header & HEADER_FSYNC_TAG != 0,
            accel_odr_changed: header & HEADER_ACCEL_ODR != 0,
            gyro_odr_changed: header & HEADER_GYRO_ODR != 0,
        }
    }
}

/// Accel-only frame, 8 bytes (5.1, p39).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelFrame {
    /// Header flags.
    pub flags: FifoFlags,
    /// Accel X, Y, Z in LSB of the configured full scale.
    pub accel: [i16; 3],
    /// Temperature, 8-bit FIFO encoding (see [`temperature_c_8bit`]).
    pub temp: i8,
}

impl AccelFrame {
    /// Whether all three accel axes carry [`INVALID_SAMPLE_I16`].
    pub const fn accel_invalid(&self) -> bool {
        all_invalid(&self.accel)
    }
}

/// Gyro-only frame, 8 bytes (5.1, p39).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroFrame {
    /// Header flags.
    pub flags: FifoFlags,
    /// Gyro X, Y, Z in LSB of the configured full scale.
    pub gyro: [i16; 3],
    /// Temperature, 8-bit FIFO encoding (see [`temperature_c_8bit`]).
    pub temp: i8,
}

impl GyroFrame {
    /// Whether all three gyro axes carry [`INVALID_SAMPLE_I16`].
    pub const fn gyro_invalid(&self) -> bool {
        all_invalid(&self.gyro)
    }
}

/// Accel + gyro frame, 16 bytes (5.1, p39).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelGyroFrame {
    /// Header flags.
    pub flags: FifoFlags,
    /// Accel X, Y, Z in LSB of the configured full scale.
    pub accel: [i16; 3],
    /// Gyro X, Y, Z in LSB of the configured full scale.
    pub gyro: [i16; 3],
    /// Temperature, 8-bit FIFO encoding (see [`temperature_c_8bit`]).
    pub temp: i8,
    /// Timestamp ticks of the device (a nominal 1 us or 16 us, about 0.44 % fast against real time
    /// even with the external clock; see [`timestamp_tick_us`]), or the FSYNC delay when
    /// `flags.fsync_tag`; 0 when the timestamp is off (FIFO_TMST_FSYNC_EN = 0, 16.33, p83).
    pub timestamp: u16,
}

impl AccelGyroFrame {
    /// Whether all three accel axes carry [`INVALID_SAMPLE_I16`].
    pub const fn accel_invalid(&self) -> bool {
        all_invalid(&self.accel)
    }

    /// Whether all three gyro axes carry [`INVALID_SAMPLE_I16`].
    pub const fn gyro_invalid(&self) -> bool {
        all_invalid(&self.gyro)
    }
}

/// High-resolution accel + gyro frame, 20 bytes (5 and 5.1, p39).
///
/// p39 says the lowest bit of the accel is always 0 in the 20-bit format. The decoder does not
/// change it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HighResFrame {
    /// Header flags.
    pub flags: FifoFlags,
    /// Accel X, Y, Z, 20-bit sign-extended; 16384 LSB/g (see [`hires_accel_ms2`]).
    pub accel: [i32; 3],
    /// Gyro X, Y, Z, 20-bit sign-extended; 131.072 LSB/dps (see [`hires_gyro_rads`]).
    pub gyro: [i32; 3],
    /// Temperature, 16-bit FIFO encoding (see [`temperature_c_16bit`]).
    pub temp: i16,
    /// Timestamp ticks of the device (a nominal 1 us or 16 us, about 0.44 % fast against real time
    /// even with the external clock; see [`timestamp_tick_us`]), or the FSYNC delay when
    /// `flags.fsync_tag`; 0 when the timestamp is off.
    pub timestamp: u16,
}

/// One item of the FIFO byte stream. The variants that carry bytes borrow them from the input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoFrame<'a> {
    /// Accel-only frame.
    Accel(AccelFrame),
    /// Gyro-only frame.
    Gyro(GyroFrame),
    /// Accel + gyro frame.
    AccelGyro(AccelGyroFrame),
    /// High-resolution frame.
    HighRes(HighResFrame),
    /// A frame whose header is not one of the four layouts of 5.1 (extended header, no sensor
    /// bits, inconsistent bits), or all the bytes when the configuration holds no sensor. The
    /// datasheet defines no marker for an empty FIFO slot, so empty data lands here too.
    Invalid(&'a [u8]),
    /// A frame with a valid header of another layout than the configured one.
    WrongFormat(&'a [u8]),
    /// Fewer bytes than one frame at the end of the input.
    Trailing(&'a [u8]),
}

impl FifoFrame<'_> {
    /// Number of input bytes this item stands for.
    pub const fn size_bytes(&self) -> usize {
        match self {
            Self::Accel(_) | Self::Gyro(_) => FRAME_BYTES_SINGLE,
            Self::AccelGyro(_) => FRAME_BYTES_ACCEL_GYRO,
            Self::HighRes(_) => FRAME_BYTES_HIRES,
            Self::Invalid(b) | Self::WrongFormat(b) | Self::Trailing(b) => b.len(),
        }
    }
}

/// Iterator over the frames of a FIFO read, see [`fifo_frames`].
#[derive(Clone, Debug)]
pub struct FifoFrames<'a> {
    rest: &'a [u8],
    expected: u8,
    /// Header bits that rule out the fast path of `next`: EXT_HEADER, and in the 8-byte formats
    /// the timestamp and FSYNC bits as well.
    reject: u8,
    size: usize,
}

/// Decodes `buf` in place against the frame layout `config` selects (`config.format`, `accel`,
/// `gyro`). Never skips a byte and never panics; the sizes of the items add up to `buf.len()`.
///
/// Every item but the last is as long as one configured frame. The last item is
/// [`FifoFrame::Trailing`] when fewer bytes remain. With no sensor configured every byte comes out
/// as one [`FifoFrame::Invalid`].
#[inline]
pub const fn fifo_frames<'a>(buf: &'a [u8], config: &FifoConfig) -> FifoFrames<'a> {
    let (expected, size) = match config.format {
        FifoFormat::HighResolution => (KIND_HIRES, FRAME_BYTES_HIRES),
        _ => match (config.accel, config.gyro) {
            (true, true) => (KIND_ACCEL_GYRO, FRAME_BYTES_ACCEL_GYRO),
            (true, false) => (KIND_ACCEL, FRAME_BYTES_SINGLE),
            (false, true) => (KIND_GYRO, FRAME_BYTES_SINGLE),
            (false, false) => (0, 0),
        },
    };
    let reject = if size == FRAME_BYTES_SINGLE {
        HEADER_EXT | HEADER_TMST_FIELD | HEADER_FSYNC_TAG
    } else {
        HEADER_EXT
    };
    FifoFrames {
        rest: buf,
        expected,
        reject,
        size,
    }
}

impl<'a> Iterator for FifoFrames<'a> {
    type Item = FifoFrame<'a>;

    // perf: ns per 8 / 16 / 20-byte frame through `fifo_frames` (MARV V2 Rev 1, RP2354B at 150 MHz,
    // 2026-11-11): 498 / 557 / 771 with `inline(always)` and the fast path; 839 / 866 / 1085 with
    // plain `#[inline]`; 1013 / 1272 / 1452 before either.
    #[inline(always)]
    fn next(&mut self) -> Option<FifoFrame<'a>> {
        let rest = self.rest;
        if rest.is_empty() {
            return None;
        }
        let split = if self.size == 0 {
            None
        } else {
            rest.split_at_checked(self.size)
        };
        let Some((frame, tail)) = split else {
            self.rest = &[];
            return Some(if self.size == 0 {
                FifoFrame::Invalid(rest)
            } else {
                FifoFrame::Trailing(rest)
            });
        };
        self.rest = tail;
        // Fast path: only headers with the expected kind and none of the `reject` bits (EXT_HEADER,
        // and in 8-byte frames TMST_FIELD_EN and FSYNC_TAG_EN) take it; `decode` would accept
        // exactly these. Every other header goes through `decode` unchanged.
        if let Some(&header) = frame.first()
            && header & self.reject == 0
            && (header >> KIND_SHIFT) & KIND_MASK == self.expected
        {
            let fast = match self.expected {
                KIND_ACCEL => <&[u8; FRAME_BYTES_SINGLE]>::try_from(frame)
                    .ok()
                    .map(|b| FifoFrame::Accel(decode_accel(header, b))),
                KIND_GYRO => <&[u8; FRAME_BYTES_SINGLE]>::try_from(frame)
                    .ok()
                    .map(|b| FifoFrame::Gyro(decode_gyro(header, b))),
                KIND_ACCEL_GYRO => <&[u8; FRAME_BYTES_ACCEL_GYRO]>::try_from(frame)
                    .ok()
                    .map(|b| FifoFrame::AccelGyro(decode_accel_gyro(header, b))),
                _ => <&[u8; FRAME_BYTES_HIRES]>::try_from(frame)
                    .ok()
                    .map(|b| FifoFrame::HighRes(decode_hires(header, b))),
            };
            if let Some(decoded) = fast {
                return Some(decoded);
            }
        }
        Some(decode(frame, self.expected))
    }
}

impl core::iter::FusedIterator for FifoFrames<'_> {}

#[inline(never)]
fn decode(frame: &[u8], expected: u8) -> FifoFrame<'_> {
    let Some(&header) = frame.first() else {
        return FifoFrame::Invalid(frame);
    };
    let kind = (header >> KIND_SHIFT) & KIND_MASK;
    let flags = FifoFlags::from_header(header);
    let single = matches!(kind, KIND_ACCEL | KIND_GYRO);
    let known = single || matches!(kind, KIND_ACCEL_GYRO | KIND_HIRES);
    // A timestamp field needs accel and gyro or high resolution (5.2, p40): not in 8-byte frames.
    if header & HEADER_EXT != 0 || !known || (single && (flags.timestamp_field || flags.fsync_tag))
    {
        return FifoFrame::Invalid(frame);
    }
    if kind != expected {
        return FifoFrame::WrongFormat(frame);
    }
    let decoded = match kind {
        KIND_ACCEL => <&[u8; FRAME_BYTES_SINGLE]>::try_from(frame)
            .ok()
            .map(|b| FifoFrame::Accel(decode_accel(header, b))),
        KIND_GYRO => <&[u8; FRAME_BYTES_SINGLE]>::try_from(frame)
            .ok()
            .map(|b| FifoFrame::Gyro(decode_gyro(header, b))),
        KIND_ACCEL_GYRO => <&[u8; FRAME_BYTES_ACCEL_GYRO]>::try_from(frame)
            .ok()
            .map(|b| FifoFrame::AccelGyro(decode_accel_gyro(header, b))),
        _ => <&[u8; FRAME_BYTES_HIRES]>::try_from(frame)
            .ok()
            .map(|b| FifoFrame::HighRes(decode_hires(header, b))),
    };
    decoded.unwrap_or(FifoFrame::Invalid(frame))
}

// perf: part of the decode path measured at FifoFrames::next
#[inline(always)]
const fn word(lo: u8, hi: u8) -> i16 {
    i16::from_le_bytes([lo, hi])
}

// perf: part of the decode path measured at FifoFrames::next
#[inline(always)]
const fn decode_accel(header: u8, b: &[u8; FRAME_BYTES_SINGLE]) -> AccelFrame {
    let [_, ax0, ax1, ay0, ay1, az0, az1, t] = *b;
    AccelFrame {
        flags: FifoFlags::from_header(header),
        accel: [word(ax0, ax1), word(ay0, ay1), word(az0, az1)],
        temp: i8::from_le_bytes([t]),
    }
}

// perf: part of the decode path measured at FifoFrames::next
#[inline(always)]
const fn decode_gyro(header: u8, b: &[u8; FRAME_BYTES_SINGLE]) -> GyroFrame {
    let [_, gx0, gx1, gy0, gy1, gz0, gz1, t] = *b;
    GyroFrame {
        flags: FifoFlags::from_header(header),
        gyro: [word(gx0, gx1), word(gy0, gy1), word(gz0, gz1)],
        temp: i8::from_le_bytes([t]),
    }
}

// perf: part of the decode path measured at FifoFrames::next
#[inline(always)]
const fn decode_accel_gyro(header: u8, b: &[u8; FRAME_BYTES_ACCEL_GYRO]) -> AccelGyroFrame {
    let [
        _,
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
        t,
        ts0,
        ts1,
    ] = *b;
    AccelGyroFrame {
        flags: FifoFlags::from_header(header),
        accel: [word(ax0, ax1), word(ay0, ay1), word(az0, az1)],
        gyro: [word(gx0, gx1), word(gy0, gy1), word(gz0, gz1)],
        temp: i8::from_le_bytes([t]),
        timestamp: u16::from_le_bytes([ts0, ts1]),
    }
}

/// 20-bit value: the 16 upper bits from the word, the 4 lower bits from a nibble (5.1, p39).
// perf: part of the decode path measured at FifoFrames::next
#[inline(always)]
const fn wide(lo: u8, hi: u8, nibble: u8) -> i32 {
    ((word(lo, hi) as i32) << 4) | (nibble & 0x0F) as i32
}

// perf: part of the decode path measured at FifoFrames::next
#[inline(always)]
const fn decode_hires(header: u8, b: &[u8; FRAME_BYTES_HIRES]) -> HighResFrame {
    let [
        _,
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
        ts0,
        ts1,
        lx,
        ly,
        lz,
    ] = *b;
    // Byte 17 to 19 hold "Ax LSB | Gx LSB" etc. (5.1, p39). The placement (accel nibble in bits 7:4,
    // gyro nibble in bits 3:0) was taken from the figure on p39 and is confirmed on the board,
    // 2026-11-10 (in 321 frames the accel's lowest bit was never set, the gyro's about half the time).
    HighResFrame {
        flags: FifoFlags::from_header(header),
        accel: [
            wide(ax0, ax1, lx >> 4),
            wide(ay0, ay1, ly >> 4),
            wide(az0, az1, lz >> 4),
        ],
        gyro: [wide(gx0, gx1, lx), wide(gy0, gy1, ly), wide(gz0, gz1, lz)],
        temp: word(t0, t1),
        timestamp: u16::from_le_bytes([ts0, ts1]),
    }
}

/// Temperature in degrees Celsius from the 8-bit FIFO encoding: raw / 2 + 25 (16.14, p69;
/// 2 LSB/degC, electrical characteristics, p21). The raw byte is read as signed.
pub fn temperature_c_8bit(raw: i8) -> f32 {
    f32::from(raw) / 2.0 + 25.0
}

/// Temperature in degrees Celsius from the 16-bit FIFO encoding of the high-resolution frame:
/// raw / 128 + 25, the scale of TEMP_DATA (16.14, p69). The datasheet states no scale for the
/// 2-byte FIFO temperature; this is the 16-bit register scale, confirmed on the board, 2026-11-10
/// (within 0.3 C of the register; the 8-bit encoding, raw / 2 + 25, also).
pub fn temperature_c_16bit(raw: i16) -> f32 {
    f32::from(raw) / 128.0 + 25.0
}

/// Acceleration in m/s^2 from high-resolution accel counts (16384 LSB/g, 5, p39).
pub fn hires_accel_ms2(raw: [i32; 3]) -> [f32; 3] {
    let k = G / HIRES_ACCEL_LSB_PER_G;
    raw.map(|v| v as f32 * k)
}

/// Angular rate in rad/s from high-resolution gyro counts (131.072 LSB/dps, 5, p39).
pub fn hires_gyro_rads(raw: [i32; 3]) -> [f32; 3] {
    let k = DEG_TO_RAD / HIRES_GYRO_LSB_PER_DPS;
    raw.map(|v| v as f32 * k)
}

/// Nominal microseconds per tick, TMST_RESOL (16.34, p84). The tick is not real time. With the
/// internal oscillator it was 0.45 % fast on the test board (measured 2026-11-10: frames 1250 ticks
/// apart at 800 Hz while the wall-clock period was 1244.4 us). With the external 32 kHz clock on
/// CLKIN it was still about 0.44 % fast (measured on the MARV V2 board, 2026-11-11: frames at 200 Hz
/// 5022.5 us apart in timestamp units, at 1 us and at 16 us per tick, while the MCU clock measured
/// 5000.5 us per frame and the output rate on that clock was exact), so the tick does not follow the
/// external clock. Timestamps are for spacing frames relative to each other; an application that
/// needs real time calibrates the tick period against its own clock and converts with
/// [`timestamp_us_with_period`].
pub const fn timestamp_tick_us(resolution: TimestampResolution) -> u32 {
    match resolution {
        TimestampResolution::Us1 => 1,
        TimestampResolution::Us16 => 16,
    }
}

/// Timestamp ticks to nominal microseconds, in integers (see [`timestamp_tick_us`]: this is not
/// exactly real time, the tick runs about 0.44 % fast). The 16-bit field wraps after 65536 ticks
/// (65.5 ms at 1 us, 1.05 s at 16 us); the largest result is 1 048 560.
pub const fn timestamp_us(ticks: u16, resolution: TimestampResolution) -> u32 {
    ticks as u32 * timestamp_tick_us(resolution)
}

/// Timestamp ticks to microseconds with a tick period the caller supplies, the way to apply a
/// calibration (for example 0.9956 us per tick measured against the application's own clock instead
/// of the nominal 1 us). The field is 16 bits wide and wraps: take the difference of two timestamps
/// with `later.wrapping_sub(earlier)` first, then convert it.
pub const fn timestamp_us_with_period(ticks: u16, tick_us: f32) -> f32 {
    ticks as f32 * tick_us
}

#[cfg(test)]
mod tests;
