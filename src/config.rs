//! Typed configuration. Pure code, no bus: [`Config`] and its settings structs, the datasheet tables
//! as enums, and [`Config::validate`] (in [`crate::validate`]). The register images and the write
//! plan computed from a [`Config`] are in [`crate::image`] and [`crate::plan`].
//!
//! Every settings struct is `#[non_exhaustive]` with public fields, `Default`, a `const fn new()`
//! equal to the default and a `const fn with_<field>(self, value) -> Self` per field (contract F3).
//! Outside the crate, build one with `Config::new().with_accel_fs(..)`. Enums that copy a complete
//! datasheet table are exhaustive; the others are `#[non_exhaustive]`.

use crate::reg;

/// Accelerometer full-scale range, ACCEL_UI_FS_SEL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelFullScale {
    /// +-32 g.
    G32 = 0,
    /// +-16 g.
    G16 = 1,
    /// +-8 g.
    G8 = 2,
    /// +-4 g.
    G4 = 3,
    /// +-2 g.
    G2 = 4,
}

impl AccelFullScale {
    /// ACCEL_UI_FS_SEL code.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Full-scale range in g.
    pub const fn range_g(self) -> f32 {
        match self {
            Self::G32 => 32.0,
            Self::G16 => 16.0,
            Self::G8 => 8.0,
            Self::G4 => 4.0,
            Self::G2 => 2.0,
        }
    }

    /// Sensitivity in LSB per g (1024 at +-32 g up to 16384 at +-2 g).
    pub const fn lsb_per_g(self) -> f32 {
        32768.0 / self.range_g()
    }
}

/// Gyroscope full-scale range, GYRO_UI_FS_SEL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GyroFullScale {
    /// +-4000 dps.
    Dps4000 = 0,
    /// +-2000 dps.
    Dps2000 = 1,
    /// +-1000 dps.
    Dps1000 = 2,
    /// +-500 dps.
    Dps500 = 3,
    /// +-250 dps.
    Dps250 = 4,
    /// +-125 dps.
    Dps125 = 5,
    /// +-62.5 dps.
    Dps62_5 = 6,
    /// +-31.25 dps.
    Dps31_25 = 7,
    /// +-15.625 dps.
    Dps15_625 = 8,
}

impl GyroFullScale {
    /// GYRO_UI_FS_SEL code.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Full-scale range in degrees per second.
    pub const fn range_dps(self) -> f32 {
        match self {
            Self::Dps4000 => 4000.0,
            Self::Dps2000 => 2000.0,
            Self::Dps1000 => 1000.0,
            Self::Dps500 => 500.0,
            Self::Dps250 => 250.0,
            Self::Dps125 => 125.0,
            Self::Dps62_5 => 62.5,
            Self::Dps31_25 => 31.25,
            Self::Dps15_625 => 15.625,
        }
    }

    /// Sensitivity in LSB per degree per second (131.07 at +-250 dps).
    pub const fn lsb_per_dps(self) -> f32 {
        32768.0 / self.range_dps()
    }
}

/// Output data rate, ACCEL_ODR / GYRO_ODR. The device runs in low-noise mode, which supports
/// 12.5 Hz and above; the three lowest rates are low-power-only and [`Config::validate`] rejects them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Odr {
    /// 6.4 kHz (low noise only).
    Hz6400 = 0x3,
    /// 3.2 kHz (low noise only).
    Hz3200 = 0x4,
    /// 1.6 kHz (low noise only).
    Hz1600 = 0x5,
    /// 800 Hz (low noise only).
    Hz800 = 0x6,
    /// 400 Hz.
    Hz400 = 0x7,
    /// 200 Hz.
    Hz200 = 0x8,
    /// 100 Hz.
    Hz100 = 0x9,
    /// 50 Hz.
    Hz50 = 0xA,
    /// 25 Hz.
    Hz25 = 0xB,
    /// 12.5 Hz.
    Hz12_5 = 0xC,
    /// 6.25 Hz (low power only).
    Hz6_25 = 0xD,
    /// 3.125 Hz (low power only).
    Hz3_125 = 0xE,
    /// 1.5625 Hz (low power only).
    Hz1_5625 = 0xF,
}

impl Odr {
    /// ODR code, bits 3:0 of ACCEL_CONFIG0 / GYRO_CONFIG0.
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Nominal rate in hertz (CLKIN 32 kHz scales it exactly; the internal oscillator is about 0.4 % fast).
    pub const fn hz(self) -> f32 {
        match self {
            Self::Hz6400 => 6400.0,
            Self::Hz3200 => 3200.0,
            Self::Hz1600 => 1600.0,
            Self::Hz800 => 800.0,
            Self::Hz400 => 400.0,
            Self::Hz200 => 200.0,
            Self::Hz100 => 100.0,
            Self::Hz50 => 50.0,
            Self::Hz25 => 25.0,
            Self::Hz12_5 => 12.5,
            Self::Hz6_25 => 6.25,
            Self::Hz3_125 => 3.125,
            Self::Hz1_5625 => 1.5625,
        }
    }

    /// Base-2 logarithm of 6400 Hz over this rate: 0 at 6.4 kHz, 12 at 1.5625 Hz. Each step halves the
    /// rate, so two rates are equal exactly when their shifts are.
    pub const fn rate_shift(self) -> u8 {
        self.code() - Self::Hz6400.code()
    }

    /// Whether low-noise mode supports this rate.
    pub const fn low_noise(self) -> bool {
        self.code() <= Self::Hz12_5.code()
    }
}

/// Power state of one sensor, the mode field of PWR_MGMT0 (16.17, p70). Low-power mode (code 2) is
/// outside this crate's scope.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SensorPower {
    /// Sensor off (code 0).
    Off = 0,
    /// Standby (code 1): gyro drive on, no data. The accelerometer has no standby: its code 1 is
    /// off, so [`Config::validate`] rejects this for the accelerometer.
    Standby = 1,
    /// Low-noise mode (code 3).
    LowNoise = 3,
}

impl SensorPower {
    /// GYRO_MODE / ACCEL_MODE code.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Bandwidth of the UI path low-pass filter, GYRO_UI_LPFBW_SEL / ACCEL_UI_LPFBW_SEL (20.11, p192;
/// 21.10, p196). Code 7 repeats ODR/128 and is not offered. With the anti-alias FIR on, the signal
/// bandwidth is set by both filters (application note AN-000365, not in the datasheet text).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UiBandwidth {
    /// Low-pass filter bypassed (code 0).
    Bypass = 0,
    /// ODR/4 (code 1).
    OdrDiv4 = 1,
    /// ODR/8 (code 2).
    OdrDiv8 = 2,
    /// ODR/16 (code 3).
    OdrDiv16 = 3,
    /// ODR/32 (code 4).
    OdrDiv32 = 4,
    /// ODR/64 (code 5).
    OdrDiv64 = 5,
    /// ODR/128 (code 6).
    OdrDiv128 = 6,
}

impl UiBandwidth {
    /// LPFBW_SEL code.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Anti-alias FIR filter and interpolator of one sensor, GYRO_SRC_CTRL / ACCEL_SRC_CTRL (20.7,
/// p190; 21.7, p194). Code 3 is reserved (debug mode) and not offered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AntiAlias {
    /// Interpolator and FIR filter off (code 0, the reset value).
    Off = 0,
    /// Interpolator off, FIR filter on (code 1).
    Fir = 1,
    /// Interpolator and FIR filter on (code 2).
    InterpolatorFir = 2,
}

impl AntiAlias {
    /// SRC_CTRL code.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Source of the sensor clock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ClockSource {
    /// Internal oscillator.
    Internal,
    /// External 32 kHz clock on pin 9 (CLKIN), 20 kHz to 40 kHz accepted (4.14, p37). Pin 9 is
    /// shared with FSYNC (Table 9, p28); this crate does not offer FSYNC.
    ExternalClkin,
}

/// INT1 interrupt mode, INT1_MODE (16.23, p75).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Int1Mode {
    /// Pulse of the configured width (0).
    Pulse = 0,
    /// Level held until the status register is read (1).
    Latched = 1,
}

/// INT1 polarity, INT1_POLARITY (16.23, p75).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Int1Polarity {
    /// Active low (0).
    ActiveLow = 0,
    /// Active high (1).
    ActiveHigh = 1,
}

/// INT1 output stage, INT1_DRIVE (16.23, p75).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Int1Drive {
    /// Push-pull (0).
    PushPull = 0,
    /// Open-drain (1).
    OpenDrain = 1,
}

/// Minimum INT1 pulse width in pulse mode, INT1_TPULSE_DURATION (19.53, p181).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PulseWidth {
    /// 100 us (code 0): only if both output data rates are below 4 kHz.
    Us100 = 0,
    /// 8 us (code 1, the reset value): required if an output data rate is 4 kHz or more.
    Us8 = 1,
}

impl PulseWidth {
    /// INT1_TPULSE_DURATION code.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// INT1 pin: which events drive it and how (16.21 to 16.23, p72-p75; 19.53, p181).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Int1Config {
    /// New sample in the data registers (INT1_STATUS_EN_DRDY).
    pub data_ready: bool,
    /// FIFO count reached the watermark (INT1_STATUS_EN_FIFO_THS).
    pub fifo_threshold: bool,
    /// FIFO full (INT1_STATUS_EN_FIFO_FULL). Enable it if the application wants the
    /// `fifo_full` flag of [`InterruptStatus`](crate::InterruptStatus): with only the threshold
    /// source enabled the flag stays 0 (measured on the board, 2026-11-10). The driver's own
    /// overflow detection in `fifo_read` uses the count and does not need it.
    pub fifo_full: bool,
    /// UI AGC ready (INT1_STATUS_EN_AP_AGC_RDY).
    pub agc_ready: bool,
    /// Pulse or latched.
    pub mode: Int1Mode,
    /// Polarity.
    pub polarity: Int1Polarity,
    /// Output stage.
    pub drive: Int1Drive,
    /// Pulse width in pulse mode.
    pub pulse_width: PulseWidth,
}

impl Int1Config {
    /// No event enabled; pulse, active high, push-pull (what `init` configures today), 8 us (the
    /// device's reset value).
    pub const fn new() -> Self {
        Self {
            data_ready: false,
            fifo_threshold: false,
            fifo_full: false,
            agc_ready: false,
            mode: Int1Mode::Pulse,
            polarity: Int1Polarity::ActiveHigh,
            drive: Int1Drive::PushPull,
            pulse_width: PulseWidth::Us8,
        }
    }

    /// Whether an event that comes from the FIFO is enabled.
    pub const fn uses_fifo(&self) -> bool {
        self.fifo_threshold || self.fifo_full
    }
}

impl Default for Int1Config {
    fn default() -> Self {
        Self::new()
    }
}

/// FIFO operating mode, FIFO_MODE (16.28, p80).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoMode {
    /// Bypass: the FIFO is disabled (code 0).
    Off = 0,
    /// Stream: the oldest frames are overwritten when the FIFO is full (code 1).
    Stream = 1,
    /// Stop on full: new frames are dropped when the FIFO is full (code 2).
    StopOnFull = 2,
}

impl FifoMode {
    /// FIFO_MODE code.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// FIFO size, FIFO_DEPTH (16.28, p80).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoDepth {
    /// 2 KB, the recommended setting (code 000111).
    Kib2,
    /// 8 KB, valid when all APEX features are disabled (code 011111).
    Kib8,
}

impl FifoDepth {
    /// FIFO_DEPTH code.
    pub const fn code(self) -> u8 {
        match self {
            Self::Kib2 => reg::FIFO_DEPTH_2K,
            Self::Kib8 => reg::FIFO_DEPTH_8K,
        }
    }

    /// Size in bytes.
    pub const fn bytes(self) -> u16 {
        match self {
            Self::Kib2 => 2048,
            Self::Kib8 => 8192,
        }
    }
}

/// FIFO frame format (5.1, p39). The size follows from the format and the sensors selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum FifoFormat {
    /// 16-bit data: 8 bytes with one sensor, 16 bytes with both.
    Standard,
    /// 20-bit data, 20 bytes, accel and gyro together, scaled to +-32 g and +-4000 dps whatever the
    /// full scale (5, p39; FIFO_HIRES_EN, 16.32, p82).
    HighResolution,
}

/// Timestamp resolution, TMST_RESOL (16.34, p84).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimestampResolution {
    /// 1 us per LSB (0).
    Us1 = 0,
    /// 16 us per LSB (1).
    Us16 = 1,
}

/// FIFO decimation factor of one sensor, GYRO_FIFO_ODR_DEC / ACCEL_FIFO_ODR_DEC (16.39, p87).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Decimation {
    /// 1, the FIFO gets every sample (code 0).
    Div1 = 0,
    /// 2 (code 1).
    Div2 = 1,
    /// 4 (code 2).
    Div4 = 2,
    /// 8 (code 3).
    Div8 = 3,
    /// 16 (code 4).
    Div16 = 4,
    /// 32 (code 5).
    Div32 = 5,
    /// 64 (code 6).
    Div64 = 6,
    /// 128 (code 7).
    Div128 = 7,
    /// 256 (code 8).
    Div256 = 8,
    /// 512 (code 9).
    Div512 = 9,
    /// 1024 (code 10).
    Div1024 = 10,
    /// 2048 (code 11).
    Div2048 = 11,
    /// 4096 (code 12).
    Div4096 = 12,
}

impl Decimation {
    /// Decimation code, the base-2 logarithm of the factor.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// FIFO settings (16.28 to 16.34, p80-p84; 16.39, p87).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FifoConfig {
    /// Off, stream or stop on full.
    pub mode: FifoMode,
    /// Frame format.
    pub format: FifoFormat,
    /// Accel data in the frame.
    pub accel: bool,
    /// Gyro data in the frame.
    pub gyro: bool,
    /// Timestamp field in the frame (FIFO_TMST_FSYNC_EN). The field exists only in 16-byte and
    /// 20-byte frames (5.2, p40).
    pub timestamp: bool,
    /// Timestamp resolution.
    pub timestamp_resolution: TimestampResolution,
    /// Watermark in frames; 0 disables it. For N >= 1 the driver writes N + 1 to FIFO_WM_TH
    /// (vendor erratum AN-000364: never read the FIFO empty in stream mode), so reading N frames
    /// when the threshold interrupt fires always leaves one frame in the FIFO (confirmed on the
    /// board, 2026-11-10: exactly one frame left in 200 batches; the erratum itself did not
    /// reproduce there, and the driver keeps it). `validate` accepts N only when N + 1 frames fit
    /// the FIFO, which was valid whether the device counts frames or bytes (the datasheet does not
    /// settle the unit of FIFO_WM_TH; the board showed frames).
    pub watermark: u16,
    /// FIFO size.
    pub depth: FifoDepth,
    /// Decimation of the accel data written to the FIFO.
    pub accel_decimation: Decimation,
    /// Decimation of the gyro data written to the FIFO.
    pub gyro_decimation: Decimation,
}

impl FifoConfig {
    /// FIFO off; when switched on it holds accel and gyro in 16-byte frames without timestamp. Off
    /// is the device's reset state.
    pub const fn new() -> Self {
        Self {
            mode: FifoMode::Off,
            format: FifoFormat::Standard,
            accel: true,
            gyro: true,
            timestamp: false,
            timestamp_resolution: TimestampResolution::Us1,
            watermark: 0,
            depth: FifoDepth::Kib2,
            accel_decimation: Decimation::Div1,
            gyro_decimation: Decimation::Div1,
        }
    }

    /// Size of one frame in bytes (8, 16 or 20), or 0 when the frame holds no sensor.
    pub const fn frame_bytes(&self) -> u8 {
        match self.format {
            FifoFormat::HighResolution => 20,
            FifoFormat::Standard => match (self.accel, self.gyro) {
                (true, true) => 16,
                (false, false) => 0,
                _ => 8,
            },
        }
    }

    /// Whether the FIFO is on.
    pub const fn enabled(&self) -> bool {
        !matches!(self.mode, FifoMode::Off)
    }
}

impl Default for FifoConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Slew rate of the SPI pads, PADS_SPI_SLEW (16.48, p93). Names give the typical edge time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiSlew {
    /// 38 ns typical, 12 to 106 ns (000).
    Ns38 = 0,
    /// 14 ns typical, 4 to 45 ns (001).
    Ns14 = 1,
    /// 10 ns typical, 3 to 37 ns (010).
    Ns10 = 2,
    /// 7 ns typical, 2 to 25 ns (011).
    Ns7 = 3,
    /// 5 ns typical, 1 to 17 ns (100).
    Ns5 = 4,
    /// 4 ns typical, 1 to 14 ns (101).
    Ns4 = 5,
    /// 0.5 ns typical, 0.1 to 6 ns (11x; written as 110, the reset value).
    Ns0_5 = 6,
}

impl SpiSlew {
    /// PADS_SPI_SLEW code.
    pub const fn code(self) -> u8 {
        self as u8
    }
}

/// Driver configuration for `init` and `configure`.
///
/// Build one with `Config::new()` and the `with_*` setters. The default reproduces what `init`
/// configures today for the original fields (+-4 g, +-2000 dps, 800 Hz, both sensors in low-noise
/// mode, internal clock, INT1 push-pull pulse, active high, no event) and the device's reset values
/// for the others. Check a configuration with [`Config::validate`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Accelerometer full scale.
    pub accel_fs: AccelFullScale,
    /// Gyroscope full scale.
    pub gyro_fs: GyroFullScale,
    /// Accelerometer output data rate.
    pub accel_odr: Odr,
    /// Gyroscope output data rate.
    pub gyro_odr: Odr,
    /// Accelerometer power state.
    pub accel_power: SensorPower,
    /// Gyroscope power state.
    pub gyro_power: SensorPower,
    /// Accelerometer UI low-pass bandwidth.
    pub accel_bandwidth: UiBandwidth,
    /// Gyroscope UI low-pass bandwidth.
    pub gyro_bandwidth: UiBandwidth,
    /// Accelerometer anti-alias FIR and interpolator. `None` leaves the choice to the clock: off on
    /// the internal clock (the reset value), interpolator and FIR on CLKIN (what the vendor driver's
    /// CLKIN sequence writes, and what `init` does today).
    pub accel_anti_alias: Option<AntiAlias>,
    /// Gyroscope anti-alias FIR and interpolator; `None` as for the accelerometer.
    pub gyro_anti_alias: Option<AntiAlias>,
    /// Sensor clock source.
    pub clock: ClockSource,
    /// Send the RTC re-align command after the external clock is switched on (RTC_ALIGN).
    pub rtc_align: bool,
    /// INT1 pin.
    pub int1: Int1Config,
    /// FIFO.
    pub fifo: FifoConfig,
    /// SPI pad slew rate.
    pub spi_slew: SpiSlew,
}

impl Config {
    /// The default configuration.
    pub const fn new() -> Self {
        Self {
            accel_fs: AccelFullScale::G4,
            gyro_fs: GyroFullScale::Dps2000,
            accel_odr: Odr::Hz800,
            gyro_odr: Odr::Hz800,
            accel_power: SensorPower::LowNoise,
            gyro_power: SensorPower::LowNoise,
            accel_bandwidth: UiBandwidth::Bypass,
            gyro_bandwidth: UiBandwidth::Bypass,
            accel_anti_alias: None,
            gyro_anti_alias: None,
            clock: ClockSource::Internal,
            rtc_align: false,
            int1: Int1Config::new(),
            fifo: FifoConfig::new(),
            spi_slew: SpiSlew::Ns0_5,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

macro_rules! setters {
    ($($with:ident, $field:ident: $ty:ty;)*) => {
        $(
            #[doc = concat!("Returns the configuration with `", stringify!($field), "` replaced.")]
            #[must_use]
            pub const fn $with(mut self, value: $ty) -> Self {
                self.$field = value;
                self
            }
        )*
    };
}

impl Config {
    setters! {
        with_accel_fs, accel_fs: AccelFullScale;
        with_gyro_fs, gyro_fs: GyroFullScale;
        with_accel_odr, accel_odr: Odr;
        with_gyro_odr, gyro_odr: Odr;
        with_accel_power, accel_power: SensorPower;
        with_gyro_power, gyro_power: SensorPower;
        with_accel_bandwidth, accel_bandwidth: UiBandwidth;
        with_gyro_bandwidth, gyro_bandwidth: UiBandwidth;
        with_accel_anti_alias, accel_anti_alias: Option<AntiAlias>;
        with_gyro_anti_alias, gyro_anti_alias: Option<AntiAlias>;
        with_clock, clock: ClockSource;
        with_rtc_align, rtc_align: bool;
        with_int1, int1: Int1Config;
        with_fifo, fifo: FifoConfig;
        with_spi_slew, spi_slew: SpiSlew;
    }
}

impl Int1Config {
    setters! {
        with_data_ready, data_ready: bool;
        with_fifo_threshold, fifo_threshold: bool;
        with_fifo_full, fifo_full: bool;
        with_agc_ready, agc_ready: bool;
        with_mode, mode: Int1Mode;
        with_polarity, polarity: Int1Polarity;
        with_drive, drive: Int1Drive;
        with_pulse_width, pulse_width: PulseWidth;
    }
}

impl FifoConfig {
    setters! {
        with_mode, mode: FifoMode;
        with_format, format: FifoFormat;
        with_accel, accel: bool;
        with_gyro, gyro: bool;
        with_timestamp, timestamp: bool;
        with_timestamp_resolution, timestamp_resolution: TimestampResolution;
        with_watermark, watermark: u16;
        with_depth, depth: FifoDepth;
        with_accel_decimation, accel_decimation: Decimation;
        with_gyro_decimation, gyro_decimation: Decimation;
    }
}

/// ACCEL_CONFIG0 value.
pub const fn accel_config0(fs: AccelFullScale, odr: Odr) -> u8 {
    (fs.code() << 4) | odr.code()
}

/// GYRO_CONFIG0 value.
pub const fn gyro_config0(fs: GyroFullScale, odr: Odr) -> u8 {
    (fs.code() << 4) | odr.code()
}

/// INT1_CONFIG0 value: data-ready enable only (reset-done reporting off).
pub const fn int1_config0(data_ready: bool) -> u8 {
    if data_ready { reg::DRDY } else { 0 }
}
