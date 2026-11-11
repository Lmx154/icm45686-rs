//! Validation of a [`Config`]: the combinations the datasheet rules out, found before any bus
//! access. Each rule cites the datasheet section and printed page; a rule the datasheet does not
//! state but the safer reading needs says so.

use crate::config::{
    ClockSource, Config, FifoConfig, FifoFormat, Int1Mode, Odr, PulseWidth, SensorPower,
};

/// Why a [`Config`] cannot be applied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The accelerometer has no standby mode: ACCEL_MODE code 1 turns it off (16.17, p70).
    AccelStandby,
    /// The accelerometer rate is 6.25 Hz or lower, which only low-power mode supports; this crate
    /// runs low-noise mode only (16.26, p78).
    AccelOdrLowPowerOnly,
    /// The gyroscope rate is 6.25 Hz or lower, which only low-power mode supports (16.27, p79).
    GyroOdrLowPowerOnly,
    /// The FIFO is on but its frame holds neither accel nor gyro data (5.1, p39).
    FifoNoSensors,
    /// High-resolution frames carry accel and gyro together, so both must be selected (5.1 and 5.2,
    /// p39-p40: the 20-byte frame, ACCEL_EN and GYRO_EN set whenever high resolution is on).
    FifoHighResNeedsBothSensors,
    /// The FIFO frame holds accel data but the accelerometer is not in low-noise mode, so no data
    /// would arrive (16.17, p70; 16.32, p82).
    FifoAccelNotRunning,
    /// The FIFO frame holds gyro data but the gyroscope is not in low-noise mode.
    FifoGyroNotRunning,
    /// The timestamp field exists only with high resolution or with both accel and gyro in the
    /// frame (5.2, TMST_FIELD_EN, p40); the 8-byte frame has none.
    FifoTimestampNeedsBothSensors,
    /// The watermark N plus one is larger than the number of frames the FIFO can hold (16.29, p80;
    /// 16.28, p80): the driver programs N + 1 (vendor erratum, see [`crate::image::fifo_watermark`]),
    /// so the largest accepted N is the capacity minus one. The datasheet does not say whether the threshold counts frames or bytes (16.18, p71
    /// against 5, p39), so only values valid under both readings are accepted.
    FifoWatermarkTooLarge,
    /// Accel and gyro share a frame but arrive at different rates after decimation. The datasheet
    /// flags an ODR change in the frame header (5.2, p41) and does not define frames with unequal
    /// rates, so the safer reading rejects them.
    FifoRateMismatch,
    /// An INT1 FIFO event is enabled while the FIFO is off: in bypass mode no frame is stored
    /// (16.28, p80), so the event could never fire. The datasheet does not forbid it; this rule is
    /// the safer reading.
    Int1FifoEventWithFifoOff,
    /// The INT1 FIFO threshold event is enabled while the watermark is 0, which disables it (16.29,
    /// p80).
    Int1ThresholdWithoutWatermark,
    /// RTC_ALIGN is set but the clock is internal; the RTC is not running (16.37, p86).
    RtcAlignWithoutClkin,
    /// The 100 us INT1 pulse is "only if ODR < 4 kHz" and 8 us is "required if ODR >= 4 kHz"
    /// (19.53, p181), but an output data rate is 6.4 kHz.
    Int1LongPulseAtHighOdr,
    /// The write plan has more steps than [`crate::plan::PLAN_CAPACITY`]. No valid configuration
    /// reaches it (a test checks the largest); the error exists so that a step is never dropped.
    PlanOverflow,
}

impl FifoConfig {
    /// Frames the FIFO holds: its size over the frame size, 0 when the frame has no sensor.
    pub const fn capacity_frames(&self) -> u16 {
        match self.frame_bytes() {
            0 => 0,
            n => self.depth.bytes() / n as u16,
        }
    }
}

const fn runs(power: SensorPower) -> bool {
    matches!(power, SensorPower::LowNoise)
}

impl Config {
    /// Checks the whole configuration. `Ok` means every combination is one the datasheet allows
    /// (within the safer readings documented on [`ConfigError`]).
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.accel_power == SensorPower::Standby {
            return Err(ConfigError::AccelStandby);
        }
        if !self.accel_odr.low_noise() {
            return Err(ConfigError::AccelOdrLowPowerOnly);
        }
        if !self.gyro_odr.low_noise() {
            return Err(ConfigError::GyroOdrLowPowerOnly);
        }
        self.validate_fifo()?;
        self.validate_pins()?;
        Ok(())
    }

    fn validate_fifo(&self) -> Result<(), ConfigError> {
        let fifo = &self.fifo;
        let high_res = fifo.format == FifoFormat::HighResolution;
        if fifo.enabled() {
            if high_res && !(fifo.accel && fifo.gyro) {
                return Err(ConfigError::FifoHighResNeedsBothSensors);
            }
            if fifo.frame_bytes() == 0 {
                return Err(ConfigError::FifoNoSensors);
            }
            if fifo.accel && !runs(self.accel_power) {
                return Err(ConfigError::FifoAccelNotRunning);
            }
            if fifo.gyro && !runs(self.gyro_power) {
                return Err(ConfigError::FifoGyroNotRunning);
            }
            if fifo.timestamp && fifo.frame_bytes() == 8 {
                return Err(ConfigError::FifoTimestampNeedsBothSensors);
            }
            if fifo.watermark >= fifo.capacity_frames() {
                return Err(ConfigError::FifoWatermarkTooLarge);
            }
            if fifo.accel
                && fifo.gyro
                && self.accel_odr.rate_shift() + fifo.accel_decimation.code()
                    != self.gyro_odr.rate_shift() + fifo.gyro_decimation.code()
            {
                return Err(ConfigError::FifoRateMismatch);
            }
        }
        if self.int1.uses_fifo() && !fifo.enabled() {
            return Err(ConfigError::Int1FifoEventWithFifoOff);
        }
        if self.int1.fifo_threshold && fifo.watermark == 0 {
            return Err(ConfigError::Int1ThresholdWithoutWatermark);
        }
        Ok(())
    }

    fn validate_pins(&self) -> Result<(), ConfigError> {
        if self.rtc_align && self.clock == ClockSource::Internal {
            return Err(ConfigError::RtcAlignWithoutClkin);
        }
        if self.int1.mode == Int1Mode::Pulse
            && self.int1.pulse_width == PulseWidth::Us100
            && (self.accel_odr == Odr::Hz6400 || self.gyro_odr == Odr::Hz6400)
        {
            return Err(ConfigError::Int1LongPulseAtHighOdr);
        }
        Ok(())
    }
}
