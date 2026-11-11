//! Register images: the value of each register (or register field) that a [`Config`] asks for. Pure
//! functions, no bus. A function returning a whole register keeps the reserved bits at their reset
//! value (p66); one for an indirect register returns the field value, already shifted, to be
//! applied under the mask of the same name in [`crate::reg`].

use crate::config::{
    AntiAlias, ClockSource, Config, FifoFormat, Int1Drive, Int1Mode, Int1Polarity,
    TimestampResolution,
};
use crate::reg;

/// PWR_MGMT0 (16.17, p70): gyro mode in bits 3:2, accel mode in bits 1:0.
pub const fn pwr_mgmt0(c: &Config) -> u8 {
    (c.gyro_power.code() << reg::PWR_MGMT0_GYRO_SHIFT)
        | (c.accel_power.code() << reg::PWR_MGMT0_ACCEL_SHIFT)
}

/// ACCEL_CONFIG0 (16.26, p78).
pub const fn accel_config0(c: &Config) -> u8 {
    crate::config::accel_config0(c.accel_fs, c.accel_odr)
}

/// GYRO_CONFIG0 (16.27, p79).
pub const fn gyro_config0(c: &Config) -> u8 {
    crate::config::gyro_config0(c.gyro_fs, c.gyro_odr)
}

/// INT1_CONFIG0 (16.21, p72-p73): the enabled events. Reset-done reporting is left off, as `init`
/// does today.
pub const fn int1_config0(c: &Config) -> u8 {
    let i = &c.int1;
    let mut v = 0;
    if i.data_ready {
        v |= reg::DRDY;
    }
    if i.fifo_threshold {
        v |= reg::FIFO_THS;
    }
    if i.fifo_full {
        v |= reg::FIFO_FULL;
    }
    if i.agc_ready {
        v |= reg::AP_AGC_RDY;
    }
    v
}

/// INT1_CONFIG2 (16.23, p75).
pub const fn int1_config2(c: &Config) -> u8 {
    let mut v = 0;
    if matches!(c.int1.drive, Int1Drive::OpenDrain) {
        v |= reg::INT1_DRIVE_OPEN_DRAIN;
    }
    if matches!(c.int1.mode, Int1Mode::Latched) {
        v |= reg::INT1_MODE_LATCHED;
    }
    if matches!(c.int1.polarity, Int1Polarity::ActiveHigh) {
        v |= reg::INT1_POLARITY_HIGH;
    }
    v
}

/// INT1_TPULSE_DURATION field of INT_PULSE_MIN_ON_INTF1 (19.53, p181).
pub const fn int1_pulse_width(c: &Config) -> u8 {
    c.int1.pulse_width.code()
}

/// FIFO_CONFIG0 with the mode (16.28, p80): zero when the FIFO is off.
pub const fn fifo_config0(c: &Config) -> u8 {
    if c.fifo.enabled() {
        (c.fifo.mode.code() << reg::FIFO_MODE_SHIFT) | c.fifo.depth.code()
    } else {
        reg::FIFO_CONFIG0_RESET
    }
}

/// FIFO_CONFIG0 with the depth and the FIFO still in bypass, the first write when the FIFO is
/// switched on (the depth "can be changed when FIFO is disabled", 16.28, p80).
pub const fn fifo_config0_bypass(c: &Config) -> u8 {
    if c.fifo.enabled() {
        (reg::FIFO_MODE_BYPASS << reg::FIFO_MODE_SHIFT) | c.fifo.depth.code()
    } else {
        reg::FIFO_CONFIG0_RESET
    }
}

/// FIFO_CONFIG1_0 and FIFO_CONFIG1_1 (16.29, p80; 16.30, p81): the watermark, low byte first.
///
/// The caller's watermark of N frames (N >= 1) is written as N + 1 (vendor erratum AN-000364: when
/// the threshold interrupt fires with M frames in the FIFO, the host must not read all M; the
/// driver reads N and so always leaves one frame; confirmed on the board, 2026-11-10: one frame
/// left in 200 batches, though the erratum itself did not reproduce there). 0 stays 0, which disables the watermark (16.29,
/// p80), and the watermark is 0 when the FIFO is off. `validate` guarantees that N + 1 fits.
pub const fn fifo_watermark(c: &Config) -> [u8; 2] {
    if c.fifo.enabled() && c.fifo.watermark != 0 {
        c.fifo.watermark.saturating_add(1).to_le_bytes()
    } else {
        [0, 0]
    }
}

/// FIFO_CONFIG2 (16.31, p81) without the flush command: the reserved bit 5 keeps its reset value;
/// with the FIFO on, FIFO_WR_WM_GT_TH is set so that the watermark event fires at "count >=
/// threshold" and cannot be stepped over.
pub const fn fifo_config2(c: &Config) -> u8 {
    if c.fifo.enabled() {
        reg::FIFO_CONFIG2_RESET | reg::FIFO_WR_WM_GT_TH
    } else {
        reg::FIFO_CONFIG2_RESET
    }
}

/// FIFO_CONFIG3 (16.32, p82): the frame content. `interface` sets FIFO_IF_EN, which is written last
/// when switching on and first when switching off. High resolution sets the accel and gyro enables
/// too. Zero when the FIFO is off.
pub const fn fifo_config3(c: &Config, interface: bool) -> u8 {
    if !c.fifo.enabled() {
        return reg::FIFO_CONFIG3_RESET;
    }
    let mut v = 0;
    if matches!(c.fifo.format, FifoFormat::HighResolution) {
        v |= reg::FIFO_HIRES_EN | reg::FIFO_ACCEL_EN | reg::FIFO_GYRO_EN;
    }
    if c.fifo.accel {
        v |= reg::FIFO_ACCEL_EN;
    }
    if c.fifo.gyro {
        v |= reg::FIFO_GYRO_EN;
    }
    if interface {
        v |= reg::FIFO_IF_EN;
    }
    v
}

/// FIFO_CONFIG4 (16.33, p83): the timestamp field, only when the FIFO is on.
pub const fn fifo_config4(c: &Config) -> u8 {
    if c.fifo.enabled() && c.fifo.timestamp {
        reg::FIFO_TMST_FSYNC_EN
    } else {
        reg::FIFO_CONFIG4_RESET
    }
}

/// TMST_WOM_CONFIG (16.34, p84): the timestamp resolution.
pub const fn tmst_wom_config(c: &Config) -> u8 {
    match c.fifo.timestamp_resolution {
        TimestampResolution::Us1 => 0,
        TimestampResolution::Us16 => reg::TMST_RESOL_16US,
    }
}

/// TMST_EN bit of SMC_CONTROL_0 (19.47, p178-p179), under the mask [`reg::TMST_EN`]. TMST_FSYNC_EN
/// (FSYNC is not offered) is outside the mask and keeps its reset value 0.
pub const fn smc_timestamp(c: &Config) -> u8 {
    let mut v = 0;
    if c.fifo.enabled() && c.fifo.timestamp {
        v |= reg::TMST_EN;
    }
    v
}

/// ODR_DECIMATE_CONFIG (16.39, p87): gyro in bits 7:4, accel in bits 3:0. The driver writes the
/// decimation only when the FIFO is on and the reset value 0 when it is off. With the FIFO on the
/// decimation also lowers the data-register and INT1 data-ready rate (measured on the MARV V2 board,
/// 2026-11-11).
pub const fn odr_decimate_config(c: &Config) -> u8 {
    if c.fifo.enabled() {
        (c.fifo.gyro_decimation.code() << 4) | c.fifo.accel_decimation.code()
    } else {
        reg::ODR_DECIMATE_CONFIG_RESET
    }
}

/// RTC_MODE bit of RTC_CONFIG (16.37, p86), under the mask [`reg::RTC_MODE`].
pub const fn rtc_mode(c: &Config) -> u8 {
    match c.clock {
        ClockSource::ExternalClkin => reg::RTC_MODE,
        ClockSource::Internal => 0,
    }
}

/// ACCEL_LP_CLK_SEL bit of SMC_CONTROL_0 (19.47, p178), under the mask [`reg::ACCEL_LP_CLK_SEL`]:
/// RTC mode needs it set (16.37, p86).
pub const fn accel_lp_clk_sel(c: &Config) -> u8 {
    match c.clock {
        ClockSource::ExternalClkin => reg::ACCEL_LP_CLK_SEL,
        ClockSource::Internal => 0,
    }
}

/// DRIVE_CONFIG0 (16.48, p93): the SPI slew; PADS_I2C_SLEW keeps its reset value.
pub const fn drive_config0(c: &Config) -> u8 {
    (reg::DRIVE_CONFIG0_RESET & !reg::DRIVE_CONFIG0_FIELDS)
        | (c.spi_slew.code() << reg::PADS_SPI_SLEW_SHIFT)
}

/// The anti-alias setting in effect for the accelerometer: the configured one, or by the clock
/// (interpolator and FIR on CLKIN, off otherwise).
pub const fn accel_anti_alias(c: &Config) -> AntiAlias {
    effective_anti_alias(c.accel_anti_alias, c.clock)
}

/// The anti-alias setting in effect for the gyroscope.
pub const fn gyro_anti_alias(c: &Config) -> AntiAlias {
    effective_anti_alias(c.gyro_anti_alias, c.clock)
}

const fn effective_anti_alias(set: Option<AntiAlias>, clock: ClockSource) -> AntiAlias {
    match (set, clock) {
        (Some(v), _) => v,
        (None, ClockSource::ExternalClkin) => AntiAlias::InterpolatorFir,
        (None, ClockSource::Internal) => AntiAlias::Off,
    }
}

/// ACCEL_SRC_CTRL field of IPREG_SYS2_REG_123 (21.7, p194), under [`reg::ACCEL_SRC_CTRL_MASK`].
pub const fn accel_src_ctrl(c: &Config) -> u8 {
    accel_anti_alias(c).code() << reg::ACCEL_SRC_CTRL_SHIFT
}

/// GYRO_SRC_CTRL field of IPREG_SYS1_REG_166 (20.7, p190), under [`reg::GYRO_SRC_CTRL_MASK`].
pub const fn gyro_src_ctrl(c: &Config) -> u8 {
    gyro_anti_alias(c).code() << reg::GYRO_SRC_CTRL_SHIFT
}

/// ACCEL_UI_LPFBW_SEL field of IPREG_SYS2_REG_131 (21.10, p196), under [`reg::ACCEL_UI_LPFBW_MASK`].
pub const fn accel_ui_lpfbw(c: &Config) -> u8 {
    c.accel_bandwidth.code()
}

/// GYRO_UI_LPFBW_SEL field of IPREG_SYS1_REG_172 (20.11, p192), under [`reg::GYRO_UI_LPFBW_MASK`].
pub const fn gyro_ui_lpfbw(c: &Config) -> u8 {
    c.gyro_bandwidth.code()
}
