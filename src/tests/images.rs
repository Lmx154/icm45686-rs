//! D2: register addresses, reset values, field codes and images against the datasheet tables
//! (DS-000489 Rev 1.1). Every group names the section and the printed page.

use crate::config::*;
use crate::image;
use crate::reg;

fn with(f: impl FnOnce(Config) -> Config) -> Config {
    f(Config::new())
}

#[test]
fn register_addresses_match_the_register_maps() {
    // 15.1 User Bank 0 register map, p58-p60, and descriptions 16.x. Each pair is (name, address).
    let direct = [
        (reg::ACCEL_DATA_X1_UI, 0x00),
        (reg::GYRO_DATA_X1_UI, 0x06),
        (reg::TEMP_DATA1_UI, 0x0C),
        (reg::TMST_FSYNCH, 0x0E),
        (reg::TMST_FSYNCL, 0x0F),
        (reg::PWR_MGMT0, 0x10),
        (reg::FIFO_COUNT_0, 0x12),
        (reg::FIFO_COUNT_1, 0x13),
        (reg::FIFO_DATA, 0x14),
        (reg::INT1_CONFIG0, 0x16),
        (reg::INT1_CONFIG1, 0x17),
        (reg::INT1_CONFIG2, 0x18),
        (reg::INT1_STATUS0, 0x19),
        (reg::INT1_STATUS1, 0x1A),
        (reg::ACCEL_CONFIG0, 0x1B),
        (reg::GYRO_CONFIG0, 0x1C),
        (reg::FIFO_CONFIG0, 0x1D),
        (reg::FIFO_CONFIG1_0, 0x1E),
        (reg::FIFO_CONFIG1_1, 0x1F),
        (reg::FIFO_CONFIG2, 0x20),
        (reg::FIFO_CONFIG3, 0x21),
        (reg::FIFO_CONFIG4, 0x22),
        (reg::TMST_WOM_CONFIG, 0x23),
        (reg::FSYNC_CONFIG0, 0x24),
        (reg::FSYNC_CONFIG1, 0x25),
        (reg::RTC_CONFIG, 0x26),
        (reg::ODR_DECIMATE_CONFIG, 0x28),
        (reg::DRIVE_CONFIG0, 0x32),
        (reg::WHO_AM_I, 0x72),
        (reg::IREG_ADDR_15_8, 0x7C),
        (reg::IREG_ADDR_7_0, 0x7D),
        (reg::IREG_DATA, 0x7E),
        (reg::REG_MISC2, 0x7F),
    ];
    for (got, want) in direct {
        assert_eq!(got, want);
    }
    // 13.4, p55: bank bases. 15.4-15.6, p64-p66: offsets inside the banks.
    assert_eq!(reg::IMEM_SRAM, 0x0000);
    assert_eq!(reg::IPREG_TOP1, 0xA200);
    assert_eq!(reg::IPREG_SYS1, 0xA400);
    assert_eq!(reg::IPREG_SYS2, 0xA500);
    let indirect = [
        (reg::SMC_CONTROL_0, 0xA258),
        (reg::SREG_CTRL, 0xA267),
        (reg::SIFS_I3C_STC_CFG, 0xA268),
        (reg::INT_PULSE_MIN_ON_INTF0, 0xA269),
        (reg::INT_PULSE_MIN_OFF_INTF1, 0xA26C),
        (reg::SELFTEST, 0xA290),
        (reg::GYRO_X_OFFUSER, 0xA42A),
        (reg::GYRO_Y_OFFUSER, 0xA438),
        (reg::GYRO_Z_OFFUSER, 0xA446),
        (reg::IPREG_SYS1_REG_166, 0xA4A6),
        (reg::IPREG_SYS1_REG_172, 0xA4AC),
        (reg::ACCEL_X_OFFUSER, 0xA518),
        (reg::ACCEL_Y_OFFUSER, 0xA520),
        (reg::ACCEL_Z_OFFUSER, 0xA528),
        (reg::IPREG_SYS2_REG_123, 0xA57B),
        (reg::IPREG_SYS2_REG_131, 0xA583),
        (reg::GYRO_X_STR_FT, 0x0000),
        (reg::GYRO_Y_STR_FT, 0x0002),
        (reg::GYRO_Z_STR_FT, 0x0004),
        (reg::IMEM_SRAM_REG_56, 0x0038),
        (reg::IMEM_SRAM_REG_57, 0x0039),
        (reg::IMEM_SRAM_REG_64, 0x0040),
        (reg::IMEM_SRAM_REG_68, 0x0044),
    ];
    for (got, want) in indirect {
        assert_eq!(got, want);
    }
}

#[test]
fn reset_values_match_the_register_descriptions() {
    // "Reset value" lines of 16.17 (p70), 16.21-16.23 (p72-p75), 16.26-16.39 (p78-p87), 16.48 (p93),
    // 19.47-19.55 (p178-p181), 20.7-20.11 (p190-p192), 21.7-21.10 (p194-p196).
    assert_eq!(reg::PWR_MGMT0_RESET, 0x00);
    assert_eq!(reg::INT1_CONFIG0_RESET, 0x80);
    assert_eq!(reg::INT1_CONFIG1_RESET, 0x00);
    assert_eq!(reg::INT1_CONFIG2_RESET, 0x04);
    assert_eq!(reg::ACCEL_CONFIG0_RESET, 0x06);
    assert_eq!(reg::GYRO_CONFIG0_RESET, 0x06);
    assert_eq!(reg::FIFO_CONFIG0_RESET, 0x00);
    assert_eq!(reg::FIFO_CONFIG2_RESET, 0x20);
    assert_eq!(reg::FIFO_CONFIG3_RESET, 0x00);
    assert_eq!(reg::FIFO_CONFIG4_RESET, 0x00);
    assert_eq!(reg::TMST_WOM_CONFIG_RESET, 0x00);
    assert_eq!(reg::FSYNC_CONFIG0_RESET, 0x00);
    assert_eq!(reg::RTC_CONFIG_RESET, 0x03);
    assert_eq!(reg::ODR_DECIMATE_CONFIG_RESET, 0x00);
    assert_eq!(reg::DRIVE_CONFIG0_RESET, 0x6C);
    assert_eq!(reg::SMC_CONTROL_0_RESET, 0x60);
    assert_eq!(reg::SIFS_I3C_STC_CFG_RESET, 0x23);
    assert_eq!(reg::INT_PULSE_MIN_ON_INTF0_RESET, 0x01);
    assert_eq!(reg::INT_PULSE_MIN_OFF_INTF1_RESET, 0x01);
    assert_eq!(reg::IPREG_SYS1_REG_166_RESET, 0x1B);
    assert_eq!(reg::IPREG_SYS1_REG_172_RESET, 0x80);
    assert_eq!(reg::IPREG_SYS2_REG_123_RESET, 0x14);
    assert_eq!(reg::IPREG_SYS2_REG_131_RESET, 0x00);
}

#[test]
fn bit_masks_match_the_register_descriptions() {
    // 16.21 INT1_CONFIG0, p72-p73: bit 7 reset done .. bit 0 FIFO full.
    assert_eq!(
        [
            reg::RESET_DONE,
            reg::AUX1_AGC_RDY,
            reg::AP_AGC_RDY,
            reg::AP_FSYNC,
            reg::AUX1_DRDY,
            reg::DRDY,
            reg::FIFO_THS,
            reg::FIFO_FULL
        ],
        [0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01]
    );
    // 16.23 INT1_CONFIG2, p75.
    assert_eq!(reg::INT1_DRIVE_OPEN_DRAIN, 1 << 2);
    assert_eq!(reg::INT1_MODE_LATCHED, 1 << 1);
    assert_eq!(reg::INT1_POLARITY_HIGH, 1 << 0);
    // 16.31 FIFO_CONFIG2, p81: bit 7 flush, bit 3 FIFO_WR_WM_GT_TH.
    assert_eq!(reg::FIFO_FLUSH, 1 << 7);
    assert_eq!(reg::FIFO_WR_WM_GT_TH, 1 << 3);
    // 16.32 FIFO_CONFIG3, p82: bit 3 HIRES, 2 GYRO, 1 ACCEL, 0 IF_EN.
    assert_eq!(
        [
            reg::FIFO_HIRES_EN,
            reg::FIFO_GYRO_EN,
            reg::FIFO_ACCEL_EN,
            reg::FIFO_IF_EN
        ],
        [8, 4, 2, 1]
    );
    // 16.33 FIFO_CONFIG4, p83: bit 1 FIFO_TMST_FSYNC_EN. 16.34, p84: bit 5 TMST_RESOL.
    assert_eq!(reg::FIFO_TMST_FSYNC_EN, 1 << 1);
    assert_eq!(reg::TMST_RESOL_16US, 1 << 5);
    // 16.37 RTC_CONFIG, p86: bit 6 RTC_ALIGN, bit 5 RTC_MODE.
    assert_eq!(reg::RTC_ALIGN, 1 << 6);
    assert_eq!(reg::RTC_MODE, 1 << 5);
    // 19.47 SMC_CONTROL_0, p178-p179: bit 4 ACCEL_LP_CLK_SEL, 3 TEMP_DIS, 1 TMST_FSYNC_EN, 0 TMST_EN.
    assert_eq!(reg::ACCEL_LP_CLK_SEL, 1 << 4);
    assert_eq!(reg::TEMP_DIS, 1 << 3);
    assert_eq!(reg::TMST_FSYNC_EN, 1 << 1);
    assert_eq!(reg::TMST_EN, 1 << 0);
    // 19.50 SREG_CTRL, p180: bit 1. 19.51 SIFS_I3C_STC_CFG, p180: bit 2.
    assert_eq!(reg::SREG_DATA_ENDIAN_SEL, 1 << 1);
    assert_eq!(reg::I3C_STC_MODE, 1 << 2);
    // 19.66 SELFTEST, p187: bits 5:3 gyro Z, Y, X; bits 2:0 accel Z, Y, X.
    assert_eq!(
        [
            reg::EN_AX_ST,
            reg::EN_AY_ST,
            reg::EN_AZ_ST,
            reg::EN_GX_ST,
            reg::EN_GY_ST,
            reg::EN_GZ_ST
        ],
        [1, 2, 4, 8, 16, 32]
    );
    // 17.13 and 17.16, p113-p114.
    assert_eq!(
        [reg::ST_GYRO_EN, reg::ST_ACCEL_EN, reg::STC_INIT_EN],
        [4, 2, 1]
    );
    assert_eq!(
        [
            reg::ST_STATUS_DONE,
            reg::ST_STATUS_IN_PROGRESS,
            reg::ST_STATUS_ERROR
        ],
        [0x00, 0x40, 0x80]
    );
    // 16.84 REG_MISC2, p109: bit 1 SOFT_RST.
    assert_eq!(reg::SOFT_RST, 1 << 1);
    // 16.79 WHO_AM_I, p107.
    assert_eq!(reg::WHO_AM_I_VALUE, 0xE9);
}

#[test]
fn field_codes_match_the_tables() {
    // 16.17 PWR_MGMT0, p70.
    assert_eq!(SensorPower::Off.code(), 0b00);
    assert_eq!(SensorPower::Standby.code(), 0b01);
    assert_eq!(SensorPower::LowNoise.code(), 0b11);
    assert_eq!(reg::POWER_LOW_POWER, 0b10);
    // 20.11 and 21.10, p192 and p196.
    let lpf = [
        (UiBandwidth::Bypass, 0b000),
        (UiBandwidth::OdrDiv4, 0b001),
        (UiBandwidth::OdrDiv8, 0b010),
        (UiBandwidth::OdrDiv16, 0b011),
        (UiBandwidth::OdrDiv32, 0b100),
        (UiBandwidth::OdrDiv64, 0b101),
        (UiBandwidth::OdrDiv128, 0b110),
    ];
    for (v, code) in lpf {
        assert_eq!(v.code(), code);
    }
    // 20.7 and 21.7, p190 and p194.
    assert_eq!(AntiAlias::Off.code(), 0);
    assert_eq!(AntiAlias::Fir.code(), 1);
    assert_eq!(AntiAlias::InterpolatorFir.code(), 2);
    // 19.52, p181.
    assert_eq!(PulseWidth::Us100.code(), 0);
    assert_eq!(PulseWidth::Us8.code(), 1);
    // 16.28, p80.
    assert_eq!(FifoMode::Off.code(), 0b00);
    assert_eq!(FifoMode::Stream.code(), 0b01);
    assert_eq!(FifoMode::StopOnFull.code(), 0b10);
    assert_eq!(FifoDepth::Kib2.code(), 0b000111);
    assert_eq!(FifoDepth::Kib8.code(), 0b011111);
    assert_eq!(FifoDepth::Kib2.bytes(), 2048);
    assert_eq!(FifoDepth::Kib8.bytes(), 8192);
    // 16.39, p87: code n is a factor 2^n, n = 0..=12.
    let dec = [
        Decimation::Div1,
        Decimation::Div2,
        Decimation::Div4,
        Decimation::Div8,
        Decimation::Div16,
        Decimation::Div32,
        Decimation::Div64,
        Decimation::Div128,
        Decimation::Div256,
        Decimation::Div512,
        Decimation::Div1024,
        Decimation::Div2048,
        Decimation::Div4096,
    ];
    for (n, d) in dec.into_iter().enumerate() {
        assert_eq!(usize::from(d.code()), n);
    }
    // 16.48, p93.
    let slew = [
        (SpiSlew::Ns38, 0b000),
        (SpiSlew::Ns14, 0b001),
        (SpiSlew::Ns10, 0b010),
        (SpiSlew::Ns7, 0b011),
        (SpiSlew::Ns5, 0b100),
        (SpiSlew::Ns4, 0b101),
        (SpiSlew::Ns0_5, 0b110),
    ];
    for (s, code) in slew {
        assert_eq!(s.code(), code);
    }
}

#[test]
fn odr_rate_shift_halves_the_rate_each_step() {
    // 16.26, p78: 6.4 kHz is code 3, every next code halves the rate down to 1.5625 Hz.
    assert_eq!(Odr::Hz6400.rate_shift(), 0);
    assert_eq!(Odr::Hz800.rate_shift(), 3);
    assert_eq!(Odr::Hz1_5625.rate_shift(), 12);
    for o in [
        Odr::Hz6400,
        Odr::Hz3200,
        Odr::Hz400,
        Odr::Hz12_5,
        Odr::Hz1_5625,
    ] {
        assert_eq!(o.hz() * f32::from(1u16 << o.rate_shift()), 6400.0);
    }
}

#[test]
fn pwr_mgmt0_image() {
    // 16.17, p70: gyro mode in bits 3:2, accel mode in bits 1:0.
    use SensorPower::*;
    let cases = [
        (LowNoise, LowNoise, 0x0F),
        (Off, Off, 0x00),
        (Off, LowNoise, 0x0C),
        (LowNoise, Off, 0x03),
        (Off, Standby, 0x04),
        (LowNoise, Standby, 0x07),
    ];
    for (accel, gyro, want) in cases {
        let c = with(|c| c.with_accel_power(accel).with_gyro_power(gyro));
        assert_eq!(image::pwr_mgmt0(&c), want);
    }
}

#[test]
fn accel_and_gyro_config0_images() {
    // 16.26, p78: full scale in bits 6:4, ODR in bits 3:0.
    let c = with(|c| {
        c.with_accel_fs(AccelFullScale::G2)
            .with_accel_odr(Odr::Hz12_5)
    });
    assert_eq!(image::accel_config0(&c), 0x4C);
    // 16.27, p79: full scale in bits 7:4, ODR in bits 3:0.
    let c = with(|c| {
        c.with_gyro_fs(GyroFullScale::Dps15_625)
            .with_gyro_odr(Odr::Hz6400)
    });
    assert_eq!(image::gyro_config0(&c), 0x83);
    assert_eq!(image::accel_config0(&Config::new()), 0x36);
    assert_eq!(image::gyro_config0(&Config::new()), 0x16);
}

#[test]
fn int1_images() {
    // 16.21, p72-p73.
    let all = Int1Config::new()
        .with_data_ready(true)
        .with_fifo_threshold(true)
        .with_fifo_full(true)
        .with_agc_ready(true);
    assert_eq!(image::int1_config0(&with(|c| c.with_int1(all))), 0x27);
    assert_eq!(image::int1_config0(&Config::new()), 0x00);
    let one = |i: Int1Config| image::int1_config0(&with(|c| c.with_int1(i)));
    assert_eq!(one(Int1Config::new().with_data_ready(true)), 0x04);
    assert_eq!(one(Int1Config::new().with_fifo_threshold(true)), 0x02);
    assert_eq!(one(Int1Config::new().with_fifo_full(true)), 0x01);
    assert_eq!(one(Int1Config::new().with_agc_ready(true)), 0x20);
    // 16.23, p75: bit 2 drive, bit 1 mode, bit 0 polarity.
    let two = |i: Int1Config| image::int1_config2(&with(|c| c.with_int1(i)));
    assert_eq!(two(Int1Config::new()), 0x01);
    let low_pp = Int1Config::new().with_polarity(Int1Polarity::ActiveLow);
    assert_eq!(two(low_pp), 0x00);
    let reset = low_pp.with_drive(Int1Drive::OpenDrain);
    assert_eq!(two(reset), reg::INT1_CONFIG2_RESET);
    let all = Int1Config::new()
        .with_drive(Int1Drive::OpenDrain)
        .with_mode(Int1Mode::Latched);
    assert_eq!(two(all), 0x07);
    // 19.52, p181.
    let pw = |w| {
        image::int1_pulse_width(&with(|c| {
            c.with_int1(Int1Config::new().with_pulse_width(w))
        }))
    };
    assert_eq!(pw(PulseWidth::Us100), 0);
    assert_eq!(pw(PulseWidth::Us8), 1);
}

#[test]
fn fifo_images() {
    let on = |f: FifoConfig| with(|c| c.with_fifo(f));
    let stream = FifoConfig::new().with_mode(FifoMode::Stream);
    // 16.28, p80: mode in bits 7:6, depth in bits 5:0.
    assert_eq!(image::fifo_config0(&on(stream)), 0x47);
    assert_eq!(image::fifo_config0_bypass(&on(stream)), 0x07);
    let stop8 = stream
        .with_mode(FifoMode::StopOnFull)
        .with_depth(FifoDepth::Kib8);
    assert_eq!(image::fifo_config0(&on(stop8)), 0x9F);
    assert_eq!(image::fifo_config0_bypass(&on(stop8)), 0x1F);
    assert_eq!(image::fifo_config0(&Config::new()), 0x00);
    // 16.29-16.30, p80-p81: low byte first.
    // Vendor erratum: the caller's N is programmed as N + 1; 0 stays 0.
    let wm = stream.with_watermark(0x0123);
    assert_eq!(image::fifo_watermark(&on(wm)), [0x24, 0x01]);
    assert_eq!(
        image::fifo_watermark(&on(stream.with_watermark(0x00FF))),
        [0x00, 0x01]
    );
    assert_eq!(
        image::fifo_watermark(&on(stream.with_watermark(1))),
        [0x02, 0x00]
    );
    assert_eq!(image::fifo_watermark(&on(stream.with_watermark(0))), [0, 0]);
    assert_eq!(image::fifo_watermark(&Config::new()), [0, 0]);
    // 16.31, p81: reset 0x20, bit 3 FIFO_WR_WM_GT_TH.
    assert_eq!(image::fifo_config2(&on(stream)), 0x28);
    assert_eq!(image::fifo_config2(&Config::new()), 0x20);
    // 16.32, p82: bit 3 HIRES, 2 GYRO, 1 ACCEL, 0 IF_EN.
    let cfg3 = |f: FifoConfig, iface| image::fifo_config3(&on(f), iface);
    assert_eq!(cfg3(stream, false), 0x06);
    assert_eq!(cfg3(stream, true), 0x07);
    assert_eq!(cfg3(stream.with_gyro(false), true), 0x03);
    assert_eq!(cfg3(stream.with_accel(false), false), 0x04);
    let hires = stream.with_format(FifoFormat::HighResolution);
    assert_eq!(cfg3(hires, true), 0x0F);
    assert_eq!(cfg3(FifoConfig::new(), true), 0x00);
    // 16.33, p83: bit 1. 16.34, p84: bit 5.
    assert_eq!(image::fifo_config4(&on(stream.with_timestamp(true))), 0x02);
    assert_eq!(image::fifo_config4(&on(stream)), 0x00);
    assert_eq!(
        image::fifo_config4(&on(FifoConfig::new().with_timestamp(true))),
        0x00
    );
    let res16 = stream.with_timestamp_resolution(TimestampResolution::Us16);
    assert_eq!(image::tmst_wom_config(&on(res16)), 0x20);
    assert_eq!(image::tmst_wom_config(&on(stream)), 0x00);
    // 19.47, p178-p179: TMST_EN bit 0 with the FIFO timestamp; TMST_FSYNC_EN (bit 1) is never set.
    assert_eq!(image::smc_timestamp(&on(stream.with_timestamp(true))), 0x01);
    assert_eq!(image::smc_timestamp(&Config::new()), 0x00);
    // 16.39, p87: gyro in bits 7:4, accel in bits 3:0.
    let dec = stream
        .with_gyro_decimation(Decimation::Div16)
        .with_accel_decimation(Decimation::Div2);
    assert_eq!(image::odr_decimate_config(&on(dec)), 0x41);
    assert_eq!(image::odr_decimate_config(&on(stream)), 0x00);
}

#[test]
fn fifo_frame_sizes() {
    // 5.1, p39: 8 bytes for one sensor, 16 for both, 20 for high resolution.
    let f = FifoConfig::new();
    assert_eq!(f.frame_bytes(), 16);
    assert_eq!(f.with_gyro(false).frame_bytes(), 8);
    assert_eq!(f.with_accel(false).frame_bytes(), 8);
    assert_eq!(f.with_accel(false).with_gyro(false).frame_bytes(), 0);
    assert_eq!(f.with_format(FifoFormat::HighResolution).frame_bytes(), 20);
    // 2 KB over 16 bytes is 128 frames, 8 KB over 16 bytes is 512 (scope document).
    assert_eq!(f.capacity_frames(), 128);
    assert_eq!(f.with_depth(FifoDepth::Kib8).capacity_frames(), 512);
    assert_eq!(
        f.with_format(FifoFormat::HighResolution).capacity_frames(),
        102
    );
    assert_eq!(f.with_accel(false).with_gyro(false).capacity_frames(), 0);
}

#[test]
fn rtc_slew_and_filter_images() {
    // 16.37, p86 and 19.47, p178.
    let clkin = with(|c| c.with_clock(ClockSource::ExternalClkin));
    assert_eq!(image::rtc_mode(&clkin), 0x20);
    assert_eq!(image::accel_lp_clk_sel(&clkin), 0x10);
    assert_eq!(image::rtc_mode(&Config::new()), 0);
    assert_eq!(image::accel_lp_clk_sel(&Config::new()), 0);
    // 16.48, p93: PADS_SPI_SLEW in bits 3:1, the rest keeps 0x6C.
    let s = |v| image::drive_config0(&with(|c| c.with_spi_slew(v)));
    assert_eq!(s(SpiSlew::Ns38), 0x60);
    assert_eq!(s(SpiSlew::Ns14), 0x62);
    assert_eq!(s(SpiSlew::Ns4), 0x6A);
    assert_eq!(s(SpiSlew::Ns0_5), 0x6C);
    assert_eq!(s(SpiSlew::Ns0_5), reg::DRIVE_CONFIG0_RESET);
    // 20.7, p190: GYRO_SRC_CTRL in bits 6:5. 21.7, p194: ACCEL_SRC_CTRL in bits 1:0.
    let aa = |a, g| with(|c| c.with_accel_anti_alias(a).with_gyro_anti_alias(g));
    let c = aa(Some(AntiAlias::Fir), Some(AntiAlias::InterpolatorFir));
    assert_eq!(image::accel_src_ctrl(&c), 1);
    assert_eq!(image::gyro_src_ctrl(&c), 0x40);
    // Unset: reset value on the internal clock, interpolator and FIR on CLKIN.
    assert_eq!(image::accel_src_ctrl(&Config::new()), 0);
    assert_eq!(image::gyro_src_ctrl(&Config::new()), 0);
    assert_eq!(image::accel_src_ctrl(&clkin), 2);
    assert_eq!(image::gyro_src_ctrl(&clkin), 0x40);
    let c = with(|c| {
        c.with_accel_bandwidth(UiBandwidth::OdrDiv128)
            .with_gyro_bandwidth(UiBandwidth::OdrDiv4)
    });
    assert_eq!(image::accel_ui_lpfbw(&c), 6);
    assert_eq!(image::gyro_ui_lpfbw(&c), 1);
}
