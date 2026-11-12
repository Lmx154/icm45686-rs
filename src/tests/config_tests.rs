//! `Config` family: construction, defaults, setters and validation.

use crate::config::*;
use crate::validate::ConfigError;

const BUILT: Config = Config::new()
    .with_accel_fs(AccelFullScale::G8)
    .with_gyro_fs(GyroFullScale::Dps500)
    .with_accel_odr(Odr::Hz400)
    .with_gyro_odr(Odr::Hz400)
    .with_accel_power(SensorPower::LowNoise)
    .with_gyro_power(SensorPower::Standby)
    .with_accel_bandwidth(UiBandwidth::OdrDiv8)
    .with_gyro_bandwidth(UiBandwidth::OdrDiv16)
    .with_accel_anti_alias(Some(AntiAlias::Fir))
    .with_gyro_anti_alias(Some(AntiAlias::Off))
    .with_clock(ClockSource::ExternalClkin)
    .with_rtc_align(true)
    .with_int1(
        Int1Config::new()
            .with_data_ready(true)
            .with_mode(Int1Mode::Latched)
            .with_polarity(Int1Polarity::ActiveLow)
            .with_drive(Int1Drive::OpenDrain)
            .with_pulse_width(PulseWidth::Us100),
    )
    .with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::StopOnFull)
            .with_format(FifoFormat::Standard)
            .with_gyro(false)
            .with_watermark(10)
            .with_depth(FifoDepth::Kib8)
            .with_accel_decimation(Decimation::Div4)
            .with_gyro_decimation(Decimation::Div8)
            .with_timestamp(false)
            .with_timestamp_resolution(TimestampResolution::Us16)
            .with_accel(true),
    )
    .with_spi_slew(SpiSlew::Ns10);

#[test]
fn new_equals_default() {
    assert_eq!(Config::new(), Config::default());
    assert_eq!(Int1Config::new(), Int1Config::default());
    assert_eq!(FifoConfig::new(), FifoConfig::default());
}

#[test]
fn default_reproduces_what_init_configures_today() {
    let c = Config::new();
    assert_eq!(c.accel_fs, AccelFullScale::G4);
    assert_eq!(c.gyro_fs, GyroFullScale::Dps2000);
    assert_eq!((c.accel_odr, c.gyro_odr), (Odr::Hz800, Odr::Hz800));
    assert_eq!(c.clock, ClockSource::Internal);
    assert!(!c.int1.data_ready);
    assert_eq!(
        (c.accel_power, c.gyro_power),
        (SensorPower::LowNoise, SensorPower::LowNoise)
    );
    assert_eq!(c.int1.mode, Int1Mode::Pulse);
    assert_eq!(c.int1.polarity, Int1Polarity::ActiveHigh);
    assert_eq!(c.int1.drive, Int1Drive::PushPull);
    assert_eq!(c.validate(), Ok(()));
}

#[test]
fn const_config_built_with_setters_has_those_values() {
    let built = std::hint::black_box(BUILT);
    assert_eq!(built.accel_fs, AccelFullScale::G8);
    assert_eq!(built.gyro_fs, GyroFullScale::Dps500);
    assert_eq!((built.accel_odr, built.gyro_odr), (Odr::Hz400, Odr::Hz400));
    assert_eq!(built.gyro_power, SensorPower::Standby);
    assert_eq!(built.accel_bandwidth, UiBandwidth::OdrDiv8);
    assert_eq!(built.gyro_bandwidth, UiBandwidth::OdrDiv16);
    assert_eq!(built.accel_anti_alias, Some(AntiAlias::Fir));
    assert_eq!(built.gyro_anti_alias, Some(AntiAlias::Off));
    assert_eq!(built.clock, ClockSource::ExternalClkin);
    assert!(built.rtc_align);
    assert!(built.int1.data_ready);
    assert_eq!(built.int1.pulse_width, PulseWidth::Us100);
    assert_eq!(built.fifo.mode, FifoMode::StopOnFull);
    assert_eq!(built.fifo.depth, FifoDepth::Kib8);
    assert_eq!(built.fifo.watermark, 10);
    assert_eq!(built.fifo.accel_decimation, Decimation::Div4);
    assert_eq!(built.fifo.timestamp_resolution, TimestampResolution::Us16);
    assert_eq!(built.spi_slew, SpiSlew::Ns10);
    assert_ne!(built, Config::new());
}

#[test]
fn setters_replace_one_field_and_nothing_else() {
    let base = Config::new();
    assert_eq!(
        base.with_accel_fs(AccelFullScale::G2),
        Config {
            accel_fs: AccelFullScale::G2,
            ..base
        }
    );
    assert_eq!(
        base.with_spi_slew(SpiSlew::Ns38),
        Config {
            spi_slew: SpiSlew::Ns38,
            ..base
        }
    );
    let i = Int1Config::new();
    assert_eq!(
        i.with_fifo_full(true),
        Int1Config {
            fifo_full: true,
            ..i
        }
    );
    let f = FifoConfig::new();
    assert_eq!(f.with_watermark(5), FifoConfig { watermark: 5, ..f });
}

#[test]
fn helper_predicates() {
    let i = Int1Config::new();
    assert!(!i.uses_fifo());
    assert!(i.with_fifo_threshold(true).uses_fifo());
    assert!(i.with_fifo_full(true).uses_fifo());
    assert!(!FifoConfig::new().enabled());
    assert!(FifoConfig::new().with_mode(FifoMode::Stream).enabled());
}

fn fifo(f: FifoConfig) -> Config {
    Config::new().with_fifo(f.with_mode(FifoMode::Stream))
}

/// Names every variant, so a new one cannot be added without a decision here.
fn covered(e: ConfigError) -> &'static str {
    match e {
        ConfigError::AccelStandby => "accel standby",
        ConfigError::AccelOdrLowPowerOnly => "accel odr",
        ConfigError::GyroOdrLowPowerOnly => "gyro odr",
        ConfigError::FifoNoSensors => "fifo no sensors",
        ConfigError::FifoHighResNeedsBothSensors => "hires both",
        ConfigError::FifoAccelNotRunning => "fifo accel off",
        ConfigError::FifoGyroNotRunning => "fifo gyro off",
        ConfigError::FifoTimestampNeedsBothSensors => "timestamp both",
        ConfigError::FifoWatermarkTooLarge => "watermark",
        ConfigError::FifoRateMismatch => "rate mismatch",
        ConfigError::Int1FifoEventWithFifoOff => "int1 fifo event",
        ConfigError::Int1ThresholdWithoutWatermark => "int1 threshold",
        ConfigError::RtcAlignWithoutClkin => "rtc align",
        ConfigError::Int1LongPulseAtHighOdr => "long pulse",
        // No Config produces it; the plan module tests it with a plan forced past its capacity.
        ConfigError::PlanOverflow => "plan overflow",
    }
}

#[test]
fn every_config_error_is_produced() {
    let hires = FifoConfig::new().with_format(FifoFormat::HighResolution);
    let cases = [
        (
            Config::new().with_accel_power(SensorPower::Standby),
            ConfigError::AccelStandby,
        ),
        (
            Config::new().with_accel_odr(Odr::Hz6_25),
            ConfigError::AccelOdrLowPowerOnly,
        ),
        (
            Config::new().with_gyro_odr(Odr::Hz1_5625),
            ConfigError::GyroOdrLowPowerOnly,
        ),
        (
            fifo(FifoConfig::new().with_accel(false).with_gyro(false)),
            ConfigError::FifoNoSensors,
        ),
        (
            fifo(hires.with_gyro(false)),
            ConfigError::FifoHighResNeedsBothSensors,
        ),
        (
            fifo(FifoConfig::new()).with_accel_power(SensorPower::Off),
            ConfigError::FifoAccelNotRunning,
        ),
        (
            fifo(FifoConfig::new()).with_gyro_power(SensorPower::Standby),
            ConfigError::FifoGyroNotRunning,
        ),
        (
            fifo(FifoConfig::new().with_gyro(false).with_timestamp(true)),
            ConfigError::FifoTimestampNeedsBothSensors,
        ),
        (
            fifo(FifoConfig::new().with_watermark(128)),
            ConfigError::FifoWatermarkTooLarge,
        ),
        (
            fifo(FifoConfig::new()).with_gyro_odr(Odr::Hz400),
            ConfigError::FifoRateMismatch,
        ),
        (
            Config::new().with_int1(Int1Config::new().with_fifo_full(true)),
            ConfigError::Int1FifoEventWithFifoOff,
        ),
        (
            fifo(FifoConfig::new()).with_int1(Int1Config::new().with_fifo_threshold(true)),
            ConfigError::Int1ThresholdWithoutWatermark,
        ),
        (
            Config::new().with_rtc_align(true),
            ConfigError::RtcAlignWithoutClkin,
        ),
        (
            Config::new()
                .with_accel_odr(Odr::Hz6400)
                .with_gyro_odr(Odr::Hz6400)
                .with_int1(Int1Config::new().with_pulse_width(PulseWidth::Us100)),
            ConfigError::Int1LongPulseAtHighOdr,
        ),
    ];
    for (config, want) in cases {
        assert_eq!(config.validate(), Err(want), "{}", covered(want));
    }
}

#[test]
fn valid_edges_are_accepted() {
    // The longest pulse is fine below 4 kHz and in latched mode (19.52, p181).
    let long = Int1Config::new().with_pulse_width(PulseWidth::Us100);
    assert_eq!(Config::new().with_int1(long).validate(), Ok(()));
    let at_6400 = Config::new()
        .with_accel_odr(Odr::Hz6400)
        .with_gyro_odr(Odr::Hz6400);
    assert_eq!(
        at_6400
            .with_int1(long.with_mode(Int1Mode::Latched))
            .validate(),
        Ok(())
    );
    // The largest watermark (capacity - 1: N + 1 is programmed), with decimation making unequal ODRs
    // equal rates.
    let f = FifoConfig::new().with_watermark(127);
    assert_eq!(fifo(f).validate(), Ok(()));
    let f = f.with_accel_decimation(Decimation::Div2);
    assert_eq!(fifo(f).with_accel_odr(Odr::Hz1600).validate(), Ok(()));
    // High resolution, 8 KB: 20-byte frames.
    let hires = FifoConfig::new()
        .with_format(FifoFormat::HighResolution)
        .with_depth(FifoDepth::Kib8)
        .with_timestamp(true)
        .with_watermark(408);
    assert_eq!(fifo(hires).validate(), Ok(()));
    assert_eq!(
        fifo(hires.with_watermark(409)).validate(),
        Err(ConfigError::FifoWatermarkTooLarge)
    );
    // A single-sensor FIFO needs only that sensor; gyro may be off.
    let accel_only = FifoConfig::new().with_gyro(false);
    assert_eq!(
        fifo(accel_only)
            .with_gyro_power(SensorPower::Off)
            .validate(),
        Ok(())
    );
    // Gyro standby is allowed; the accelerometer cannot.
    assert_eq!(
        Config::new()
            .with_gyro_power(SensorPower::Standby)
            .validate(),
        Ok(())
    );
    assert_eq!(
        Config::new()
            .with_clock(ClockSource::ExternalClkin)
            .with_rtc_align(true)
            .validate(),
        Ok(())
    );
}
