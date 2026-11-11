//! The register write plan: order, content and agreement with what `init` writes today.

use super::NoDelay;
use super::model::{Device, LogSpi, Wr};
use crate::config::*;
use crate::plan::{
    ACCEL_START_UP_US, GYRO_START_UP_US, GYRO_STOP_US, PLAN_CAPACITY, Plan, SLEW_SETTLE_US, Step,
};
use crate::reg;
use crate::validate::ConfigError;

fn plan(c: &Config) -> Plan {
    Plan::after_reset(c).unwrap()
}

fn steps(c: &Config) -> Vec<Step> {
    plan(c).steps().to_vec()
}

fn w(reg: u8, value: u8) -> Step {
    Step::Write { reg, value }
}

/// The twelve user-offset registers written to zero by every post-reset plan (20.1-21.6, p189-p194).
fn offset_zeros() -> Vec<Step> {
    [
        0xA42A, 0xA42B, 0xA438, 0xA439, 0xA446, 0xA447, 0xA518, 0xA519, 0xA520, 0xA521, 0xA528,
        0xA529,
    ]
    .into_iter()
    .map(|addr| Step::IregWrite { addr, value: 0 })
    .collect()
}

#[test]
fn default_plan_is_todays_init_in_todays_order() {
    // lib.rs `init` after the soft reset and the identity check: INT1_CONFIG0 = 0, INT1_CONFIG2 =
    // push-pull pulse high, ACCEL_CONFIG0, GYRO_CONFIG0, INT1_CONFIG0, PWR_MGMT0 = 0x0F, then the
    // start-up wait (`init` waits the vendor's 60 ms); the twelve offset registers are written to
    // zero after INT1_CONFIG2. Datasheet values: 16.21 p72, 16.23 p75, 16.26 p78, 16.27 p79, 16.17 p70.
    let want: Vec<Step> = [w(reg::INT1_CONFIG0, 0x00), w(reg::INT1_CONFIG2, 0x01)]
        .into_iter()
        .chain(offset_zeros())
        .chain([
            w(reg::ACCEL_CONFIG0, 0x36),
            w(reg::GYRO_CONFIG0, 0x16),
            w(reg::INT1_CONFIG0, 0x00),
            w(reg::PWR_MGMT0, 0x0F),
            Step::DelayUs(60_000),
        ])
        .collect();
    assert_eq!(steps(&Config::new()), want);
}

#[test]
fn clkin_plan_is_todays_clock_sequence() {
    // lib.rs `external_clock`: pad override, SIFS_I3C_STC_CFG bit 2 clear, accel SRC 2, gyro SRC 2,
    // ACCEL_LP_CLK_SEL set, RTC_MODE set; then the rates.
    let c = Config::new()
        .with_accel_fs(AccelFullScale::G8)
        .with_gyro_fs(GyroFullScale::Dps500)
        .with_accel_odr(Odr::Hz200)
        .with_gyro_odr(Odr::Hz200)
        .with_clock(ClockSource::ExternalClkin)
        .with_int1(Int1Config::new().with_data_ready(true));
    let want: Vec<Step> = [w(reg::INT1_CONFIG0, 0x00), w(reg::INT1_CONFIG2, 0x01)]
        .into_iter()
        .chain(offset_zeros())
        .chain([
            w(reg::IOC_PAD_SCENARIO_OVRD, 0x06),
            Step::IregModify {
                addr: 0xA268,
                mask: 0x04,
                value: 0x00,
            },
            Step::IregModify {
                addr: 0xA57B,
                mask: 0x03,
                value: 0x02,
            },
            Step::IregModify {
                addr: 0xA4A6,
                mask: 0x60,
                value: 0x40,
            },
            Step::IregModify {
                addr: 0xA258,
                mask: 0x10,
                value: 0x10,
            },
            Step::Modify {
                reg: reg::RTC_CONFIG,
                mask: 0x20,
                value: 0x20,
            },
            w(reg::ACCEL_CONFIG0, 0x28),
            w(reg::GYRO_CONFIG0, 0x38),
            w(reg::INT1_CONFIG0, 0x04),
            w(reg::PWR_MGMT0, 0x0F),
            Step::DelayUs(60_000),
        ])
        .collect();
    assert_eq!(steps(&c), want);
}

/// The writes `init` makes on the bus, from a reset device; the soft reset is not part of a plan.
fn init_writes(c: &Config) -> Vec<Wr> {
    let mut imu = crate::blocking::Icm45686::new(LogSpi(Device::reset()));
    imu.init(&mut NoDelay, c).unwrap();
    let log = imu.release().0.log;
    log.into_iter()
        .filter(|w| *w != Wr::Direct(reg::REG_MISC2, reg::SOFT_RST))
        .collect()
}

fn plan_writes(c: &Config) -> Vec<Wr> {
    let mut d = Device::reset();
    for s in plan(c).steps() {
        d.exec(s);
    }
    d.log
}

#[test]
fn plan_writes_what_init_writes_on_the_bus() {
    let configs = [
        Config::new(),
        Config::new().with_int1(Int1Config::new().with_data_ready(true)),
        Config::new()
            .with_accel_fs(AccelFullScale::G16)
            .with_gyro_fs(GyroFullScale::Dps125)
            .with_accel_odr(Odr::Hz6400)
            .with_gyro_odr(Odr::Hz6400),
        Config::new()
            .with_clock(ClockSource::ExternalClkin)
            .with_accel_odr(Odr::Hz200)
            .with_gyro_odr(Odr::Hz200),
        Config::new()
            .with_clock(ClockSource::ExternalClkin)
            .with_int1(Int1Config::new().with_data_ready(true)),
    ];
    for c in configs {
        assert_eq!(plan_writes(&c), init_writes(&c), "{c:?}");
    }
}

#[test]
fn power_state_sets_the_start_up_wait() {
    // Vendor (TDK's Linux driver): 60 ms start-up for the gyro and for the accel; the datasheet
    // gives 35 ms (Table 1, p18) and 10 ms (Table 2, p19).
    assert_eq!(GYRO_START_UP_US, 60_000);
    assert_eq!(ACCEL_START_UP_US, 60_000);
    let both = steps(&Config::new());
    assert_eq!(both.last(), Some(&Step::DelayUs(GYRO_START_UP_US)));
    let accel = Config::new().with_gyro_power(SensorPower::Off);
    let s = steps(&accel);
    assert_eq!(s.last(), Some(&Step::DelayUs(ACCEL_START_UP_US)));
    assert_eq!(s[s.len() - 2], w(reg::PWR_MGMT0, 0x03));
    let standby = Config::new()
        .with_accel_power(SensorPower::Off)
        .with_gyro_power(SensorPower::Standby);
    let s = steps(&standby);
    assert_eq!(s.last(), Some(&Step::DelayUs(GYRO_START_UP_US)));
    assert_eq!(s[s.len() - 2], w(reg::PWR_MGMT0, 0x04));
    let none = Config::new()
        .with_accel_power(SensorPower::Off)
        .with_gyro_power(SensorPower::Off);
    let s = steps(&none);
    assert_eq!(s.last(), Some(&w(reg::PWR_MGMT0, 0x00)));
}

fn position(steps: &[Step], want: Step) -> usize {
    steps
        .iter()
        .position(|s| *s == want)
        .unwrap_or_else(|| panic!("{want:?} missing from {steps:?}"))
}

#[test]
fn fifo_plan_order() {
    let c = Config::new()
        .with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Stream)
                .with_depth(FifoDepth::Kib8)
                .with_watermark(0x0102)
                .with_timestamp(true)
                .with_timestamp_resolution(TimestampResolution::Us16)
                .with_accel_decimation(Decimation::Div2)
                .with_gyro_decimation(Decimation::Div2),
        )
        .with_int1(Int1Config::new().with_fifo_threshold(true));
    let s = steps(&c);
    let depth_bypass = position(&s, w(reg::FIFO_CONFIG0, 0x1F));
    let wm_lo = position(&s, w(reg::FIFO_CONFIG1_0, 0x03));
    let wm_hi = position(&s, w(reg::FIFO_CONFIG1_1, 0x01));
    let config2 = position(&s, w(reg::FIFO_CONFIG2, 0x28));
    let content = position(&s, w(reg::FIFO_CONFIG3, 0x06));
    let config4 = position(&s, w(reg::FIFO_CONFIG4, 0x02));
    let decimate = position(&s, w(reg::ODR_DECIMATE_CONFIG, 0x11));
    let tmst = position(&s, w(reg::TMST_WOM_CONFIG, 0x20));
    let rates = position(&s, w(reg::ACCEL_CONFIG0, 0x36));
    let mode = position(&s, w(reg::FIFO_CONFIG0, 0x5F));
    let iface = position(&s, w(reg::FIFO_CONFIG3, 0x07));
    let int1 = position(&s, w(reg::INT1_CONFIG0, 0x02));
    let power = position(&s, w(reg::PWR_MGMT0, 0x0F));
    // Depth in bypass, then watermark low byte before high byte (16.29, p80), content and timestamp
    // before the rates, the mode after the content, FIFO_IF_EN after the mode (16.32, p82), the
    // events and the power state last.
    assert!(depth_bypass < wm_lo && wm_lo < wm_hi);
    assert!(wm_hi < config2 && config2 < content && content < config4);
    assert!(decimate < rates && tmst < rates && config4 < rates);
    assert!(rates < mode && mode < iface && iface < int1 && int1 < power);
    // Decimation is written while the sensors are still off (5, p39).
    assert!(decimate < power);
    assert!(s.contains(&Step::IregModify {
        addr: reg::SMC_CONTROL_0,
        mask: 0x01,
        value: 0x01,
    }));
}

#[test]
fn skipped_writes_are_those_already_at_reset() {
    // Reset values (16.x): nothing is written for FIFO, timestamps, slew or filters; FSYNC is never written.
    let s = steps(&Config::new());
    for forbidden in [
        reg::FIFO_CONFIG0,
        reg::FIFO_CONFIG1_0,
        reg::FIFO_CONFIG2,
        reg::FIFO_CONFIG3,
        reg::FIFO_CONFIG4,
        reg::TMST_WOM_CONFIG,
        reg::FSYNC_CONFIG0,
        reg::ODR_DECIMATE_CONFIG,
        reg::DRIVE_CONFIG0,
        reg::RTC_CONFIG,
    ] {
        assert!(!s.iter().any(
            |x| matches!(x, Step::Write { reg, .. } | Step::Modify { reg, .. } if *reg == forbidden)
        ));
    }
    assert!(!s.iter().any(|x| matches!(x, Step::IregModify { .. })));
    // A value that differs from the reset value is written, and the pad slew waits 1.5 us (p93).
    let slow = steps(&Config::new().with_spi_slew(SpiSlew::Ns38));
    let at = position(&slow, w(reg::DRIVE_CONFIG0, 0x60));
    assert_eq!(slow[at + 1], Step::DelayUs(SLEW_SETTLE_US));
    const { assert!(SLEW_SETTLE_US >= 2) };
    let long =
        steps(&Config::new().with_int1(Int1Config::new().with_pulse_width(PulseWidth::Us100)));
    assert!(long.contains(&Step::IregModify {
        addr: reg::INT_PULSE_MIN_ON_INTF1,
        mask: 0x07,
        value: 0x00,
    }));
    let lpf = steps(&Config::new().with_gyro_bandwidth(UiBandwidth::OdrDiv16));
    assert!(lpf.contains(&Step::IregModify {
        addr: reg::IPREG_SYS1_REG_172,
        mask: 0x07,
        value: 0x03,
    }));
}

#[test]
fn rtc_align_follows_rtc_mode() {
    let c = Config::new()
        .with_clock(ClockSource::ExternalClkin)
        .with_rtc_align(true);
    let s = steps(&c);
    let mode = position(
        &s,
        Step::Modify {
            reg: reg::RTC_CONFIG,
            mask: 0x20,
            value: 0x20,
        },
    );
    assert_eq!(
        s[mode + 1],
        Step::Modify {
            reg: reg::RTC_CONFIG,
            mask: 0x40,
            value: 0x40,
        }
    );
}

#[test]
fn reconfigure_stops_everything_first_and_writes_every_field() {
    let s = Plan::reconfigure(&Config::new()).unwrap();
    let s = s.steps();
    assert_eq!(s[0], w(reg::PWR_MGMT0, 0x00));
    // Vendor (TDK's Linux driver): 150 ms after the gyro is stopped, before it is started again.
    assert_eq!(GYRO_STOP_US, 150_000);
    assert_eq!(s[1], Step::DelayUs(GYRO_STOP_US));
    assert_eq!(s[2], w(reg::INT1_CONFIG0, 0x00));
    // The FIFO interface goes off before the mode (16.32, p82).
    assert_eq!(s[3], w(reg::FIFO_CONFIG3, 0x00));
    assert_eq!(s[4], w(reg::FIFO_CONFIG0, 0x00));
    assert_eq!(s[5], w(reg::INT1_CONFIG2, 0x01));
    // The flush command is written and then cleared.
    let flush = position(s, w(reg::FIFO_CONFIG2, 0xA0));
    assert_eq!(s[flush + 1], w(reg::FIFO_CONFIG2, 0x20));
    assert_eq!(s.last(), Some(&Step::DelayUs(GYRO_START_UP_US)));
    assert!(s.len() > steps(&Config::new()).len());
    // RTC mode and the clock select are switched off explicitly on the internal clock.
    assert!(s.contains(&Step::Modify {
        reg: reg::RTC_CONFIG,
        mask: 0x20,
        value: 0x00,
    }));
    assert!(s.contains(&Step::IregModify {
        addr: reg::SMC_CONTROL_0,
        mask: 0x10,
        value: 0x00,
    }));
}

#[test]
fn reconfigure_waits_after_the_stop_only_when_the_gyro_comes_back_on() {
    // The wait comes after the power-off write and before any step that turns the gyro on.
    let stop_wait = |c: &Config| {
        let r = Plan::reconfigure(c).unwrap();
        let s = r.steps();
        let on = s
            .iter()
            .rposition(|x| matches!(x, Step::Write { reg: r, value } if *r == reg::PWR_MGMT0 && *value != 0))
            .unwrap();
        let wait = s.iter().position(|x| *x == Step::DelayUs(GYRO_STOP_US));
        (wait, on)
    };
    let (wait, on) = stop_wait(&Config::new());
    assert_eq!(wait, Some(1));
    assert!(on > 1);
    let accel_only = Config::new().with_gyro_power(SensorPower::Off);
    assert_eq!(stop_wait(&accel_only).0, None);
    // After a reset the sensors are already off: no stop wait.
    assert!(!steps(&Config::new()).contains(&Step::DelayUs(GYRO_STOP_US)));
}

#[test]
fn invalid_config_gives_no_plan() {
    let bad = Config::new().with_accel_power(SensorPower::Standby);
    assert_eq!(Plan::after_reset(&bad), Err(ConfigError::AccelStandby));
    assert_eq!(Plan::reconfigure(&bad), Err(ConfigError::AccelStandby));
}

#[test]
fn plan_accessors_and_iteration() {
    let p = plan(&Config::new());
    assert_eq!(p.len(), 19);
    assert!(!p.is_empty());
    assert_eq!((&p).into_iter().count(), p.len());
    assert_eq!(p.steps().len(), p.len());
}

/// The most options on: every optional group of the longest plan.
fn busiest() -> Config {
    Config::new()
        .with_clock(ClockSource::ExternalClkin)
        .with_rtc_align(true)
        .with_accel_bandwidth(UiBandwidth::OdrDiv4)
        .with_gyro_bandwidth(UiBandwidth::OdrDiv4)
        .with_spi_slew(SpiSlew::Ns38)
        .with_int1(
            Int1Config::new()
                .with_pulse_width(PulseWidth::Us100)
                .with_fifo_threshold(true)
                .with_data_ready(true),
        )
        .with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Stream)
                .with_watermark(3)
                .with_timestamp(true)
                .with_timestamp_resolution(TimestampResolution::Us16)
                .with_accel_decimation(Decimation::Div2)
                .with_gyro_decimation(Decimation::Div2),
        )
}

#[test]
fn longest_plans_fit_with_room_to_spare() {
    // `Plan::push` reports overflow as `ConfigError::PlanOverflow` (see the test in plan.rs), so
    // the largest plan must stay below the capacity.
    let c = busiest();
    let r = Plan::reconfigure(&c).unwrap();
    assert!(r.len() < PLAN_CAPACITY, "{}", r.len());
    let c = busiest()
        .with_clock(ClockSource::Internal)
        .with_rtc_align(false);
    let r = Plan::reconfigure(&c).unwrap();
    assert!(r.len() < PLAN_CAPACITY, "{}", r.len());
    assert!(plan(&c).len() < PLAN_CAPACITY);
}
