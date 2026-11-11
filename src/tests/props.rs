//! Property tests (D5): every configuration that validates yields plans that keep reserved bits at
//! their reset value, write fields in range and agree with the configuration; the two plans reach
//! the same device state; an invalid configuration yields no plan.

use proptest::prelude::*;
use proptest::sample::select;

use super::model::{DIRECT, Device, IREG, RTC};
use crate::config::*;
use crate::plan::{PLAN_CAPACITY, Plan};
use crate::reg;

const ACCEL_FS: [AccelFullScale; 5] = [
    AccelFullScale::G32,
    AccelFullScale::G16,
    AccelFullScale::G8,
    AccelFullScale::G4,
    AccelFullScale::G2,
];
const GYRO_FS: [GyroFullScale; 9] = [
    GyroFullScale::Dps4000,
    GyroFullScale::Dps2000,
    GyroFullScale::Dps1000,
    GyroFullScale::Dps500,
    GyroFullScale::Dps250,
    GyroFullScale::Dps125,
    GyroFullScale::Dps62_5,
    GyroFullScale::Dps31_25,
    GyroFullScale::Dps15_625,
];
const ODR: [Odr; 13] = [
    Odr::Hz6400,
    Odr::Hz3200,
    Odr::Hz1600,
    Odr::Hz800,
    Odr::Hz400,
    Odr::Hz200,
    Odr::Hz100,
    Odr::Hz50,
    Odr::Hz25,
    Odr::Hz12_5,
    Odr::Hz6_25,
    Odr::Hz3_125,
    Odr::Hz1_5625,
];
const BANDWIDTH: [UiBandwidth; 7] = [
    UiBandwidth::Bypass,
    UiBandwidth::OdrDiv4,
    UiBandwidth::OdrDiv8,
    UiBandwidth::OdrDiv16,
    UiBandwidth::OdrDiv32,
    UiBandwidth::OdrDiv64,
    UiBandwidth::OdrDiv128,
];
const DECIMATION: [Decimation; 13] = [
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
const SLEW: [SpiSlew; 7] = [
    SpiSlew::Ns38,
    SpiSlew::Ns14,
    SpiSlew::Ns10,
    SpiSlew::Ns7,
    SpiSlew::Ns5,
    SpiSlew::Ns4,
    SpiSlew::Ns0_5,
];

fn power() -> impl Strategy<Value = SensorPower> {
    prop_oneof![
        8 => Just(SensorPower::LowNoise),
        1 => Just(SensorPower::Off),
        1 => Just(SensorPower::Standby),
    ]
}

fn odr() -> impl Strategy<Value = Odr> {
    prop_oneof![9 => select(ODR[..10].to_vec()), 1 => select(ODR.to_vec())]
}

fn anti_alias() -> impl Strategy<Value = Option<AntiAlias>> {
    select(vec![
        None,
        Some(AntiAlias::Off),
        Some(AntiAlias::Fir),
        Some(AntiAlias::InterpolatorFir),
    ])
}

fn sensors() -> impl Strategy<Value = Config> {
    (
        (select(ACCEL_FS.to_vec()), select(GYRO_FS.to_vec())),
        (odr(), odr()),
        (power(), power()),
        (select(BANDWIDTH.to_vec()), select(BANDWIDTH.to_vec())),
        (anti_alias(), anti_alias()),
    )
        .prop_map(|((af, gf), (ao, go), (ap, gp), (ab, gb), (aa, ga))| {
            Config::new()
                .with_accel_fs(af)
                .with_gyro_fs(gf)
                .with_accel_odr(ao)
                .with_gyro_odr(go)
                .with_accel_power(ap)
                .with_gyro_power(gp)
                .with_accel_bandwidth(ab)
                .with_gyro_bandwidth(gb)
                .with_accel_anti_alias(aa)
                .with_gyro_anti_alias(ga)
        })
}

fn int1() -> impl Strategy<Value = Int1Config> {
    (
        (any::<bool>(), any::<bool>(), any::<bool>(), any::<bool>()),
        select(vec![Int1Mode::Pulse, Int1Mode::Latched]),
        select(vec![Int1Polarity::ActiveLow, Int1Polarity::ActiveHigh]),
        select(vec![Int1Drive::PushPull, Int1Drive::OpenDrain]),
        select(vec![PulseWidth::Us100, PulseWidth::Us8]),
    )
        .prop_map(|((dr, th, fu, ag), mode, pol, drive, pw)| {
            Int1Config::new()
                .with_data_ready(dr)
                .with_fifo_threshold(th)
                .with_fifo_full(fu)
                .with_agc_ready(ag)
                .with_mode(mode)
                .with_polarity(pol)
                .with_drive(drive)
                .with_pulse_width(pw)
        })
}

fn fifo() -> impl Strategy<Value = FifoConfig> {
    (
        select(vec![FifoMode::Off, FifoMode::Stream, FifoMode::StopOnFull]),
        select(vec![FifoFormat::Standard, FifoFormat::HighResolution]),
        (any::<bool>(), any::<bool>(), any::<bool>()),
        select(vec![TimestampResolution::Us1, TimestampResolution::Us16]),
        prop_oneof![2 => Just(0u16), 3 => 1u16..100, 1 => 0u16..9000],
        select(vec![FifoDepth::Kib2, FifoDepth::Kib8]),
        (select(DECIMATION.to_vec()), select(DECIMATION.to_vec())),
    )
        .prop_map(|(mode, format, (a, g, t), res, wm, depth, (ad, gd))| {
            FifoConfig::new()
                .with_mode(mode)
                .with_format(format)
                .with_accel(a)
                .with_gyro(g)
                .with_timestamp(t)
                .with_timestamp_resolution(res)
                .with_watermark(wm)
                .with_depth(depth)
                .with_accel_decimation(ad)
                .with_gyro_decimation(gd)
        })
}

pub(crate) fn config() -> impl Strategy<Value = Config> {
    (
        sensors(),
        select(vec![ClockSource::Internal, ClockSource::ExternalClkin]),
        any::<bool>(),
        int1(),
        fifo(),
        select(SLEW.to_vec()),
    )
        .prop_map(|(c, clock, align, int1, fifo, slew)| {
            c.with_clock(clock)
                .with_rtc_align(align)
                .with_int1(int1)
                .with_fifo(fifo)
                .with_spi_slew(slew)
        })
}

fn settings() -> ProptestConfig {
    ProptestConfig {
        cases: 600,
        max_global_rejects: 1_000_000,
        ..ProptestConfig::default()
    }
}

fn run(plan: &Plan, mut d: Device) -> Device {
    for s in plan {
        d.exec(s);
    }
    d
}

/// The managed registers with the mask of the bits that are state (RTC_ALIGN is a command and
/// SIFS_I3C_STC_CFG is only touched on CLKIN, so neither is compared across plans).
type Direct = (u8, u8, u8);
type Indirect = (u16, u8, u8);

fn managed() -> (Vec<Direct>, Vec<Indirect>) {
    let direct = DIRECT.into_iter().chain([RTC]).collect();
    let ireg = IREG.into_iter().collect();
    (direct, ireg)
}

fn bit(v: u8, mask: u8) -> bool {
    v & mask != 0
}

/// Checks the final register state against the configuration, field by field.
fn check_fields(c: &Config, d: &Device) {
    let r = |a: u8| d.direct[a as usize];
    let i = |a: u16| d.read_ireg(a);
    let accel = r(reg::ACCEL_CONFIG0);
    assert!((accel >> 4) & 7 <= 4 && (3..=12).contains(&(accel & 0x0F)));
    assert_eq!((accel >> 4) & 7, c.accel_fs.code());
    assert_eq!(accel & 0x0F, c.accel_odr.code());
    assert_eq!(accel & 0x80, 0);
    let gyro = r(reg::GYRO_CONFIG0);
    assert!(gyro >> 4 <= 8 && (3..=12).contains(&(gyro & 0x0F)));
    assert_eq!(gyro >> 4, c.gyro_fs.code());
    assert_eq!(gyro & 0x0F, c.gyro_odr.code());
    let pwr = r(reg::PWR_MGMT0);
    assert_eq!(pwr >> 4, 0);
    assert_eq!((pwr >> 2) & 3, c.gyro_power.code());
    assert_eq!(pwr & 3, c.accel_power.code());
    assert_ne!(pwr & 3, 1);
    let int0 = r(reg::INT1_CONFIG0);
    assert_eq!(bit(int0, reg::DRDY), c.int1.data_ready);
    assert_eq!(bit(int0, reg::FIFO_THS), c.int1.fifo_threshold);
    assert_eq!(bit(int0, reg::FIFO_FULL), c.int1.fifo_full);
    assert_eq!(int0 & reg::AP_FSYNC, 0);
    assert_eq!(bit(int0, reg::AP_AGC_RDY), c.int1.agc_ready);
    assert_eq!(
        int0 & (reg::RESET_DONE | reg::AUX1_AGC_RDY | reg::AUX1_DRDY),
        0
    );
    let int2 = r(reg::INT1_CONFIG2);
    assert_eq!(
        bit(int2, reg::INT1_DRIVE_OPEN_DRAIN),
        c.int1.drive == Int1Drive::OpenDrain
    );
    assert_eq!(
        bit(int2, reg::INT1_MODE_LATCHED),
        c.int1.mode == Int1Mode::Latched
    );
    assert_eq!(
        bit(int2, reg::INT1_POLARITY_HIGH),
        c.int1.polarity == Int1Polarity::ActiveHigh
    );
    assert_eq!(i(reg::INT_PULSE_MIN_ON_INTF1), c.int1.pulse_width.code());
    let f = &c.fifo;
    let cfg0 = r(reg::FIFO_CONFIG0);
    let cfg3 = r(reg::FIFO_CONFIG3);
    if f.enabled() {
        assert_eq!(cfg0 >> 6, f.mode.code());
        assert!(cfg0 >> 6 <= 2);
        assert_eq!(cfg0 & 0x3F, f.depth.code());
        assert!(matches!(cfg0 & 0x3F, 0x07 | 0x1F));
        assert_eq!(
            u16::from_le_bytes([r(reg::FIFO_CONFIG1_0), r(reg::FIFO_CONFIG1_1)]),
            if f.watermark == 0 { 0 } else { f.watermark + 1 }
        );
        assert!(bit(cfg3, reg::FIFO_IF_EN));
        let hires = f.format == FifoFormat::HighResolution;
        assert_eq!(bit(cfg3, reg::FIFO_HIRES_EN), hires);
        assert_eq!(bit(cfg3, reg::FIFO_ACCEL_EN), f.accel || hires);
        assert_eq!(bit(cfg3, reg::FIFO_GYRO_EN), f.gyro || hires);
        assert_eq!(
            bit(r(reg::FIFO_CONFIG4), reg::FIFO_TMST_FSYNC_EN),
            f.timestamp
        );
        assert!(bit(r(reg::FIFO_CONFIG2), reg::FIFO_WR_WM_GT_TH));
        let dec = r(reg::ODR_DECIMATE_CONFIG);
        assert_eq!(dec & 0x0F, f.accel_decimation.code());
        assert_eq!(dec >> 4, f.gyro_decimation.code());
        assert!(dec & 0x0F <= 12 && dec >> 4 <= 12);
    } else {
        assert_eq!(cfg0, 0);
        assert_eq!(cfg3, 0);
        assert_eq!(r(reg::FIFO_CONFIG4), 0);
        assert_eq!(r(reg::ODR_DECIMATE_CONFIG), 0);
    }
    assert_eq!(
        bit(r(reg::TMST_WOM_CONFIG), reg::TMST_RESOL_16US),
        f.timestamp_resolution == TimestampResolution::Us16
    );
    assert_eq!(r(reg::FSYNC_CONFIG0), reg::FSYNC_CONFIG0_RESET);
    let smc = i(reg::SMC_CONTROL_0);
    assert_eq!(bit(smc, reg::TMST_EN), f.enabled() && f.timestamp);
    assert!(!bit(smc, reg::TMST_FSYNC_EN));
    let clkin = c.clock == ClockSource::ExternalClkin;
    assert_eq!(bit(smc, reg::ACCEL_LP_CLK_SEL), clkin);
    assert_eq!(bit(r(reg::RTC_CONFIG), reg::RTC_MODE), clkin);
    assert_eq!((r(reg::DRIVE_CONFIG0) >> 1) & 7, c.spi_slew.code());
    let accel_src = i(reg::IPREG_SYS2_REG_123) & reg::ACCEL_SRC_CTRL_MASK;
    let gyro_src =
        (i(reg::IPREG_SYS1_REG_166) & reg::GYRO_SRC_CTRL_MASK) >> reg::GYRO_SRC_CTRL_SHIFT;
    assert!(accel_src <= 2 && gyro_src <= 2);
    let default_src = if clkin { 2 } else { 0 };
    assert_eq!(
        accel_src,
        c.accel_anti_alias.map_or(default_src, AntiAlias::code)
    );
    assert_eq!(
        gyro_src,
        c.gyro_anti_alias.map_or(default_src, AntiAlias::code)
    );
    assert_eq!(i(reg::IPREG_SYS2_REG_131) & 7, c.accel_bandwidth.code());
    assert_eq!(i(reg::IPREG_SYS1_REG_172) & 7, c.gyro_bandwidth.code());
}

/// Reserved bits are at their reset value in every managed register.
fn check_reserved(d: &Device) {
    let (direct, ireg) = managed();
    for (a, mask, reset) in direct {
        let mask = if a == reg::RTC_CONFIG {
            reg::RTC_CONFIG_FIELDS
        } else {
            mask
        };
        assert_eq!(
            d.direct[a as usize] & !mask,
            reset & !mask,
            "register {a:#04x}"
        );
    }
    for (a, mask, reset) in ireg {
        assert_eq!(d.read_ireg(a) & !mask, reset & !mask, "indirect {a:#06x}");
    }
}

/// Whether two devices hold the same state in the compared bits of every managed register.
fn same_state(a: &Device, b: &Device) {
    let (direct, ireg) = managed();
    for (r, mask, _) in direct {
        let mask = if r == reg::RTC_CONFIG {
            reg::RTC_MODE
        } else {
            mask
        };
        assert_eq!(
            a.direct[r as usize] & mask,
            b.direct[r as usize] & mask,
            "register {r:#04x}"
        );
    }
    for (r, mask, _) in ireg {
        if r == reg::SIFS_I3C_STC_CFG {
            continue;
        }
        assert_eq!(
            a.read_ireg(r) & mask,
            b.read_ireg(r) & mask,
            "indirect {r:#06x}"
        );
    }
}

proptest! {
    #![proptest_config(settings())]

    #[test]
    fn valid_config_gives_plans_with_reserved_bits_kept_and_fields_in_range(c in config()) {
        prop_assume!(c.validate().is_ok());
        let after_reset = Plan::after_reset(&c).unwrap();
        let reconfigure = Plan::reconfigure(&c).unwrap();
        prop_assert!(after_reset.len() < PLAN_CAPACITY);
        prop_assert!(reconfigure.len() < PLAN_CAPACITY);
        for plan in [after_reset, reconfigure] {
            let d = run(&plan, Device::reset());
            check_reserved(&d);
            check_fields(&c, &d);
        }
    }

    #[test]
    fn both_plans_reach_the_same_state(c in config()) {
        prop_assume!(c.validate().is_ok());
        let a = run(&Plan::after_reset(&c).unwrap(), Device::reset());
        let b = run(&Plan::reconfigure(&c).unwrap(), Device::reset());
        same_state(&a, &b);
    }

    #[test]
    fn reconfigure_overwrites_whatever_the_device_held(
        c in config(),
        junk in prop::collection::vec(any::<u8>(), 23),
    ) {
        prop_assume!(c.validate().is_ok());
        let (direct, ireg) = managed();
        let mut dirty = Device::reset();
        let mut junk = junk.into_iter();
        for (a, mask, reset) in direct {
            let j = junk.next().unwrap_or(0);
            dirty.direct[a as usize] = (reset & !mask) | (j & mask);
        }
        for (a, mask, reset) in ireg {
            let j = junk.next().unwrap_or(0);
            dirty.ireg.insert(a, (reset & !mask) | (j & mask));
        }
        let clean = run(&Plan::reconfigure(&c).unwrap(), Device::reset());
        let after = run(&Plan::reconfigure(&c).unwrap(), dirty);
        same_state(&clean, &after);
        check_reserved(&after);
    }

    #[test]
    fn no_plan_for_an_invalid_config(c in config()) {
        let v = c.validate();
        prop_assert_eq!(Plan::after_reset(&c).map(|_| ()), v);
        prop_assert_eq!(Plan::reconfigure(&c).map(|_| ()), v);
    }

    #[test]
    fn default_with_any_valid_rates_and_scales_plans_exactly_the_init_writes(
        af in select(ACCEL_FS.to_vec()),
        gf in select(GYRO_FS.to_vec()),
        ao in select(ODR[..10].to_vec()),
        go in select(ODR[..10].to_vec()),
        dr in any::<bool>(),
    ) {
        let c = Config::new()
            .with_accel_fs(af)
            .with_gyro_fs(gf)
            .with_accel_odr(ao)
            .with_gyro_odr(go)
            .with_int1(Int1Config::new().with_data_ready(dr));
        let plan = Plan::after_reset(&c).unwrap();
        prop_assert_eq!(plan.len(), 19);
    }
}
