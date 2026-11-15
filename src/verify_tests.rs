//! Host tests of `verify` and `recover` and of the owned-register list, on the fake bus of
//! [`crate::testbus`]: golden traces, counts, the detection sweep over every bit, a one-off glitch,
//! the unchanged validity state, bus errors and cancels at every point, and the blocking front-end.

use std::vec;
use std::vec::Vec;

use proptest::prelude::*;

use crate::driver_tests::{Imu, Res, Snapshot, clkin, configs, configured, r, rig, snapshot, us};
use crate::owned::{OWNED_CAPACITY, OwnedRegister, OwnedRegisters, owned_registers};
use crate::testbus::{BusFault, Event, Op, Polled, Shared, TestDelay, poll_then_drop, run};
use crate::tests::props::config;
use crate::{
    Config, Error, FifoConfig, FifoFormat, FifoMode, Int1Config, Mismatch, Odr, Offsets,
    SensorPower, Space, TimestampResolution, blocking, reg,
};

/// The owned list of `config` with the offsets at zero.
fn owned(config: &Config) -> OwnedRegisters {
    owned_registers(config, &Offsets::new())
}

/// Offsets that set every bit of every offset register at least once.
fn some_offsets() -> Offsets {
    Offsets::new()
        .with_gyro([0x1555, -0x0AAB, 1])
        .with_accel([-1, 0x2AA, -8192])
}

/// An indirect read as it appears on the bus: address write, wait, data read, wait (13.3, p55).
fn ir(addr: u16, got: u8) -> Vec<Event> {
    let [hi, lo] = addr.to_be_bytes();
    vec![
        Event::Spi(vec![Op::Write(vec![0x7C, hi, lo])]),
        us(10),
        r(0x7E, got),
        us(10),
    ]
}

fn rich() -> Config {
    Config::new()
        .with_accel_odr(Odr::Hz200)
        .with_gyro_odr(Odr::Hz200)
        .with_accel_power(SensorPower::LowNoise)
        .with_int1(Int1Config::new().with_fifo_threshold(true))
        .with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Stream)
                .with_format(FifoFormat::HighResolution)
                .with_watermark(40)
                .with_timestamp(true)
                .with_timestamp_resolution(TimestampResolution::Us16),
        )
}

/// A driver configured with `config`, by `init`, or by `init` of the default then `configure`.
fn applied(config: &Config, via_configure: bool) -> (Shared, Imu, TestDelay) {
    if via_configure {
        let (shared, mut imu, mut delay) = configured();
        run(imu.configure(&mut delay, config)).unwrap();
        (shared, imu, delay)
    } else {
        let (shared, mut imu, mut delay) = rig();
        run(imu.init(&mut delay, config)).unwrap();
        (shared, imu, delay)
    }
}

fn restore(shared: &Shared, saved: &Snapshot) {
    let mut c = shared.chip();
    c.regs = saved.0;
    c.ireg = saved.1.clone();
}

fn flip(shared: &Shared, space: Space, register: u16, bit: u32) {
    let mut c = shared.chip();
    match space {
        Space::Direct => c.regs[usize::from(register & 0xFF)] ^= 1 << bit,
        Space::Indirect => *c.ireg.entry(register).or_insert(0) ^= 1 << bit,
    }
}

fn mismatch(item: &OwnedRegister, found: u8) -> Error<BusFault> {
    Error::ConfigMismatch(Mismatch {
        space: item.space,
        register: item.register,
        mask: item.mask,
        expected: item.expected,
        found,
    })
}

#[derive(Clone, Copy, Debug)]
enum Which {
    Verify,
    Recover,
}

async fn go(imu: &mut Imu, delay: &mut TestDelay, which: Which) -> Res<()> {
    match which {
        Which::Verify => imu.verify(delay).await,
        Which::Recover => imu.recover(delay).await,
    }
}

/// The driver is still configured with `config` and `read` works.
fn assert_state(imu: &mut Imu, config: &Config) {
    assert_eq!(imu.config(), Some(config));
    assert!(run(imu.read()).is_ok());
}

// ---------------------------------------------------------------------------------------------
// The owned list (pure)
// ---------------------------------------------------------------------------------------------

fn tuples(config: &Config) -> Vec<(Space, u16, u8, u8)> {
    owned(config)
        .registers()
        .iter()
        .map(|o| (o.space, o.register, o.mask, o.expected))
        .collect()
}

#[test]
fn owned_registers_default_config_exact_list() {
    use Space::{Direct, Indirect};
    // Addresses, masks and values from reg.rs and image.rs; pages in owned.rs.
    let want = vec![
        (Direct, 0x10, 0x0F, 0x0F),
        (Direct, 0x16, 0xFF, 0x00),
        (Direct, 0x18, 0x07, 0x01),
        (Direct, 0x1B, 0x7F, 0x36),
        (Direct, 0x1C, 0xFF, 0x16),
        (Direct, 0x1D, 0xFF, 0x00),
        (Direct, 0x1E, 0xFF, 0x00),
        (Direct, 0x1F, 0xFF, 0x00),
        (Direct, 0x20, 0x08, 0x00),
        (Direct, 0x21, 0x0F, 0x00),
        (Direct, 0x22, 0x02, 0x00),
        (Direct, 0x23, 0x20, 0x00),
        (Direct, 0x26, 0x20, 0x00),
        (Direct, 0x28, 0xFF, 0x00),
        (Direct, 0x32, 0x0E, 0x0C),
        (Indirect, 0xA258, 0x11, 0x00),
        (Indirect, 0xA269, 0x07, 0x01),
        (Indirect, 0xA42A, 0xFF, 0x00),
        (Indirect, 0xA42B, 0x3F, 0x00),
        (Indirect, 0xA438, 0xFF, 0x00),
        (Indirect, 0xA439, 0x3F, 0x00),
        (Indirect, 0xA446, 0xFF, 0x00),
        (Indirect, 0xA447, 0x3F, 0x00),
        (Indirect, 0xA4A6, 0x60, 0x00),
        (Indirect, 0xA4AC, 0x07, 0x00),
        (Indirect, 0xA518, 0xFF, 0x00),
        (Indirect, 0xA519, 0x3F, 0x00),
        (Indirect, 0xA520, 0xFF, 0x00),
        (Indirect, 0xA521, 0x3F, 0x00),
        (Indirect, 0xA528, 0xFF, 0x00),
        (Indirect, 0xA529, 0x3F, 0x00),
        (Indirect, 0xA57B, 0x03, 0x00),
        (Indirect, 0xA583, 0x07, 0x00),
    ];
    assert_eq!(tuples(&Config::new()), want);
}

#[test]
fn owned_registers_external_clock_adds_the_i3c_field_and_fills_the_capacity() {
    use Space::{Direct, Indirect};
    let want = vec![
        (Direct, 0x10, 0x0F, 0x0F),
        (Direct, 0x16, 0xFF, 0x04),
        (Direct, 0x18, 0x07, 0x01),
        (Direct, 0x1B, 0x7F, 0x38),
        (Direct, 0x1C, 0xFF, 0x18),
        (Direct, 0x1D, 0xFF, 0x00),
        (Direct, 0x1E, 0xFF, 0x00),
        (Direct, 0x1F, 0xFF, 0x00),
        (Direct, 0x20, 0x08, 0x00),
        (Direct, 0x21, 0x0F, 0x00),
        (Direct, 0x22, 0x02, 0x00),
        (Direct, 0x23, 0x20, 0x00),
        (Direct, 0x26, 0x20, 0x20),
        (Direct, 0x28, 0xFF, 0x00),
        (Direct, 0x32, 0x0E, 0x0C),
        (Indirect, 0xA258, 0x11, 0x10),
        (Indirect, 0xA268, 0x04, 0x00),
        (Indirect, 0xA269, 0x07, 0x01),
        (Indirect, 0xA42A, 0xFF, 0x00),
        (Indirect, 0xA42B, 0x3F, 0x00),
        (Indirect, 0xA438, 0xFF, 0x00),
        (Indirect, 0xA439, 0x3F, 0x00),
        (Indirect, 0xA446, 0xFF, 0x00),
        (Indirect, 0xA447, 0x3F, 0x00),
        (Indirect, 0xA4A6, 0x60, 0x40),
        (Indirect, 0xA4AC, 0x07, 0x00),
        (Indirect, 0xA518, 0xFF, 0x00),
        (Indirect, 0xA519, 0x3F, 0x00),
        (Indirect, 0xA520, 0xFF, 0x00),
        (Indirect, 0xA521, 0x3F, 0x00),
        (Indirect, 0xA528, 0xFF, 0x00),
        (Indirect, 0xA529, 0x3F, 0x00),
        (Indirect, 0xA57B, 0x03, 0x02),
        (Indirect, 0xA583, 0x07, 0x00),
    ];
    let list = tuples(&clkin());
    assert_eq!(list, want);
    assert_eq!(list.len(), OWNED_CAPACITY);
    for c in [rich(), Config::new()] {
        assert!(owned(&c).len() <= OWNED_CAPACITY);
    }
}

#[test]
fn registers_len_is_empty_and_verify_transactions_agree() {
    for (c, direct, indirect) in [(Config::new(), 15, 18), (clkin(), 15, 19), (rich(), 15, 18)] {
        let o = owned(&c);
        assert_eq!(o.registers().len(), o.len());
        assert!(!o.is_empty());
        assert_eq!(o.len(), direct + indirect);
        assert_eq!(o.verify_transactions(), direct + 2 * indirect);
    }
}

// ---------------------------------------------------------------------------------------------
// Golden traces (D2, E1)
// ---------------------------------------------------------------------------------------------

fn default_verify_trace() -> Vec<Event> {
    let mut want = vec![
        r(0x10, 0x0F), // PWR_MGMT0 (16.17, p70)
        r(0x16, 0x00), // INT1_CONFIG0 (16.21, p72)
        r(0x18, 0x01), // INT1_CONFIG2 (16.23, p75)
        r(0x1B, 0x36), // ACCEL_CONFIG0 (16.26, p78)
        r(0x1C, 0x16), // GYRO_CONFIG0 (16.27, p79)
        r(0x1D, 0x00), // FIFO_CONFIG0 (16.28, p80)
        r(0x1E, 0x00), // FIFO_CONFIG1_0 (16.29, p80)
        r(0x1F, 0x00), // FIFO_CONFIG1_1 (16.30, p81)
        r(0x20, 0x20), // FIFO_CONFIG2, reset value, compared under 0x08 (16.31, p81)
        r(0x21, 0x00), // FIFO_CONFIG3 (16.32, p82)
        r(0x22, 0x00), // FIFO_CONFIG4 (16.33, p83)
        r(0x23, 0x00), // TMST_WOM_CONFIG (16.34, p84)
        r(0x26, 0x03), // RTC_CONFIG (16.37, p86)
        r(0x28, 0x00), // ODR_DECIMATE_CONFIG (16.39, p87)
        r(0x32, 0x6C), // DRIVE_CONFIG0 (16.48, p93)
    ];
    for (addr, got) in [
        (0xA258, 0x60), // SMC_CONTROL_0 (19.47, p178)
        (0xA269, 0x01), // INT_PULSE_MIN_ON_INTF0 (19.52, p181)
        (0xa42a, 0x00), // GYRO_X_OFFUSER low (20.1, p189-p190)
        (0xa42b, 0x00), // GYRO_X_OFFUSER high (20.2, p189-p190)
        (0xa438, 0x00), // GYRO_Y_OFFUSER low (20.3, p189-p190)
        (0xa439, 0x00), // GYRO_Y_OFFUSER high (20.4, p189-p190)
        (0xa446, 0x00), // GYRO_Z_OFFUSER low (20.5, p189-p190)
        (0xa447, 0x00), // GYRO_Z_OFFUSER high (20.6, p189-p190)
        (0xA4A6, 0x1B), // IPREG_SYS1_REG_166 (20.7, p190)
        (0xA4AC, 0x80), // IPREG_SYS1_REG_172 (20.11, p192)
        (0xa518, 0x00), // ACCEL_X_OFFUSER low (21.1, p193-p194)
        (0xa519, 0x00), // ACCEL_X_OFFUSER high (21.2, p193-p194)
        (0xa520, 0x00), // ACCEL_Y_OFFUSER low (21.3, p193-p194)
        (0xa521, 0x00), // ACCEL_Y_OFFUSER high (21.4, p193-p194)
        (0xa528, 0x00), // ACCEL_Z_OFFUSER low (21.5, p193-p194)
        (0xa529, 0x00), // ACCEL_Z_OFFUSER high (21.6, p193-p194)
        (0xA57B, 0x14), // IPREG_SYS2_REG_123 (21.7, p194)
        (0xA583, 0x00), // IPREG_SYS2_REG_131 (21.10, p196)
    ] {
        want.extend(ir(addr, got));
    }
    want
}

fn clkin_verify_trace() -> Vec<Event> {
    let mut want = vec![
        r(0x10, 0x0F),
        r(0x16, 0x04),
        r(0x18, 0x01),
        r(0x1B, 0x38),
        r(0x1C, 0x18),
        r(0x1D, 0x00),
        r(0x1E, 0x00),
        r(0x1F, 0x00),
        r(0x20, 0x20),
        r(0x21, 0x00),
        r(0x22, 0x00),
        r(0x23, 0x00),
        r(0x26, 0x23),
        r(0x28, 0x00),
        r(0x32, 0x6C),
    ];
    for (addr, got) in [
        (0xA258, 0x70),
        (0xA268, 0x23),
        (0xA269, 0x01),
        (0xa42a, 0x00),
        (0xa42b, 0x00),
        (0xa438, 0x00),
        (0xa439, 0x00),
        (0xa446, 0x00),
        (0xa447, 0x00),
        (0xA4A6, 0x5B),
        (0xA4AC, 0x80),
        (0xa518, 0x00),
        (0xa519, 0x00),
        (0xa520, 0x00),
        (0xa521, 0x00),
        (0xa528, 0x00),
        (0xa529, 0x00),
        (0xA57B, 0x16),
        (0xA583, 0x00),
    ] {
        want.extend(ir(addr, got));
    }
    want
}

fn who_am_i_frame() -> Vec<Event> {
    // The one-byte frame of the board measurement: the read address of WHO_AM_I (16.79, p107), then
    // the identity read.
    vec![Event::Spi(vec![Op::Write(vec![0xF2])]), r(0x72, 0xE9)]
}

#[test]
fn verify_golden_trace_and_count_default_and_external_clock() {
    for (config, want, count) in [
        (Config::new(), default_verify_trace(), 51),
        (clkin(), clkin_verify_trace(), 53),
    ] {
        let (shared, mut imu, mut delay) = applied(&config, false);
        shared.arm(None, None);
        assert_eq!(run(imu.verify(&mut delay)), Ok(()));
        assert_eq!(shared.log(), want);
        assert_eq!(shared.transactions(), count);
        assert_eq!(shared.violations(), 0);
    }
}

#[test]
fn recover_golden_trace_and_count_default_and_external_clock() {
    for (config, verify, count) in [
        (Config::new(), default_verify_trace(), 53),
        (clkin(), clkin_verify_trace(), 55),
    ] {
        let (shared, mut imu, mut delay) = applied(&config, false);
        shared.arm(None, None);
        assert_eq!(run(imu.recover(&mut delay)), Ok(()));
        let mut want = who_am_i_frame();
        want.extend(verify);
        assert_eq!(shared.log(), want);
        assert_eq!(shared.transactions(), count);
        assert_eq!(shared.violations(), 0);
    }
}

#[test]
fn verify_not_configured_sends_nothing() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.verify(&mut delay)), Err(Error::NotConfigured));
    assert_eq!(shared.log(), vec![]);
    assert_eq!(imu.config(), None);
}

// ---------------------------------------------------------------------------------------------
// After init and after configure
// ---------------------------------------------------------------------------------------------

fn check_ok_and_count(config: &Config, via_configure: bool) -> Result<(), TestCaseError> {
    let (shared, mut imu, mut delay) = applied(config, via_configure);
    let n = owned(config).verify_transactions();
    shared.arm(None, None);
    prop_assert_eq!(run(imu.verify(&mut delay)), Ok(()));
    prop_assert_eq!(shared.transactions(), n);
    prop_assert_eq!(shared.violations(), 0);
    prop_assert_eq!(imu.config(), Some(config));
    Ok(())
}

#[test]
fn verify_ok_after_init_and_after_configure_for_unlike_configs() {
    for c in [Config::new(), clkin(), rich()] {
        for via_configure in [false, true] {
            check_ok_and_count(&c, via_configure).unwrap();
        }
    }
    // Configure over a driver that held another configuration.
    let (_shared, mut imu, mut delay) = applied(&rich(), false);
    run(imu.configure(&mut delay, &clkin())).unwrap();
    assert_eq!(run(imu.verify(&mut delay)), Ok(()));
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 48,
        max_global_rejects: 1_000_000,
        ..ProptestConfig::default()
    })]

    #[test]
    fn verify_ok_after_init_and_configure_for_any_valid_config(c in config(), d in config()) {
        prop_assume!(c.validate().is_ok() && d.validate().is_ok());
        check_ok_and_count(&c, false)?;
        let (shared, mut imu, mut delay) = applied(&c, false);
        run(imu.configure(&mut delay, &d)).unwrap();
        shared.arm(None, None);
        prop_assert_eq!(run(imu.verify(&mut delay)), Ok(()));
        prop_assert_eq!(shared.transactions(), owned(&d).verify_transactions());
    }
}

// ---------------------------------------------------------------------------------------------
// Detection sweep
// ---------------------------------------------------------------------------------------------

fn detection_sweep(config: &Config, offsets: &Offsets) {
    let (shared, mut imu, mut delay) = applied(config, false);
    run(imu.set_offsets(&mut delay, offsets)).unwrap();
    assert_eq!(imu.offsets(), Some(*offsets));
    let owned = owned_registers(config, offsets);
    let n = owned.verify_transactions();
    let saved = snapshot(&shared);
    let mut before = 0;
    for item in owned.registers() {
        let cost = if item.space == Space::Direct { 1 } else { 2 };
        before += cost;
        for bit in 0..8u32 {
            restore(&shared, &saved);
            flip(&shared, item.space, item.register, bit);
            shared.arm(None, None);
            let got = run(imu.verify(&mut delay));
            if item.mask >> bit & 1 == 1 {
                // `found` is the whole register byte as read, not masked.
                let raw = match item.space {
                    Space::Direct => saved.0[usize::from(item.register & 0xFF)],
                    Space::Indirect => saved.1.get(&item.register).copied().unwrap_or(0),
                };
                let found = raw ^ (1 << bit);
                assert_eq!(got, Err(mismatch(item, found)), "{item:?} bit {bit}");
                // The reads up to this register, then its read once more.
                assert_eq!(shared.transactions(), before + cost, "{item:?} bit {bit}");
            } else {
                assert_eq!(got, Ok(()), "{item:?} bit {bit} is outside the mask");
                assert_eq!(shared.transactions(), n);
            }
            assert_eq!(imu.config(), Some(config));
        }
    }
    let is_owned = |space: Space, register: u16| {
        owned
            .registers()
            .iter()
            .any(|o| o.space == space && o.register == register)
    };
    for register in 0..=255u16 {
        if is_owned(Space::Direct, register) {
            continue;
        }
        for bit in 0..8u32 {
            restore(&shared, &saved);
            flip(&shared, Space::Direct, register, bit);
            shared.arm(None, None);
            assert_eq!(
                run(imu.verify(&mut delay)),
                Ok(()),
                "{register:#x} bit {bit}"
            );
        }
    }
    let mut others: Vec<u16> = saved.1.keys().copied().collect();
    others.extend([0xA200, 0xA25A, 0xA400, 0xA500, 0xA4A7, 0xA584, 0x0000]);
    for register in others {
        if is_owned(Space::Indirect, register) {
            continue;
        }
        for bit in 0..8u32 {
            restore(&shared, &saved);
            flip(&shared, Space::Indirect, register, bit);
            shared.arm(None, None);
            assert_eq!(
                run(imu.verify(&mut delay)),
                Ok(()),
                "{register:#x} bit {bit}"
            );
        }
    }
}

#[test]
fn verify_detects_every_bit_of_every_owned_mask_and_ignores_every_other_bit() {
    for c in [Config::new(), clkin(), rich()] {
        detection_sweep(&c, &Offsets::new());
        detection_sweep(&c, &some_offsets());
    }
}

#[test]
fn verify_masks_leave_out_the_flush_the_align_and_the_reserved_bits() {
    let c = Config::new();
    let owned = owned(&c);
    let mask = |register: u16| {
        owned
            .registers()
            .iter()
            .find(|o| o.register == register)
            .map(|o| o.mask)
    };
    assert_eq!(
        mask(u16::from(reg::FIFO_CONFIG2)).map(|m| m & reg::FIFO_FLUSH),
        Some(0)
    );
    assert_eq!(
        mask(u16::from(reg::RTC_CONFIG)).map(|m| m & reg::RTC_ALIGN),
        Some(0)
    );
    assert_eq!(mask(u16::from(reg::REG_MISC2)), None);
    assert_eq!(mask(u16::from(reg::INT1_STATUS0)), None);
}

// ---------------------------------------------------------------------------------------------
// One-off glitch
// ---------------------------------------------------------------------------------------------

#[test]
fn verify_one_off_corrupt_read_costs_exactly_one_extra_read() {
    for config in configs() {
        let n = owned(&config).verify_transactions();
        let mut hits = 0;
        for t in 1..=n {
            let (shared, mut imu, mut delay) = applied(&config, false);
            shared.arm(None, None);
            shared.chip().glitch_at = Some(t);
            let got = run(imu.verify(&mut delay));
            assert_eq!(got, Ok(()), "glitch in transaction {t}");
            let consumed = shared.chip().glitch_at.is_none();
            if consumed {
                hits += 1;
                let log = shared.log();
                let first_data_reads = log
                    .iter()
                    .filter(|e| matches!(e, Event::Spi(ops) if ops.len() == 2))
                    .count();
                let per_register = owned(&config).len();
                assert_eq!(first_data_reads, per_register + 1, "one extra read at {t}");
                let owned = owned(&config);
                let mut first = 1;
                let mut extra = None;
                for o in owned.registers() {
                    let txns = if o.space == Space::Direct { 1 } else { 2 };
                    if (first..first + txns).contains(&t) {
                        extra = Some(txns);
                    }
                    first += txns;
                }
                assert_eq!(
                    shared.transactions(),
                    n + extra.unwrap(),
                    "re-read of the register hit at {t}"
                );
            } else {
                assert_eq!(shared.transactions(), n, "address write at {t}");
            }
        }
        assert_eq!(hits, owned(&config).len());
    }
}

// ---------------------------------------------------------------------------------------------
// The validity state is never changed (A3)
// ---------------------------------------------------------------------------------------------

#[test]
fn verify_and_recover_leave_config_and_validity_unchanged_in_every_outcome() {
    for which in [Which::Verify, Which::Recover] {
        let c = Config::new();
        // Ok.
        let (shared, mut imu, mut delay) = applied(&c, false);
        shared.arm(None, None);
        assert_eq!(run(go(&mut imu, &mut delay, which)), Ok(()));
        assert_state(&mut imu, &c);
        // Mismatch.
        flip(&shared, Space::Direct, 0x1B, 0);
        flip(&shared, Space::Indirect, reg::IPREG_SYS2_REG_131, 1);
        shared.arm(None, None);
        assert!(matches!(
            run(go(&mut imu, &mut delay, which)),
            Err(Error::ConfigMismatch(m)) if m.register == 0x1B
        ));
        assert_state(&mut imu, &c);
        // Bus error and cancel, early and late.
        for at in [1usize, 2, 9, 20, 27, 51] {
            let (shared, mut imu, mut delay) = applied(&c, false);
            shared.arm(Some(at), None);
            assert_eq!(
                run(go(&mut imu, &mut delay, which)),
                Err(Error::Bus(BusFault))
            );
            assert_state(&mut imu, &c);
            shared.arm(None, Some(at as u32));
            assert!(matches!(
                poll_then_drop(go(&mut imu, &mut delay, which)),
                Polled::Cancelled
            ));
            assert_state(&mut imu, &c);
        }
        // Not configured stays not configured.
        let (shared, mut imu, mut delay) = rig();
        shared.arm(None, None);
        assert_eq!(
            run(go(&mut imu, &mut delay, which)),
            Err(Error::NotConfigured)
        );
        assert_eq!(imu.config(), None);
        assert_eq!(run(imu.read()), Err(Error::NotConfigured));
    }
}

// ---------------------------------------------------------------------------------------------
// D3 bus error at every transaction, A2/D4 cancel at every await
// ---------------------------------------------------------------------------------------------

fn bus_error_sweep(which: Which, config: &Config) -> usize {
    let mut idx = 0;
    loop {
        idx += 1;
        let (shared, mut imu, mut delay) = applied(config, false);
        shared.arm(Some(idx), None);
        match run(go(&mut imu, &mut delay, which)) {
            Ok(()) => return idx - 1,
            Err(Error::Bus(BusFault)) => {}
            Err(e) => panic!("{e:?} at transaction {idx}"),
        }
        assert_eq!(shared.violations(), 0);
        assert_state(&mut imu, config);
        shared.arm(None, None);
        assert_eq!(
            run(go(&mut imu, &mut delay, which)),
            Ok(()),
            "{which:?} at {idx}"
        );
        assert_state(&mut imu, config);
    }
}

fn cancel_sweep(which: Which, config: &Config) -> u32 {
    let reference = {
        let (shared, mut imu, mut delay) = applied(config, false);
        run(go(&mut imu, &mut delay, which)).unwrap();
        snapshot(&shared)
    };
    let mut k = 0;
    loop {
        k += 1;
        let (shared, mut imu, mut delay) = applied(config, false);
        shared.arm(None, Some(k));
        match poll_then_drop(go(&mut imu, &mut delay, which)) {
            Polled::Done(out) => {
                assert_eq!(out, Ok(()));
                return k - 1;
            }
            Polled::Cancelled => {}
        }
        shared.arm(None, None);
        assert_state(&mut imu, config);
        assert_eq!(
            run(go(&mut imu, &mut delay, which)),
            Ok(()),
            "{which:?} at {k}"
        );
        assert_eq!(snapshot(&shared), reference, "{which:?} cancelled at {k}");
    }
}

#[test]
fn verify_bus_error_at_every_transaction_then_the_same_call_works() {
    assert_eq!(bus_error_sweep(Which::Verify, &Config::new()), 51);
    assert_eq!(bus_error_sweep(Which::Verify, &clkin()), 53);
}

#[test]
fn verify_cancel_at_every_await_then_the_same_call_works() {
    // 15 direct awaits + 4 per indirect register (transaction, wait, transaction, wait).
    assert_eq!(cancel_sweep(Which::Verify, &Config::new()), 15 + 18 * 4);
    assert_eq!(cancel_sweep(Which::Verify, &clkin()), 15 + 19 * 4);
}

#[test]
fn recover_bus_error_at_every_transaction_then_the_same_call_works() {
    assert_eq!(bus_error_sweep(Which::Recover, &Config::new()), 53);
    assert_eq!(bus_error_sweep(Which::Recover, &clkin()), 55);
}

#[test]
fn recover_cancel_at_every_await_then_the_same_call_works() {
    assert_eq!(
        cancel_sweep(Which::Recover, &Config::new()),
        2 + 15 + 18 * 4
    );
    assert_eq!(cancel_sweep(Which::Recover, &clkin()), 2 + 15 + 19 * 4);
}

// ---------------------------------------------------------------------------------------------
// recover: not configured, wrong identity
// ---------------------------------------------------------------------------------------------

#[test]
fn recover_not_configured_writes_the_frame_checks_identity_and_says_so() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.recover(&mut delay)), Err(Error::NotConfigured));
    assert_eq!(shared.log(), who_am_i_frame());
    assert_eq!(shared.transactions(), 2);
    assert_eq!(imu.config(), None);
}

#[test]
fn recover_wrong_identity_is_the_wrong_id_error_and_stops_there() {
    for configured_first in [false, true] {
        let (shared, mut imu, mut delay) = if configured_first {
            configured()
        } else {
            rig()
        };
        shared.chip().regs[reg::WHO_AM_I as usize] = 0x42;
        shared.arm(None, None);
        assert_eq!(
            run(imu.recover(&mut delay)),
            Err(Error::WrongId { found: 0x42 })
        );
        assert_eq!(
            shared.log(),
            vec![Event::Spi(vec![Op::Write(vec![0xF2])]), r(0x72, 0x42)]
        );
        assert_eq!(imu.config().is_some(), configured_first);
    }
}

#[test]
fn recover_writes_no_register_except_the_indirect_address() {
    let (shared, mut imu, mut delay) = applied(&clkin(), false);
    let before = snapshot(&shared);
    shared.arm(None, None);
    run(imu.recover(&mut delay)).unwrap();
    let after = snapshot(&shared);
    for i in 0..256 {
        if i != 0x7C && i != 0x7D {
            assert_eq!(before.0[i], after.0[i], "register {i:#x}");
        }
    }
    assert_eq!(before.1, after.1);
    let writes = shared
        .log()
        .into_iter()
        .filter(|e| matches!(e, Event::Spi(ops) if matches!(ops.first(), Some(Op::Write(w)) if w.first().is_some_and(|b| b & 0x80 == 0) && w.len() > 1)))
        .count();
    assert_eq!(writes, 19);
}

#[test]
fn recover_after_a_stale_byte_style_misframe_gives_the_right_answer() {
    // The fake cannot model a receive FIFO; what the driver controls is that the first frame after
    // a cancel is the one-byte write, whatever came before.
    let (shared, mut imu, mut delay) = applied(&Config::new(), false);
    shared.arm(None, Some(3));
    assert!(matches!(
        poll_then_drop(imu.verify(&mut delay)),
        Polled::Cancelled
    ));
    shared.arm(None, None);
    assert_eq!(run(imu.recover(&mut delay)), Ok(()));
    assert_eq!(shared.log().first(), who_am_i_frame().first());
}

// ---------------------------------------------------------------------------------------------
// A11: the blocking front-end
// ---------------------------------------------------------------------------------------------

#[test]
fn verify_and_recover_through_blocking_match_the_async_driver() {
    for config in configs() {
        for scenario in 0..4 {
            let (a_shared, mut a_imu, mut a_delay) = if scenario == 3 {
                rig()
            } else {
                applied(&config, false)
            };
            let b_shared = Shared::new();
            let mut b_imu = blocking::Icm45686::new(b_shared.bus());
            let mut b_delay = b_shared.delay();
            if scenario != 3 {
                b_imu.init(&mut b_delay, &config).unwrap();
            }
            if scenario == 1 {
                flip(&a_shared, Space::Direct, 0x1C, 3);
                flip(&b_shared, Space::Direct, 0x1C, 3);
            }
            let fail = if scenario == 2 { Some(5) } else { None };
            a_shared.arm(fail, None);
            b_shared.arm(fail, None);
            let a = run(a_imu.verify(&mut a_delay));
            let b = b_imu.verify(&mut b_delay);
            assert_eq!(a, b, "verify, scenario {scenario}");
            assert_eq!(a_shared.log(), b_shared.log());

            a_shared.arm(fail, None);
            b_shared.arm(fail, None);
            let a = run(a_imu.recover(&mut a_delay));
            let b = b_imu.recover(&mut b_delay);
            assert_eq!(a, b, "recover, scenario {scenario}");
            assert_eq!(a_shared.log(), b_shared.log());
            assert_eq!(snapshot(&a_shared), snapshot(&b_shared));
            assert_eq!(a_imu.config(), b_imu.config());
            match scenario {
                0 => assert_eq!(a, Ok(())),
                1 => assert!(matches!(a, Err(Error::ConfigMismatch(_)))),
                2 => assert_eq!(a, Err(Error::Bus(BusFault))),
                _ => assert_eq!(a, Err(Error::NotConfigured)),
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// verify_step
// ---------------------------------------------------------------------------------------------

#[test]
fn verify_steps_in_order_send_what_verify_sends_and_each_finds_its_own_mismatch() {
    let (shared, mut imu, mut delay) = rig();
    assert_eq!(imu.verify_steps(), 0);
    shared.arm(None, None);
    assert_eq!(
        run(imu.verify_step(&mut delay, 0)),
        Err(Error::NotConfigured)
    );
    assert_eq!(shared.transactions(), 0);
    for (config, steps) in [
        (Config::new(), 33),
        (clkin(), 34),
        (rich(), owned(&rich()).len()),
    ] {
        let (shared, mut imu, mut delay) = applied(&config, false);
        let list = owned(&config);
        assert_eq!(imu.verify_steps(), steps);
        assert_eq!(imu.verify_steps(), list.registers().len());
        shared.arm(None, None);
        assert_eq!(run(imu.verify(&mut delay)), Ok(()));
        let whole = shared.log();
        shared.arm(None, None);
        for index in 0..imu.verify_steps() {
            assert_eq!(run(imu.verify_step(&mut delay, index)), Ok(()));
        }
        assert_eq!(shared.log(), whole);
        assert_eq!(shared.violations(), 0);
        shared.arm(None, None);
        assert_eq!(run(imu.verify_step(&mut delay, steps)), Ok(()));
        assert_eq!(shared.transactions(), 0);
        let saved = snapshot(&shared);
        for item in list.registers() {
            let bit = item.mask.trailing_zeros();
            restore(&shared, &saved);
            flip(&shared, item.space, item.register, bit);
            let raw = match item.space {
                Space::Direct => saved.0[usize::from(item.register & 0xFF)],
                Space::Indirect => saved.1.get(&item.register).copied().unwrap_or(0),
            };
            let hit = |o: &OwnedRegister| {
                o.space == item.space && o.register == item.register && o.mask >> bit & 1 == 1
            };
            let whole = run(imu.verify(&mut delay));
            let first = list.registers().iter().position(hit).unwrap();
            assert_eq!(
                whole,
                Err(mismatch(&list.registers()[first], raw ^ (1 << bit)))
            );
            for (j, other) in list.registers().iter().enumerate() {
                let got = run(imu.verify_step(&mut delay, j));
                if hit(other) {
                    assert_eq!(got, Err(mismatch(other, raw ^ (1 << bit))), "{other:?}");
                } else {
                    assert_eq!(got, Ok(()), "{item:?} at step {j}");
                }
            }
            assert_eq!(imu.config(), Some(&config));
        }
        restore(&shared, &saved);
    }
}
