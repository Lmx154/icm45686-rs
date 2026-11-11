//! Host tests of `set_offsets`, `set_power`, `soft_reset` and `offsets` on the fake bus of
//! [`crate::testbus`]: golden traces, transaction counts, errors found before any bus access, a bus
//! error at every transaction and a cancel at every await, the state afterwards, and the blocking
//! front-end.

use std::vec;
use std::vec::Vec;

use crate::driver_tests::{Imu, configured, r, rig, snapshot, us, w};
use crate::plan::{ACCEL_START_UP_US, GYRO_START_UP_US, GYRO_STOP_US};
use crate::testbus::{BusFault, Event, Op, Polled, Shared, TestDelay, poll_then_drop, run};
use crate::{
    Config, ConfigError, Error, FifoConfig, FifoFormat, FifoMode, Info, Int1Config, OffsetAxis,
    OffsetError, Offsets, SensorPower, blocking,
};

const OFF: SensorPower = SensorPower::Off;
const STANDBY: SensorPower = SensorPower::Standby;
const LOW_NOISE: SensorPower = SensorPower::LowNoise;

/// An indirect write as it appears on the bus: address and data in one write, then 10 us (13.3, p55).
fn iw(addr: u16, value: u8) -> Vec<Event> {
    let [hi, lo] = addr.to_be_bytes();
    vec![
        Event::Spi(vec![Op::Write(vec![0x7C, hi, lo, value])]),
        us(10),
    ]
}

/// Twelve registers that all differ from zero.
fn nonzero() -> Offsets {
    Offsets::new()
        .with_gyro([0x0123, -0x0123, 0x1ABC])
        .with_accel([0x0456, -0x0789, 0x1DEF])
}

/// The register images of `nonzero()`: two's complement in 14 bits, low byte then bits 13:8.
const NONZERO_IMAGES: [(u16, u8); 12] = [
    (0xA42A, 0x23), // GYRO_X_OFFUSER[7:0] (20.1, p189)
    (0xA42B, 0x01), // GYRO_X_OFFUSER[13:8] (20.2, p189)
    (0xA438, 0xDD), // GYRO_Y_OFFUSER[7:0] (20.3, p189), -0x123 = 0x3EDD
    (0xA439, 0x3E), // GYRO_Y_OFFUSER[13:8] (20.4, p189)
    (0xA446, 0xBC), // GYRO_Z_OFFUSER[7:0] (20.5, p190)
    (0xA447, 0x1A), // GYRO_Z_OFFUSER[13:8] (20.6, p190)
    (0xA518, 0x56), // ACCEL_X_OFFUSER[7:0] (21.1, p193)
    (0xA519, 0x04), // ACCEL_X_OFFUSER[13:8] (21.2, p193)
    (0xA520, 0x77), // ACCEL_Y_OFFUSER[7:0] (21.3, p193), -0x789 = 0x3877
    (0xA521, 0x38), // ACCEL_Y_OFFUSER[13:8] (21.4, p193)
    (0xA528, 0xEF), // ACCEL_Z_OFFUSER[7:0] (21.5, p194)
    (0xA529, 0x1D), // ACCEL_Z_OFFUSER[13:8] (21.6, p194)
];

fn ireg_of(shared: &Shared, addr: u16) -> u8 {
    shared.chip().ireg.get(&addr).copied().unwrap_or(0)
}

fn is_ok_and_configured(imu: &mut Imu, delay: &mut TestDelay, config: &Config) {
    assert_eq!(imu.config(), Some(config));
    assert_eq!(run(imu.verify(delay)), Ok(()));
    assert!(run(imu.read()).is_ok());
}

fn not_configured(imu: &mut Imu) {
    assert_eq!(imu.config(), None);
    assert_eq!(imu.offsets(), None);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
}

fn config_with(accel: SensorPower, gyro: SensorPower) -> Config {
    Config::new().with_accel_power(accel).with_gyro_power(gyro)
}

fn started(accel: SensorPower, gyro: SensorPower) -> (Shared, Imu, TestDelay) {
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, &config_with(accel, gyro))).unwrap();
    (shared, imu, delay)
}

// ---------------------------------------------------------------------------------------------
// set_offsets and offsets
// ---------------------------------------------------------------------------------------------

#[test]
fn offsets_are_none_before_init_zero_after_it_and_cost_no_bus_access() {
    let (shared, imu, mut delay) = rig();
    shared.arm(None, None);
    assert_eq!(imu.offsets(), None);
    assert_eq!(shared.transactions(), 0);
    let (shared, mut imu, _) = configured();
    shared.arm(None, None);
    assert_eq!(imu.offsets(), Some(Offsets::new()));
    assert_eq!(shared.log(), vec![]);
    assert_eq!(run(imu.verify(&mut delay)), Ok(()));
}

#[test]
fn set_offsets_golden_trace_count_and_state() {
    let (shared, mut imu, mut delay) = configured();
    shared.arm(None, None);
    assert_eq!(run(imu.set_offsets(&mut delay, &nonzero())), Ok(()));
    let want: Vec<Event> = NONZERO_IMAGES
        .iter()
        .flat_map(|&(addr, value)| iw(addr, value))
        .collect();
    assert_eq!(shared.log(), want);
    assert_eq!(shared.transactions(), 12);
    assert_eq!(shared.awaits(), 24);
    assert_eq!(shared.violations(), 0);
    assert_eq!(imu.offsets(), Some(nonzero()));
    for (addr, value) in NONZERO_IMAGES {
        assert_eq!(ireg_of(&shared, addr), value, "{addr:#x}");
    }
    is_ok_and_configured(&mut imu, &mut delay, &Config::new());
}

#[test]
fn set_offsets_extremes_and_zero_images() {
    let (shared, mut imu, mut delay) = configured();
    let o = Offsets::new()
        .with_gyro([-8192, 8191, -1])
        .with_accel([1, 0, -8192]);
    run(imu.set_offsets(&mut delay, &o)).unwrap();
    let want = [
        (0xA42A, 0x00),
        (0xA42B, 0x20),
        (0xA438, 0xFF),
        (0xA439, 0x1F),
        (0xA446, 0xFF),
        (0xA447, 0x3F),
        (0xA518, 0x01),
        (0xA519, 0x00),
        (0xA520, 0x00),
        (0xA521, 0x00),
        (0xA528, 0x00),
        (0xA529, 0x20),
    ];
    for (addr, value) in want {
        assert_eq!(ireg_of(&shared, addr), value, "{addr:#x}");
    }
    assert_eq!(run(imu.verify(&mut delay)), Ok(()));
    run(imu.set_offsets(&mut delay, &Offsets::new())).unwrap();
    assert_eq!(imu.offsets(), Some(Offsets::new()));
    assert_eq!(run(imu.verify(&mut delay)), Ok(()));
}

#[test]
fn set_offsets_not_configured_and_out_of_range_send_nothing() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    assert_eq!(
        run(imu.set_offsets(&mut delay, &nonzero())),
        Err(Error::NotConfigured)
    );
    assert_eq!(shared.log(), vec![]);

    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
    let bad = [
        (Offsets::new().with_gyro([8192, 0, 0]), OffsetAxis::GyroX),
        (Offsets::new().with_gyro([0, -8193, 0]), OffsetAxis::GyroY),
        (
            Offsets::new().with_gyro([0, 0, i16::MAX]),
            OffsetAxis::GyroZ,
        ),
        (
            Offsets::new().with_accel([i16::MIN, 0, 0]),
            OffsetAxis::AccelX,
        ),
        (Offsets::new().with_accel([0, 8192, 0]), OffsetAxis::AccelY),
        (Offsets::new().with_accel([0, 0, -8193]), OffsetAxis::AccelZ),
    ];
    for (offsets, axis) in bad {
        shared.arm(None, None);
        assert_eq!(
            run(imu.set_offsets(&mut delay, &offsets)),
            Err(Error::InvalidOffsets(OffsetError::AxisOutOfRange(axis)))
        );
        assert_eq!(shared.log(), vec![]);
        assert_eq!(imu.offsets(), Some(nonzero()));
    }
    assert_eq!(run(imu.verify(&mut delay)), Ok(()));
}

#[test]
fn set_offsets_bus_error_at_every_transaction_leaves_the_stored_offsets_and_verify_reports() {
    for at in 1..=12usize {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(Some(at), None);
        assert_eq!(
            run(imu.set_offsets(&mut delay, &nonzero())),
            Err(Error::Bus(BusFault))
        );
        // Stored state unchanged, still configured.
        assert_eq!(imu.offsets(), Some(Offsets::new()));
        assert_eq!(imu.config(), Some(&Config::new()));
        assert!(run(imu.read()).is_ok());
        // The registers written before the failed one already changed: verify reports the first.
        shared.arm(None, None);
        match run(imu.verify(&mut delay)) {
            Ok(()) => assert_eq!(at, 1, "nothing was written"),
            Err(Error::ConfigMismatch(m)) => {
                assert!(at > 1);
                assert_eq!((m.register, m.expected, m.found), (0xA42A, 0x00, 0x23));
            }
            Err(e) => panic!("{e:?}"),
        }
        // The same call again puts it right.
        shared.arm(None, None);
        assert_eq!(run(imu.set_offsets(&mut delay, &nonzero())), Ok(()));
        assert_eq!(imu.offsets(), Some(nonzero()));
        is_ok_and_configured(&mut imu, &mut delay, &Config::new());
    }
}

#[test]
fn set_offsets_cancel_at_every_await_leaves_the_stored_offsets_and_verify_reports() {
    let mut k = 0;
    loop {
        k += 1;
        let (shared, mut imu, mut delay) = configured();
        shared.arm(None, Some(k));
        match poll_then_drop(imu.set_offsets(&mut delay, &nonzero())) {
            Polled::Done(out) => {
                assert_eq!(out, Ok(()));
                assert_eq!(k, 25, "24 awaits");
                break;
            }
            Polled::Cancelled => {}
        }
        assert_eq!(imu.offsets(), Some(Offsets::new()));
        assert_eq!(imu.config(), Some(&Config::new()));
        // A write has taken effect once its own await is passed: await 2 is the wait after write 1.
        let written = k / 2;
        shared.arm(None, None);
        match run(imu.verify(&mut delay)) {
            Ok(()) => assert_eq!(written, 0, "cancel at {k}"),
            Err(Error::ConfigMismatch(m)) => {
                assert!(written > 0, "cancel at {k}");
                assert_eq!(m.register, 0xA42A);
            }
            Err(e) => panic!("{e:?}"),
        }
        assert_eq!(run(imu.set_offsets(&mut delay, &nonzero())), Ok(()));
        is_ok_and_configured(&mut imu, &mut delay, &Config::new());
    }
}

/// The offset writes of `NONZERO_IMAGES` as they appear on the bus.
fn nonzero_offset_writes() -> Vec<Event> {
    NONZERO_IMAGES
        .iter()
        .flat_map(|&(addr, value)| iw(addr, value))
        .collect()
}

/// Whether `event` is a write to one of the twelve offset registers (0xA42A.. and 0xA518..).
fn is_offset_write(event: &Event) -> bool {
    matches!(event, Event::Spi(ops) if matches!(ops.first(),
        Some(Op::Write(b)) if b.len() == 4 && matches!(b.get(1), Some(0xA4 | 0xA5))
            && b.get(2).is_some_and(|&a| matches!(a, 0x2A | 0x2B | 0x38 | 0x39 | 0x46 | 0x47 | 0x18 | 0x19 | 0x20 | 0x21 | 0x28 | 0x29))))
}

/// Offsets that differ from `nonzero()` in every one of the twelve registers.
fn other() -> Offsets {
    Offsets::new()
        .with_gyro([0x0211, -0x0222, 0x1333])
        .with_accel([-0x0344, 0x0555, -0x1666])
}

/// The state after a `configure` that must have put the device right: the stored offsets are
/// `want`, the twelve registers hold them and `verify` agrees.
fn assert_device_holds(shared: &Shared, imu: &mut Imu, delay: &mut TestDelay, want: &Offsets) {
    assert_eq!(imu.offsets(), Some(*want));
    for (addr, value, _) in want.registers() {
        assert_eq!(ireg_of(shared, addr), value, "{addr:#x}");
    }
    is_ok_and_configured(imu, delay, &Config::new());
}

#[test]
fn configure_writes_the_stored_offsets_and_set_power_writes_none() {
    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
    let rich = Config::new()
        .with_int1(Int1Config::new().with_fifo_threshold(true))
        .with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Stream)
                .with_format(FifoFormat::HighResolution)
                .with_watermark(40),
        );
    shared.arm(None, None);
    run(imu.configure(&mut delay, &rich)).unwrap();
    assert_eq!(imu.offsets(), Some(nonzero()));
    // The writes are the last 24 events, after the plan and its start-up wait, in the order of
    // `set_offsets`, and nothing else wrote an offset register.
    let log = shared.log();
    assert_eq!(
        log.get(log.len() - 24..),
        Some(nonzero_offset_writes().as_slice())
    );
    assert_eq!(log.iter().filter(|e| is_offset_write(e)).count(), 12);
    is_ok_and_configured(&mut imu, &mut delay, &rich);
    shared.arm(None, None);
    run(imu.set_power(&mut delay, LOW_NOISE, LOW_NOISE)).unwrap();
    assert_eq!(imu.offsets(), Some(nonzero()));
    assert!(!shared.log().iter().any(is_offset_write));
    for (addr, value) in NONZERO_IMAGES {
        assert_eq!(ireg_of(&shared, addr), value, "{addr:#x}");
    }
}

#[test]
fn a_failed_init_at_every_transaction_keeps_the_stored_offsets_and_configure_restores_them() {
    let mut at = 0;
    loop {
        at += 1;
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
        shared.arm(Some(at), None);
        if run(imu.init(&mut delay, &Config::new())).is_ok() {
            assert_eq!(at, 23, "22 transactions");
            assert_eq!(imu.offsets(), Some(Offsets::new()));
            break;
        }
        assert_eq!(imu.offsets(), None, "at {at}");
        assert_eq!(imu.offsets, nonzero(), "at {at}");
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_device_holds(&shared, &mut imu, &mut delay, &nonzero());
    }
}

#[test]
fn an_init_cancelled_at_every_await_keeps_the_stored_offsets_and_configure_restores_them() {
    let mut k = 0;
    loop {
        k += 1;
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
        shared.arm(None, Some(k));
        match poll_then_drop(imu.init(&mut delay, &Config::new())) {
            Polled::Done(out) => {
                assert!(out.is_ok());
                assert_eq!(k, 37, "36 awaits");
                break;
            }
            Polled::Cancelled => {}
        }
        assert_eq!(imu.offsets(), None, "at {k}");
        assert_eq!(imu.offsets, nonzero(), "at {k}");
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_device_holds(&shared, &mut imu, &mut delay, &nonzero());
    }
}

#[test]
fn an_init_with_an_invalid_config_keeps_the_stored_offsets() {
    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
    let bad = Config::new()
        .with_accel_odr(crate::Odr::Hz6_25)
        .with_gyro_odr(crate::Odr::Hz6_25);
    shared.arm(None, None);
    assert_eq!(
        run(imu.init(&mut delay, &bad)),
        Err(Error::InvalidConfig(ConfigError::AccelOdrLowPowerOnly))
    );
    assert_eq!(shared.log(), vec![]);
    assert_eq!(imu.offsets, nonzero());
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_device_holds(&shared, &mut imu, &mut delay, &nonzero());
}

#[test]
fn a_failed_soft_reset_keeps_the_stored_offsets_and_configure_restores_them() {
    for at in 1..=3usize {
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
        shared.arm(Some(at), None);
        assert_eq!(run(imu.soft_reset(&mut delay)), Err(Error::Bus(BusFault)));
        assert_eq!(imu.offsets, nonzero(), "at {at}");
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_device_holds(&shared, &mut imu, &mut delay, &nonzero());
    }
    for k in 1..=4u32 {
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
        shared.arm(None, Some(k));
        assert!(matches!(
            poll_then_drop(imu.soft_reset(&mut delay)),
            Polled::Cancelled
        ));
        assert_eq!(imu.offsets, nonzero(), "cancel at {k}");
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_device_holds(&shared, &mut imu, &mut delay, &nonzero());
    }
    // A completed soft_reset does not touch them either; the device cleared its own registers.
    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
    run(imu.soft_reset(&mut delay)).unwrap();
    assert_eq!(imu.offsets, nonzero());
    assert_eq!(
        ireg_of(&shared, 0xA42A),
        0,
        "the fake clears them like the device"
    );
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_device_holds(&shared, &mut imu, &mut delay, &nonzero());
}

#[test]
fn a_fresh_driver_on_a_calibrated_device_clears_the_registers_at_configure() {
    let (shared, mut imu, mut delay) = rig();
    for (addr, value) in NONZERO_IMAGES {
        shared.chip().ireg.insert(addr, value);
    }
    assert_eq!(imu.offsets(), None);
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_device_holds(&shared, &mut imu, &mut delay, &Offsets::new());
}

#[test]
fn a_set_offsets_cut_off_part_way_is_undone_by_configure() {
    assert_ne!(
        other().registers().map(|r| r.1),
        nonzero().registers().map(|r| r.1)
    );
    for at in 1..=12usize {
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
        shared.arm(Some(at), None);
        assert_eq!(
            run(imu.set_offsets(&mut delay, &other())),
            Err(Error::Bus(BusFault))
        );
        assert_eq!(imu.offsets(), Some(nonzero()));
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_device_holds(&shared, &mut imu, &mut delay, &nonzero());
    }
    for k in 1..=24u32 {
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
        shared.arm(None, Some(k));
        assert!(matches!(
            poll_then_drop(imu.set_offsets(&mut delay, &other())),
            Polled::Cancelled
        ));
        assert_eq!(imu.offsets(), Some(nonzero()));
        shared.arm(None, None);
        run(imu.configure(&mut delay, &Config::new())).unwrap();
        assert_device_holds(&shared, &mut imu, &mut delay, &nonzero());
    }
}

#[test]
fn init_after_set_offsets_zeroes_the_offsets_whether_or_not_the_reset_clears_them() {
    for keeps in [false, true] {
        let (shared, mut imu, mut delay) = configured();
        run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
        shared.chip().reset_keeps_offsets = keeps;
        run(imu.init(&mut delay, &Config::new())).unwrap();
        assert_eq!(imu.offsets(), Some(Offsets::new()), "keeps {keeps}");
        for (addr, _) in NONZERO_IMAGES {
            assert_eq!(ireg_of(&shared, addr), 0, "{addr:#x}, keeps {keeps}");
        }
        is_ok_and_configured(&mut imu, &mut delay, &Config::new());
    }
}

// ---------------------------------------------------------------------------------------------
// soft_reset
// ---------------------------------------------------------------------------------------------

#[test]
fn soft_reset_golden_trace_then_not_configured_until_init() {
    let (shared, mut imu, mut delay) = configured();
    run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
    shared.arm(None, None);
    assert_eq!(run(imu.soft_reset(&mut delay)), Ok(()));
    assert_eq!(
        shared.log(),
        vec![
            r(0x19, 0x00), // INT1_STATUS0 read to clear it (16.24, p76)
            w(0x7F, 0x02), // REG_MISC2 = SOFT_RST (16.84, p109)
            us(1_000),     // 1 ms after the soft reset (vendor)
            r(0x19, 0x80), // INT1_STATUS0 bit 7 RESET_DONE (16.24, p76)
        ]
    );
    assert_eq!(shared.transactions(), 3);
    assert_eq!(shared.awaits(), 4);
    not_configured(&mut imu);
    assert_eq!(run(imu.verify(&mut delay)), Err(Error::NotConfigured));
    assert_eq!(
        run(imu.set_offsets(&mut delay, &nonzero())),
        Err(Error::NotConfigured)
    );
    assert_eq!(
        run(imu.set_power(&mut delay, OFF, OFF)),
        Err(Error::NotConfigured)
    );
    run(imu.init(&mut delay, &Config::new())).unwrap();
    assert_eq!(imu.offsets(), Some(Offsets::new()));
    is_ok_and_configured(&mut imu, &mut delay, &Config::new());
}

#[test]
fn soft_reset_timeout_bus_error_and_cancel_leave_it_not_configured_and_the_same_call_works() {
    let (shared, mut imu, mut delay) = configured();
    shared.chip().reset_stuck = true;
    shared.arm(None, None);
    assert_eq!(run(imu.soft_reset(&mut delay)), Err(Error::ResetTimeout));
    assert_eq!(shared.transactions(), 102);
    not_configured(&mut imu);
    shared.chip().reset_stuck = false;

    for at in 1..=3usize {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(Some(at), None);
        assert_eq!(run(imu.soft_reset(&mut delay)), Err(Error::Bus(BusFault)));
        not_configured(&mut imu);
        shared.arm(None, None);
        assert_eq!(run(imu.soft_reset(&mut delay)), Ok(()));
        not_configured(&mut imu);
        run(imu.init(&mut delay, &Config::new())).unwrap();
        is_ok_and_configured(&mut imu, &mut delay, &Config::new());
    }
    for k in 1..=4u32 {
        let (shared, mut imu, mut delay) = configured();
        shared.arm(None, Some(k));
        assert!(matches!(
            poll_then_drop(imu.soft_reset(&mut delay)),
            Polled::Cancelled
        ));
        not_configured(&mut imu);
        shared.arm(None, None);
        assert_eq!(run(imu.soft_reset(&mut delay)), Ok(()));
        run(imu.init(&mut delay, &Config::new())).unwrap();
        is_ok_and_configured(&mut imu, &mut delay, &Config::new());
    }
}

// ---------------------------------------------------------------------------------------------
// set_power
// ---------------------------------------------------------------------------------------------

#[test]
fn set_power_golden_traces_for_each_transition_class() {
    let stop = us(u64::from(GYRO_STOP_US / 1_000) * 1_000);
    let gyro_up = us(u64::from(GYRO_START_UP_US / 1_000) * 1_000);
    let accel_up = us(u64::from(ACCEL_START_UP_US / 1_000) * 1_000);
    // (from accel, from gyro, to accel, to gyro, trace, awaits). PWR_MGMT0 gyro bits 3:2, accel 1:0
    // (16.17, p70).
    let cases: [(
        SensorPower,
        SensorPower,
        SensorPower,
        SensorPower,
        Vec<Event>,
        u32,
    ); 8] = [
        // Unchanged: the one write, no wait.
        (
            LOW_NOISE,
            LOW_NOISE,
            LOW_NOISE,
            LOW_NOISE,
            vec![w(0x10, 0x0F)],
            1,
        ),
        // Both off: no wait.
        (LOW_NOISE, LOW_NOISE, OFF, OFF, vec![w(0x10, 0x00)], 1),
        // Gyroscope off to on (and accelerometer off to on): stop wait, write, gyro start-up.
        (
            OFF,
            OFF,
            LOW_NOISE,
            LOW_NOISE,
            vec![stop.clone(), w(0x10, 0x0F), gyro_up.clone()],
            3,
        ),
        // Gyroscope stays on, accelerometer off to on: the accelerometer's start-up wait only.
        (
            OFF,
            LOW_NOISE,
            LOW_NOISE,
            LOW_NOISE,
            vec![w(0x10, 0x0F), accel_up.clone()],
            2,
        ),
        // Accelerometer stays on, gyroscope off to standby.
        (
            LOW_NOISE,
            OFF,
            LOW_NOISE,
            STANDBY,
            vec![stop, w(0x10, 0x07), gyro_up.clone()],
            3,
        ),
        // Gyroscope standby to low-noise: no stop wait, start-up wait.
        (
            LOW_NOISE,
            STANDBY,
            LOW_NOISE,
            LOW_NOISE,
            vec![w(0x10, 0x0F), gyro_up],
            2,
        ),
        // Gyroscope low-noise to standby: no wait.
        (
            LOW_NOISE,
            LOW_NOISE,
            LOW_NOISE,
            STANDBY,
            vec![w(0x10, 0x07)],
            1,
        ),
        // Gyroscope off, accelerometer low-noise to off: no wait.
        (LOW_NOISE, OFF, OFF, OFF, vec![w(0x10, 0x00)], 1),
    ];
    for (fa, fg, ta, tg, want, awaits) in cases {
        let (shared, mut imu, mut delay) = started(fa, fg);
        run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
        shared.arm(None, None);
        let info = run(imu.set_power(&mut delay, ta, tg)).unwrap();
        assert_eq!(shared.log(), want, "{fa:?} {fg:?} -> {ta:?} {tg:?}");
        assert_eq!(shared.transactions(), 1);
        assert_eq!(shared.awaits(), awaits);
        assert_eq!(shared.violations(), 0);
        let to = config_with(ta, tg);
        assert_eq!(info.who_am_i, 0xE9);
        assert_eq!(imu.info(), Some(info));
        assert_eq!(info.accel_odr_mhz != 0, ta == LOW_NOISE);
        assert_eq!(info.gyro_odr_mhz != 0, tg == LOW_NOISE);
        assert_eq!(imu.offsets(), Some(nonzero()));
        is_ok_and_configured(&mut imu, &mut delay, &to);
    }
}

#[test]
fn set_power_to_the_applied_states_writes_only_pwr_mgmt0_and_no_other_register() {
    let (shared, mut imu, mut delay) = configured();
    let before = snapshot(&shared);
    shared.arm(None, None);
    run(imu.set_power(&mut delay, LOW_NOISE, LOW_NOISE)).unwrap();
    assert_eq!(snapshot(&shared), before);
    assert_eq!(shared.log(), vec![w(0x10, 0x0F)]);
}

#[test]
fn set_power_errors_are_found_before_any_bus_access() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    assert_eq!(
        run(imu.set_power(&mut delay, OFF, OFF)),
        Err(Error::NotConfigured)
    );
    assert_eq!(shared.log(), vec![]);

    let fifo = Config::new().with_fifo(
        FifoConfig::new()
            .with_mode(FifoMode::Stream)
            .with_format(FifoFormat::Standard),
    );
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, &fifo)).unwrap();
    shared.arm(None, None);
    assert_eq!(
        run(imu.set_power(&mut delay, OFF, LOW_NOISE)),
        Err(Error::InvalidConfig(ConfigError::FifoAccelNotRunning))
    );
    assert_eq!(
        run(imu.set_power(&mut delay, LOW_NOISE, OFF)),
        Err(Error::InvalidConfig(ConfigError::FifoGyroNotRunning))
    );
    assert_eq!(
        run(imu.set_power(&mut delay, STANDBY, LOW_NOISE)),
        Err(Error::InvalidConfig(ConfigError::AccelStandby))
    );
    assert_eq!(shared.log(), vec![]);
    // The driver is unchanged: still configured with the same configuration.
    is_ok_and_configured(&mut imu, &mut delay, &fifo);
}

#[test]
fn set_power_bus_error_and_cancel_leave_it_not_configured_and_configure_recovers() {
    let cases = [
        (OFF, OFF, LOW_NOISE, LOW_NOISE),
        (LOW_NOISE, LOW_NOISE, OFF, OFF),
        (LOW_NOISE, STANDBY, LOW_NOISE, LOW_NOISE),
    ];
    for (fa, fg, ta, tg) in cases {
        let to = config_with(ta, tg);
        // A bus error at the only transaction: the register was not written.
        let (shared, mut imu, mut delay) = started(fa, fg);
        shared.arm(Some(1), None);
        assert_eq!(
            run(imu.set_power(&mut delay, ta, tg)),
            Err(Error::Bus(BusFault))
        );
        not_configured(&mut imu);
        assert_eq!(
            run(imu.set_power(&mut delay, ta, tg)),
            Err(Error::NotConfigured)
        );
        shared.arm(None, None);
        run(imu.configure(&mut delay, &to)).unwrap();
        is_ok_and_configured(&mut imu, &mut delay, &to);

        // A cancel at every await.
        let mut k = 0;
        loop {
            k += 1;
            let (shared, mut imu, mut delay) = started(fa, fg);
            run(imu.set_offsets(&mut delay, &nonzero())).unwrap();
            shared.arm(None, Some(k));
            match poll_then_drop(imu.set_power(&mut delay, ta, tg)) {
                Polled::Done(out) => {
                    assert!(out.is_ok());
                    break;
                }
                Polled::Cancelled => {}
            }
            assert_eq!(imu.config(), None);
            assert_eq!(imu.offsets(), None);
            assert_eq!(run(imu.read()), Err(Error::NotConfigured));
            shared.arm(None, None);
            run(imu.configure(&mut delay, &to)).unwrap();
            // `configure` keeps the offsets, so they are applied again.
            assert_eq!(imu.offsets(), Some(nonzero()));
            is_ok_and_configured(&mut imu, &mut delay, &to);
        }
        assert!(k >= 2, "{fa:?} {fg:?} -> {ta:?} {tg:?}");
    }
}

// ---------------------------------------------------------------------------------------------
// A11: the blocking front-end
// ---------------------------------------------------------------------------------------------

#[test]
fn blocking_set_offsets_set_power_soft_reset_match_the_async_driver() {
    for scenario in 0..3 {
        let (a_shared, mut a_imu, mut a_delay) = if scenario == 0 { rig() } else { configured() };
        let b_shared = Shared::new();
        let mut b_imu = blocking::Icm45686::new(b_shared.bus());
        let mut b_delay = b_shared.delay();
        if scenario != 0 {
            b_imu.init(&mut b_delay, &Config::new()).unwrap();
        }
        let fail = if scenario == 2 { Some(5) } else { None };
        a_shared.arm(fail, None);
        b_shared.arm(fail, None);
        let a = run(a_imu.set_offsets(&mut a_delay, &nonzero()));
        let b = b_imu.set_offsets(&mut b_delay, &nonzero());
        assert_eq!(a, b, "set_offsets, scenario {scenario}");
        assert_eq!(a_imu.offsets(), b_imu.offsets());
        assert_eq!(a_shared.log(), b_shared.log());

        a_shared.arm(None, None);
        b_shared.arm(None, None);
        let a: Result<Info, Error<BusFault>> = run(a_imu.set_power(&mut a_delay, OFF, OFF));
        let b = b_imu.set_power(&mut b_delay, OFF, OFF);
        assert_eq!(a, b, "set_power, scenario {scenario}");
        assert_eq!(a_shared.log(), b_shared.log());
        assert_eq!(a_imu.config(), b_imu.config());

        a_shared.arm(None, None);
        b_shared.arm(None, None);
        let a = run(a_imu.soft_reset(&mut a_delay));
        let b = b_imu.soft_reset(&mut b_delay);
        assert_eq!(a, b, "soft_reset, scenario {scenario}");
        assert_eq!(a, Ok(()));
        assert_eq!(a_shared.log(), b_shared.log());
        assert_eq!(a_imu.offsets(), None);
        assert_eq!(b_imu.offsets(), None);
        assert_eq!(snapshot(&a_shared), snapshot(&b_shared));
    }
}
