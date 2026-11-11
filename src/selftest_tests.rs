//! Host tests of the self-test (`crate::selftest`) on the fake bus of [`crate::testbus`]: the pure
//! parameter word and result decoding with hand-computed words, two golden traces (default and a
//! non-default configuration), pass, fail, timeout and random-SRAM (the vendor zeros overwrite it)
//! paths, a bus error at every transaction and a cancel at every await (each checking that the final
//! soft reset was attempted, or what a dropped future leaves), and the blocking front-end. "vendor" = TDK's public driver, zephyr.hal_tdk fa54cb6, icm456xx/imu.

use std::vec;
use std::vec::Vec;

use crate::driver_tests::{Imu, configured, r, rig, us, w};
use crate::selftest::POLLS;
use crate::testbus::{BusFault, Event, Op, Polled, Shared, TestDelay, poll_then_drop, run};
use crate::{
    AxisVerdict, Config, Error, SelfTestConfig, SelfTestConfigError, SelfTestReport,
    SelfTestStatus, blocking,
};

const ALL_PASS: AxisVerdict = AxisVerdict {
    x: true,
    y: true,
    z: true,
};

/// An indirect write on the bus: address and data in one write, then 10 us (13.3, p55).
fn iw(addr: u16, value: u8) -> Vec<Event> {
    let [hi, lo] = addr.to_be_bytes();
    vec![
        Event::Spi(vec![Op::Write(vec![0x7C, hi, lo, value])]),
        us(10),
    ]
}

/// An indirect read on the bus: address write, 10 us, IREG_DATA read, 10 us (13.3, p55).
fn ir(addr: u16, got: u8) -> Vec<Event> {
    let [hi, lo] = addr.to_be_bytes();
    vec![
        Event::Spi(vec![Op::Write(vec![0x7C, hi, lo])]),
        us(10),
        r(0x7E, got),
        us(10),
    ]
}

fn soft_reset_trace() -> Vec<Event> {
    vec![
        r(0x19, 0x00), // INT1_STATUS0 read to clear it (16.24, p76)
        w(0x7F, 0x02), // REG_MISC2 SOFT_RST (16.84, p109); vendor inv_imu_selftest.c:41, :54
        us(1_000),     // 1 ms (vendor inv_imu_driver.c:34)
        r(0x19, 0x80), // RESET_DONE (16.24, p76)
    ]
}

fn not_configured(imu: &mut Imu) {
    assert_eq!(imu.config(), None);
    assert_eq!(imu.offsets(), None);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
}

/// Transactions of a clean self-test when done shows at poll `polls`: 44 plus one per further poll.
fn clean_transactions(polls: u32) -> usize {
    44 + polls as usize - 1
}

/// After a bus error at transaction `at` of a self-test that has `total` transactions when clean:
/// the final soft reset was attempted. If the error came before the final reset (its three
/// transactions are `total - 2..=total`), the reset ran after it: exactly three more transactions,
/// and the last logged events are its write of SOFT_RST, the 1 ms wait and the RESET_DONE read; if
/// the error is inside the final reset itself, the run ended at that transaction. Either way the
/// eDMP is left stopped.
fn assert_final_reset_attempted(shared: &Shared, at: usize, total: usize) {
    if at + 3 <= total {
        assert_eq!(shared.transactions(), at + 3, "transaction {at}");
        let log = shared.log();
        assert_eq!(
            log[log.len() - 3..],
            [w(0x7F, 0x02), us(1_000), r(0x19, 0x80)],
            "transaction {at}"
        );
    } else {
        assert_eq!(shared.transactions(), at, "transaction {at}");
    }
    assert_eq!(shared.chip().selftest_left, None, "transaction {at}");
}

/// The 1-based transaction number of the first logged transaction equal to `event`.
fn transaction_of(log: &[Event], event: &Event) -> usize {
    let mut n = 0;
    for e in log {
        if matches!(e, Event::Spi(_)) {
            n += 1;
            if e == event {
                return n;
            }
        }
    }
    panic!("{event:?} is not in the log");
}

fn init_works(imu: &mut Imu, delay: &mut TestDelay) {
    run(imu.init(delay, &Config::new())).unwrap();
    assert_eq!(run(imu.verify(delay)), Ok(()));
    assert!(run(imu.read()).is_ok());
}

// ---------------------------------------------------------------------------------------------
// Pure code
// ---------------------------------------------------------------------------------------------

#[test]
fn new_equals_default_and_the_vendor_defaults() {
    // Vendor inv_imu_selftest.c:23-28: both sensors, 320 ms, 50 %, 50 %.
    let c = SelfTestConfig::new();
    assert_eq!(c, SelfTestConfig::default());
    assert!(c.accel && c.gyro);
    assert_eq!(
        (c.averaging_ms, c.accel_tolerance_pct, c.gyro_tolerance_pct),
        (320, 50, 50)
    );
}

#[test]
fn with_accel_sets_only_accel() {
    let c = SelfTestConfig::new().with_accel(false);
    assert!(!c.accel && c.gyro);
}

#[test]
fn with_gyro_sets_only_gyro() {
    let c = SelfTestConfig::new().with_gyro(false);
    assert!(c.accel && !c.gyro);
}

#[test]
fn with_averaging_ms_sets_it() {
    assert_eq!(SelfTestConfig::new().with_averaging_ms(40).averaging_ms, 40);
}

#[test]
fn with_accel_tolerance_pct_sets_it() {
    assert_eq!(
        SelfTestConfig::new()
            .with_accel_tolerance_pct(15)
            .accel_tolerance_pct,
        15
    );
}

#[test]
fn with_gyro_tolerance_pct_sets_it() {
    assert_eq!(
        SelfTestConfig::new()
            .with_gyro_tolerance_pct(25)
            .gyro_tolerance_pct,
        25
    );
}

#[test]
fn param_word_hand_computed() {
    // Bits: 0 init, 1 accel, 2 gyro, 9:7 averaging, 12:10 accel tolerance, 15:13 gyro tolerance
    // (17.13-17.14, p113-p114; vendor inv_imu_defs.h:742-787).
    // Default: 0x0007 | 5 << 7 (0x0280) | 7 << 10 (0x1C00) | 7 << 13 (0xE000) = 0xFE87.
    assert_eq!(SelfTestConfig::new().param_word(), Ok(0xFE87));
    // Accelerometer only, 10 ms, 5 %, 5 %: 0x0001 | 0x0002 = 0x0003.
    let a = SelfTestConfig::new()
        .with_gyro(false)
        .with_averaging_ms(10)
        .with_accel_tolerance_pct(5)
        .with_gyro_tolerance_pct(5);
    assert_eq!(a.param_word(), Ok(0x0003));
    // Gyroscope only, 40 ms, accel 25 %, gyro 30 %: 0x0005 | 2 << 7 | 4 << 10 | 5 << 13 = 0xB105.
    let g = SelfTestConfig::new()
        .with_accel(false)
        .with_averaging_ms(40)
        .with_accel_tolerance_pct(25)
        .with_gyro_tolerance_pct(30);
    assert_eq!(g.param_word(), Ok(0xB105));
    // Every averaging time and tolerance code.
    for (i, ms) in [10u16, 20, 40, 80, 160, 320].into_iter().enumerate() {
        let word = SelfTestConfig::new().with_averaging_ms(ms).param_word();
        assert_eq!(word, Ok((0xFE87 & !0x0380) | ((i as u16) << 7)));
    }
    for (i, pct) in [5u8, 10, 15, 20, 25, 30, 40, 50].into_iter().enumerate() {
        let a = SelfTestConfig::new().with_accel_tolerance_pct(pct);
        assert_eq!(a.param_word(), Ok((0xFE87 & !0x1C00) | ((i as u16) << 10)));
        let g = SelfTestConfig::new().with_gyro_tolerance_pct(pct);
        assert_eq!(g.param_word(), Ok((0xFE87 & !0xE000) | ((i as u16) << 13)));
    }
}

#[test]
fn param_word_rejects_values_outside_the_documented_choices() {
    let c = SelfTestConfig::new();
    assert_eq!(
        c.with_accel(false).with_gyro(false).param_word(),
        Err(SelfTestConfigError::NoSensor)
    );
    for ms in [0u16, 30, 640, 321] {
        assert_eq!(
            c.with_averaging_ms(ms).param_word(),
            Err(SelfTestConfigError::AveragingMs(ms))
        );
    }
    for pct in [0u8, 35, 45, 100] {
        assert_eq!(
            c.with_accel_tolerance_pct(pct).param_word(),
            Err(SelfTestConfigError::AccelTolerancePct(pct))
        );
        assert_eq!(
            c.with_gyro_tolerance_pct(pct).param_word(),
            Err(SelfTestConfigError::GyroTolerancePct(pct))
        );
    }
}

#[test]
fn decode_hand_computed_words() {
    let both = SelfTestConfig::new();
    // 0x00: done, every bit 0 = every axis passed (vendor sense, inv_imu_selftest.c:143-178).
    let r0 = SelfTestReport::decode(0x00, &both);
    assert_eq!(r0.accel, Some(ALL_PASS));
    assert_eq!(r0.gyro, Some(ALL_PASS));
    assert_eq!((r0.status, r0.raw), (SelfTestStatus::Done, 0x00));
    // 0x05: AX (bit 0) and AZ (bit 2) failed.
    let r5 = SelfTestReport::decode(0x05, &both);
    assert_eq!(
        r5.accel,
        Some(AxisVerdict {
            x: false,
            y: true,
            z: false
        })
    );
    assert_eq!(r5.gyro, Some(ALL_PASS));
    // 0x10: GY (bit 4) failed.
    let r10 = SelfTestReport::decode(0x10, &both);
    assert_eq!(
        r10.gyro,
        Some(AxisVerdict {
            x: true,
            y: false,
            z: true
        })
    );
    // Status field, bits 7:6 (17.16, p114).
    assert_eq!(
        SelfTestReport::decode(0x40, &both).status,
        SelfTestStatus::InProgress
    );
    assert_eq!(
        SelfTestReport::decode(0x80, &both).status,
        SelfTestStatus::Error
    );
    assert_eq!(
        SelfTestReport::decode(0xC0, &both).status,
        SelfTestStatus::Reserved
    );
    // Accelerometer only: the gyro bits are ignored (vendor inv_imu_selftest.c:159-163).
    let a = SelfTestReport::decode(0x38, &both.with_gyro(false));
    assert_eq!((a.accel, a.gyro), (Some(ALL_PASS), None));
}

#[test]
fn passed_needs_done_a_tested_sensor_and_every_tested_axis() {
    let both = SelfTestConfig::new();
    assert!(SelfTestReport::decode(0x00, &both).passed());
    assert!(!SelfTestReport::decode(0x01, &both).passed());
    assert!(!SelfTestReport::decode(0x20, &both).passed());
    assert!(!SelfTestReport::decode(0x40, &both).passed());
    assert!(!SelfTestReport::decode(0x80, &both).passed());
    assert!(SelfTestReport::decode(0x38, &both.with_gyro(false)).passed());
    assert!(SelfTestReport::decode(0x07, &both.with_accel(false)).passed());
    assert!(!SelfTestReport::default().passed());
}

#[test]
fn all_is_true_only_for_three_passes() {
    assert!(ALL_PASS.all());
    assert!(
        !AxisVerdict {
            x: true,
            y: true,
            z: false
        }
        .all()
    );
}

// ---------------------------------------------------------------------------------------------
// The sequence
// ---------------------------------------------------------------------------------------------

/// The whole sequence for `SelfTestConfig::new()` when done shows at the first poll and the eDMP
/// leaves `result`. Steps as numbered in `crate::selftest`.
fn golden(result: u8) -> Vec<Event> {
    golden_for(0x87, 0xFE, result)
}

/// The same sequence for the parameter word `low` (IMEM_SRAM 0x38) and `high` (0x39).
fn golden_for(low: u8, high: u8, result: u8) -> Vec<Event> {
    let mut t = Vec::new();
    t.extend(soft_reset_trace()); // 1: soft reset (vendor inv_imu_selftest.c:41)
    t.push(us(10_000)); // 2: 10 ms (vendor inv_imu_selftest.c:43)
    // 3: eDMP start addresses (vendor inv_imu_edmp.c:795-811, :10-12; inv_imu_edmp_defs.h:15, 17).
    t.extend(iw(0xA24F, 0x00)); // EDMP_PRGRM_IRQ0_0 (19.40, p176)
    t.extend(iw(0xA250, 0x40)); // EDMP_PRGRM_IRQ0_1 (19.41, p176)
    t.extend(iw(0xA251, 0x04)); // EDMP_PRGRM_IRQ1_0 (19.42, p177)
    t.extend(iw(0xA252, 0x40)); // EDMP_PRGRM_IRQ1_1 (19.43, p177)
    t.extend(iw(0xA253, 0x08)); // EDMP_PRGRM_IRQ2_0 (19.44, p177)
    t.extend(iw(0xA254, 0x40)); // EDMP_PRGRM_IRQ2_1 (19.45, p177)
    t.extend(iw(0xA255, 0x05)); // EDMP_SP_START_ADDR (19.46, p178)
    // 4: FIFO_SRAM_SLEEP bits 1:0 = 3 (19.69, p188; vendor inv_imu_driver_advanced.c:1297).
    t.extend(ir(0xA2A7, 0x00));
    t.extend(iw(0xA2A7, 0x03));
    // 5: parameter word (17.13-17.14, p113-p114; vendor inv_imu_selftest.c:77, 4 bytes).
    t.extend(iw(0x0038, low)); // IMEM_SRAM_REG_56
    t.extend(iw(0x0039, high)); // IMEM_SRAM_REG_57
    t.extend(iw(0x003A, 0x00)); // vendor, not in the datasheet
    t.extend(iw(0x003B, 0x00)); // vendor, not in the datasheet
    // 6: debug word (17.15, p114; vendor inv_imu_selftest.c:80, 4 bytes).
    t.extend(iw(0x0040, 0x00)); // IMEM_SRAM_REG_64, ST_DEBUG_EN = 0
    t.extend(iw(0x0041, 0x00)); // vendor, not in the datasheet
    t.extend(iw(0x0042, 0x00)); // vendor, not in the datasheet
    t.extend(iw(0x0043, 0x00)); // vendor, not in the datasheet
    // 7: patch-enable word = 0 (vendor inv_imu_selftest.c:28, 82; not in the datasheet).
    for addr in [0x3Cu16, 0x3D, 0x3E, 0x3F] {
        t.extend(iw(addr, 0x00));
    }
    t.push(r(0x73, 0x00)); // 8: REG_HOST_MSG (16.80, p108)
    t.push(w(0x73, 0x01)); //    TESTOPENABLE (vendor inv_imu_selftest.c:94-96)
    t.push(r(0x3A, 0x00)); // 9: INT_APEX_CONFIG1 (16.53, p96)
    t.push(w(0x3A, 0x00)); //    bit 2 = 0 (vendor inv_imu_selftest.c:99-101)
    // 10: STATUS_MASK_PIN_16_23 bit 5 = 0 (19.61, p185; vendor inv_imu_edmp.c:774-793, :818).
    t.extend(ir(0xA273, 0x3F));
    t.extend(iw(0xA273, 0x1F));
    t.push(r(0x2A, 0x00)); // 11: EDMP_APEX_EN1 (16.41, p88)
    t.push(w(0x2A, 0x40)); //     EDMP_ENABLE (vendor inv_imu_edmp.c:368-378)
    t.push(r(0x73, 0x01)); // 12: REG_HOST_MSG
    t.push(w(0x73, 0x21)); //     EDMP_ON_DEMAND_EN (vendor inv_imu_edmp.c:822-824)
    t.push(r(0x3C, 0x04)); // 13: INT_APEX_STATUS1 bit 2 (16.55, p97; vendor inv_imu_selftest.c:109)
    t.push(r(0x3A, 0x00)); // 14: INT_APEX_CONFIG1
    t.push(w(0x3A, 0x04)); //     bit 2 = 1 (vendor inv_imu_selftest.c:121-123)
    t.extend(ir(0x0044, result)); // 15: IMEM_SRAM_REG_68 (17.16, p114)
    t.extend(soft_reset_trace()); // 16: soft reset (vendor inv_imu_selftest.c:54)
    t
}

#[test]
fn self_test_golden_trace_default_config() {
    let (shared, mut imu, mut delay) = configured();
    shared.arm(None, None);
    let report = run(imu.self_test(&mut delay, &SelfTestConfig::new())).unwrap();
    assert_eq!(shared.log(), golden(0x00));
    assert_eq!(shared.transactions(), 44);
    assert_eq!(shared.awaits(), 74);
    assert_eq!(shared.violations(), 0);
    assert_eq!(shared.chip().selftest_starts, 1);
    assert!(report.passed());
    not_configured(&mut imu);
    init_works(&mut imu, &mut delay);
}

#[test]
fn self_test_golden_trace_non_default_config() {
    // Accelerometer only, 160 ms, accelerometer tolerance 15 %, gyroscope tolerance 40 % (still
    // encoded although the gyroscope is not tested). Word, by hand (17.13-17.14, p113-p114):
    //   bit 0 STC_INIT_EN + bit 1 accelerometer          = 0x0003
    //   averaging 160 ms = code 4, bits 9:7:  4 << 7     = 0x0200
    //   accelerometer 15 % = code 2, bits 12:10: 2 << 10 = 0x0800
    //   gyroscope 40 % = code 6, bits 15:13: 6 << 13     = 0xC000
    //   sum 0xCA03: IMEM_SRAM 0x38 = 0x03, 0x39 = 0xCA.
    // The eDMP leaves 0x38 (gyroscope bits set): ignored, because the gyroscope was not tested.
    let config = SelfTestConfig::new()
        .with_gyro(false)
        .with_averaging_ms(160)
        .with_accel_tolerance_pct(15)
        .with_gyro_tolerance_pct(40);
    assert_eq!(config.param_word(), Ok(0xCA03));
    let (shared, mut imu, mut delay) = configured();
    shared.arm(None, None);
    shared.chip().selftest_result = 0x38;
    let report = run(imu.self_test(&mut delay, &config)).unwrap();
    assert_eq!(shared.log(), golden_for(0x03, 0xCA, 0x38));
    assert_eq!(shared.transactions(), 44);
    assert_eq!(shared.awaits(), 74);
    assert_eq!(shared.violations(), 0);
    assert_eq!(shared.chip().selftest_starts, 1);
    assert_eq!((report.accel, report.gyro), (Some(ALL_PASS), None));
    assert!(report.passed());
    not_configured(&mut imu);
    init_works(&mut imu, &mut delay);
}

#[test]
fn self_test_pass_and_fail_reports_and_later_polls() {
    for (result, polls, passed) in [(0x00u8, 1u32, true), (0x00, 7, true), (0x21, 3, false)] {
        let (shared, mut imu, mut delay) = configured();
        shared.chip().selftest_result = result;
        shared.chip().selftest_polls = polls;
        shared.arm(None, None);
        let report = run(imu.self_test(&mut delay, &SelfTestConfig::new())).unwrap();
        assert_eq!(report.raw, result);
        assert_eq!(report.passed(), passed);
        assert_eq!(shared.transactions(), 44 + polls as usize - 1);
        not_configured(&mut imu);
        init_works(&mut imu, &mut delay);
    }
    // A failed axis: Ok with the axis reported (0x21: AX and GZ failed).
    let (shared, mut imu, mut delay) = configured();
    shared.chip().selftest_result = 0x21;
    let report = run(imu.self_test(&mut delay, &SelfTestConfig::new())).unwrap();
    assert_eq!(report.accel.map(|a| a.x), Some(false));
    assert_eq!(report.gyro.map(|g| g.z), Some(false));
    assert_eq!(report.status, SelfTestStatus::Done);
}

#[test]
fn self_test_timeout_still_soft_resets() {
    let (shared, mut imu, mut delay) = configured();
    shared.chip().selftest_never = true;
    shared.arm(None, None);
    let got = run(imu.self_test(&mut delay, &SelfTestConfig::new()));
    assert_eq!(got, Err(Error::SelfTestTimeout { apex_status: 0x00 }));
    // 36 transactions up to the start, POLLS polls, then the soft reset (3).
    assert_eq!(shared.transactions(), 36 + POLLS as usize + 3);
    let log = shared.log();
    assert_eq!(log[log.len() - 5..], {
        let mut tail = vec![us(100)];
        tail.extend(soft_reset_trace());
        tail
    });
    assert_eq!(shared.chip().selftest_left, None);
    not_configured(&mut imu);
    init_works(&mut imu, &mut delay);
}

#[test]
fn self_test_overwrites_random_sram_with_the_vendor_zeros() {
    // After a reset the SRAM holds random bytes (17, p110); measured on the board: 0x3C = 0xCD.
    let junk: Vec<(u16, u8)> = (0x38u16..=0x47).map(|a| (a, 0xCD)).collect();
    let (shared, mut imu, mut delay) = configured();
    shared.chip().sram_after_reset = junk;
    shared.arm(None, None);
    let report = run(imu.self_test(&mut delay, &SelfTestConfig::new())).unwrap();
    assert!(report.passed());
    assert_eq!(shared.chip().selftest_starts, 1);
    // Without the nine vendor zeros the fake's eDMP does not start: the trace up to the start
    // writes every one of them.
    let log = shared.log();
    for addr in [0x3Au16, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x41, 0x42, 0x43] {
        let [hi, lo] = addr.to_be_bytes();
        let write = Event::Spi(vec![Op::Write(vec![0x7C, hi, lo, 0x00])]);
        assert!(log.contains(&write), "0x{addr:02X}");
    }
    not_configured(&mut imu);
    init_works(&mut imu, &mut delay);
}

#[test]
fn self_test_invalid_config_has_no_bus_access_and_leaves_the_driver_configured() {
    let (shared, mut imu, mut delay) = configured();
    shared.arm(None, None);
    let bad = SelfTestConfig::new().with_averaging_ms(30);
    assert_eq!(
        run(imu.self_test(&mut delay, &bad)),
        Err(Error::InvalidSelfTestConfig(
            SelfTestConfigError::AveragingMs(30)
        ))
    );
    assert_eq!(shared.transactions(), 0);
    assert_eq!(imu.config(), Some(&Config::new()));
}

#[test]
fn self_test_bus_error_at_every_transaction_attempts_the_final_reset_and_init_works() {
    // Done at the first poll (44 transactions) and at the third (46): the second case puts bus
    // errors on a poll that does not yet show done as well.
    for polls in [1u32, 3] {
        let total = clean_transactions(polls);
        for at in 1..=total {
            let (shared, mut imu, mut delay) = configured();
            shared.chip().selftest_polls = polls;
            shared.arm(Some(at), None);
            let got = run(imu.self_test(&mut delay, &SelfTestConfig::new()));
            assert_eq!(got, Err(Error::Bus(BusFault)), "transaction {at}");
            assert_final_reset_attempted(&shared, at, total);
            not_configured(&mut imu);
            init_works(&mut imu, &mut delay);
        }
    }
}

#[test]
fn self_test_final_reset_follows_a_failed_poll_and_a_failed_result_read() {
    // Transaction numbers taken from a clean run with done at the third poll and the result 0x38
    // (the only IREG_DATA read that returns it).
    let (shared, mut imu, mut delay) = configured();
    shared.chip().selftest_polls = 3;
    shared.chip().selftest_result = 0x38;
    shared.arm(None, None);
    run(imu.self_test(&mut delay, &SelfTestConfig::new())).unwrap();
    let log = shared.log();
    let first_poll = transaction_of(&log, &r(0x3C, 0x00));
    let done_poll = transaction_of(&log, &r(0x3C, 0x04));
    let result_read = transaction_of(&log, &r(0x7E, 0x38));
    assert_eq!(done_poll, first_poll + 2);
    assert_eq!(result_read, done_poll + 4);
    for at in [first_poll, first_poll + 1, done_poll, result_read] {
        let (shared, mut imu, mut delay) = configured();
        shared.chip().selftest_polls = 3;
        shared.chip().selftest_result = 0x38;
        shared.arm(Some(at), None);
        let got = run(imu.self_test(&mut delay, &SelfTestConfig::new()));
        assert_eq!(got, Err(Error::Bus(BusFault)), "transaction {at}");
        assert_final_reset_attempted(&shared, at, clean_transactions(3));
        assert_eq!(shared.transactions(), at + 3, "transaction {at}");
        not_configured(&mut imu);
        init_works(&mut imu, &mut delay);
    }
}

#[test]
fn self_test_cancel_at_every_await_leaves_it_not_configured_and_init_resets_and_works() {
    // A dropped future cannot run the final soft reset, unlike a bus error. So after the drop the
    // driver is not configured and nothing more happens on the bus; the eDMP may still be running
    // (only once it was started, after the writes of step 12), and `init`, which starts with a soft
    // reset, stops it and works.
    for (polls, awaits) in [(1u32, 74u32), (3, 78)] {
        for k in 1..=awaits {
            let (shared, mut imu, mut delay) = configured();
            shared.chip().selftest_polls = polls;
            shared.arm(None, Some(k));
            assert!(
                matches!(
                    poll_then_drop(imu.self_test(&mut delay, &SelfTestConfig::new())),
                    Polled::Cancelled
                ),
                "await {k}"
            );
            not_configured(&mut imu);
            let transactions = shared.transactions();
            if shared.chip().selftest_left.is_some() {
                assert_eq!(shared.chip().selftest_starts, 1, "await {k}");
            }
            init_works(&mut imu, &mut delay);
            assert!(shared.transactions() > transactions, "await {k}");
            assert_eq!(shared.chip().selftest_left, None, "await {k}");
        }
        let (shared, mut imu, mut delay) = configured();
        shared.chip().selftest_polls = polls;
        shared.arm(None, Some(awaits + 1));
        assert!(matches!(
            poll_then_drop(imu.self_test(&mut delay, &SelfTestConfig::new())),
            Polled::Done(Ok(_))
        ));
        assert_eq!(shared.chip().selftest_left, None);
    }
}

#[test]
fn self_test_blocking_front_end_has_the_same_traffic_and_result() {
    let mut logs = Vec::new();
    let mut results = Vec::new();
    for result in [0x00u8, 0x12] {
        let a = Shared::new();
        a.chip().selftest_result = result;
        let mut imu = crate::Icm45686::new(a.bus());
        let mut delay = a.delay();
        let got_a = run(imu.self_test(&mut delay, &SelfTestConfig::new()));

        let b = Shared::new();
        b.chip().selftest_result = result;
        let mut bimu = blocking::Icm45686::new(b.bus());
        let mut bdelay = b.delay();
        let got_b = bimu.self_test(&mut bdelay, &SelfTestConfig::new());
        assert_eq!(got_a, got_b);
        assert_eq!(a.log(), b.log());
        logs.push(a.log());
        results.push(got_a);
    }
    assert_ne!(logs[0], logs[1]);
    assert!(matches!(results[1], Ok(r) if !r.passed()));
    let (_, mut imu, mut delay) = rig();
    assert!(run(imu.self_test(&mut delay, &SelfTestConfig::new())).is_ok());
}
