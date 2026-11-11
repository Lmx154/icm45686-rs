//! Host tests of the FIFO methods, `interrupt_status`, `stats` and `info` on the fake bus: golden
//! traces (D2, E1), the two vendor errata, proptests, errors before any bus access, bus errors and
//! cancels at every point (D3, D4), the watermark image and limits, blocking parity (A11) and `Send`
//! (A5). Page numbers are DS-000489 Rev 1.1 printed pages.

use proptest::prelude::*;

use crate::driver_tests::{Imu, Res, rig, snapshot, us, w};
use crate::testbus::{BusFault, Event, Op, Polled, Shared, TestDelay, poll_then_drop, run};
use crate::{
    Config, ConfigError, Error, FifoConfig, FifoDepth, FifoFormat, FifoMode, FifoRead, Int1Config,
    InterruptStatus, Stats, blocking, fifo_frames, image, reg,
};

#[derive(Clone, Copy, Debug)]
enum Fmt {
    Accel8,
    Both16,
    HiRes20,
}

const FORMATS: [Fmt; 3] = [Fmt::Accel8, Fmt::Both16, Fmt::HiRes20];
const DEPTHS: [FifoDepth; 2] = [FifoDepth::Kib2, FifoDepth::Kib8];

fn fifo_of(fmt: Fmt, depth: FifoDepth) -> FifoConfig {
    let f = FifoConfig::new()
        .with_mode(FifoMode::Stream)
        .with_depth(depth);
    match fmt {
        Fmt::Accel8 => f.with_gyro(false),
        Fmt::Both16 => f,
        Fmt::HiRes20 => f.with_format(FifoFormat::HighResolution),
    }
}

fn size(fmt: Fmt) -> usize {
    match fmt {
        Fmt::Accel8 => 8,
        Fmt::Both16 => 16,
        Fmt::HiRes20 => 20,
    }
}

fn config_of(f: FifoConfig) -> Config {
    Config::new().with_fifo(f)
}

/// Frame number `i` of `n` bytes: a plausible header, then bytes that differ per frame.
fn frame(i: usize, n: usize) -> Vec<u8> {
    let mut v: Vec<u8> = (0..n).map(|j| (i * 31 + j * 7 + 1) as u8).collect();
    v[0] = match n {
        8 => 0x40,
        16 => 0x60,
        _ => 0x70,
    };
    v
}

fn push(shared: &Shared, from: usize, count: usize, n: usize) -> Vec<u8> {
    let mut all = Vec::new();
    for i in from..from + count {
        let f = frame(i, n);
        shared.chip().push_frame(&f);
        all.extend(f);
    }
    all
}

fn setup(f: FifoConfig) -> (Shared, Imu, TestDelay) {
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, &config_of(f))).unwrap();
    shared.arm(None, None);
    (shared, imu, delay)
}

fn rd(addr: u8, bytes: Vec<u8>) -> Event {
    Event::Spi(vec![Op::Write(vec![addr | 0x80]), Op::Read(bytes)])
}

fn std16() -> FifoConfig {
    fifo_of(Fmt::Both16, FifoDepth::Kib2)
}

// ---------------------------------------------------------------------------------------------
// Golden traces (D2, E1)
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_read_with_frames_waiting_is_three_transactions() {
    // 16.18-16.20, p71: FIFO_COUNT_0 (0x12), FIFO_COUNT_1 (0x13), FIFO_DATA (0x14); the count is read
    // twice (vendor erratum AN-000364 2.2) and at most count - 1 frames are read (vendor).
    let (shared, mut imu, _d) = setup(std16());
    let all = push(&shared, 0, 10, 16);
    shared.arm(None, None);
    let mut buf = [0u8; 1000];
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    assert_eq!(
        shared.log(),
        vec![
            rd(reg::FIFO_COUNT_0, vec![10, 0]),
            rd(reg::FIFO_COUNT_0, vec![10, 0]),
            rd(reg::FIFO_DATA, all[..9 * 16].to_vec()),
        ]
    );
    assert_eq!(
        got,
        FifoRead {
            bytes: 144,
            frames: 9,
            left: Some(1),
            overflow: Some(false),
        }
    );
    assert_eq!(&buf[..144], &all[..144]);
    assert_eq!(shared.violations(), 0);
}

#[test]
fn fifo_read_is_limited_by_the_buffer_to_whole_frames() {
    let (shared, mut imu, _d) = setup(std16());
    let all = push(&shared, 0, 10, 16);
    shared.arm(None, None);
    let mut buf = [0u8; 3 * 16 + 15];
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    assert_eq!((got.frames, got.bytes, got.left), (3, 48, Some(7)));
    assert_eq!(&buf[..48], &all[..48]);
    assert_eq!(shared.transactions(), 3);
}

#[test]
fn fifo_read_with_zero_or_one_frame_waiting_is_two_transactions() {
    for n in [0usize, 1] {
        let (shared, mut imu, _d) = setup(std16());
        push(&shared, 0, n, 16);
        shared.arm(None, None);
        let mut buf = [0u8; 64];
        let got = run(imu.fifo_read(&mut buf)).unwrap();
        assert_eq!(shared.transactions(), 2, "{n} waiting");
        assert_eq!(
            got,
            FifoRead {
                bytes: 0,
                frames: 0,
                left: Some(n as u16),
                overflow: Some(false),
            }
        );
        assert_eq!(shared.chip().fifo_frames(), n);
    }
}

#[test]
fn fifo_read_exact_is_one_transaction_and_claims_nothing_else() {
    let (shared, mut imu, _d) = setup(std16());
    let all = push(&shared, 0, 5, 16);
    shared.arm(None, None);
    let mut buf = [0u8; 100];
    let got = run(imu.fifo_read_exact(&mut buf, 4)).unwrap();
    assert_eq!(shared.log(), vec![rd(reg::FIFO_DATA, all[..64].to_vec())]);
    assert_eq!(
        got,
        FifoRead {
            bytes: 64,
            frames: 4,
            left: None,
            overflow: None,
        }
    );
    assert_ne!(got.left, Some(0));
    assert_eq!(imu.stats().fifo_frames_read, 4);
    assert_eq!(shared.chip().fifo_frames(), 1);
}

#[test]
fn fifo_count_is_two_transactions_in_frames() {
    let (shared, mut imu, _d) = setup(std16());
    push(&shared, 0, 100, 16);
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_count()), Ok(100));
    assert_eq!(
        shared.log(),
        vec![
            rd(reg::FIFO_COUNT_0, vec![100, 0]),
            rd(reg::FIFO_COUNT_0, vec![100, 0])
        ]
    );
}

#[test]
fn fifo_flush_writes_the_flush_bit_then_polls_until_it_clears() {
    // 16.31, p81: FIFO_FLUSH is bit 7 of FIFO_CONFIG2; the applied value is 0x28 (FIFO_WR_WM_GT_TH).
    // Poll every 100 us (vendor).
    let (shared, mut imu, mut delay) = setup(std16());
    push(&shared, 0, 20, 16);
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_flush(&mut delay)), Ok(()));
    assert_eq!(
        shared.log(),
        vec![
            w(reg::FIFO_CONFIG2, 0xA8),
            us(100),
            rd(reg::FIFO_CONFIG2, vec![0x28])
        ]
    );
    assert_eq!(shared.chip().fifo_frames(), 0);
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_count()), Ok(0));
}

#[test]
fn fifo_flush_times_out_after_the_polling_limit() {
    let (shared, mut imu, mut delay) = setup(std16());
    shared.chip().flush_stuck = true;
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_flush(&mut delay)), Err(Error::FlushTimeout));
    assert_eq!(shared.transactions(), 1 + 10_000);
    assert_eq!(imu.stats().bus_errors, 0);
}

#[test]
fn interrupt_status_is_one_transaction_and_clears_on_read() {
    // 16.24-16.25, p76-p77: INT1_STATUS0 (0x19) and INT1_STATUS1 (0x1A), both R/C.
    let (shared, mut imu, _d) = setup(std16());
    {
        let mut c = shared.chip();
        c.regs[reg::INT1_STATUS0 as usize] = reg::FIFO_THS | reg::DRDY | reg::AP_AGC_RDY;
        c.regs[reg::INT1_STATUS1 as usize] = 0x01;
    }
    shared.arm(None, None);
    let got = run(imu.interrupt_status()).unwrap();
    assert_eq!(shared.log(), vec![rd(reg::INT1_STATUS0, vec![0x26, 0x01])]);
    assert_eq!(got, InterruptStatus::decode(0x26, 0x01));
    assert!(got.data_ready && got.fifo_threshold && got.agc_ready);
    assert!(!got.fifo_full && !got.reset_done);
    assert_eq!((got.status0, got.status1), (0x26, 0x01));
    assert_eq!(run(imu.interrupt_status()), Ok(InterruptStatus::default()));
    assert_eq!(imu.stats().fifo_overflows, 0);
}

#[test]
fn interrupt_status_works_while_not_configured() {
    let (shared, mut imu, _d) = rig();
    shared.chip().regs[reg::INT1_STATUS0 as usize] = reg::RESET_DONE;
    let got = run(imu.interrupt_status()).unwrap();
    assert!(got.reset_done);
    assert_eq!(imu.config(), None);
}

// ---------------------------------------------------------------------------------------------
// The two vendor errata
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_read_uses_the_second_count_when_the_first_is_stale() {
    for stale in [0u16, 3, 50] {
        let (shared, mut imu, _d) = setup(std16());
        push(&shared, 0, 8, 16);
        shared.chip().count_script.push_back(stale);
        shared.arm(None, None);
        let mut buf = [0u8; 400];
        let got = run(imu.fifo_read(&mut buf)).unwrap();
        assert_eq!(got.frames, 7, "stale first read {stale}");
        assert_eq!(got.left, Some(1));
        assert_eq!(run(imu.fifo_count()), Ok(1));
    }
    // fifo_count also returns the second value.
    let (shared, mut imu, _d) = setup(std16());
    push(&shared, 0, 8, 16);
    shared.chip().count_script.push_back(99);
    assert_eq!(run(imu.fifo_count()), Ok(8));
}

#[test]
fn fifo_read_never_drains_the_fifo() {
    for m in 0usize..40 {
        let (shared, mut imu, _d) = setup(std16());
        push(&shared, 0, m, 16);
        let mut buf = [0u8; 1000];
        let got = run(imu.fifo_read(&mut buf)).unwrap();
        assert_eq!(got.frames, m.saturating_sub(1), "{m} waiting");
        assert_eq!(shared.chip().fifo_frames(), m.min(1));
    }
}

// ---------------------------------------------------------------------------------------------
// Proptest
// ---------------------------------------------------------------------------------------------

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn fifo_read_never_splits_a_frame_and_returns_the_queued_bytes(
        waiting in 0usize..=70,
        buf_len in 0usize..=1500,
        fmt in 0usize..3,
        depth in 0usize..2,
    ) {
        let fmt = FORMATS[fmt];
        let n = size(fmt);
        let fc = fifo_of(fmt, DEPTHS[depth]);
        let (shared, mut imu, _d) = setup(fc);
        let all = push(&shared, 0, waiting, n);
        shared.arm(None, None);
        let mut buf = vec![0xEEu8; buf_len];
        let got = run(imu.fifo_read(&mut buf));
        if buf_len < n {
            prop_assert_eq!(got, Err(Error::BufferTooSmall));
            prop_assert_eq!(shared.transactions(), 0);
            return Ok(());
        }
        let got = got.unwrap();
        prop_assert_eq!(got.bytes, got.frames * n);
        prop_assert!(got.bytes <= buf_len);
        prop_assert_eq!(got.frames, waiting.saturating_sub(1).min(buf_len / n));
        prop_assert_eq!(&buf[..got.bytes], &all[..got.bytes]);
        prop_assert!(buf[got.bytes..].iter().all(|b| *b == 0xEE));
        prop_assert_eq!(got.overflow, Some(false));
        prop_assert_eq!(got.left, Some((waiting - got.frames) as u16));
        prop_assert_eq!(shared.chip().fifo_frames(), waiting - got.frames);
        prop_assert_eq!(shared.transactions(), if got.frames == 0 { 2 } else { 3 });
        let read: Vec<_> = fifo_frames(&buf[..got.bytes], &fc).collect();
        let queued: Vec<_> = fifo_frames(&all[..got.bytes], &fc).collect();
        prop_assert_eq!(read.len(), got.frames);
        prop_assert_eq!(read, queued);
    }
}

// ---------------------------------------------------------------------------------------------
// Overflow and stats (C4)
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_read_reports_overflow_from_the_capacity_up_to_the_saturated_count() {
    // Measured on the board, 2026-11-10: in stream mode the count saturates at the nominal capacity
    // plus 2 frames (130 for 2 KB and 16-byte frames, 514 for 8 KB); the fake does the same.
    for depth in DEPTHS {
        for fmt in FORMATS {
            let n = size(fmt);
            let fc = fifo_of(fmt, depth);
            let capacity = usize::from(fc.capacity_frames());
            let mut buf = vec![0u8; 20];
            let (shared, mut imu, _d) = setup(fc);
            push(&shared, 0, capacity - 1, n);
            let got = run(imu.fifo_read(&mut buf)).unwrap();
            assert_eq!(got.overflow, Some(false), "{fmt:?} {depth:?}");
            assert_eq!(imu.stats().fifo_overflows, 0);
            for (pushed, count) in [
                (capacity, capacity),
                (capacity + 1, capacity + 1),
                (capacity + 2, capacity + 2),
                (capacity + 3, capacity + 2),
                (capacity + 9, capacity + 2),
            ] {
                let (shared, mut imu, _d) = setup(fc);
                push(&shared, 0, pushed, n);
                assert_eq!(shared.chip().fifo_frames(), count, "{fmt:?} {depth:?}");
                assert_eq!(run(imu.fifo_count()), Ok(count as u16));
                let got = run(imu.fifo_read(&mut buf)).unwrap();
                assert_eq!(got.overflow, Some(true), "{fmt:?} {depth:?} {pushed}");
                assert_eq!(imu.stats().fifo_overflows, 1, "once per call");
                assert_eq!(imu.stats().fifo_frames_read, got.frames as u64);
            }
        }
    }
}

#[test]
fn the_saturated_count_is_130_and_514_for_16_byte_frames() {
    for (depth, capacity) in [(FifoDepth::Kib2, 128usize), (FifoDepth::Kib8, 512)] {
        let (shared, mut imu, _d) = setup(fifo_of(Fmt::Both16, depth));
        push(&shared, 0, capacity + 40, 16);
        assert_eq!(run(imu.fifo_count()), Ok(capacity as u16 + 2));
    }
}

#[test]
fn stats_count_overflows_once_per_call_and_frames() {
    let (shared, mut imu, _d) = setup(std16());
    assert_eq!(*imu.stats(), Stats::default());
    push(&shared, 0, 128, 16);
    // The harness set FIFO_FULL in INT1_STATUS0: interrupt_status shows it and counts it once.
    assert!(run(imu.interrupt_status()).unwrap().fifo_full);
    assert_eq!(imu.stats().fifo_overflows, 1);
    assert!(!run(imu.interrupt_status()).unwrap().fifo_full);
    assert_eq!(imu.stats().fifo_overflows, 1);
    let mut buf = [0u8; 16 * 20];
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    assert_eq!((got.frames, got.overflow), (20, Some(true)));
    assert_eq!(imu.stats().fifo_overflows, 2);
    assert_eq!(imu.stats().fifo_frames_read, 20);
    run(imu.fifo_read_exact(&mut buf, 5)).unwrap();
    assert_eq!(imu.stats().fifo_frames_read, 25);
    assert_eq!(imu.stats().fifo_overflows, 2);
    // Nothing clears them: not a new init.
    let mut delay = shared.delay();
    run(imu.init(&mut delay, &config_of(std16()))).unwrap();
    assert_eq!(imu.stats().fifo_frames_read, 25);
}

#[test]
fn stats_saturate_and_bus_errors_are_counted() {
    let (shared, mut imu, _d) = setup(std16());
    imu.set_stats(Stats {
        fifo_overflows: u32::MAX,
        fifo_frames_read: u64::MAX,
        bus_errors: u32::MAX,
    });
    push(&shared, 0, 128, 16);
    let mut buf = [0u8; 64];
    let got = run(imu.fifo_read(&mut buf)).unwrap();
    assert_eq!(got.overflow, Some(true));
    shared.arm(Some(1), None);
    assert_eq!(run(imu.fifo_count()), Err(Error::Bus(BusFault)));
    assert_eq!(
        *imu.stats(),
        Stats {
            fifo_overflows: u32::MAX,
            fifo_frames_read: u64::MAX,
            bus_errors: u32::MAX,
        }
    );
    // And an unsaturated counter counts one bus error.
    let (shared, mut imu, _d) = setup(std16());
    shared.arm(Some(2), None);
    assert_eq!(run(imu.fifo_count()), Err(Error::Bus(BusFault)));
    assert_eq!(imu.stats().bus_errors, 1);
}

#[test]
fn info_is_none_until_configured_then_what_init_returned() {
    let (shared, mut imu, mut delay) = rig();
    assert_eq!(imu.info(), None);
    let info = run(imu.init(&mut delay, &config_of(std16()))).unwrap();
    assert_eq!(imu.info(), Some(info));
    assert_eq!(info.fifo_frame_bytes, 16);
    shared.arm(Some(1), None);
    assert!(run(imu.configure(&mut delay, &Config::new())).is_err());
    assert_eq!(imu.info(), None);
    shared.arm(None, None);
    let info = run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_eq!(imu.info(), Some(info));
    assert_eq!(info.fifo_frame_bytes, 0);
}

// ---------------------------------------------------------------------------------------------
// Errors before any bus access
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_read_errors_are_found_before_any_bus_access() {
    let mut buf = [0u8; 100];
    // Buffer shorter than one frame.
    let (shared, mut imu, _d) = setup(std16());
    assert_eq!(
        run(imu.fifo_read(&mut buf[..15])),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(run(imu.fifo_read(&mut [])), Err(Error::BufferTooSmall));
    assert_eq!(shared.transactions(), 0);
    // FIFO off.
    let (shared, mut imu, _d) = setup(FifoConfig::new());
    assert_eq!(run(imu.fifo_read(&mut buf)), Err(Error::FifoOff));
    assert_eq!(shared.transactions(), 0);
    // Not configured.
    let (shared, mut imu, _d) = rig();
    assert_eq!(run(imu.fifo_read(&mut buf)), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
}

#[test]
fn fifo_read_exact_errors_are_found_before_any_bus_access() {
    let mut buf = [0u8; 100];
    let (shared, mut imu, _d) = setup(std16());
    assert_eq!(
        run(imu.fifo_read_exact(&mut buf, 0)),
        Err(Error::ZeroFrames)
    );
    assert_eq!(
        run(imu.fifo_read_exact(&mut buf, 7)),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(
        run(imu.fifo_read_exact(&mut buf, usize::MAX)),
        Err(Error::BufferTooSmall)
    );
    assert_eq!(shared.transactions(), 0);
    assert!(run(imu.fifo_read_exact(&mut buf, 6)).is_ok());
    let (shared, mut imu, _d) = setup(FifoConfig::new());
    assert_eq!(run(imu.fifo_read_exact(&mut buf, 1)), Err(Error::FifoOff));
    assert_eq!(shared.transactions(), 0);
    let (shared, mut imu, _d) = rig();
    assert_eq!(
        run(imu.fifo_read_exact(&mut buf, 1)),
        Err(Error::NotConfigured)
    );
    assert_eq!(shared.transactions(), 0);
}

#[test]
fn fifo_count_and_fifo_flush_errors_are_found_before_any_bus_access() {
    let (shared, mut imu, mut delay) = setup(FifoConfig::new());
    assert_eq!(run(imu.fifo_count()), Err(Error::FifoOff));
    assert_eq!(run(imu.fifo_flush(&mut delay)), Err(Error::FifoOff));
    assert_eq!(shared.transactions(), 0);
    let (shared, mut imu, mut delay) = rig();
    assert_eq!(run(imu.fifo_count()), Err(Error::NotConfigured));
    assert_eq!(run(imu.fifo_flush(&mut delay)), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
    assert!(shared.log().is_empty());
}

// ---------------------------------------------------------------------------------------------
// D3 and D4: a bus error at every transaction index, a cancel at every await point
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum M {
    Read,
    Exact,
    Count,
    Flush,
    Status,
}

const METHODS: [M; 5] = [M::Read, M::Exact, M::Count, M::Flush, M::Status];

#[derive(Debug, PartialEq, Eq)]
enum Out {
    Read(FifoRead, Vec<u8>),
    Count(u16),
    Done,
    Status(InterruptStatus),
}

async fn call(imu: &mut Imu, delay: &mut TestDelay, m: M) -> Res<Out> {
    let mut buf = [0u8; 400];
    Ok(match m {
        M::Read => {
            let r = imu.fifo_read(&mut buf).await?;
            Out::Read(r, buf[..r.bytes].to_vec())
        }
        M::Exact => {
            let r = imu.fifo_read_exact(&mut buf, 4).await?;
            Out::Read(r, buf[..r.bytes].to_vec())
        }
        M::Count => Out::Count(imu.fifo_count().await?),
        M::Flush => {
            imu.fifo_flush(delay).await?;
            Out::Done
        }
        M::Status => Out::Status(imu.interrupt_status().await?),
    })
}

/// 10 frames waiting and FIFO_THS flagged. In the fake a failed or cancelled transaction has no
/// effect, so nothing is popped or cleared before the transaction that fails; what a real device
/// does with a data transfer that stops half-way is in the module notes of `fifo_io`.
fn loaded() -> (Shared, Imu, TestDelay) {
    let (shared, imu, delay) = setup(std16());
    push(&shared, 0, 10, 16);
    shared.chip().regs[reg::INT1_STATUS0 as usize] = reg::FIFO_THS;
    shared.arm(None, None);
    (shared, imu, delay)
}

fn reference(m: M) -> (Out, usize, u32, usize) {
    let (shared, mut imu, mut delay) = loaded();
    let out = run(call(&mut imu, &mut delay, m)).unwrap();
    let left = shared.chip().fifo_frames();
    (out, shared.transactions(), shared.awaits(), left)
}

#[test]
fn fifo_methods_bus_error_at_every_transaction_then_the_same_call_works() {
    for m in METHODS {
        let (expected, transactions, _, left) = reference(m);
        let mut idx = 0;
        loop {
            idx += 1;
            let (shared, mut imu, mut delay) = loaded();
            shared.arm(Some(idx), None);
            match run(call(&mut imu, &mut delay, m)) {
                Ok(out) => {
                    assert_eq!(out, expected);
                    assert_eq!(idx - 1, transactions, "{m:?}");
                    break;
                }
                Err(Error::Bus(BusFault)) => {}
                Err(e) => panic!("{m:?} at {idx}: {e:?}"),
            }
            assert_eq!(imu.stats().bus_errors, 1);
            assert_eq!(imu.stats().fifo_frames_read, 0);
            assert_eq!(shared.violations(), 0);
            assert!(imu.config().is_some());
            shared.arm(None, None);
            assert_eq!(
                run(call(&mut imu, &mut delay, m)).unwrap(),
                expected,
                "{m:?} {idx}"
            );
            assert_eq!(shared.chip().fifo_frames(), left);
        }
    }
}

#[test]
fn fifo_methods_cancel_at_every_await_then_the_same_call_gives_the_right_result() {
    for m in METHODS {
        let (expected, _, awaits, left) = reference(m);
        let mut k = 0;
        loop {
            k += 1;
            let (shared, mut imu, mut delay) = loaded();
            shared.arm(None, Some(k));
            match poll_then_drop(call(&mut imu, &mut delay, m)) {
                Polled::Done(out) => {
                    assert_eq!(out.unwrap(), expected);
                    assert_eq!(k - 1, awaits, "{m:?}");
                    break;
                }
                Polled::Cancelled => {}
            }
            assert!(imu.config().is_some());
            shared.arm(None, None);
            assert_eq!(
                run(call(&mut imu, &mut delay, m)).unwrap(),
                expected,
                "{m:?} {k}"
            );
            assert_eq!(shared.chip().fifo_frames(), left);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Watermark: N is programmed as N + 1
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_watermark_image_is_n_plus_one_and_zero_stays_zero() {
    for n in [1u16, 2, 10, 127, 255, 256] {
        let c = config_of(std16().with_watermark(n));
        assert_eq!(image::fifo_watermark(&c), (n + 1).to_le_bytes());
    }
    assert_eq!(image::fifo_watermark(&config_of(std16())), [0, 0]);
}

#[test]
fn fifo_watermark_largest_accepted_value_per_frame_size_and_depth() {
    // (frame size, depth) -> capacity frames -> the largest N is capacity - 1.
    let cases = [
        (Fmt::Accel8, FifoDepth::Kib2, 255u16),
        (Fmt::Both16, FifoDepth::Kib2, 127),
        (Fmt::HiRes20, FifoDepth::Kib2, 101),
        (Fmt::Accel8, FifoDepth::Kib8, 1023),
        (Fmt::Both16, FifoDepth::Kib8, 511),
        (Fmt::HiRes20, FifoDepth::Kib8, 408),
    ];
    for (fmt, depth, largest) in cases {
        let f = fifo_of(fmt, depth);
        assert_eq!(f.capacity_frames(), largest + 1, "{fmt:?} {depth:?}");
        assert_eq!(config_of(f.with_watermark(largest)).validate(), Ok(()));
        assert_eq!(
            config_of(f.with_watermark(largest + 1)).validate(),
            Err(ConfigError::FifoWatermarkTooLarge),
            "{fmt:?} {depth:?}"
        );
        assert_eq!(config_of(f.with_watermark(0)).validate(), Ok(()));
    }
}

#[test]
fn fifo_watermark_init_writes_n_plus_one_and_verify_is_ok() {
    let c = config_of(std16().with_watermark(40))
        .with_int1(Int1Config::new().with_fifo_threshold(true));
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, &c)).unwrap();
    {
        let chip = shared.chip();
        assert_eq!(chip.regs[reg::FIFO_CONFIG1_0 as usize], 41);
        assert_eq!(chip.regs[reg::FIFO_CONFIG1_1 as usize], 0);
    }
    assert_eq!(run(imu.verify(&mut delay)), Ok(()));
    run(imu.configure(&mut delay, &c)).unwrap();
    assert_eq!(run(imu.verify(&mut delay)), Ok(()));
}

// ---------------------------------------------------------------------------------------------
// A11 blocking parity and A5 Send
// ---------------------------------------------------------------------------------------------

#[test]
fn fifo_methods_blocking_front_end_matches_the_async_driver() {
    let c = config_of(std16());
    let (a_shared, mut a_imu, mut a_delay) = rig();
    let b_shared = Shared::new();
    let mut b_imu = blocking::Icm45686::new(b_shared.bus());
    let mut b_delay = b_shared.delay();
    assert_eq!(b_imu.info(), None);
    run(a_imu.init(&mut a_delay, &c)).unwrap();
    b_imu.init(&mut b_delay, &c).unwrap();
    assert_eq!(a_imu.info(), b_imu.info());
    for s in [&a_shared, &b_shared] {
        push(s, 0, 30, 16);
        s.chip().regs[reg::INT1_STATUS0 as usize] = reg::FIFO_THS | reg::FIFO_FULL;
        s.arm(None, None);
    }
    let (mut ab, mut bb) = ([0u8; 200], [0u8; 200]);
    assert_eq!(run(a_imu.fifo_count()), b_imu.fifo_count());
    assert_eq!(run(a_imu.interrupt_status()), b_imu.interrupt_status());
    assert_eq!(run(a_imu.fifo_read(&mut ab)), b_imu.fifo_read(&mut bb));
    assert_eq!(ab, bb);
    assert_eq!(
        run(a_imu.fifo_read_exact(&mut ab, 3)),
        b_imu.fifo_read_exact(&mut bb, 3)
    );
    assert_eq!(ab, bb);
    assert_eq!(
        run(a_imu.fifo_read(&mut ab[..5])),
        b_imu.fifo_read(&mut bb[..5])
    );
    assert_eq!(
        run(a_imu.fifo_read_exact(&mut ab, 0)),
        b_imu.fifo_read_exact(&mut bb, 0)
    );
    assert_eq!(
        run(a_imu.fifo_flush(&mut a_delay)),
        b_imu.fifo_flush(&mut b_delay)
    );
    assert_eq!(run(a_imu.fifo_count()), b_imu.fifo_count());
    assert_eq!(a_imu.stats(), b_imu.stats());
    assert_eq!(a_shared.log(), b_shared.log());
    assert_eq!(snapshot(&a_shared), snapshot(&b_shared));
    assert!(a_imu.stats().fifo_frames_read > 0);
}

fn assert_send<T: Send>(_: &T) {}

#[test]
fn fifo_method_futures_are_send_when_bus_and_delay_are() {
    let (_shared, mut imu, mut delay) = rig();
    let mut buf = [0u8; 32];
    assert_send(&imu.fifo_count());
    assert_send(&imu.fifo_flush(&mut delay));
    assert_send(&imu.interrupt_status());
    assert_send(&imu.fifo_read_exact(&mut buf, 1));
    assert_send(&imu.fifo_read(&mut buf));
}

#[test]
fn fifo_count_is_a_little_endian_word() {
    // 14, p57: the FIFO count is little endian by default, so 0x12 carries the low byte. A count
    // above 255 tells the two byte orders apart: 300 = 0x012C is sent as 2C 01.
    let (shared, mut imu, _d) = setup(std16());
    shared.chip().count_script.push_back(300);
    shared.chip().count_script.push_back(300);
    shared.arm(None, None);
    assert_eq!(run(imu.fifo_count()), Ok(300));
    assert_eq!(
        shared.log(),
        vec![
            rd(reg::FIFO_COUNT_0, vec![0x2C, 0x01]),
            rd(reg::FIFO_COUNT_0, vec![0x2C, 0x01])
        ]
    );
}
