//! Host tests of the driver methods on the fake bus of [`crate::testbus`]: golden traces,
//! transaction counts, the validity state, bus errors and cancels at every point, the blocking
//! front-end against the async one, and `Send`.

use std::collections::BTreeMap;
use std::vec;

use crate::plan::{Plan, Step};
use crate::testbus::{
    BusFault, Event, Op, Polled, Shared, TestBus, TestDelay, poll_then_drop, run,
};
use crate::{
    ClockSource, Config, ConfigError, Error, Icm45686, Info, Int1Config, Odr, RawSample,
    SensorPower, blocking, reg,
};

pub(crate) type Imu = Icm45686<TestBus>;
pub(crate) type Res<T> = Result<T, Error<BusFault>>;

pub(crate) fn rig() -> (Shared, Imu, TestDelay) {
    let shared = Shared::new();
    let imu = Icm45686::new(shared.bus());
    let delay = shared.delay();
    (shared, imu, delay)
}

pub(crate) fn configured() -> (Shared, Imu, TestDelay) {
    let (shared, mut imu, mut delay) = rig();
    run(imu.init(&mut delay, &Config::new())).unwrap();
    (shared, imu, delay)
}

pub(crate) fn clkin() -> Config {
    Config::new()
        .with_clock(ClockSource::ExternalClkin)
        .with_accel_odr(Odr::Hz200)
        .with_gyro_odr(Odr::Hz200)
        .with_int1(Int1Config::new().with_data_ready(true))
}

pub(crate) fn configs() -> [Config; 2] {
    [Config::new(), clkin()]
}

pub(crate) fn w(reg: u8, value: u8) -> Event {
    Event::Spi(vec![Op::Write(vec![reg, value])])
}

pub(crate) fn r(reg: u8, got: u8) -> Event {
    Event::Spi(vec![Op::Write(vec![reg | 0x80]), Op::Read(vec![got])])
}

pub(crate) fn us(us: u64) -> Event {
    Event::Delay(us * 1000)
}

/// Transactions of a plan run: a write is one, a read-modify-write is two, an indirect write is
/// one, an indirect read-modify-write is three.
fn plan_transactions(plan: &Plan) -> usize {
    plan.steps()
        .iter()
        .map(|s| match s {
            Step::Write { .. } | Step::IregWrite { .. } => 1,
            Step::Modify { .. } => 2,
            Step::IregModify { .. } => 3,
            _ => 0,
        })
        .sum()
}

/// Awaits of a plan run beyond its transactions: its delays and the 10 us indirect waits.
fn plan_delays(plan: &Plan) -> u32 {
    plan.steps()
        .iter()
        .map(|s| match s {
            Step::DelayUs(_) => 1,
            Step::IregWrite { .. } => 1,
            Step::IregModify { .. } => 3,
            _ => 0,
        })
        .sum()
}

#[derive(Clone, Copy, Debug)]
enum Method {
    Init,
    Configure,
    Read,
}

#[derive(Debug, PartialEq, Eq)]
enum Out {
    Info(Info),
    Sample(RawSample),
}

async fn call(imu: &mut Imu, delay: &mut TestDelay, config: &Config, m: Method) -> Res<Out> {
    Ok(match m {
        Method::Init => Out::Info(imu.init(delay, config).await?),
        Method::Configure => Out::Info(imu.configure(delay, config).await?),
        Method::Read => Out::Sample(imu.read().await?),
    })
}

pub(crate) fn expected_read(shared: &Shared) -> RawSample {
    let c = shared.chip();
    let mut b = [0u8; 14];
    b.copy_from_slice(&c.regs[..14]);
    RawSample::decode(&b)
}

pub(crate) type Snapshot = ([u8; 256], BTreeMap<u16, u8>);

pub(crate) fn snapshot(shared: &Shared) -> Snapshot {
    let c = shared.chip();
    (c.regs, c.ireg.clone())
}

// ---------------------------------------------------------------------------------------------
// Golden traces (D2) and transaction counts (E1)
// ---------------------------------------------------------------------------------------------

/// Events of the twelve offset writes: 12 transactions, each followed by the 10 us wait.
const OFFSET_EVENTS: usize = 24;

/// The twelve offset writes of zero as they appear on the bus, in write order.
fn zero_offset_writes() -> Vec<Event> {
    let mut v = Vec::new();
    for addr in [
        0xA42A_u16, 0xA42B, 0xA438, 0xA439, 0xA446, 0xA447, 0xA518, 0xA519, 0xA520, 0xA521, 0xA528,
        0xA529,
    ] {
        let [hi, lo] = addr.to_be_bytes();
        v.push(Event::Spi(vec![Op::Write(vec![0x7C, hi, lo, 0x00])]));
        v.push(us(10));
    }
    v
}

#[test]
fn init_default_golden_trace() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    let info = run(imu.init(&mut delay, &Config::new())).unwrap();
    // Register addresses and values from the datasheet (DS-000489 Rev 1.1), page in each comment.
    let mut want = vec![
        r(0x19, 0x00), // INT1_STATUS0 read to clear it (16.24, p76)
        w(0x7F, 0x02), // REG_MISC2 = SOFT_RST (16.84, p109)
        us(1_000),     // 1 ms after the soft reset (vendor)
        r(0x19, 0x80), // INT1_STATUS0 bit 7 RESET_DONE (16.24, p76)
        r(0x72, 0xE9), // WHO_AM_I = 0xE9 (16.79, p107)
        w(0x16, 0x00), // INT1_CONFIG0 = 0, sources off (16.21, p72)
        w(0x18, 0x01), // INT1_CONFIG2: push-pull, pulse, active high (16.23, p75)
    ];
    // The twelve user-offset registers written to zero (20.1-21.6, p189-p194).
    for addr in [
        0xA42A_u16, 0xA42B, 0xA438, 0xA439, 0xA446, 0xA447, 0xA518, 0xA519, 0xA520, 0xA521, 0xA528,
        0xA529,
    ] {
        let [hi, lo] = addr.to_be_bytes();
        want.push(Event::Spi(vec![Op::Write(vec![0x7C, hi, lo, 0x00])]));
        want.push(us(10));
    }
    want.extend([
        w(0x1B, 0x36), // ACCEL_CONFIG0: +-4 g (code 3), 800 Hz (code 6) (16.26, p78)
        w(0x1C, 0x16), // GYRO_CONFIG0: +-2000 dps (code 1), 800 Hz (code 6) (16.27, p79)
        w(0x16, 0x00), // INT1_CONFIG0 = 0, no event routed (16.21, p72)
        w(0x10, 0x0F), // PWR_MGMT0: gyro and accel low-noise (16.17, p70)
        us(60_000),    // start-up (vendor: 60 ms; Table 1, p18)
    ]);
    assert_eq!(shared.log(), want);
    assert_eq!(shared.violations(), 0);
    assert_eq!(
        info,
        Info {
            who_am_i: 0xE9,
            accel_odr_mhz: 800_000,
            gyro_odr_mhz: 800_000,
            fifo_frame_bytes: 0,
        }
    );
    assert_eq!(imu.config(), Some(&Config::new()));
}

#[test]
fn init_clkin_golden_indirect_sequence() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    run(imu.init(&mut delay, &clkin())).unwrap();
    let log = shared.log();
    // After the 5 reset events, INT1_CONFIG0, INT1_CONFIG2 and the 12 offset writes (24 events): the
    // pad override, then the first
    // indirect read-modify-write of SIFS_I3C_STC_CFG (IPREG_TOP1 0xA200 + 0x68, 20.x p191): address
    // write, 10 us, data read (IREG_DATA 0x7E, 16.83, p108), 10 us, address and data write, 10 us.
    // The gap between indirect accesses is at least 4 us (13.3, p55); the driver waits 10 us, the
    // value that has run on the board.
    let want = vec![
        w(0x31, 0x06),
        Event::Spi(vec![Op::Write(vec![0x7C, 0xA2, 0x68])]),
        us(10),
        r(0x7E, 0x23),
        us(10),
        Event::Spi(vec![Op::Write(vec![0x7C, 0xA2, 0x68, 0x23])]),
        us(10),
    ];
    assert_eq!(log.get(31..38), Some(want.as_slice()));
    let c = shared.chip();
    assert_eq!(c.ireg.get(&reg::SIFS_I3C_STC_CFG), Some(&0x23));
    assert_eq!(c.ireg.get(&reg::IPREG_SYS2_REG_123), Some(&0x16));
    assert_eq!(c.ireg.get(&reg::IPREG_SYS1_REG_166), Some(&0x5B));
    assert_eq!(c.ireg.get(&reg::SMC_CONTROL_0), Some(&0x70));
    assert_eq!(c.regs[reg::RTC_CONFIG as usize], 0x23);
    assert_eq!(c.regs[reg::INT1_CONFIG0 as usize], 0x04);
}

#[test]
fn init_counts_match_the_plan() {
    for config in configs() {
        let (shared, mut imu, mut delay) = rig();
        shared.arm(None, None);
        run(imu.init(&mut delay, &config)).unwrap();
        let plan = Plan::after_reset(&config).unwrap();
        // Reset: status read, reset write, one poll; identity: one read.
        assert_eq!(shared.transactions(), 4 + plan_transactions(&plan));
        assert_eq!(
            shared.awaits(),
            4 + 1 + (plan_transactions(&plan) as u32) + plan_delays(&plan)
        );
    }
}

#[test]
fn init_returns_the_applied_rates_and_frame_size() {
    let (_shared, mut imu, mut delay) = rig();
    let config = Config::new()
        .with_accel_odr(Odr::Hz200)
        .with_gyro_odr(Odr::Hz1600)
        .with_gyro_power(SensorPower::Off);
    let info = run(imu.init(&mut delay, &config)).unwrap();
    assert_eq!(info.who_am_i, 0xE9);
    assert_eq!(info.accel_odr_mhz, 200_000);
    assert_eq!(info.gyro_odr_mhz, 0);
    assert_eq!(info.fifo_frame_bytes, 0);
}

#[test]
fn configure_golden_trace_head_and_tail() {
    let (shared, mut imu, mut delay) = configured();
    shared.arm(None, None);
    let info = run(imu.configure(&mut delay, &Config::new())).unwrap();
    let log = shared.log();
    // Datasheet pages as in `init_default_golden_trace`; FIFO_CONFIG3 0x21 / FIFO_CONFIG0 0x1D
    // (16.32, p82; 16.28, p80). The 150 ms wait after stopping the gyroscope is the vendor's.
    let head = vec![
        r(0x72, 0xE9),
        w(0x10, 0x00),
        us(150_000),
        w(0x16, 0x00),
        w(0x21, 0x00),
        w(0x1D, 0x00),
    ];
    assert_eq!(log.get(..6), Some(head.as_slice()));
    // The plan ends with PWR_MGMT0 and the start-up wait; then the twelve offset registers are
    // written from the stored value, here zero (20.1-21.6, p189-p194).
    let plan_end = vec![w(0x16, 0x00), w(0x10, 0x0F), us(60_000)];
    assert_eq!(
        log.get(log.len() - OFFSET_EVENTS - 3..log.len() - OFFSET_EVENTS),
        Some(plan_end.as_slice())
    );
    assert_eq!(
        log.get(log.len() - OFFSET_EVENTS..),
        Some(zero_offset_writes().as_slice())
    );
    let plan = Plan::reconfigure(&Config::new()).unwrap();
    assert_eq!(shared.transactions(), 1 + plan_transactions(&plan) + 12);
    assert_eq!(shared.violations(), 0);
    assert_eq!(info.who_am_i, 0xE9);
    assert_eq!(imu.config(), Some(&Config::new()));
}

#[test]
fn read_is_one_write_then_read_transaction() {
    let (shared, mut imu, _delay) = configured();
    shared.arm(None, None);
    let s = run(imu.read()).unwrap();
    let c = shared.chip();
    assert_eq!(c.log.len(), 1);
    let Some(Event::Spi(ops)) = c.log.first() else {
        panic!("not a transaction");
    };
    assert_eq!(ops.len(), 2);
    assert_eq!(ops.first(), Some(&Op::Write(vec![0x80])));
    let Some(Op::Read(got)) = ops.get(1) else {
        panic!("second operation is not a read");
    };
    assert_eq!(got.len(), 14);
    assert_eq!(c.transactions, 1);
    assert_eq!(c.awaits, 1);
    assert_eq!(c.violations, 0);
    drop(c);
    assert_eq!(s, expected_read(&shared));
}

#[test]
fn read_register_is_one_write_then_read_transaction() {
    let (shared, mut imu, _delay) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.read_register(reg::WHO_AM_I)), Ok(0xE9));
    assert_eq!(shared.log(), vec![r(0x72, 0xE9)]);
    assert_eq!(shared.violations(), 0);
}

#[test]
fn read_registers_is_one_write_then_read_transaction() {
    let (shared, mut imu, _delay) = rig();
    shared.arm(None, None);
    let mut buf = [0u8; 4];
    run(imu.read_registers(0x02, &mut buf)).unwrap();
    let want = {
        let c = shared.chip();
        [c.regs[2], c.regs[3], c.regs[4], c.regs[5]]
    };
    assert_eq!(buf, want);
    assert_eq!(
        shared.log(),
        vec![Event::Spi(vec![
            Op::Write(vec![0x82]),
            Op::Read(want.to_vec())
        ])]
    );
}

#[test]
fn write_register_is_one_write_operation() {
    let (shared, mut imu, _delay) = rig();
    shared.arm(None, None);
    run(imu.write_register(reg::PWR_MGMT0, 0x0F)).unwrap();
    assert_eq!(shared.log(), vec![w(0x10, 0x0F)]);
    assert_eq!(shared.chip().regs[0x10], 0x0F);
}

#[test]
fn write_registers_is_one_transaction_with_increment() {
    let (shared, mut imu, _delay) = rig();
    shared.arm(None, None);
    run(imu.write_registers(reg::FIFO_CONFIG1_0, &[0x12, 0x34])).unwrap();
    assert_eq!(
        shared.log(),
        vec![Event::Spi(vec![
            Op::Write(vec![0x1E]),
            Op::Write(vec![0x12, 0x34])
        ])]
    );
    let c = shared.chip();
    assert_eq!((c.regs[0x1E], c.regs[0x1F]), (0x12, 0x34));
}

#[test]
fn ireg_write_and_ireg_read_use_the_indirect_file() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(None, None);
    run(imu.ireg_write(&mut delay, 0xA258, 0x5A)).unwrap();
    assert_eq!(
        shared.log(),
        vec![
            Event::Spi(vec![Op::Write(vec![0x7C, 0xA2, 0x58, 0x5A])]),
            us(10)
        ]
    );
    assert_eq!(shared.chip().ireg.get(&0xA258), Some(&0x5A));
    shared.arm(None, None);
    assert_eq!(run(imu.ireg_read(&mut delay, 0xA258)), Ok(0x5A));
    assert_eq!(
        shared.log(),
        vec![
            Event::Spi(vec![Op::Write(vec![0x7C, 0xA2, 0x58])]),
            us(10),
            r(0x7E, 0x5A),
            us(10)
        ]
    );
    assert_eq!(shared.violations(), 0);
}

#[test]
fn raw_access_works_while_not_configured_and_does_not_change_the_state() {
    let (_shared, mut imu, mut delay) = rig();
    assert_eq!(imu.config(), None);
    run(imu.write_register(reg::PWR_MGMT0, 0)).unwrap();
    run(imu.ireg_write(&mut delay, 0xA258, 1)).unwrap();
    assert_eq!(run(imu.read_register(reg::WHO_AM_I)), Ok(0xE9));
    assert_eq!(imu.config(), None);
    let (_shared, mut imu, mut delay) = configured();
    run(imu.write_register(reg::PWR_MGMT0, 0)).unwrap();
    run(imu.ireg_write(&mut delay, 0xA258, 1)).unwrap();
    assert_eq!(imu.config(), Some(&Config::new()));
}

// ---------------------------------------------------------------------------------------------
// Error variants
// ---------------------------------------------------------------------------------------------

#[test]
fn read_before_init_is_not_configured_and_sends_nothing() {
    let (shared, mut imu, _delay) = rig();
    shared.arm(None, None);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
    assert_eq!(shared.transactions(), 0);
    assert_eq!(imu.config(), None);
}

#[test]
fn init_wrong_id_is_reported_and_leaves_the_driver_not_configured() {
    let (shared, mut imu, mut delay) = configured();
    shared.chip().regs[reg::WHO_AM_I as usize] = 0x42;
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::WrongId { found: 0x42 })
    );
    assert_eq!(imu.config(), None);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
    shared.chip().regs[reg::WHO_AM_I as usize] = 0xE9;
    run(imu.init(&mut delay, &Config::new())).unwrap();
    assert!(run(imu.read()).is_ok());
}

#[test]
fn configure_wrong_id_is_reported_and_leaves_the_driver_not_configured() {
    let (shared, mut imu, mut delay) = configured();
    shared.chip().regs[reg::WHO_AM_I as usize] = 0x00;
    assert_eq!(
        run(imu.configure(&mut delay, &Config::new())),
        Err(Error::WrongId { found: 0 })
    );
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
}

#[test]
fn init_reset_timeout_is_reported_after_100_polls() {
    let (shared, mut imu, mut delay) = rig();
    shared.chip().reset_stuck = true;
    shared.arm(None, None);
    assert_eq!(
        run(imu.init(&mut delay, &Config::new())),
        Err(Error::ResetTimeout)
    );
    // Status read, reset write, 100 polls.
    assert_eq!(shared.transactions(), 102);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
}

#[test]
fn init_invalid_config_is_found_before_any_bus_access() {
    let (shared, mut imu, mut delay) = configured();
    let bad = Config::new()
        .with_accel_odr(Odr::Hz6_25)
        .with_gyro_odr(Odr::Hz6_25);
    shared.arm(None, None);
    assert_eq!(
        run(imu.init(&mut delay, &bad)),
        Err(Error::InvalidConfig(ConfigError::AccelOdrLowPowerOnly))
    );
    assert_eq!(shared.log(), vec![]);
    assert_eq!(imu.config(), None);
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
}

#[test]
fn configure_invalid_config_is_found_before_any_bus_access() {
    let (shared, mut imu, mut delay) = configured();
    let bad = Config::new().with_accel_power(SensorPower::Standby);
    shared.arm(None, None);
    assert_eq!(
        run(imu.configure(&mut delay, &bad)),
        Err(Error::InvalidConfig(ConfigError::AccelStandby))
    );
    assert_eq!(shared.log(), vec![]);
    assert_eq!(imu.config(), None);
}

#[test]
fn config_is_none_until_init_completes_then_the_applied_config() {
    let (_shared, mut imu, mut delay) = rig();
    assert_eq!(imu.config(), None);
    let c = clkin();
    run(imu.init(&mut delay, &c)).unwrap();
    assert_eq!(imu.config(), Some(&c));
    run(imu.configure(&mut delay, &Config::new())).unwrap();
    assert_eq!(imu.config(), Some(&Config::new()));
}

// ---------------------------------------------------------------------------------------------
// A3: init leaves the driver not configured; D3: bus errors; A2/D4: cancels
// ---------------------------------------------------------------------------------------------

/// Fails transaction 1, 2, ... of `m` on a configured driver until `m` completes; returns the
/// number of transactions of a complete run. After every failure the driver is usable again.
fn bus_error_sweep(m: Method, config: &Config) -> usize {
    let mut idx = 0;
    loop {
        idx += 1;
        let (shared, mut imu, mut delay) = configured();
        shared.arm(Some(idx), None);
        match run(call(&mut imu, &mut delay, config, m)) {
            Ok(_) => return idx - 1,
            Err(Error::Bus(BusFault)) => {}
            Err(e) => panic!("{e:?} at transaction {idx}"),
        }
        assert_eq!(shared.violations(), 0);
        if !matches!(m, Method::Read) {
            assert_eq!(imu.config(), None, "{m:?} at {idx}");
            shared.arm(None, None);
            assert_eq!(run(imu.read()), Err(Error::NotConfigured));
        }
        shared.arm(None, None);
        assert!(run(call(&mut imu, &mut delay, config, m)).is_ok());
        assert_eq!(run(imu.read()), Ok(expected_read(&shared)));
    }
}

/// Cancels `m` at await 1, 2, ... on a configured driver until `m` completes without pending;
/// returns the number of awaits of a complete run. After every cancel the same call gives the right
/// result and the register files equal those of an uninterrupted run.
fn cancel_sweep(m: Method, config: &Config) -> u32 {
    let reference = {
        let (shared, mut imu, mut delay) = configured();
        run(call(&mut imu, &mut delay, config, m)).unwrap();
        snapshot(&shared)
    };
    let mut k = 0;
    loop {
        k += 1;
        let (shared, mut imu, mut delay) = configured();
        shared.arm(None, Some(k));
        match poll_then_drop(call(&mut imu, &mut delay, config, m)) {
            Polled::Done(out) => {
                assert!(out.is_ok());
                return k - 1;
            }
            Polled::Cancelled => {}
        }
        shared.arm(None, None);
        if !matches!(m, Method::Read) {
            assert_eq!(imu.config(), None, "{m:?} at {k}");
            assert_eq!(run(imu.read()), Err(Error::NotConfigured));
        }
        let again = run(call(&mut imu, &mut delay, config, m)).unwrap();
        if let Out::Sample(s) = again {
            assert_eq!(s, expected_read(&shared));
        }
        assert_eq!(run(imu.read()), Ok(expected_read(&shared)));
        assert_eq!(snapshot(&shared), reference, "{m:?} cancelled at {k}");
    }
}

#[test]
fn init_bus_error_at_every_transaction_leaves_read_not_configured() {
    for config in configs() {
        let n = bus_error_sweep(Method::Init, &config);
        let plan = Plan::after_reset(&config).unwrap();
        assert_eq!(n, 4 + plan_transactions(&plan));
        std::eprintln!("init: {n} transactions");
    }
}

#[test]
fn init_cancel_at_every_await_leaves_read_not_configured() {
    for config in configs() {
        let n = cancel_sweep(Method::Init, &config);
        let plan = Plan::after_reset(&config).unwrap();
        assert_eq!(n, 5 + plan_transactions(&plan) as u32 + plan_delays(&plan));
        std::eprintln!("init: {n} awaits");
    }
}

#[test]
fn init_after_a_failure_or_cancel_completes_and_read_returns_the_registers() {
    let (shared, mut imu, mut delay) = rig();
    shared.arm(Some(3), None);
    assert!(run(imu.init(&mut delay, &Config::new())).is_err());
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
    shared.arm(None, Some(2));
    assert!(matches!(
        poll_then_drop(imu.init(&mut delay, &Config::new())),
        Polled::Cancelled
    ));
    assert_eq!(run(imu.read()), Err(Error::NotConfigured));
    shared.arm(None, None);
    run(imu.init(&mut delay, &Config::new())).unwrap();
    assert_eq!(run(imu.read()), Ok(expected_read(&shared)));
}

#[test]
fn configure_bus_error_at_every_transaction_returns_err_and_the_next_call_works() {
    for config in configs() {
        let n = bus_error_sweep(Method::Configure, &config);
        let plan = Plan::reconfigure(&config).unwrap();
        assert_eq!(n, 1 + plan_transactions(&plan) + 12);
        std::eprintln!("configure: {n} transactions");
    }
}

#[test]
fn configure_cancel_at_every_await_then_the_same_call_gives_the_right_result() {
    for config in configs() {
        let n = cancel_sweep(Method::Configure, &config);
        let plan = Plan::reconfigure(&config).unwrap();
        assert_eq!(
            n,
            1 + plan_transactions(&plan) as u32 + plan_delays(&plan) + 24
        );
        std::eprintln!("configure: {n} awaits");
    }
}

#[test]
fn read_bus_error_at_every_transaction_returns_err_and_the_next_call_works() {
    let n = bus_error_sweep(Method::Read, &Config::new());
    assert_eq!(n, 1);
}

#[test]
fn read_cancel_at_every_await_then_the_same_call_gives_the_right_result() {
    let n = cancel_sweep(Method::Read, &Config::new());
    assert_eq!(n, 1);
}

// ---------------------------------------------------------------------------------------------
// A11: the blocking front-end
// ---------------------------------------------------------------------------------------------

#[test]
fn blocking_init_configure_read_match_the_async_driver() {
    for config in configs() {
        let (a_shared, mut a_imu, mut a_delay) = rig();
        let b_shared = Shared::new();
        let mut b_imu = blocking::Icm45686::new(b_shared.bus());
        let mut b_delay = b_shared.delay();

        let a = run(a_imu.init(&mut a_delay, &config));
        let b = b_imu.init(&mut b_delay, &config);
        assert_eq!(a, b);
        assert!(a.is_ok());
        assert_eq!(a_shared.log(), b_shared.log());
        assert_eq!(a_imu.config(), b_imu.config());

        a_shared.arm(None, None);
        b_shared.arm(None, None);
        let a = run(a_imu.configure(&mut a_delay, &Config::new()));
        let b = b_imu.configure(&mut b_delay, &Config::new());
        assert_eq!(a, b);
        assert_eq!(a_shared.log(), b_shared.log());

        a_shared.arm(None, None);
        b_shared.arm(None, None);
        let a = run(a_imu.read());
        let b = b_imu.read();
        assert_eq!(a, b);
        assert_eq!(a, Ok(expected_read(&a_shared)));
        assert_eq!(a_shared.log(), b_shared.log());
        assert_eq!(snapshot(&a_shared), snapshot(&b_shared));
    }
}

#[test]
fn blocking_errors_match_the_async_driver() {
    for idx in 1..=6 {
        let (a_shared, mut a_imu, mut a_delay) = rig();
        let b_shared = Shared::new();
        let mut b_imu = blocking::Icm45686::new(b_shared.bus());
        let mut b_delay = b_shared.delay();
        a_shared.arm(Some(idx), None);
        b_shared.arm(Some(idx), None);
        let a = run(a_imu.init(&mut a_delay, &Config::new()));
        let b = b_imu.init(&mut b_delay, &Config::new());
        assert_eq!(a, b);
        assert_eq!(a, Err(Error::Bus(BusFault)));
        assert_eq!(a_shared.log(), b_shared.log());
        assert_eq!(a_imu.config(), None);
        assert_eq!(b_imu.config(), None);
        assert_eq!(run(a_imu.read()), b_imu.read());
    }
}

// ---------------------------------------------------------------------------------------------
// A5: Send
// ---------------------------------------------------------------------------------------------

fn assert_send<T: Send>(_: &T) {}

#[test]
fn driver_and_method_futures_are_send_when_bus_and_delay_are() {
    let (_shared, mut imu, mut delay) = rig();
    assert_send(&imu);
    let config = Config::new();
    assert_send(&imu.init(&mut delay, &config));
    assert_send(&imu.configure(&mut delay, &config));
    assert_send(&imu.read());
    assert_send(&imu.read_register(0));
    assert_send(&imu.ireg_read(&mut delay, 0));
    assert_send(&imu.set_offsets(&mut delay, &crate::Offsets::new()));
    assert_send(&imu.self_test(&mut delay, &crate::SelfTestConfig::new()));
    assert_send(&imu.set_power(&mut delay, SensorPower::Off, SensorPower::Off));
    assert_send(&imu.soft_reset(&mut delay));
}
