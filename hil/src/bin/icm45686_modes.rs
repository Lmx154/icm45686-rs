//! On-board check of the icm45686 FIFO and rate options no other program touches (MARV V2 Rev 1, U21,
//! SPI0, INT1 on GPIO15, external 32 kHz clock): bypass, stop-on-full against stream, decimation,
//! timestamp resolution, different rates per sensor, the 8-byte single-sensor frames. No SPI transfer
//! is ever wrapped in a timeout or cancelled, and no raw register is written. Where the device
//! disagrees with the driver the check keeps failing and the numbers are printed.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_15, PWM_SLICE7};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use icm45686::fifo::{timestamp_tick_us, timestamp_us};
use icm45686::{
    AccelFullScale, ClockSource, Config, ConfigError, Decimation, Error, FifoConfig, FifoDepth,
    FifoFormat, FifoFrame, FifoMode, GyroFullScale, Icm45686, Int1Config, Odr, RawSample,
    TimestampResolution, fifo_frames,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const G: f32 = 9.80665;
const RAD_TO_DEG: f32 = 57.295_78;
const PERIOD_800_US: u32 = 1250;

static VERIFIES: AtomicU32 = AtomicU32::new(0);

/// INT1 edge counter: a PWM slice counts every rising edge in hardware (same as icm45686_fifo).
struct DataReady<'d> {
    _input: Input<'d>,
    pwm: Pwm<'d>,
    last: u16,
    wraps: u64,
}

impl<'d> DataReady<'d> {
    fn new(slice: Peri<'d, PWM_SLICE7>, pin: Peri<'d, PIN_15>) -> Self {
        // SAFETY: the Input only arms GPIO edge interrupts and the PWM slice only counts; neither
        // reconfigures the pad after construction.
        let input = Input::new(unsafe { pin.clone_unchecked() }, Pull::None);
        let pwm = Pwm::new_input(
            slice,
            pin,
            Pull::None,
            InputMode::RisingEdge,
            pwm::Config::default(),
        );
        let last = pwm.counter();
        Self {
            _input: input,
            pwm,
            last,
            wraps: 0,
        }
    }

    fn count(&mut self) -> u64 {
        let c = self.pwm.counter();
        if c < self.last {
            self.wraps += 1;
        }
        self.last = c;
        (self.wraps << 16) | u64::from(c)
    }
}

fn verdict(failures: &mut u32, ok: bool, name: &str) {
    if ok {
        defmt::info!("[PASS] {=str}", name);
    } else {
        *failures += 1;
        defmt::error!("[FAIL] {=str}", name);
    }
}

fn bus_fail<E: core::fmt::Debug>(failures: &mut u32, what: &str, e: &E) {
    *failures += 1;
    defmt::error!("[FAIL] {=str}: {}", what, defmt::Debug2Format(e));
}

fn base() -> Config {
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_gyro_fs(GyroFullScale::Dps250)
        .with_clock(ClockSource::ExternalClkin)
}

fn fifo_base(mode: FifoMode) -> FifoConfig {
    FifoConfig::new()
        .with_mode(mode)
        .with_format(FifoFormat::Standard)
        .with_accel(true)
        .with_gyro(true)
        .with_timestamp(true)
        .with_depth(FifoDepth::Kib2)
}

fn g2(a: [f32; 3]) -> f32 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (G * G)
}

fn plausible(a2: f32) -> bool {
    a2 > 0.81 && a2 < 1.21
}

/// Applies `config` (init when `reset`, else configure) and requires `verify()` Ok afterwards.
async fn apply_verify<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    name: &str,
    config: &Config,
    reset: bool,
) -> bool {
    let r = if reset {
        imu.init(&mut Delay, config).await
    } else {
        imu.configure(&mut Delay, config).await
    };
    if let Err(e) = r {
        defmt::error!(
            "[STAT] configure refused ({=str}): {}",
            name,
            defmt::Debug2Format(&e)
        );
        bus_fail(failures, name, &e);
        return false;
    }
    match imu.verify(&mut Delay).await {
        Ok(()) => {
            VERIFIES.fetch_add(1, Ordering::Relaxed);
            true
        }
        Err(Error::ConfigMismatch(m)) => {
            defmt::error!(
                "[STAT] verify mismatch ({=str}): space={} register={=u16:#x} mask={=u8:#x} expected={=u8:#x} found={=u8:#x}",
                name,
                defmt::Debug2Format(&m.space),
                m.register,
                m.mask,
                m.expected,
                m.found
            );
            verdict(failures, false, "verify Ok after configure");
            false
        }
        Err(e) => {
            bus_fail(failures, name, &e);
            false
        }
    }
}

/// init to the default configuration, verify(), `n` samples with gravity magnitude within 10 %.
async fn health<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32, label: &str, n: u32) {
    let c = Config::new();
    if !apply_verify(imu, failures, label, &c, true).await {
        return;
    }
    let (mut ok, mut bad, mut lo, mut hi) = (0u32, 0u32, f32::MAX, 0.0f32);
    for _ in 0..n {
        Timer::after_millis(2).await;
        match imu.read().await {
            Ok(s) => {
                let m = g2(s.accel_ms2(c.accel_fs));
                lo = lo.min(m);
                hi = hi.max(m);
                if plausible(m) {
                    ok += 1;
                } else {
                    bad += 1;
                }
            }
            Err(e) => {
                bad += 1;
                defmt::error!("[STAT] health read error: {}", defmt::Debug2Format(&e));
            }
        }
    }
    defmt::info!(
        "[STAT] health ({=str}): plausible={} bad={} accel_g2 min={} max={}",
        label,
        ok,
        bad,
        lo,
        hi
    );
    verdict(
        failures,
        ok == n && bad == 0,
        "sensor healthy: init default, verify Ok, samples 1 g",
    );
}

/// Decoded frames of one or more FIFO reads.
struct Acc {
    period_us: u32,
    res: TimestampResolution,
    n: u32,
    n_accel: u32,
    n_gyro: u32,
    n_both: u32,
    other: u32,
    invalid_a: u32,
    invalid_g: u32,
    dn: u32,
    dmin: u32,
    dmax: u32,
    dsum_us: u64,
    dsum_ticks: u64,
    bad: u32,
    first_bad: i32,
    first_ts: Option<u16>,
    third_ts: Option<u16>,
    last_ts: Option<u16>,
    a2_min: f32,
    a2_max: f32,
    gmax_dps: f32,
    rep_a: u32,
    rep_g: u32,
    prev_a: Option<[i16; 3]>,
    prev_g: Option<[i16; 3]>,
    flag_changes: u32,
    hdr: [u8; 16],
    hdr_n: usize,
    overflows: u32,
}

impl Acc {
    fn new(period_us: u32, res: TimestampResolution) -> Self {
        Self {
            period_us,
            res,
            n: 0,
            n_accel: 0,
            n_gyro: 0,
            n_both: 0,
            other: 0,
            invalid_a: 0,
            invalid_g: 0,
            dn: 0,
            dmin: u32::MAX,
            dmax: 0,
            dsum_us: 0,
            dsum_ticks: 0,
            bad: 0,
            first_bad: -1,
            first_ts: None,
            third_ts: None,
            last_ts: None,
            a2_min: f32::MAX,
            a2_max: 0.0,
            gmax_dps: 0.0,
            rep_a: 0,
            rep_g: 0,
            prev_a: None,
            prev_g: None,
            flag_changes: 0,
            hdr: [0; 16],
            hdr_n: 0,
            overflows: 0,
        }
    }

    fn feed(&mut self, buf: &[u8], config: &Config) {
        let size = usize::from(config.fifo.frame_bytes());
        for (i, fr) in fifo_frames(buf, &config.fifo).enumerate() {
            if self.hdr_n < 16 {
                if let Some(&h) = buf.get(i * size) {
                    if let Some(slot) = self.hdr.get_mut(self.hdr_n) {
                        *slot = h;
                    }
                    self.hdr_n += 1;
                }
            }
            let (ts, a, g, flags) = match fr {
                FifoFrame::AccelGyro(f) => {
                    self.n_both += 1;
                    if f.accel_invalid() {
                        self.invalid_a += 1;
                    }
                    if f.gyro_invalid() {
                        self.invalid_g += 1;
                    }
                    (
                        config.fifo.timestamp.then_some(f.timestamp),
                        Some(f.accel),
                        Some(f.gyro),
                        f.flags,
                    )
                }
                FifoFrame::Accel(f) => {
                    self.n_accel += 1;
                    if f.accel_invalid() {
                        self.invalid_a += 1;
                    }
                    (None, Some(f.accel), None, f.flags)
                }
                FifoFrame::Gyro(f) => {
                    self.n_gyro += 1;
                    if f.gyro_invalid() {
                        self.invalid_g += 1;
                    }
                    (None, None, Some(f.gyro), f.flags)
                }
                _ => {
                    self.n += 1;
                    self.other += 1;
                    continue;
                }
            };
            self.n += 1;
            if self.n > 1 && (flags.accel_odr_changed || flags.gyro_odr_changed) {
                self.flag_changes += 1;
            }
            if let Some(t) = ts {
                if self.first_ts.is_none() {
                    self.first_ts = Some(t);
                }
                if self.n == 3 {
                    self.third_ts = Some(t);
                }
                if let Some(p) = self.last_ts {
                    let ticks = t.wrapping_sub(p);
                    let us = timestamp_us(ticks, self.res);
                    self.dn += 1;
                    self.dmin = self.dmin.min(us);
                    self.dmax = self.dmax.max(us);
                    self.dsum_us += u64::from(us);
                    self.dsum_ticks += u64::from(ticks);
                    if us * 100 < self.period_us * 95 || us * 100 > self.period_us * 105 {
                        self.bad += 1;
                        if self.first_bad < 0 {
                            self.first_bad = self.n as i32 - 1;
                        }
                    }
                }
                self.last_ts = Some(t);
            }
            if let Some(a) = a {
                if self.prev_a == Some(a) {
                    self.rep_a += 1;
                }
                self.prev_a = Some(a);
                if !(a == [i16::MIN; 3]) {
                    let s = RawSample {
                        accel: a,
                        gyro: [0; 3],
                        temp: 0,
                    };
                    let m = g2(s.accel_ms2(config.accel_fs));
                    self.a2_min = self.a2_min.min(m);
                    self.a2_max = self.a2_max.max(m);
                }
            }
            if let Some(g) = g {
                if self.prev_g == Some(g) {
                    self.rep_g += 1;
                }
                self.prev_g = Some(g);
                if !(g == [i16::MIN; 3]) {
                    let s = RawSample {
                        accel: [0; 3],
                        gyro: g,
                        temp: 0,
                    };
                    for v in s.gyro_rads(config.gyro_fs) {
                        self.gmax_dps = self.gmax_dps.max(v.abs() * RAD_TO_DEG);
                    }
                }
            }
        }
    }

    fn mean_us(&self) -> f32 {
        self.dsum_us as f32 / self.dn.max(1) as f32
    }

    fn mean_ticks(&self) -> f32 {
        self.dsum_ticks as f32 / self.dn.max(1) as f32
    }

    fn print(&self, name: &str) {
        defmt::info!(
            "[STAT] {=str}: frames={} both={} accel_only={} gyro_only={} other={} invalid_a={} invalid_g={} deltas={} delta_us min={} max={} mean={} mean_ticks={} bad_deltas={} first_bad={} rep_a={} rep_g={} odr_flag_frames={} accel_g2 min={} max={} gyro_max_dps={}",
            name,
            self.n,
            self.n_both,
            self.n_accel,
            self.n_gyro,
            self.other,
            self.invalid_a,
            self.invalid_g,
            self.dn,
            if self.dn == 0 { 0 } else { self.dmin },
            self.dmax,
            self.mean_us(),
            self.mean_ticks(),
            self.bad,
            self.first_bad,
            self.rep_a,
            self.rep_g,
            self.flag_changes,
            self.a2_min,
            self.a2_max,
            self.gmax_dps
        );
    }
}

/// Flushes, then reads the FIFO every `ms` for `iters` rounds. Returns the frames that entered the FIFO
/// (read plus the final count), the INT1 edges and the microseconds of the window.
async fn drain_run<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    dr: &mut DataReady<'_>,
    buf: &mut [u8],
    config: &Config,
    acc: &mut Acc,
    iters: u32,
    ms: u64,
) -> Option<(u32, u64, u64)> {
    if let Err(e) = imu.fifo_flush(&mut Delay).await {
        bus_fail(failures, "fifo_flush", &e);
        return None;
    }
    let e0 = dr.count();
    let t0 = Instant::now();
    let mut total = 0u32;
    for _ in 0..iters {
        Timer::after_millis(ms).await;
        match imu.fifo_read(buf).await {
            Ok(r) => {
                acc.feed(buf.get(..r.bytes).unwrap_or(&[]), config);
                total += r.frames as u32;
                if r.overflow == Some(true) {
                    acc.overflows += 1;
                }
            }
            Err(e) => {
                bus_fail(failures, "fifo_read", &e);
                return None;
            }
        }
    }
    let cnt = match imu.fifo_count().await {
        Ok(c) => c,
        Err(e) => {
            bus_fail(failures, "fifo_count", &e);
            return None;
        }
    };
    let e1 = dr.count();
    let el = t0.elapsed().as_micros();
    Some((total + u32::from(cnt), e1 - e0, el))
}

/// Polls `read()` for `ms` and reports the INT1 edge rate and how often each sensor's data changes.
/// Returns (edges/s, accel changes/s, gyro changes/s, polls, accel g2 min, accel g2 max, ok).
async fn poll_rates<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    dr: &mut DataReady<'_>,
    config: &Config,
    ms: u64,
) -> (f32, f32, f32, u32, f32, f32, bool) {
    let e0 = dr.count();
    let t0 = Instant::now();
    let (mut ca, mut cg, mut polls) = (0u32, 0u32, 0u32);
    let (mut pa, mut pg): (Option<[i16; 3]>, Option<[i16; 3]>) = (None, None);
    let (mut lo, mut hi) = (f32::MAX, 0.0f32);
    let mut ok = true;
    while t0.elapsed() < Duration::from_millis(ms) {
        match imu.read().await {
            Ok(s) => {
                polls += 1;
                if pa.is_some_and(|p| p != s.accel) {
                    ca += 1;
                }
                if pg.is_some_and(|p| p != s.gyro) {
                    cg += 1;
                }
                pa = Some(s.accel);
                pg = Some(s.gyro);
                let m = g2(s.accel_ms2(config.accel_fs));
                lo = lo.min(m);
                hi = hi.max(m);
            }
            Err(e) => {
                bus_fail(failures, "read()", &e);
                ok = false;
                break;
            }
        }
    }
    let el = t0.elapsed().as_micros() as f32;
    let edges = dr.count() - e0;
    (
        edges as f32 * 1.0e6 / el,
        ca as f32 * 1.0e6 / el,
        cg as f32 * 1.0e6 / el,
        polls,
        lo,
        hi,
        ok,
    )
}

// ---------------------------------------------------------------------------------------------
// Check 1: bypass
// ---------------------------------------------------------------------------------------------

async fn check1<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    dr: &mut DataReady<'_>,
    buf: &mut [u8],
) {
    defmt::info!("[INFO] check 1: FIFO bypass, data-ready 200 Hz");
    let c = base()
        .with_accel_odr(Odr::Hz200)
        .with_gyro_odr(Odr::Hz200)
        .with_int1(Int1Config::new().with_data_ready(true));
    if !apply_verify(imu, failures, "check 1 configure bypass", &c, false).await {
        return;
    }
    Timer::after_millis(100).await;
    let e0 = dr.count();
    let t0 = Instant::now();
    let count = imu.fifo_count().await;
    let read = imu.fifo_read(buf).await;
    let exact = imu.fifo_read_exact(buf, 1).await;
    let flush = imu.fifo_flush(&mut Delay).await;
    defmt::info!(
        "[STAT] bypass: driver FIFO methods: fifo_count={} fifo_read={} fifo_read_exact={} fifo_flush={}",
        defmt::Debug2Format(&count),
        defmt::Debug2Format(&read),
        defmt::Debug2Format(&exact),
        defmt::Debug2Format(&flush)
    );
    verdict(
        failures,
        matches!(count, Err(Error::FifoOff))
            && matches!(read, Err(Error::FifoOff))
            && matches!(exact, Err(Error::FifoOff))
            && matches!(flush, Err(Error::FifoOff)),
        "bypass: every driver FIFO method returns Error::FifoOff",
    );
    let mut raw = [0u8; 2];
    match imu
        .read_registers(icm45686::reg::FIFO_COUNT_0, &mut raw)
        .await
    {
        Ok(()) => {
            let n = u16::from_le_bytes(raw);
            defmt::info!("[STAT] bypass: raw FIFO_COUNT register (read) = {}", n);
            verdict(failures, n == 0, "bypass: raw FIFO_COUNT register is 0");
        }
        Err(e) => bus_fail(failures, "raw FIFO_COUNT read", &e),
    }
    Timer::after_millis(500).await;
    let edges = dr.count() - e0;
    let el = t0.elapsed().as_micros();
    let rate = edges as f32 * 1.0e6 / el as f32;
    defmt::info!(
        "[STAT] bypass: INT1 edges={} in {} us = {} Hz (nominal 200)",
        edges,
        el,
        rate
    );
    verdict(
        failures,
        rate > 194.0 && rate < 206.0,
        "bypass: INT1 data-ready edges keep coming at 200 Hz within 3 %",
    );
    match imu.read().await {
        Ok(s) => {
            let m = g2(s.accel_ms2(c.accel_fs));
            defmt::info!("[STAT] bypass: read() accel_g2={}", m);
            verdict(
                failures,
                plausible(m),
                "bypass: read() gives 1 g within 10 %",
            );
        }
        Err(e) => bus_fail(failures, "bypass read()", &e),
    }
}

// ---------------------------------------------------------------------------------------------
// Check 2: stop-on-full against stream
// ---------------------------------------------------------------------------------------------

async fn check2<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    buf: &mut [u8],
    mode: FifoMode,
    label: &str,
    tick_ratio: f32,
) {
    defmt::info!(
        "[INFO] check 2: {=str}, 2 KB, 800 Hz, no read until the FIFO has overflowed twice over",
        label
    );
    const RES: TimestampResolution = TimestampResolution::Us16;
    let c = base()
        .with_int1(Int1Config::new().with_fifo_full(true))
        .with_fifo(
            fifo_base(mode)
                .with_timestamp_resolution(RES)
                .with_watermark(0),
        );
    if !apply_verify(imu, failures, label, &c, false).await {
        return;
    }
    let capacity = c.fifo.capacity_frames();
    let wait_ms = 2 * u64::from(capacity) * u64::from(PERIOD_800_US) / 1000 + 30;
    if let Err(e) = imu.fifo_flush(&mut Delay).await {
        return bus_fail(failures, "fifo_flush", &e);
    }
    let _ = imu.interrupt_status().await;
    let s0 = imu.stats().fifo_overflows;
    // A marker: an early read (all frames but one) gives the device timestamp of a known host moment.
    Timer::after_millis(15).await;
    let mut marker = Acc::new(PERIOD_800_US, RES);
    match imu.fifo_read(buf).await {
        Ok(r) => marker.feed(buf.get(..r.bytes).unwrap_or(&[]), &c),
        Err(e) => return bus_fail(failures, "marker fifo_read", &e),
    }
    let tm = Instant::now();
    let Some(tm_ts) = marker.last_ts else {
        return verdict(failures, false, "marker frames decode");
    };
    Timer::after_millis(wait_ms).await;
    let st = match imu.interrupt_status().await {
        Ok(s) => s,
        Err(e) => return bus_fail(failures, "interrupt_status", &e),
    };
    let sa = imu.stats().fifo_overflows;
    let t_read = Instant::now();
    let r = match imu.fifo_read(buf).await {
        Ok(r) => r,
        Err(e) => return bus_fail(failures, "fifo_read", &e),
    };
    let s1 = imu.stats().fifo_overflows;
    let mut acc = Acc::new(PERIOD_800_US, RES);
    acc.feed(buf.get(..r.bytes).unwrap_or(&[]), &c);
    let e_read = (t_read - tm).as_micros() as i64;
    let rel = |ts: u16| i64::from(timestamp_us(ts.wrapping_sub(tm_ts), RES));
    let (rf, rl) = match (acc.first_ts, acc.last_ts) {
        (Some(f), Some(l)) => (rel(f), rel(l)),
        _ => (-1, -1),
    };
    let r3 = acc.third_ts.map_or(-1, rel);
    let total = r.frames + usize::from(r.left.unwrap_or(0));
    defmt::info!(
        "[STAT] {=str}: waited {} ms, read at {} us after the marker; frames_read={} left={:?} count_total={} capacity={} (driver doc: stream saturates at capacity+2={}) overflow={:?}",
        label,
        wait_ms,
        e_read,
        r.frames,
        r.left,
        total,
        capacity,
        capacity + 2,
        r.overflow
    );
    defmt::info!(
        "[STAT] {=str}: interrupt_status status0={=u8:#x} fifo_full={}; stats overflows before={} after_status={} after_read={}",
        label,
        st.status0,
        st.fifo_full,
        s0,
        sa,
        s1
    );
    defmt::info!(
        "[STAT] {=str}: first_ts={:?} third_ts={:?} last_ts={:?} (marker ts {}) first_rel_us={} third_rel_us={} last_rel_us={} span_us={} read_minus_last_us={} bad_deltas={} first_bad={}",
        label,
        acc.first_ts,
        acc.third_ts,
        acc.last_ts,
        tm_ts,
        rf,
        r3,
        rl,
        rl - rf,
        e_read - rl,
        acc.bad,
        acc.first_bad
    );
    verdict(
        failures,
        st.fifo_full,
        "full condition reported by interrupt_status",
    );
    verdict(
        failures,
        r.overflow == Some(true),
        "full condition reported by FifoRead.overflow",
    );
    verdict(failures, s1 > s0, "stats() overflow counter went up");
    verdict(
        failures,
        total >= usize::from(capacity) && acc.other == 0,
        "FIFO held at least its documented capacity and every frame decodes",
    );
    let period = i64::from(PERIOD_800_US);
    if mode == FifoMode::Stream {
        verdict(
            failures,
            total <= usize::from(capacity) + 2,
            "stream: the count reads at most capacity + 2 (driver doc)",
        );
        // The timestamps are in the uncalibrated tick (check 4 measured it against the MCU clock),
        // so the span since the marker is converted with that ratio before the comparison.
        let rl_real = (rl as f32 / tick_ratio) as i64;
        defmt::info!(
            "[STAT] {=str}: tick ratio {} from check 4; last frame {} us after the marker corrected (read at {} us): difference {} us, bound {} us",
            label,
            tick_ratio,
            rl_real,
            e_read,
            e_read - rl_real,
            3 * period
        );
        verdict(
            failures,
            (e_read - rl_real).abs() <= 3 * period,
            "stream: the last frame read is within 3 frame periods of the read (newest kept)",
        );
        verdict(
            failures,
            r3 - rf > e_read / 2,
            "stream: the first two frames are older than the rest by more than half the unread time (stale, driver doc)",
        );
        verdict(
            failures,
            acc.first_bad == 2 && acc.bad == 1,
            "stream: one timestamp jump after the second frame, contiguous from the third frame on (driver doc)",
        );
        verdict(
            failures,
            r3 > 100_000,
            "stream: from the third frame on the oldest were dropped (newest kept)",
        );
    } else {
        verdict(
            failures,
            rf >= 0 && rf <= 5000,
            "stop-on-full: the first frame read is from the start (oldest kept)",
        );
        verdict(
            failures,
            rl <= (i64::from(capacity) + 6) * period && e_read - rl > 100_000,
            "stop-on-full: no frame is newer than the moment the FIFO filled",
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Check 3: decimation
// ---------------------------------------------------------------------------------------------

async fn check3<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    dr: &mut DataReady<'_>,
    buf: &mut [u8],
) {
    defmt::info!("[INFO] check 3: decimation, 800 Hz, 1.5 s windows");
    for (dec, factor, name) in [
        (Decimation::Div1, 1u32, "decimation 1 (baseline)"),
        (Decimation::Div2, 2, "decimation 2"),
        (Decimation::Div8, 8, "decimation 8"),
    ] {
        let c = base()
            .with_int1(Int1Config::new().with_data_ready(true))
            .with_fifo(
                fifo_base(FifoMode::Stream)
                    .with_timestamp_resolution(TimestampResolution::Us1)
                    .with_accel_decimation(dec)
                    .with_gyro_decimation(dec),
            );
        if !apply_verify(imu, failures, name, &c, false).await {
            continue;
        }
        let mut acc = Acc::new(PERIOD_800_US * factor, TimestampResolution::Us1);
        let Some((frames, edges, el)) =
            drain_run(imu, failures, dr, buf, &c, &mut acc, 15, 100).await
        else {
            continue;
        };
        let ratio = frames as f32 / edges.max(1) as f32;
        defmt::info!(
            "[STAT] {=str}: frames={} INT1 edges={} window_us={} ratio={} expected={} overflows={}",
            name,
            frames,
            edges,
            el,
            ratio,
            1.0 / factor as f32,
            acc.overflows
        );
        acc.print(name);
        // The time base is the MCU clock (the 800 Hz output rate is exact on the external clock).
        // Decimation lowers the FIFO frame rate and the INT1 data-ready rate alike (measured).
        let wall_ratio = frames as f32 / (800.0 * el as f32 / 1.0e6);
        let int1_ratio = edges as f32 / (800.0 * el as f32 / 1.0e6);
        defmt::info!(
            "[STAT] {=str}: over the MCU clock at 800 Hz: frame ratio={} INT1 edge ratio={} expected={}; frames over INT1 edges: ratio={}",
            name,
            wall_ratio,
            int1_ratio,
            1.0 / factor as f32,
            ratio
        );
        verdict(
            failures,
            (wall_ratio * factor as f32 - 1.0).abs() <= 0.02,
            "FIFO frame rate is the rate divided by the factor within 2 % (MCU clock)",
        );
        verdict(
            failures,
            (int1_ratio * factor as f32 - 1.0).abs() <= 0.02,
            "INT1 data-ready rate is the rate divided by the factor within 2 % (MCU clock)",
        );
        let (er, ar, gr, polls, lo, hi, ok) = poll_rates(imu, failures, dr, &c, 500).await;
        defmt::info!(
            "[STAT] {=str} polled for 500 ms ({} reads): INT1 edges/s={} accel changes/s={} gyro changes/s={} accel_g2 min={} max={}",
            name,
            polls,
            er,
            ar,
            gr,
            lo,
            hi
        );
        verdict(failures, ok, "poll after decimation run");
    }
}

/// Decimation set with the FIFO off: the driver does not write it then, so the rates do not change.
async fn check3_fifo_off<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    dr: &mut DataReady<'_>,
) {
    defmt::info!("[INFO] check 3b: decimation 8 set with the FIFO off, 800 Hz");
    let name = "decimation 8 with the FIFO off";
    let c = base()
        .with_int1(Int1Config::new().with_data_ready(true))
        .with_fifo(
            FifoConfig::new()
                .with_accel_decimation(Decimation::Div8)
                .with_gyro_decimation(Decimation::Div8),
        );
    verdict(
        failures,
        c.validate().is_ok(),
        "validate accepts decimation with the FIFO off",
    );
    if !apply_verify(imu, failures, name, &c, false).await {
        return;
    }
    Timer::after_millis(100).await;
    let (er, ar, gr, polls, lo, hi, ok) = poll_rates(imu, failures, dr, &c, 1000).await;
    defmt::info!(
        "[STAT] {=str} polled for 1000 ms ({} reads): INT1 edges/s={} accel changes/s={} gyro changes/s={} accel_g2 min={} max={}",
        name,
        polls,
        er,
        ar,
        gr,
        lo,
        hi
    );
    let near = |r: f32| (r / 800.0 - 1.0).abs() <= 0.02;
    verdict(
        failures,
        ok && near(er) && near(ar) && near(gr),
        "decimation with the FIFO off: INT1 and both registers stay at the output rate within 2 %",
    );
}

// ---------------------------------------------------------------------------------------------
// Check 4: timestamp resolution
// ---------------------------------------------------------------------------------------------

async fn check4<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    dr: &mut DataReady<'_>,
    buf: &mut [u8],
) -> f32 {
    defmt::info!("[INFO] check 4: timestamp resolution, 200 Hz, 3 s windows");
    let mut tick_ratio = 1.0f32;
    for (res, name) in [
        (TimestampResolution::Us1, "timestamp 1 us per tick"),
        (TimestampResolution::Us16, "timestamp 16 us per tick"),
    ] {
        let c = base()
            .with_accel_odr(Odr::Hz200)
            .with_gyro_odr(Odr::Hz200)
            .with_int1(Int1Config::new().with_data_ready(true))
            .with_fifo(fifo_base(FifoMode::Stream).with_timestamp_resolution(res));
        if !apply_verify(imu, failures, name, &c, false).await {
            continue;
        }
        let mut acc = Acc::new(5000, res);
        let Some((frames, edges, el)) =
            drain_run(imu, failures, dr, buf, &c, &mut acc, 15, 200).await
        else {
            continue;
        };
        defmt::info!(
            "[STAT] {=str}: driver tick={} us; frames={} INT1 edges={} window_us={} wall us per frame={} ticks per frame={}",
            name,
            timestamp_tick_us(res),
            frames,
            edges,
            el,
            el as f32 / edges.max(1) as f32,
            acc.mean_ticks()
        );
        acc.print(name);
        let m = acc.mean_us();
        verdict(failures, acc.dn >= 500 && (m - 5000.0).abs() <= 250.0, name);
        // Documented: the tick does not follow the external clock (about 0.44 % fast, measured), so
        // timestamps space frames relative to each other; real time needs a calibration.
        let wall_per_frame = el as f32 / edges.max(1) as f32;
        let ratio = m / wall_per_frame;
        defmt::info!(
            "[STAT] {=str}: timestamp period {} us against MCU clock period {} us: ratio={} deviation {} %",
            name,
            m,
            wall_per_frame,
            ratio,
            (ratio - 1.0) * 100.0
        );
        verdict(
            failures,
            (ratio - 1.0).abs() <= 0.01,
            "mean frame spacing in timestamp units is within 1 % of the spacing on the MCU clock",
        );
        tick_ratio = ratio;
    }
    tick_ratio
}

// ---------------------------------------------------------------------------------------------
// Check 5: different rates for the two sensors
// ---------------------------------------------------------------------------------------------

async fn check5<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    dr: &mut DataReady<'_>,
    buf: &mut [u8],
) {
    defmt::info!("[INFO] check 5: different rates for accel and gyro");
    let mismatch = base()
        .with_accel_odr(Odr::Hz800)
        .with_gyro_odr(Odr::Hz200)
        .with_fifo(fifo_base(FifoMode::Stream));
    verdict(
        failures,
        mismatch.validate() == Err(ConfigError::FifoRateMismatch),
        "validate refuses accel 800 Hz + gyro 200 Hz in one FIFO frame without decimation",
    );
    for (name, ao, go, da, dg) in [
        (
            "accel 800 / gyro 200",
            Odr::Hz800,
            Odr::Hz200,
            Decimation::Div4,
            Decimation::Div1,
        ),
        (
            "accel 200 / gyro 800",
            Odr::Hz200,
            Odr::Hz800,
            Decimation::Div1,
            Decimation::Div4,
        ),
    ] {
        // Part A: data registers and INT1, FIFO off.
        let c = base()
            .with_accel_odr(ao)
            .with_gyro_odr(go)
            .with_int1(Int1Config::new().with_data_ready(true));
        if !apply_verify(imu, failures, name, &c, false).await {
            continue;
        }
        Timer::after_millis(100).await;
        let (er, ar, gr, polls, a2min, a2max, ok) = poll_rates(imu, failures, dr, &c, 1000).await;
        defmt::info!(
            "[STAT] {=str} FIFO off: INT1 edges/s={} (accel nominal {}, gyro nominal {}); polled read() {} times: accel changes/s={} gyro changes/s={}; accel_g2 min={} max={}",
            name,
            er,
            ao.hz(),
            go.hz(),
            polls,
            ar,
            gr,
            a2min,
            a2max
        );
        verdict(
            failures,
            ok && er > 0.0 && ar > 0.0 && gr > 0.0 && plausible(a2min) && plausible(a2max),
            "mixed rates, FIFO off: INT1 edges come, both sensors update, data plausible",
        );

        // Part B: FIFO, equal rates after decimation.
        let cf = base()
            .with_accel_odr(ao)
            .with_gyro_odr(go)
            .with_int1(Int1Config::new().with_data_ready(true))
            .with_fifo(
                fifo_base(FifoMode::Stream)
                    .with_timestamp_resolution(TimestampResolution::Us1)
                    .with_accel_decimation(da)
                    .with_gyro_decimation(dg),
            );
        if !apply_verify(imu, failures, name, &cf, false).await {
            continue;
        }
        let mut acc = Acc::new(5000, TimestampResolution::Us1);
        let Some((frames, edges, el)) =
            drain_run(imu, failures, dr, buf, &cf, &mut acc, 10, 100).await
        else {
            continue;
        };
        defmt::info!(
            "[STAT] {=str} FIFO (decimation accel {} gyro {}): frames={} INT1 edges={} window_us={}; header bytes of the first {} frames = {=[u8]:x}",
            name,
            da.code(),
            dg.code(),
            frames,
            edges,
            el,
            acc.hdr_n,
            acc.hdr.get(..acc.hdr_n).unwrap_or(&[])
        );
        acc.print(name);
        let (er, ar, gr, polls, _, _, _) = poll_rates(imu, failures, dr, &cf, 500).await;
        defmt::info!(
            "[STAT] {=str} FIFO on, polled {} reads: INT1 edges/s={} accel changes/s={} gyro changes/s={}",
            name,
            polls,
            er,
            ar,
            gr
        );
        verdict(
            failures,
            acc.n >= 150
                && acc.other == 0
                && acc.invalid_a == 0
                && acc.invalid_g == 0
                && acc.flag_changes == 0
                && acc.bad == 0
                && plausible(acc.a2_min)
                && plausible(acc.a2_max)
                && acc.gmax_dps <= 5.0,
            "mixed rates, FIFO: every frame decodes (no Invalid/WrongFormat/marker), no ODR-changed flag, period 5 ms",
        );
    }
}

// ---------------------------------------------------------------------------------------------
// Check 6: 8-byte single-sensor frames
// ---------------------------------------------------------------------------------------------

async fn check6<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32, buf: &mut [u8]) {
    defmt::info!(
        "[INFO] check 6: 8-byte frames (icm45686_fifo covers only the 16-byte and 20-byte formats)"
    );
    for accel_only in [true, false] {
        let name = if accel_only {
            "accel-only 8-byte frames"
        } else {
            "gyro-only 8-byte frames"
        };
        let (ao, go) = if accel_only {
            (Odr::Hz800, Odr::Hz200)
        } else {
            (Odr::Hz200, Odr::Hz800)
        };
        let c = base().with_accel_odr(ao).with_gyro_odr(go).with_fifo(
            fifo_base(FifoMode::Stream)
                .with_accel(accel_only)
                .with_gyro(!accel_only)
                .with_timestamp(false),
        );
        if !apply_verify(imu, failures, name, &c, false).await {
            continue;
        }
        if let Err(e) = imu.fifo_flush(&mut Delay).await {
            bus_fail(failures, "fifo_flush", &e);
            continue;
        }
        let t0 = Instant::now();
        Timer::after_millis(200).await;
        let r = match imu.fifo_read(buf).await {
            Ok(r) => r,
            Err(e) => {
                bus_fail(failures, "fifo_read", &e);
                continue;
            }
        };
        let el = t0.elapsed().as_micros() as f32;
        let total = r.frames + usize::from(r.left.unwrap_or(0));
        let mut acc = Acc::new(PERIOD_800_US, TimestampResolution::Us1);
        acc.feed(buf.get(..r.bytes).unwrap_or(&[]), &c);
        let expected = 800.0 * el / 1.0e6;
        defmt::info!(
            "[STAT] {=str}: frame_bytes={} capacity_frames={} bytes_read={} frames_read={} count_total={} expected_at_800Hz={} overflow={:?}",
            name,
            c.fifo.frame_bytes(),
            c.fifo.capacity_frames(),
            r.bytes,
            r.frames,
            total,
            expected,
            r.overflow
        );
        acc.print(name);
        let variant_ok = if accel_only {
            acc.n_accel as usize == r.frames
        } else {
            acc.n_gyro as usize == r.frames
        };
        verdict(
            failures,
            c.fifo.frame_bytes() == 8
                && r.bytes == r.frames * 8
                && c.fifo.capacity_frames() == 256
                && r.frames > 100
                && variant_ok
                && acc.other == 0
                && acc.invalid_a == 0
                && acc.invalid_g == 0,
            name,
        );
        let rate_ok = (total as f32 - expected).abs() <= 0.03 * expected;
        verdict(
            failures,
            rate_ok,
            "single-sensor frame rate equals the 800 Hz of that sensor within 3 %",
        );
        let data_ok = if accel_only {
            plausible(acc.a2_min) && plausible(acc.a2_max)
        } else {
            acc.gmax_dps <= 5.0
        };
        verdict(
            failures,
            data_ok,
            "single-sensor frame data plausible at rest",
        );
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    let mut spi_config = spi::Config::default();
    spi_config.frequency = SPI_HZ;
    let mut dr = DataReady::new(p.PWM_SLICE7.reborrow(), p.PIN_15.reborrow());
    let started = Instant::now();
    let mut buf = [0u8; 4096];

    {
        let bus = Spi::new(
            p.SPI0.reborrow(),
            p.PIN_18.reborrow(),
            p.PIN_19.reborrow(),
            p.PIN_16.reborrow(),
            p.DMA_CH0.reborrow(),
            p.DMA_CH1.reborrow(),
            Irqs,
            spi_config,
        );
        let cs = Output::new(p.PIN_20.reborrow(), Level::High);
        let device = ExclusiveDevice::new_no_delay(bus, cs).unwrap();
        let mut imu = Icm45686::new(device);

        health(&mut imu, &mut failures, "start", 20).await;
        check1(&mut imu, &mut failures, &mut dr, &mut buf).await;
        health(&mut imu, &mut failures, "after check 1", 20).await;
        let tick_ratio = check4(&mut imu, &mut failures, &mut dr, &mut buf).await;
        health(&mut imu, &mut failures, "after check 4", 20).await;
        check2(
            &mut imu,
            &mut failures,
            &mut buf,
            FifoMode::Stream,
            "check 2 stream",
            tick_ratio,
        )
        .await;
        health(&mut imu, &mut failures, "after check 2 stream", 20).await;
        check2(
            &mut imu,
            &mut failures,
            &mut buf,
            FifoMode::StopOnFull,
            "check 2 stop-on-full",
            tick_ratio,
        )
        .await;
        health(&mut imu, &mut failures, "after check 2 stop-on-full", 20).await;
        if started.elapsed().as_secs() < 100 {
            check3(&mut imu, &mut failures, &mut dr, &mut buf).await;
            check3_fifo_off(&mut imu, &mut failures, &mut dr).await;
            health(&mut imu, &mut failures, "after check 3", 20).await;
        }
        if started.elapsed().as_secs() < 110 {
            check5(&mut imu, &mut failures, &mut dr, &mut buf).await;
            health(&mut imu, &mut failures, "after check 5", 20).await;
        }
        if started.elapsed().as_secs() < 120 {
            check6(&mut imu, &mut failures, &mut buf).await;
        }
        health(&mut imu, &mut failures, "final", 200).await;
        defmt::info!(
            "[STAT] configure/init calls followed by verify Ok: {}",
            VERIFIES.load(Ordering::Relaxed)
        );
    }
    defmt::info!("[INFO] run took {} ms", started.elapsed().as_millis());

    hil_common::exit(failures)
}
