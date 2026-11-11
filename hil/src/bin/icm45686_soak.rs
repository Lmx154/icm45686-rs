//! Soak (data integrity over time) of the icm45686 driver on the MARV V2 Rev 1 board (U21, SPI0, INT1
//! on GPIO15). One-time bench test, CONTRACT rule D9. SPI mode 3 at 18.75 MHz. The driver lives in a task
//! of an interrupt executor; thread mode runs a CPU spinner and a DMA memory copy loop for the whole
//! run. A bump (motion) is a statistic and never a failure; corrupted data, a lost sample or a bus
//! error always is. No SPI transfer is ever cancelled: only the wait for an INT1 edge has a timeout.
//! No raw register write is used. Duration: SOAK_SECONDS at build time (default 1800).
#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::hint::black_box;
use core::pin::pin;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};
use core::task::Poll;

use cortex_m::peripheral::DWT;
use defmt_rtt as _;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::interrupt::{self, InterruptExt, Priority};
use embassy_rp::peripherals::{
    DMA_CH0, DMA_CH1, DMA_CH2, PIN_15, PIN_16, PIN_18, PIN_19, PIN_20, PWM_SLICE7, SPI0,
};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_rp::{Peri, bind_interrupts, dma};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use icm45686::{
    AccelFullScale, Config, Error, FifoConfig, FifoDepth, FifoFormat, FifoFrame, FifoMode,
    GyroFullScale, Icm45686, Int1Config, Odr, RawSample, fifo_frames, reg,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
});

const fn parse(s: Option<&str>, default: u32) -> u32 {
    let Some(s) = s else { return default };
    let b = s.as_bytes();
    if b.is_empty() {
        return default;
    }
    let mut i = 0;
    let mut v = 0u32;
    while i < b.len() {
        assert!(b[i] >= b'0' && b[i] <= b'9');
        v = v * 10 + (b[i] - b'0') as u32;
        i += 1;
    }
    v
}

const SOAK_SECONDS: u32 = parse(option_env!("SOAK_SECONDS"), 1800);
const _: () = assert!(SOAK_SECONDS >= 9);
const A_SECONDS: u32 = SOAK_SECONDS / 3 * 2;
const B_SECONDS: u32 = SOAK_SECONDS - A_SECONDS;
/// Seconds between two verify() calls and between two progress lines (shortened in short runs).
const VERIFY_EVERY_A: u32 = if A_SECONDS / 2 < 60 {
    A_SECONDS / 2
} else {
    60
};
const PROGRESS_EVERY: u32 = if SOAK_SECONDS / 3 < 60 {
    SOAK_SECONDS / 3
} else {
    60
};

const SPI_HZ: u32 = 18_750_000;
const EDGE_HZ: u64 = 6400;
const G: f32 = 9.80665;
const ACCEL_FS: AccelFullScale = AccelFullScale::G4;
const GYRO_FS: GyroFullScale = GyroFullScale::Dps250;
const CYC_PER_US: u32 = 150;

// ---------------------------------------------------------------------------------------------
// Shared with thread mode (progress line, aggressor counters, verdict)
// ---------------------------------------------------------------------------------------------

static PAIRS: AtomicU32 = AtomicU32::new(0);
static MISMATCH: AtomicU32 = AtomicU32::new(0);
static SKIPPED: AtomicU32 = AtomicU32::new(0);
static EDGES: AtomicU32 = AtomicU32::new(0);
static PAUSED: AtomicU32 = AtomicU32::new(0);
static MOTION: AtomicU32 = AtomicU32::new(0);
static MISSED: AtomicU32 = AtomicU32::new(0);
static FRAMES: AtomicU32 = AtomicU32::new(0);
static BAD_B: AtomicU32 = AtomicU32::new(0);
static PHASE: AtomicU32 = AtomicU32::new(0);
static FAILS: AtomicU32 = AtomicU32::new(0);
static DONE: AtomicBool = AtomicBool::new(false);
static STOP: AtomicBool = AtomicBool::new(false);
static SPIN_CHUNKS: AtomicU32 = AtomicU32::new(0);
static DMA_COPIES: AtomicU32 = AtomicU32::new(0);
static DMA_BAD: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------------------------
// INT1: hardware edge counter plus GPIO edge wake (same as icm45686_fifo)
// ---------------------------------------------------------------------------------------------

struct DataReady<'d> {
    input: Input<'d>,
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
            input,
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

    /// Waits until the hardware edge count exceeds `seen`; `None` after `timeout`.
    async fn wait_beyond(&mut self, seen: u64, timeout: Duration) -> Option<u64> {
        let Self {
            input,
            pwm,
            last,
            wraps,
        } = self;
        let wait = async {
            loop {
                let mut edge = pin!(input.wait_for_rising_edge());
                let c = poll_fn(|cx| {
                    let fired = edge.as_mut().poll(cx).is_ready();
                    let raw = pwm.counter();
                    if raw < *last {
                        *wraps += 1;
                    }
                    *last = raw;
                    let c = (*wraps << 16) | u64::from(raw);
                    if c > seen {
                        Poll::Ready(Some(c))
                    } else if fired {
                        Poll::Ready(None)
                    } else {
                        Poll::Pending
                    }
                })
                .await;
                if let Some(c) = c {
                    return c;
                }
            }
        };
        with_timeout(timeout, wait).await.ok()
    }
}

// ---------------------------------------------------------------------------------------------
// Record
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Ev {
    idx: u32,
    t_us: u64,
    edges: u64,
    a: [i16; 6],
    b: [i16; 6],
    x: u32,
    y: u32,
}

const EV0: Ev = Ev {
    idx: 0,
    t_us: 0,
    edges: 0,
    a: [0; 6],
    b: [0; 6],
    x: 0,
    y: 0,
};

struct Events {
    n: u32,
    v: [Ev; 16],
}

impl Events {
    const fn new() -> Self {
        Self { n: 0, v: [EV0; 16] }
    }

    fn push(&mut self, e: Ev) {
        if let Some(s) = self.v.get_mut(self.n as usize) {
            *s = e;
        }
        self.n = self.n.saturating_add(1);
    }

    fn print(&self, name: &str) {
        let shown = (self.n as usize).min(16);
        if shown == 0 {
            return;
        }
        defmt::info!("[REC] {=str}: {} events, first {}", name, self.n, shown);
        for e in self.v.iter().take(shown) {
            defmt::info!(
                "[REC] {=str} read={} t_us={} edges={} first={} second={} x={=u32:#x} y={=u32:#x}",
                name,
                e.idx,
                e.t_us,
                e.edges,
                &e.a[..],
                &e.b[..],
                e.x,
                e.y
            );
        }
    }
}

#[derive(Clone, Copy)]
struct Snap {
    kind: u8,
    at_read: u32,
    ring: [[i16; 6]; 32],
}

#[derive(Clone, Copy)]
struct Win {
    start_us: u64,
    len: u32,
    min_g2: f32,
    max_g2: f32,
}

/// Statistics of the verify() windows of one phase.
struct VerStat {
    n: u32,
    longest_us: u32,
    max_paused: u32,
    bad: u32,
}

impl VerStat {
    const fn new() -> Self {
        Self {
            n: 0,
            longest_us: 0,
            max_paused: 0,
            bad: 0,
        }
    }

    /// A window is valid when it lasted at most 100 ms and the edges that arrived in it are at most
    /// `duration x edge rate + 2`.
    fn judge(&mut self, dur_us: u64, window: u64, edge_hz: u64) -> bool {
        self.n += 1;
        self.longest_us = self.longest_us.max(dur_us as u32);
        self.max_paused = self.max_paused.max(window as u32);
        let ok = dur_us <= 100_000 && window <= dur_us * edge_hz / 1_000_000 + 2;
        if !ok {
            self.bad += 1;
        }
        ok
    }
}

/// The record of an invalid verify window: duration in x, paused edges in y with a marker.
fn bad_window_ev(idx: u32, t0: Instant, edges: u64, dur: u64, window: u64) -> Ev {
    Ev {
        idx,
        t_us: t0.elapsed().as_micros(),
        edges,
        a: [0; 6],
        b: [0; 6],
        x: dur as u32,
        y: (0xBAD << 16) | (window as u32).min(0xFFFF),
    }
}

struct Soak {
    // phase A
    missed: u32,
    ev_missed: Events,
    ver: VerStat,
    reads: u32,
    pairs: u32,
    counted: u32,
    skipped: u32,
    mismatch: u32,
    ident_reads: u32,
    ident_wrong: u32,
    verify_fail: u32,
    verifies: u32,
    bus_err: u32,
    no_edge: u32,
    stuck: u32,
    stuck_max_run: u32,
    paused: u64,
    e0: u64,
    e1: u64,
    a_us: u64,
    pc_mismatch: u32,
    motion_samples: u32,
    windows: u32,
    longest: u32,
    peak_gyro: i32,
    hist: [u32; 400],
    hist_n: u32,
    tim: [u32; 512],
    worst_cyc: u32,
    ring: [[i16; 6]; 32],
    ring_pos: u32,
    snaps: [Snap; 4],
    nsnap: u32,
    ev_mis: Events,
    ev_id: Events,
    ev_ver: Events,
    ev_bus: Events,
    ev_stuck: Events,
    win_cur: Option<Win>,
    wins: [Win; 16],
}

const WIN0: Win = Win {
    start_us: 0,
    len: 0,
    min_g2: 0.0,
    max_g2: 0.0,
};

impl Soak {
    fn new() -> Self {
        Self {
            reads: 0,
            pairs: 0,
            counted: 0,
            skipped: 0,
            mismatch: 0,
            ident_reads: 0,
            ident_wrong: 0,
            verify_fail: 0,
            verifies: 0,
            bus_err: 0,
            no_edge: 0,
            stuck: 0,
            missed: 0,
            ev_missed: Events::new(),
            ver: VerStat::new(),
            stuck_max_run: 0,
            paused: 0,
            e0: 0,
            e1: 0,
            a_us: 0,
            pc_mismatch: 0,
            motion_samples: 0,
            windows: 0,
            longest: 0,
            peak_gyro: 0,
            hist: [0; 400],
            hist_n: 0,
            tim: [0; 512],
            worst_cyc: 0,
            ring: [[0; 6]; 32],
            ring_pos: 0,
            snaps: [Snap {
                kind: 0,
                at_read: 0,
                ring: [[0; 6]; 32],
            }; 4],
            nsnap: 0,
            ev_mis: Events::new(),
            ev_id: Events::new(),
            ev_ver: Events::new(),
            ev_bus: Events::new(),
            ev_stuck: Events::new(),
            win_cur: None,
            wins: [WIN0; 16],
        }
    }

    fn ring_push(&mut self, v: [i16; 6]) {
        if let Some(s) = self.ring.get_mut(self.ring_pos as usize % 32) {
            *s = v;
        }
        self.ring_pos += 1;
    }

    /// Copies the ring (oldest first) for the first four events.
    fn snap(&mut self, kind: u8) {
        let n = self.nsnap as usize;
        if n >= 4 {
            return;
        }
        let mut r = [[0i16; 6]; 32];
        for (i, slot) in r.iter_mut().enumerate() {
            let p = (self.ring_pos as usize + i) % 32;
            if let Some(v) = self.ring.get(p) {
                *slot = *v;
            }
        }
        if let Some(s) = self.snaps.get_mut(n) {
            *s = Snap {
                kind,
                at_read: self.reads,
                ring: r,
            };
        }
        self.nsnap += 1;
    }

    /// One acceleration magnitude (squared, in g squared) per pair: statistic and histogram.
    fn motion(&mut self, g2: f32, t_us: u64) {
        let bin = ((g2 * 100.0) as usize).min(399);
        if let Some(h) = self.hist.get_mut(bin) {
            *h += 1;
        }
        self.hist_n += 1;
        let bad = !(0.81..=1.21).contains(&g2);
        if bad {
            self.motion_samples += 1;
            let w = self.win_cur.get_or_insert(Win {
                start_us: t_us,
                len: 0,
                min_g2: g2,
                max_g2: g2,
            });
            w.len += 1;
            w.min_g2 = w.min_g2.min(g2);
            w.max_g2 = w.max_g2.max(g2);
        } else {
            self.close_window();
        }
    }

    fn close_window(&mut self) {
        if let Some(w) = self.win_cur.take() {
            if let Some(s) = self.wins.get_mut(self.windows as usize) {
                *s = w;
            }
            self.windows += 1;
            self.longest = self.longest.max(w.len);
        }
    }
}

fn pack(s: &RawSample) -> [i16; 6] {
    [
        s.accel[0], s.accel[1], s.accel[2], s.gyro[0], s.gyro[1], s.gyro[2],
    ]
}

/// The one comparison used for pairs and for the positive control: accel and gyro, no temperature.
fn same(a: &RawSample, b: &RawSample) -> bool {
    a.accel == b.accel && a.gyro == b.gyro
}

fn g2_of(s: &RawSample, fs: AccelFullScale) -> f32 {
    let a = s.accel_ms2(fs);
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (G * G)
}

fn median_bin(h: &[u32]) -> usize {
    let total: u64 = h.iter().map(|v| u64::from(*v)).sum();
    let mut acc = 0u64;
    for (i, v) in h.iter().enumerate() {
        acc += u64::from(*v);
        if acc * 2 >= total {
            return i;
        }
    }
    0
}

fn err_code<E>(e: &Error<E>) -> u32 {
    match e {
        Error::Bus(_) => 1,
        Error::NotConfigured => 2,
        Error::ConfigMismatch(_) => 3,
        _ => 9,
    }
}

fn spi0_hz() -> u32 {
    let pre = embassy_rp::pac::SPI0.cpsr().read().cpsdvsr() as u32;
    let scr = embassy_rp::pac::SPI0.cr0().read().scr() as u32;
    embassy_rp::clocks::clk_peri_freq() / (pre * (1 + scr))
}

fn crit(failures: &mut u32, ok: bool, name: &str, value: u32) {
    if ok {
        defmt::info!("[PASS] {=str} ({=u32})", name, value);
    } else {
        *failures += 1;
        defmt::error!("[FAIL] {=str} ({=u32})", name, value);
    }
}

/// The edge being serviced: waits for the next edge and returns `(late, seen_before)`, where `late`
/// is the number of edges that arrived on top of the serviced one (missed samples in phase A). `seen`
/// becomes the hardware count read at the wake, so the read that follows serves the newest sample once.
async fn next_edge(dr: &mut DataReady<'_>, seen: &mut u64) -> Option<(u64, u64)> {
    let c = dr.wait_beyond(*seen, Duration::from_millis(100)).await?;
    let was = *seen;
    *seen = c;
    Some((c - was - 1, was))
}

/// verify() with its duration and the edges that arrived in it. Those edges are "paused"; the window
/// is returned as `Some((duration_us, edges))` when it is not valid (see `VerStat::judge`).
async fn verify_pause<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    dr: &mut DataReady<'_>,
    seen: &mut u64,
    paused: &mut u64,
    ver: &mut VerStat,
) -> (Result<(), Error<SPI::Error>>, Option<(u64, u64)>) {
    let c0 = dr.count();
    let t = Instant::now();
    let r = imu.verify(&mut Delay).await;
    let dur = t.elapsed().as_micros();
    let c1 = dr.count();
    *paused += c1.saturating_sub(*seen);
    *seen = (*seen).max(c1);
    let w = c1.saturating_sub(c0);
    let bad = (!ver.judge(dur, w, EDGE_HZ)).then_some((dur, w));
    (r, bad)
}

// ---------------------------------------------------------------------------------------------
// Phase A: single-sample path, a pair on every data-ready edge
// ---------------------------------------------------------------------------------------------

async fn phase_a<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    dr: &mut DataReady<'_>,
    s: &mut Soak,
    t0: Instant,
    end: Instant,
    vevery: Duration,
) {
    let mut seen = dr.count();
    s.e0 = seen;
    let ta0 = Instant::now();
    let mut next_v = ta0 + vevery;
    let mut prev: Option<[i16; 6]> = None;
    let mut run = 0u32;
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        // a verify never starts on top of an unserviced edge: that edge is serviced first
        if now >= next_v && dr.count() <= seen {
            next_v += vevery;
            let (r, bad) = verify_pause(imu, dr, &mut seen, &mut s.paused, &mut s.ver).await;
            PAUSED.store(s.paused as u32, Relaxed);
            s.verifies += 1;
            if let Some((dur, w)) = bad {
                s.ev_ver.push(bad_window_ev(s.reads, t0, seen, dur, w));
            }
            record_verify(s, t0, dr, r);
            continue;
        }
        let Some((late, was)) = next_edge(dr, &mut seen).await else {
            s.no_edge += 1;
            continue;
        };
        if late > 0 {
            s.missed += late as u32;
            MISSED.store(s.missed, Relaxed);
            s.snap(6);
            s.ev_missed.push(Ev {
                idx: s.reads,
                t_us: t0.elapsed().as_micros(),
                edges: seen,
                a: [0; 6],
                b: [0; 6],
                x: was as u32,
                y: seen as u32,
            });
        }
        let c0 = dr.count();
        let cy0 = DWT::cycle_count();
        let r1 = imu.read().await;
        let r2 = imu.read().await;
        let cyc = DWT::cycle_count().wrapping_sub(cy0);
        let c1 = dr.count();
        s.pairs += 1;
        s.reads += 2;
        let (r1, r2) = match (r1, r2) {
            (Ok(a), Ok(b)) => (a, b),
            (a, b) => {
                s.bus_err += 1;
                s.snap(2);
                let code = a.as_ref().err().or(b.as_ref().err()).map_or(0, err_code);
                s.ev_bus.push(Ev {
                    idx: s.reads,
                    t_us: t0.elapsed().as_micros(),
                    edges: c1,
                    a: a.map_or([0; 6], |v| pack(&v)),
                    b: b.map_or([0; 6], |v| pack(&v)),
                    x: code,
                    y: 0,
                });
                continue;
            }
        };
        let (pa, pb) = (pack(&r1), pack(&r2));
        s.ring_push(pa);
        s.ring_push(pb);
        let t_us = t0.elapsed().as_micros();
        if c1 != c0 {
            s.skipped += 1;
        } else {
            s.counted += 1;
            if !same(&r1, &r2) {
                s.mismatch += 1;
                s.snap(1);
                s.ev_mis.push(Ev {
                    idx: s.reads,
                    t_us,
                    edges: c1,
                    a: pa,
                    b: pb,
                    x: 0,
                    y: 0,
                });
            }
        }
        let us = (cyc / CYC_PER_US) as usize;
        if let Some(t) = s.tim.get_mut(us.min(511)) {
            *t += 1;
        }
        s.worst_cyc = s.worst_cyc.max(cyc);
        // stuck: no accel or gyro value moved by even one LSB since the previous pair
        if prev == Some(pa) {
            run += 1;
            s.stuck_max_run = s.stuck_max_run.max(run);
            if run == 1000 {
                s.stuck += 1;
                s.snap(5);
                s.ev_stuck.push(Ev {
                    idx: s.reads,
                    t_us,
                    edges: c1,
                    a: pa,
                    b: pb,
                    x: run,
                    y: 0,
                });
            }
        } else {
            run = 0;
        }
        prev = Some(pa);
        // motion: statistic only
        s.motion(g2_of(&r1, ACCEL_FS), t_us);
        for g in r1.gyro {
            s.peak_gyro = s.peak_gyro.max(i32::from(g).abs());
        }
        // identity every 100 pairs
        if s.pairs % 100 == 0 {
            s.ident_reads += 1;
            match imu.read_register(reg::WHO_AM_I).await {
                Ok(reg::WHO_AM_I_VALUE) => {}
                Ok(v) => {
                    s.ident_wrong += 1;
                    s.snap(3);
                    s.ev_id.push(Ev {
                        idx: s.reads,
                        t_us: t0.elapsed().as_micros(),
                        edges: dr.count(),
                        a: pa,
                        b: pb,
                        x: u32::from(reg::WHO_AM_I_VALUE),
                        y: u32::from(v),
                    });
                }
                Err(e) => {
                    s.bus_err += 1;
                    s.snap(2);
                    s.ev_bus.push(Ev {
                        idx: s.reads,
                        t_us: t0.elapsed().as_micros(),
                        edges: dr.count(),
                        a: pa,
                        b: pb,
                        x: err_code(&e),
                        y: 1,
                    });
                }
            }
        }
        PAIRS.store(s.pairs, Relaxed);
        MISMATCH.store(s.mismatch, Relaxed);
        SKIPPED.store(s.skipped, Relaxed);
        MOTION.store(s.motion_samples, Relaxed);
        EDGES.store((c1 - s.e0) as u32, Relaxed);
    }
    s.e1 = dr.count();
    s.a_us = ta0.elapsed().as_micros();
    s.close_window();
    let (r, bad) = verify_pause(imu, dr, &mut seen, &mut s.paused, &mut s.ver).await;
    s.verifies += 1;
    if let Some((dur, w)) = bad {
        s.ev_ver.push(bad_window_ev(s.reads, t0, seen, dur, w));
    }
    record_verify(s, t0, dr, r);
}

fn record_verify<E>(s: &mut Soak, t0: Instant, dr: &mut DataReady<'_>, r: Result<(), Error<E>>) {
    let (kind, x, y) = match r {
        Ok(()) => return,
        Err(Error::ConfigMismatch(m)) => (
            3u32,
            (u32::from(m.register) << 16) | (u32::from(m.mask) << 8) | u32::from(m.expected),
            u32::from(m.found),
        ),
        Err(e) => (err_code(&e), 0, 0),
    };
    s.verify_fail += 1;
    s.snap(4);
    s.ev_ver.push(Ev {
        idx: s.reads,
        t_us: t0.elapsed().as_micros(),
        edges: dr.count(),
        a: [0; 6],
        b: [0; 6],
        x,
        y: y | (kind << 16),
    });
}

// ---------------------------------------------------------------------------------------------
// Phase B: FIFO path, stream mode, 16-byte frames at 6.4 kHz, watermark 8 (800 edges per second)
// ---------------------------------------------------------------------------------------------

const WATERMARK: usize = 8;
const FRAME_BYTES: usize = 16;

#[derive(Clone, Copy)]
struct FrameEv {
    idx: u32,
    t_us: u64,
    edges: u64,
    raw: [u8; 48],
}

struct SoakB {
    frames: u32,
    malformed: u32,
    ts_bad: u32,
    fifo_over: u32,
    fifo_count_max: u32,
    late_edges: u32,
    pc_count: u32,
    pc_flagged: u32,
    pc_ok: bool,
    ver: VerStat,
    ev_late: Events,
    ev_count: Events,
    verify_fail: u32,
    verifies: u32,
    bus_err: u32,
    no_edge: u32,
    paused: u64,
    e0: u64,
    e1: u64,
    us: u64,
    median: Option<u16>,
    step_hist: [u32; 1024],
    step_n: u32,
    ev_mal: [FrameEv; 16],
    ev_ts: Events,
    ev_ver: Events,
    ev_bus: Events,
}

impl SoakB {
    fn new() -> Self {
        Self {
            frames: 0,
            malformed: 0,
            ts_bad: 0,
            fifo_over: 0,
            fifo_count_max: 0,
            late_edges: 0,
            pc_count: 0,
            pc_flagged: 0,
            pc_ok: false,
            ver: VerStat::new(),
            ev_late: Events::new(),
            ev_count: Events::new(),
            verify_fail: 0,
            verifies: 0,
            bus_err: 0,
            no_edge: 0,
            paused: 0,
            e0: 0,
            e1: 0,
            us: 0,
            median: None,
            step_hist: [0; 1024],
            step_n: 0,
            ev_mal: [FrameEv {
                idx: 0,
                t_us: 0,
                edges: 0,
                raw: [0; 48],
            }; 16],
            ev_ts: Events::new(),
            ev_ver: Events::new(),
            ev_bus: Events::new(),
        }
    }
}

/// `(expected-side, found-side)` words of a verify() failure, `None` when it passed.
fn verify_words<E>(r: &Result<(), Error<E>>) -> Option<(u32, u32)> {
    match r {
        Ok(()) => None,
        Err(Error::ConfigMismatch(m)) => Some((
            (u32::from(m.register) << 16) | (u32::from(m.mask) << 8) | u32::from(m.expected),
            (3 << 16) | u32::from(m.found),
        )),
        Err(e) => Some((0, err_code(e) << 16)),
    }
}

const FIFO_LIMIT: u16 = 4 * WATERMARK as u16;
/// Nominal capacity of the 2 KB FIFO in 16-byte frames (the driver documents 128; the stream-mode
/// count saturates at 130).
const FIFO_CAPACITY: u16 = FifoDepth::Kib2.bytes() / FRAME_BYTES as u16;

/// Reads one watermark's worth of frames and checks them. `c` is the edge count for the record.
async fn b_service<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    b: &mut SoakB,
    prev: &mut Option<u16>,
    fifo: &FifoConfig,
    t0: Instant,
    c: u64,
    check_count: bool,
) {
    if check_count {
        match imu.fifo_count().await {
            Ok(n) => {
                b.fifo_count_max = b.fifo_count_max.max(u32::from(n));
                if n > FIFO_LIMIT {
                    b.fifo_over += 1;
                    b.ev_count.push(Ev {
                        idx: b.frames,
                        t_us: t0.elapsed().as_micros(),
                        edges: c,
                        a: [0; 6],
                        b: [0; 6],
                        x: u32::from(n),
                        y: u32::from(FIFO_LIMIT),
                    });
                }
            }
            Err(e) => {
                b.bus_err += 1;
                b.ev_bus.push(Ev {
                    idx: b.frames,
                    t_us: t0.elapsed().as_micros(),
                    edges: c,
                    a: [0; 6],
                    b: [0; 6],
                    x: err_code(&e),
                    y: 3,
                });
            }
        }
    }
    let mut buf = [0u8; WATERMARK * FRAME_BYTES];
    if let Err(e) = imu.fifo_read_exact(&mut buf, WATERMARK).await {
        b.bus_err += 1;
        b.ev_bus.push(Ev {
            idx: b.frames,
            t_us: t0.elapsed().as_micros(),
            edges: c,
            a: [0; 6],
            b: [0; 6],
            x: err_code(&e),
            y: 1,
        });
        let _ = imu.fifo_flush(&mut Delay).await;
        *prev = None;
        return;
    }
    let t_us = t0.elapsed().as_micros();
    for (i, fr) in fifo_frames(&buf, fifo).enumerate() {
        let idx = b.frames;
        b.frames += 1;
        let FifoFrame::AccelGyro(f) = fr else {
            b.malformed += 1;
            if let Some(e) = b.ev_mal.get_mut(b.malformed as usize - 1) {
                let mut raw = [0u8; 48];
                let lo = i.saturating_sub(1) * FRAME_BYTES;
                let hi = ((i + 2) * FRAME_BYTES).min(buf.len());
                let src = buf.get(lo..hi).unwrap_or(&[]);
                let off = if i == 0 { FRAME_BYTES } else { 0 };
                if let Some(d) = raw.get_mut(off..off + src.len()) {
                    d.copy_from_slice(src);
                }
                *e = FrameEv {
                    idx,
                    t_us,
                    edges: c,
                    raw,
                };
            }
            continue;
        };
        if let Some(p) = *prev {
            let step = f.timestamp.wrapping_sub(p);
            match b.median {
                None => {
                    if let Some(h) = b.step_hist.get_mut(usize::from(step).min(1023)) {
                        *h += 1;
                    }
                    b.step_n += 1;
                    if b.step_n == 1000 {
                        b.median = Some(median_bin(&b.step_hist) as u16);
                    }
                }
                Some(m) => {
                    if step.abs_diff(m) > 1 {
                        b.ts_bad += 1;
                        b.ev_ts.push(Ev {
                            idx,
                            t_us,
                            edges: c,
                            a: pack(&RawSample {
                                accel: f.accel,
                                gyro: f.gyro,
                                temp: 0,
                            }),
                            b: [0; 6],
                            x: u32::from(p),
                            y: u32::from(f.timestamp) | (u32::from(m) << 16),
                        });
                    }
                }
            }
        }
        *prev = Some(f.timestamp);
    }
}

/// Waits for a watermark edge and services it. A late edge (an edge on top of the serviced one) is
/// a statistic here: the frames are still in the FIFO. False when no edge came within 100 ms.
async fn b_edge<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    dr: &mut DataReady<'_>,
    b: &mut SoakB,
    seen: &mut u64,
    prev: &mut Option<u16>,
    fifo: &FifoConfig,
    t0: Instant,
    check_count: bool,
) -> bool {
    let Some((late, was)) = next_edge(dr, seen).await else {
        b.no_edge += 1;
        return false;
    };
    if late > 0 {
        b.late_edges += late as u32;
        b.ev_late.push(Ev {
            idx: b.frames,
            t_us: t0.elapsed().as_micros(),
            edges: *seen,
            a: [0; 6],
            b: [0; 6],
            x: was as u32,
            y: *seen as u32,
        });
    }
    b_service(imu, b, prev, fifo, t0, *seen, check_count).await;
    true
}

/// Positive control of the FIFO loss checks: with the FIFO running, no read for 30 ms (it fills in
/// about 20 ms), then `fifo_count()` must exceed the limit and the next reads must show a timestamp
/// step the check flags. Returns (count, flagged steps, ok). Own counters; nothing is added to the run's.
async fn b_control<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    dr: &mut DataReady<'_>,
    fifo: &FifoConfig,
    t0: Instant,
) -> (u32, u32, bool) {
    let mut pc = SoakB::new();
    let mut seen = dr.count();
    let mut prev = None;
    let mut n = 0;
    while pc.median.is_none() && n < 400 {
        n += 1;
        b_edge(imu, dr, &mut pc, &mut seen, &mut prev, fifo, t0, false).await;
    }
    if pc.median.is_none() {
        return (0, 0, false);
    }
    Timer::after_millis(30).await;
    let cnt = imu.fifo_count().await.map_or(0, u32::from);
    let c = dr.count();
    let before = pc.ts_bad;
    for _ in 0..3 {
        b_service(imu, &mut pc, &mut prev, fifo, t0, c, false).await;
    }
    let flagged = pc.ts_bad - before;
    (
        cnt,
        flagged,
        cnt > u32::from(FIFO_LIMIT) && flagged > 0 && pc.bus_err == 0,
    )
}

async fn phase_b<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    dr: &mut DataReady<'_>,
    b: &mut SoakB,
    t0: Instant,
    end: Instant,
    vevery: Duration,
    fifo: &FifoConfig,
) {
    if imu.fifo_flush(&mut Delay).await.is_err() || imu.interrupt_status().await.is_err() {
        b.bus_err += 1;
    }
    let (cnt, flagged, ok) = b_control(imu, dr, fifo, t0).await;
    b.pc_count = cnt;
    b.pc_flagged = flagged;
    b.pc_ok = ok;
    defmt::info!(
        "[STAT] positive control B: 30 ms without reading -> fifo_count={} (limit {}, capacity {}), next 3 reads flagged {} timestamp steps (need > limit and >= 1; reported apart, not in the totals)",
        cnt,
        FIFO_LIMIT,
        FIFO_CAPACITY,
        flagged
    );
    if !ok {
        defmt::error!("[FAIL] positive control B: loss not flagged");
        return;
    }
    if imu.fifo_flush(&mut Delay).await.is_err() || imu.interrupt_status().await.is_err() {
        b.bus_err += 1;
    }
    let mut prev: Option<u16> = None;
    let mut seen = dr.count();
    b.e0 = seen;
    let tb0 = Instant::now();
    let mut next_v = tb0 + vevery;
    let mut next_cnt = tb0 + Duration::from_secs(1);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        if now >= next_v && dr.count() <= seen {
            next_v += vevery;
            let c0 = dr.count();
            let tv = Instant::now();
            let r = imu.verify(&mut Delay).await;
            let fl = imu.fifo_flush(&mut Delay).await;
            let dur = tv.elapsed().as_micros();
            prev = None;
            let c = dr.count();
            b.paused += c.saturating_sub(seen);
            seen = seen.max(c);
            b.verifies += 1;
            let w = c.saturating_sub(c0);
            if !b.ver.judge(dur, w, EDGE_HZ / WATERMARK as u64) {
                b.ev_ver.push(bad_window_ev(b.frames, t0, c, dur, w));
            }
            if let Some((x, y)) = verify_words(&r) {
                b.verify_fail += 1;
                b.ev_ver.push(Ev {
                    idx: b.frames,
                    t_us: t0.elapsed().as_micros(),
                    edges: c,
                    a: [0; 6],
                    b: [0; 6],
                    x,
                    y,
                });
            }
            if let Err(e) = fl {
                b.bus_err += 1;
                b.ev_bus.push(Ev {
                    idx: b.frames,
                    t_us: t0.elapsed().as_micros(),
                    edges: c,
                    a: [0; 6],
                    b: [0; 6],
                    x: err_code(&e),
                    y: 2,
                });
            }
            continue;
        }
        let check = now >= next_cnt;
        if check {
            next_cnt += Duration::from_secs(1);
        }
        b_edge(imu, dr, b, &mut seen, &mut prev, fifo, t0, check).await;
        FRAMES.store(b.frames, Relaxed);
        BAD_B.store(
            b.malformed + b.ts_bad + b.fifo_over + b.verify_fail + b.bus_err + b.ver.bad,
            Relaxed,
        );
    }
    b.e1 = dr.count();
    b.us = tb0.elapsed().as_micros();
    let c0 = b.e1;
    let tv = Instant::now();
    let r = imu.verify(&mut Delay).await;
    let dur = tv.elapsed().as_micros();
    let w = dr.count().saturating_sub(c0);
    b.verifies += 1;
    if !b.ver.judge(dur, w, EDGE_HZ / WATERMARK as u64) {
        b.ev_ver.push(bad_window_ev(b.frames, t0, b.e1, dur, w));
    }
    if let Some((x, y)) = verify_words(&r) {
        b.verify_fail += 1;
        b.ev_ver.push(Ev {
            idx: b.frames,
            t_us: t0.elapsed().as_micros(),
            edges: b.e1,
            a: [0; 6],
            b: [0; 6],
            x,
            y,
        });
    }
}

/// Positive control of the phase A loss counter: a 2 ms sleep between two pairs (a timer wait between
/// bus calls) must show up as missed edges. Returns the count, or `None` on a bus failure or no edge.
async fn missed_control<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    dr: &mut DataReady<'_>,
) -> Option<u32> {
    let mut seen = dr.count();
    next_edge(dr, &mut seen).await?;
    imu.read().await.ok()?;
    imu.read().await.ok()?;
    Timer::after_millis(2).await;
    let (late, _) = next_edge(dr, &mut seen).await?;
    imu.read().await.ok()?;
    imu.read().await.ok()?;
    Some(late as u32)
}

/// The positive control: a read() against a raw read that starts one register later, through the same
/// comparison. Returns 1 when it is flagged.
async fn positive_control<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    dr: &mut DataReady<'_>,
) -> Result<u32, Error<SPI::Error>> {
    let seen = dr.count();
    let _ = dr.wait_beyond(seen, Duration::from_millis(100)).await;
    let a = imu.read().await?;
    let mut raw = [0u8; 14];
    imu.read_registers(reg::ACCEL_DATA_X1_UI + 1, &mut raw)
        .await?;
    let b = RawSample::decode(&raw);
    Ok(u32::from(!same(&a, &b)))
}

// ---------------------------------------------------------------------------------------------
// The soak task, on the interrupt executor
// ---------------------------------------------------------------------------------------------

struct Parts {
    spi: Peri<'static, SPI0>,
    clk: Peri<'static, PIN_18>,
    mosi: Peri<'static, PIN_19>,
    miso: Peri<'static, PIN_16>,
    cs: Peri<'static, PIN_20>,
    d0: Peri<'static, DMA_CH0>,
    d1: Peri<'static, DMA_CH1>,
    slice: Peri<'static, PWM_SLICE7>,
    int: Peri<'static, PIN_15>,
}

fn pct(n: u32, d: u32) -> f32 {
    if d == 0 {
        0.0
    } else {
        n as f32 * 100.0 / d as f32
    }
}

#[embassy_executor::task]
async fn soak_task(mut p: Parts) {
    let mut failures = 0u32;
    let t0 = Instant::now();
    let mut s = Soak::new();
    let mut b = SoakB::new();
    let mut dr = DataReady::new(p.slice.reborrow(), p.int.reborrow());
    // Boxed away from the stack frame of the task by living in the future: Soak is large.
    let mut cfg = spi::Config::default();
    cfg.frequency = SPI_HZ;
    cfg.polarity = spi::Polarity::IdleHigh;
    cfg.phase = spi::Phase::CaptureOnSecondTransition;
    {
        let bus = Spi::new(
            p.spi.reborrow(),
            p.clk.reborrow(),
            p.mosi.reborrow(),
            p.miso.reborrow(),
            p.d0.reborrow(),
            p.d1.reborrow(),
            Irqs,
            cfg,
        );
        let hz = spi0_hz();
        defmt::info!(
            "[INFO] soak {} s (A {} s, B {} s), SPI mode 3, actual clock {} Hz",
            SOAK_SECONDS,
            A_SECONDS,
            B_SECONDS,
            hz
        );
        if hz > 24_000_000 {
            defmt::error!("[FAIL] SPI clock above 24 MHz, nothing run");
            FAILS.store(1, Relaxed);
            DONE.store(true, Relaxed);
            return;
        }
        let cs = Output::new(p.cs.reborrow(), Level::High);
        let device = ExclusiveDevice::new_no_delay(bus, cs).unwrap();
        let mut imu = Icm45686::new(device);

        // ---- phase A
        PHASE.store(1, Relaxed);
        let config = Config::new()
            .with_accel_fs(ACCEL_FS)
            .with_gyro_fs(GYRO_FS)
            .with_int1(Int1Config::new().with_data_ready(true))
            .with_accel_odr(Odr::Hz6400)
            .with_gyro_odr(Odr::Hz6400);
        if let Err(e) = imu.init(&mut Delay, &config).await {
            defmt::error!("[FAIL] init A: {}", defmt::Debug2Format(&e));
            FAILS.store(1, Relaxed);
            DONE.store(true, Relaxed);
            return;
        }
        match positive_control(&mut imu, &mut dr).await {
            Ok(n) => {
                s.pc_mismatch = n;
                defmt::info!(
                    "[STAT] positive control: shifted read counted as {} mismatch (must be 1; reported apart, not in the totals)",
                    n
                );
                if n != 1 {
                    defmt::error!("[FAIL] positive control not flagged by the comparison");
                    FAILS.store(1, Relaxed);
                    DONE.store(true, Relaxed);
                    return;
                }
            }
            Err(e) => {
                defmt::error!("[FAIL] positive control read: {}", defmt::Debug2Format(&e));
                FAILS.store(1, Relaxed);
                DONE.store(true, Relaxed);
                return;
            }
        }
        match missed_control(&mut imu, &mut dr).await {
            Some(n) if n >= 5 => {
                defmt::info!(
                    "[STAT] positive control A: 2 ms sleep between two pairs -> missed rose by {} (need >= 5; reported apart, not in the totals)",
                    n
                );
            }
            other => {
                defmt::error!(
                    "[FAIL] positive control A: missed rose by {} (need >= 5; 0 also stands for a failed read)",
                    other.unwrap_or(0)
                );
                FAILS.store(1, Relaxed);
                DONE.store(true, Relaxed);
                return;
            }
        }
        let end_a = Instant::now() + Duration::from_secs(u64::from(A_SECONDS));
        phase_a(
            &mut imu,
            &mut dr,
            &mut s,
            t0,
            end_a,
            Duration::from_secs(u64::from(VERIFY_EVERY_A)),
        )
        .await;
        PHASE.store(2, Relaxed);

        // ---- phase B
        let config_b = Config::new()
            .with_accel_fs(ACCEL_FS)
            .with_gyro_fs(GYRO_FS)
            .with_accel_odr(Odr::Hz6400)
            .with_gyro_odr(Odr::Hz6400)
            .with_int1(Int1Config::new().with_fifo_threshold(true))
            .with_fifo(
                FifoConfig::new()
                    .with_mode(FifoMode::Stream)
                    .with_format(FifoFormat::Standard)
                    .with_accel(true)
                    .with_gyro(true)
                    .with_timestamp(true)
                    .with_watermark(WATERMARK as u16)
                    .with_depth(FifoDepth::Kib2),
            );
        match imu.configure(&mut Delay, &config_b).await {
            Ok(_) => {
                let end_b = Instant::now() + Duration::from_secs(u64::from(B_SECONDS));
                phase_b(
                    &mut imu,
                    &mut dr,
                    &mut b,
                    t0,
                    end_b,
                    Duration::from_secs(u64::from(B_SECONDS / 2).min(60)),
                    &config_b.fifo,
                )
                .await;
            }
            Err(e) => {
                defmt::error!("[FAIL] configure B: {}", defmt::Debug2Format(&e));
                b.bus_err += 1;
            }
        }
    }

    // ---- report, after the timed sections
    let elapsed_a = (s.a_us as f32) / 1e6;
    let edges_a = s.e1 - s.e0;
    let lost_a = edges_a
        .saturating_sub(s.paused)
        .saturating_sub(u64::from(s.pairs))
        .saturating_sub(1) as u32
        + s.no_edge;
    let rate_a = edges_a as f32 / elapsed_a.max(0.001);
    let med_cyc = median_bin(&s.tim);
    defmt::info!(
        "[STAT] integrity A: pairs={} mismatch={} skipped={}({}%) identity_wrong={} verify_fail={} bus_err={} lost={} stuck={} missed={}",
        s.pairs,
        s.mismatch,
        s.skipped,
        pct(s.skipped, s.pairs),
        s.ident_wrong,
        s.verify_fail,
        s.bus_err,
        lost_a,
        s.stuck,
        s.missed
    );
    defmt::info!(
        "[STAT] verify A: n={} longest_us={} paused_max={} window_bad={}",
        s.ver.n,
        s.ver.longest_us,
        s.ver.max_paused,
        s.ver.bad
    );
    defmt::info!(
        "[STAT] A detail: counted={} identity_reads={} verifies={} paused_edges={} no_edge_timeouts={} edges={} edge_rate_hz={} pairs_per_s={} pair_us median_bin={} worst={} longest_identical_run={}",
        s.counted,
        s.ident_reads,
        s.verifies,
        s.paused,
        s.no_edge,
        edges_a,
        rate_a,
        s.pairs as f32 / elapsed_a.max(0.001),
        med_cyc,
        s.worst_cyc as f32 / CYC_PER_US as f32,
        s.stuck_max_run
    );
    let mp = RawSample {
        accel: [0; 3],
        gyro: [s.peak_gyro.min(32767) as i16, 0, 0],
        temp: 0,
    };
    defmt::info!(
        "[STAT] motion: samples={} windows={} longest={} peak_gyro={}dps",
        s.motion_samples,
        s.windows,
        s.longest,
        mp.gyro_rads(GYRO_FS)[0] * 57.295_78
    );
    let med_bin = median_bin(&s.hist);
    defmt::info!(
        "[STAT] phase A median |a|^2 = {} g^2 (bin {}/100)",
        med_bin as f32 / 100.0,
        med_bin
    );
    defmt::info!(
        "[STAT] aggressors: spinner_chunks={} dma_copies={} dma_copy_bad={}",
        SPIN_CHUNKS.load(Relaxed),
        DMA_COPIES.load(Relaxed),
        DMA_BAD.load(Relaxed)
    );
    let edges_b = b.e1.saturating_sub(b.e0);
    let elapsed_b = (b.us as f32) / 1e6;
    let lost_b = (WATERMARK as u64 * edges_b.saturating_sub(b.paused))
        .saturating_sub(u64::from(b.frames))
        .saturating_sub(WATERMARK as u64) as u32
        + b.no_edge;
    let rate_b = edges_b as f32 / elapsed_b.max(0.001);
    defmt::info!(
        "[STAT] integrity B: frames={} malformed={} timestamp_bad={} overflow={} fifo_count_max={} (limit {}, capacity {} frames) verify_fail={} lost={} late_edges={}",
        b.frames,
        b.malformed,
        b.ts_bad,
        b.fifo_over,
        b.fifo_count_max,
        FIFO_LIMIT,
        FIFO_CAPACITY,
        b.verify_fail,
        lost_b,
        b.late_edges
    );
    defmt::info!(
        "[STAT] verify B: n={} longest_us={} paused_max={} window_bad={}",
        b.ver.n,
        b.ver.longest_us,
        b.ver.max_paused,
        b.ver.bad
    );
    defmt::info!(
        "[STAT] B detail: bus_err={} no_edge_timeouts={} verifies={} edges={} edge_rate_hz={} paused_edges={} median_step_ticks={}",
        b.bus_err,
        b.no_edge,
        b.verifies,
        edges_b,
        rate_b,
        b.paused,
        b.median.map_or(0, u32::from)
    );
    b.ev_ts.print("timestamp");
    b.ev_count.print("fifo_count over limit");
    b.ev_late.print("late edge B");
    s.ev_missed.print("missed A");
    b.ev_ver.print("verify B");
    b.ev_bus.print("bus_err B");
    for (i, e) in b
        .ev_mal
        .iter()
        .enumerate()
        .take((b.malformed as usize).min(16))
    {
        defmt::info!(
            "[REC] malformed {} frame={} t_us={} edges={} prev|frame|next={}",
            i,
            e.idx,
            e.t_us,
            e.edges,
            &e.raw[..]
        );
    }
    s.ev_mis.print("mismatch");
    s.ev_id.print("identity");
    s.ev_ver.print("verify");
    s.ev_bus.print("bus_err");
    s.ev_stuck.print("stuck");
    for (i, sn) in s.snaps.iter().enumerate().take(s.nsnap as usize) {
        defmt::info!(
            "[REC] ring {} kind={} at_read={} (32 reads, oldest first)",
            i,
            sn.kind,
            sn.at_read
        );
        for v in sn.ring.iter() {
            defmt::info!("[REC]   {}", &v[..]);
        }
    }
    for (i, w) in s.wins.iter().enumerate().take((s.windows as usize).min(16)) {
        defmt::info!(
            "[REC] motion window {}: start_us={} len={} min_g2={} max_g2={}",
            i,
            w.start_us,
            w.len,
            w.min_g2,
            w.max_g2
        );
    }

    crit(&mut failures, s.mismatch == 0, "A mismatch = 0", s.mismatch);
    crit(
        &mut failures,
        s.skipped * 100 <= s.pairs,
        "A skipped <= 1 %",
        s.skipped,
    );
    crit(
        &mut failures,
        s.ident_wrong == 0,
        "A identity_wrong = 0",
        s.ident_wrong,
    );
    crit(
        &mut failures,
        s.verify_fail == 0,
        "A verify_fail = 0",
        s.verify_fail,
    );
    crit(&mut failures, s.bus_err == 0, "A bus_err = 0", s.bus_err);
    crit(&mut failures, lost_a == 0, "A lost = 0", lost_a);
    crit(&mut failures, s.missed == 0, "A missed = 0", s.missed);
    crit(
        &mut failures,
        s.ver.bad == 0,
        "A verify windows valid (<= 100 ms, paused <= duration x rate + 2)",
        s.ver.bad,
    );
    crit(&mut failures, s.stuck == 0, "A stuck = 0", s.stuck);
    crit(
        &mut failures,
        rate_a > EDGE_HZ as f32 * 0.95 && rate_a < EDGE_HZ as f32 * 1.05,
        "A edge rate within 5 % of 6400 Hz",
        rate_a as u32,
    );
    crit(
        &mut failures,
        (81..=121).contains(&med_bin),
        "median |a|^2 within 0.81..1.21 g^2 (bin/100)",
        med_bin as u32,
    );

    crit(
        &mut failures,
        b.malformed == 0,
        "B malformed = 0",
        b.malformed,
    );
    crit(
        &mut failures,
        b.ts_bad == 0,
        "B timestamp_bad = 0",
        b.ts_bad,
    );
    crit(
        &mut failures,
        b.fifo_over == 0 && b.fifo_count_max <= u32::from(FIFO_LIMIT),
        "B fifo_count_max <= 32 frames (never more than 4 watermarks)",
        b.fifo_count_max,
    );
    crit(
        &mut failures,
        b.ver.bad == 0,
        "B verify windows valid (<= 100 ms, paused <= duration x rate + 2)",
        b.ver.bad,
    );
    crit(
        &mut failures,
        b.pc_ok,
        "B positive control (fifo_count above limit and a timestamp step flagged)",
        b.pc_count,
    );
    crit(
        &mut failures,
        b.verify_fail == 0,
        "B verify_fail = 0",
        b.verify_fail,
    );
    crit(&mut failures, b.bus_err == 0, "B bus_err = 0", b.bus_err);
    crit(&mut failures, lost_b == 0, "B lost = 0", lost_b);
    crit(
        &mut failures,
        rate_b > EDGE_HZ as f32 / WATERMARK as f32 * 0.95
            && rate_b < EDGE_HZ as f32 / WATERMARK as f32 * 1.05
            && b.median.is_some(),
        "B edge rate within 5 % of 800 Hz and median step found",
        rate_b as u32,
    );

    // ---- default configuration in the smoke test's bus setting (8 MHz, mode 0)
    let mut cfg0 = spi::Config::default();
    cfg0.frequency = 8_000_000;
    {
        let bus = Spi::new(
            p.spi.reborrow(),
            p.clk.reborrow(),
            p.mosi.reborrow(),
            p.miso.reborrow(),
            p.d0.reborrow(),
            p.d1.reborrow(),
            Irqs,
            cfg0,
        );
        let cs = Output::new(p.cs.reborrow(), Level::High);
        let device = ExclusiveDevice::new_no_delay(bus, cs).unwrap();
        let mut imu = Icm45686::new(device);
        let config = Config::new().with_int1(Int1Config::new().with_data_ready(true));
        let init = imu.init(&mut Delay, &config).await;
        crit(&mut failures, init.is_ok(), "final init default", 0);
        let ver = imu.verify(&mut Delay).await;
        crit(&mut failures, ver.is_ok(), "final verify Ok", 0);
        let mut seen = dr.count();
        let (mut ok, mut plausible) = (0u32, 0u32);
        for _ in 0..200 {
            if dr
                .wait_beyond(seen, Duration::from_millis(500))
                .await
                .is_none()
            {
                break;
            }
            seen += 1;
            if let Ok(r) = imu.read().await {
                ok += 1;
                if (0.81..=1.21).contains(&g2_of(&r, config.accel_fs)) {
                    plausible += 1;
                }
            }
        }
        crit(&mut failures, ok == 200, "final 200 reads Ok", ok);
        crit(
            &mut failures,
            plausible >= 100,
            "final samples plausible (>= half)",
            plausible,
        );
    }

    FAILS.store(failures, Relaxed);
    DONE.store(true, Relaxed);
}

// ---------------------------------------------------------------------------------------------
// Aggressors, thread mode
// ---------------------------------------------------------------------------------------------

async fn yield_now() {
    let mut done = false;
    poll_fn(|cx| {
        if done {
            Poll::Ready(())
        } else {
            done = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
    .await
}

/// Arithmetic in chunks of about 1000 cycles, then yield.
#[embassy_executor::task]
async fn spinner() {
    let mut x = 12345u32;
    while !STOP.load(Relaxed) {
        for _ in 0..100 {
            x = black_box(x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223));
        }
        SPIN_CHUNKS.fetch_add(1, Relaxed);
        yield_now().await;
    }
    black_box(x);
}

static mut SRC: [u32; 1024] = [0; 1024];
static mut DST: [u32; 1024] = [0; 1024];

/// DMA memory-to-memory copy of 4 KiB, over and over, on a channel the driver does not use.
#[embassy_executor::task]
async fn dma_copier(mut ch: dma::Channel<'static>) {
    // SAFETY: SRC and DST are used by this task only.
    let (src, dst) = unsafe { (&mut *addr_of_mut!(SRC), &mut *addr_of_mut!(DST)) };
    for (i, v) in src.iter_mut().enumerate() {
        *v = (i as u32).wrapping_mul(0x9E37_79B1) ^ 0xA5A5_5A5A;
    }
    while !STOP.load(Relaxed) {
        // SAFETY: both slices are in SRAM, reachable by DMA, and live for the whole task.
        unsafe { ch.copy(&src[..], &mut dst[..]) }.await;
        let n = DMA_COPIES.fetch_add(1, Relaxed) + 1;
        if n % 256 == 0 {
            if src[..] != dst[..] {
                DMA_BAD.fetch_add(1, Relaxed);
            }
            dst.fill(0);
        }
    }
}

static EXEC_HI: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
unsafe fn SWI_IRQ_0() {
    // SAFETY: the executor was started on this interrupt.
    unsafe { EXEC_HI.on_interrupt() }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // SAFETY: only the DWT cycle counter is set up here; nothing else uses the core peripherals.
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    spawner.spawn(spinner().unwrap());
    let ch = dma::Channel::new(p.DMA_CH2, Irqs);
    spawner.spawn(dma_copier(ch).unwrap());

    let parts = Parts {
        spi: p.SPI0,
        clk: p.PIN_18,
        mosi: p.PIN_19,
        miso: p.PIN_16,
        cs: p.PIN_20,
        d0: p.DMA_CH0,
        d1: p.DMA_CH1,
        slice: p.PWM_SLICE7,
        int: p.PIN_15,
    };
    interrupt::SWI_IRQ_0.set_priority(Priority::P2);
    let hi = EXEC_HI.start(interrupt::SWI_IRQ_0);
    let start = Instant::now();
    hi.spawn(soak_task(parts).unwrap());

    let deadline = start + Duration::from_secs(u64::from(SOAK_SECONDS) + 30);
    let mut next = start + Duration::from_secs(u64::from(PROGRESS_EVERY));
    while !DONE.load(Relaxed) {
        Timer::after_millis(100).await;
        let now = Instant::now();
        if now >= next {
            next += Duration::from_secs(u64::from(PROGRESS_EVERY));
            let pairs = PAIRS.load(Relaxed);
            let lost = EDGES
                .load(Relaxed)
                .saturating_sub(pairs)
                .saturating_sub(PAUSED.load(Relaxed));
            if PHASE.load(Relaxed) == 1 {
                defmt::info!(
                    "[STAT] progress t={}s phase=A pairs={} mismatch={} skipped={} missed={} lost~{} motion_samples={}",
                    (now - start).as_secs(),
                    pairs,
                    MISMATCH.load(Relaxed),
                    SKIPPED.load(Relaxed),
                    MISSED.load(Relaxed),
                    lost,
                    MOTION.load(Relaxed)
                );
            } else {
                defmt::info!(
                    "[STAT] progress t={}s phase=B (A: pairs={} mismatch={} missed={}) B frames={} B bad={}",
                    (now - start).as_secs(),
                    pairs,
                    MISMATCH.load(Relaxed),
                    MISSED.load(Relaxed),
                    FRAMES.load(Relaxed),
                    BAD_B.load(Relaxed)
                );
            }
        }
        if now >= deadline {
            defmt::error!("[FAIL] soak did not finish within SOAK_SECONDS + 30 s");
            STOP.store(true, Relaxed);
            hil_common::exit(FAILS.load(Relaxed) + 1);
        }
    }
    STOP.store(true, Relaxed);
    hil_common::exit(FAILS.load(Relaxed))
}
