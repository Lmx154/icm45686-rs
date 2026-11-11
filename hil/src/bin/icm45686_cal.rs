//! On-board check of the user-offset registers, `set_power`, `soft_reset` and `recover` of the
//! icm45686 driver on the MARV V2 Rev 1 board (U21, SPI0, INT1 on GPIO15). The board lies still.
#![no_std]
#![no_main]

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::task::Poll;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_15, PWM_SLICE7};
use embassy_rp::pwm::{self, InputMode, Pwm};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use icm45686::{ConfigError, Config, Error, Icm45686, Int1Config, Offsets, SensorPower};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const STALL: Duration = Duration::from_millis(100);
const N: u32 = 400;
const DISCARD: u32 = 50;
/// No new check starts after this; the cleanup and the verdict follow.
const DEADLINE: Duration = Duration::from_secs(48);

/// Limits for "unchanged" and "no cross-talk". They come from the bench, measured on the board,
/// 2026-11-10: the gyroscope mean wanders by about +-0.1 dps over seconds and the accelerometer bias
/// moves a few mg in the first seconds after `init`. 3 mg and 0.3 dps are that wander with margin.
/// Cross-talk is judged by half the difference of the +step and -step means, in which a slow
/// wander cancels, and "unchanged" against a reference taken immediately before the step, so the
/// limits only have to cover what wanders within a few seconds.
const NOISE_A_MG: f32 = 3.0;
const NOISE_G_DPS: f32 = 0.3;
/// Allowed deviation of a step from its expected size: 3 %. The expected sizes follow the scale
/// measured on the board, 2026-11-10: 1 g / 8192 per LSB (800 LSB = 97.66 mg) and 62.5 dps / 8192
/// per LSB (1333 LSB = 10.17 dps).
const STEP_WINDOW: f32 = 0.03;
const ACCEL_MG_PER_LSB: f32 = 1000.0 / 8192.0;
const GYRO_DPS_PER_LSB: f32 = 62.5 / 8192.0;

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

    /// Waits until the hardware edge count exceeds `seen`; `None` after `timeout`. The timeout
    /// wraps only the wait on the pin, never a driver call.
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

/// Means (and standard deviations) of N register samples, in mg and dps, and in raw LSB.
#[derive(Clone, Copy)]
struct Meas {
    a: [f32; 3],
    g: [f32; 3],
    a_sd: [f32; 3],
    g_sd: [f32; 3],
    a_raw: [f32; 3],
    g_raw: [f32; 3],
    temp_c: f32,
    errors: u32,
    stalls: u32,
}

fn sqrt(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let mut r = if x > 1.0 { x } else { 1.0 };
    for _ in 0..30 {
        r = 0.5 * (r + x / r);
    }
    r
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn within(d: [f32; 3], skip: Option<usize>, limit: f32) -> bool {
    (0..3).all(|i| Some(i) == skip || d[i].abs() <= limit)
}

struct Ctx<'d, SPI: SpiDevice> {
    imu: Icm45686<SPI>,
    dr: DataReady<'d>,
    delay: Delay,
    failures: u32,
    cfg: Config,
    t0: Instant,
}

impl<SPI: SpiDevice> Ctx<'_, SPI> {
    fn verdict(&mut self, ok: bool, name: &str) {
        if ok {
            defmt::info!("[PASS] {=str}", name);
        } else {
            self.failures += 1;
            defmt::error!("[FAIL] {=str}", name);
        }
    }

    fn time_left(&self) -> bool {
        self.t0.elapsed() < DEADLINE
    }

    fn skipped(&mut self, name: &str) {
        self.failures += 1;
        defmt::error!("[FAIL] {=str}: NOT RUN (out of time)", name);
    }

    /// Waits for the next data-ready edge (bounded; on a stall waits 2 ms), then reads.
    async fn next(&mut self, seen: &mut u64) -> Result<icm45686::RawSample, bool> {
        match self.dr.wait_beyond(*seen, STALL).await {
            Some(c) => *seen = c,
            None => Timer::after(Duration::from_millis(2)).await,
        }
        self.imu.read().await.map_err(|_| false)
    }

    /// Discards `discard` samples, then averages N.
    async fn measure(&mut self, discard: u32) -> Meas {
        let mut seen = self.dr.count();
        let mut sum = [0i64; 7];
        let mut sq = [0i64; 7];
        let mut errors = 0;
        let mut stalls = 0;
        let mut n = 0i64;
        for i in 0..discard + N {
            let before = seen;
            let r = self.next(&mut seen).await;
            if seen == before {
                stalls += 1;
            }
            match r {
                Ok(s) => {
                    if i >= discard {
                        n += 1;
                        for (k, v) in s.accel.iter().chain(&s.gyro).chain(core::iter::once(&s.temp)).enumerate() {
                            let v = i64::from(*v);
                            sum[k] += v;
                            sq[k] += v * v;
                        }
                    }
                }
                Err(_) => errors += 1,
            }
        }
        let n = n.max(1);
        let a_k = 1000.0 / self.cfg.accel_fs.lsb_per_g();
        let g_k = 1.0 / self.cfg.gyro_fs.lsb_per_dps();
        let mean = |k: usize| sum[k] as f32 / n as f32;
        let sd = |k: usize| sqrt((n * sq[k] - sum[k] * sum[k]) as f32 / (n * n) as f32);
        Meas {
            a: [mean(0) * a_k, mean(1) * a_k, mean(2) * a_k],
            g: [mean(3) * g_k, mean(4) * g_k, mean(5) * g_k],
            a_sd: [sd(0) * a_k, sd(1) * a_k, sd(2) * a_k],
            g_sd: [sd(3) * g_k, sd(4) * g_k, sd(5) * g_k],
            a_raw: [mean(0), mean(1), mean(2)],
            g_raw: [mean(3), mean(4), mean(5)],
            temp_c: mean(6) / 128.0 + 25.0,
            errors,
            stalls,
        }
    }

    async fn verify(&mut self, name: &str) {
        let r = self.imu.verify(&mut self.delay).await;
        if let Err(e) = &r {
            defmt::error!("verify error: {}", defmt::Debug2Format(e));
        }
        self.verdict(r.is_ok(), name);
    }

    /// `set_offsets`, then `offsets()` and `verify()` checks, then the means.
    async fn set(&mut self, o: &Offsets, name: &str) -> Meas {
        let r = self.imu.set_offsets(&mut self.delay, o).await;
        if let Err(e) = &r {
            defmt::error!("set_offsets error: {}", defmt::Debug2Format(e));
        }
        self.verdict(r.is_ok(), name);
        let same = self.imu.offsets() == Some(*o);
        self.verdict(same, "offsets() returns what was set");
        self.verify("verify Ok after set_offsets").await;
        self.measure(DISCARD).await
    }

    async fn init(&mut self, name: &str) {
        let cfg = self.cfg;
        let r = self.imu.init(&mut self.delay, &cfg).await;
        if let Err(e) = &r {
            defmt::error!("init error: {}", defmt::Debug2Format(e));
        }
        self.verdict(r.is_ok(), name);
    }

    /// One axis, +raw, -raw, 0 (checks 2 and 3). `zero` is the last measurement taken with all
    /// offsets zero, immediately before this call; it is the reference of every difference here and
    /// is replaced by this call's own last measurement (offsets zero again).
    async fn axis(&mut self, zero: &mut Meas, gyro: bool, axis: usize, raw: i16, nominal: f32) {
        let mk = |v: i16| {
            let mut r = [0i16; 3];
            r[axis] = v;
            if gyro {
                Offsets::new().with_gyro(r)
            } else {
                Offsets::new().with_accel(r)
            }
        };
        let unit = if gyro { "dps" } else { "mg" };
        let name = match (gyro, axis) {
            (false, 0) => "accel X",
            (false, 1) => "accel Y",
            (false, 2) => "accel Z",
            (true, 0) => "gyro X",
            (true, 1) => "gyro Y",
            _ => "gyro Z",
        };
        let mut d = [([0.0f32; 3], [0.0f32; 3]); 3];
        for (i, v) in [raw, -raw, 0].into_iter().enumerate() {
            let m = self.set(&mk(v), "set_offsets Ok").await;
            d[i] = (sub(m.a, zero.a), sub(m.g, zero.g));
            if i == 2 {
                *zero = m;
            }
            defmt::info!(
                "[STAT] {=str} offset {=i16}: dAccel_mg={} dGyro_dps={} stalls={} errors={}",
                name,
                v,
                d[i].0,
                d[i].1,
                m.stalls,
                m.errors
            );
        }
        let moved = |x: &([f32; 3], [f32; 3])| if gyro { x.1[axis] } else { x.0[axis] };
        let (cp, cn) = (moved(&d[0]), moved(&d[1]));
        let sign = if cp > 0.0 { "RAISES" } else { "LOWERS" };
        defmt::info!(
            "[STAT] {=str}: +{=i16} moves {=f32} {=str}, -{=i16} moves {=f32} {=str}: a positive raw offset {=str} the output (scale {=f32} per LSB x1000)",
            name,
            raw,
            cp,
            unit,
            raw,
            cn,
            unit,
            sign,
            (cp - cn) / (2.0 * f32::from(raw)) * 1000.0
        );
        let skip_a = if gyro { None } else { Some(axis) };
        let skip_g = if gyro { Some(axis) } else { None };
        // A step is within 3 % of nominal, plus the wander allowance because a single side is judged
        // against one reference; the half difference of the two sides, in which the wander cancels,
        // is within 3 % alone.
        let allow = if gyro { NOISE_G_DPS } else { NOISE_A_MG };
        let side = |c: f32, sign: f32| (c - sign * nominal).abs() <= STEP_WINDOW * nominal + allow;
        let half = (cp - cn) / 2.0;
        let cross = [
            [(d[0].0[0] - d[1].0[0]) / 2.0, (d[0].0[1] - d[1].0[1]) / 2.0, (d[0].0[2] - d[1].0[2]) / 2.0],
            [(d[0].1[0] - d[1].1[0]) / 2.0, (d[0].1[1] - d[1].1[1]) / 2.0, (d[0].1[2] - d[1].1[2]) / 2.0],
        ];
        defmt::info!(
            "[STAT] {=str}: half the difference of the + and - steps: changed axis {=f32} {=str} (expected {=f32}); cross-talk dAccel_mg={} dGyro_dps={}",
            name,
            half,
            unit,
            nominal,
            cross[0],
            cross[1]
        );
        self.verdict(
            (half - nominal).abs() <= STEP_WINDOW * nominal,
            "changed axis: half the difference of the two steps is the expected size within 3 %",
        );
        self.verdict(
            side(cp, 1.0) && side(cn, -1.0),
            "changed axis: +raw raises and -raw lowers the output by the expected size",
        );
        self.verdict(cp * cn < 0.0, "-raw moves the opposite way");
        self.verdict(
            within(cross[0], skip_a, NOISE_A_MG) && within(cross[1], skip_g, NOISE_G_DPS),
            "other axes: half the difference of the two steps within the wander limit",
        );
        self.verdict(
            within(d[2].0, None, NOISE_A_MG) && within(d[2].1, None, NOISE_G_DPS),
            "back at the reference taken before the step after 0",
        );
    }

    /// Check 4 on one sensor's X axis.
    async fn extremes(&mut self, base: &Meas, zero: &mut Meas, gyro: bool) {
        let (unit, lsb_out, step, full_pos, full_neg) = if gyro {
            (
                "dps",
                self.cfg.gyro_fs.lsb_per_dps(),
                GYRO_DPS_PER_LSB,
                8191.0 * GYRO_DPS_PER_LSB,
                8192.0 * GYRO_DPS_PER_LSB,
            )
        } else {
            (
                "mg",
                self.cfg.accel_fs.lsb_per_g() / 1000.0,
                ACCEL_MG_PER_LSB,
                8191.0 * ACCEL_MG_PER_LSB,
                8192.0 * ACCEL_MG_PER_LSB,
            )
        };
        let mut dx = [0.0f32; 4];
        for (i, v) in [1i16, -1, 8191, -8192].into_iter().enumerate() {
            let o = if gyro {
                Offsets::new().with_gyro([v, 0, 0])
            } else {
                Offsets::new().with_accel([v, 0, 0])
            };
            let m = self.set(&o, "set_offsets Ok").await;
            dx[i] = if gyro { m.g[0] - zero.g[0] } else { m.a[0] - zero.a[0] };
            let sd = if gyro { base.g_sd[0] } else { base.a_sd[0] };
            defmt::info!(
                "[STAT] {=str} X offset {=i16}: change {=f32} {=str} = {=f32} output LSB (baseline sd {=f32}, sd of mean {=f32})",
                if gyro { "gyro" } else { "accel" },
                v,
                dx[i],
                unit,
                dx[i] * lsb_out,
                sd,
                sd / 20.0
            );
        }
        let sd_mean = if gyro { base.g_sd[0] } else { base.a_sd[0] } / 20.0;
        if step > 5.0 * sd_mean {
            self.verdict(
                dx[0] * dx[1] < 0.0 && (dx[0] + dx[1]).abs() < step,
                "-1 moves one step the opposite way of +1",
            );
        } else {
            defmt::info!(
                "[SKIP] +-1 not resolvable: nominal step {=f32} {=str} <= 5 x sd of mean {=f32}",
                step,
                unit,
                sd_mean
            );
        }
        let near = |d: f32, nom: f32| (d.abs() / nom - 1.0).abs() <= 0.10;
        self.verdict(
            dx[2] * dx[3] < 0.0 && near(dx[2], full_pos) && near(dx[3], full_neg),
            "+8191 and -8192 move the output in opposite directions by the full-range amounts within 10 %",
        );
        let m = self.set(&Offsets::new(), "set_offsets back to 0 Ok").await;
        let back = if gyro { m.g[0] - zero.g[0] } else { m.a[0] - zero.a[0] };
        self.verdict(
            back.abs() <= if gyro { NOISE_G_DPS } else { NOISE_A_MG },
            "back at the reference taken before the steps",
        );
        *zero = m;
    }

    /// First 100 raw samples after a power change: the first 20 per axis, then block means of 20.
    async fn first20(&mut self, gyro: bool) {
        let mut seen = self.dr.count();
        let mut x = [[0i16; 100]; 3];
        for i in 0..100 {
            if let Ok(s) = self.next(&mut seen).await {
                let v = if gyro { s.gyro } else { s.accel };
                for k in 0..3 {
                    x[k][i] = v[k];
                }
            }
        }
        let name = if gyro { "gyro" } else { "accel" };
        defmt::info!(
            "[STAT] first 20 {=str} samples after power on, raw X={} Y={} Z={}",
            name,
            &x[0][..20],
            &x[1][..20],
            &x[2][..20]
        );
        for b in 1..5 {
            let m = |k: usize| {
                x[k][b * 20..b * 20 + 20].iter().map(|v| i32::from(*v)).sum::<i32>() as f32 / 20.0
            };
            defmt::info!(
                "[STAT] {=str} after power on, mean raw of samples {}..{}: {} {} {}",
                name,
                b * 20 + 1,
                b * 20 + 20,
                m(0),
                m(1),
                m(2)
            );
        }
    }

    /// Check 6 for one sensor.
    async fn power_cycle(&mut self, base: &Meas, gyro: bool) {
        let name = if gyro { "gyro" } else { "accel" };
        let (off_a, off_g) = if gyro {
            (SensorPower::LowNoise, SensorPower::Off)
        } else {
            (SensorPower::Off, SensorPower::LowNoise)
        };
        let t = Instant::now();
        let r = self.imu.set_power(&mut self.delay, off_a, off_g).await;
        defmt::info!("[STAT] {=str} off: set_power took {=u64} ms", name, t.elapsed().as_millis());
        if let Err(e) = &r {
            defmt::error!("set_power error: {}", defmt::Debug2Format(e));
        }
        self.verdict(r.is_ok(), "set_power off Ok");
        if let Ok(i) = r {
            defmt::info!("[STAT] info after off: accel_odr_mhz={} gyro_odr_mhz={}", i.accel_odr_mhz, i.gyro_odr_mhz);
        }
        self.verify("verify Ok with the sensor off").await;
        let m = self.measure(DISCARD).await;
        if gyro {
            defmt::info!(
                "[STAT] gyro registers while off: mean raw={} sd raw={} (dps sd {}) stalls={} errors={}",
                m.g_raw,
                [m.g_sd[0] * self.cfg.gyro_fs.lsb_per_dps(), m.g_sd[1] * self.cfg.gyro_fs.lsb_per_dps(), m.g_sd[2] * self.cfg.gyro_fs.lsb_per_dps()],
                m.g_sd,
                m.stalls,
                m.errors
            );
            self.verdict(within(sub(m.a, base.a), None, NOISE_A_MG), "accel unchanged while gyro off");
        } else {
            defmt::info!(
                "[STAT] accel registers while off: mean raw={} sd mg={} stalls={} errors={}",
                m.a_raw,
                m.a_sd,
                m.stalls,
                m.errors
            );
            self.verdict(within(sub(m.g, base.g), None, NOISE_G_DPS), "gyro unchanged while accel off");
        }
        let t = Instant::now();
        let r = self
            .imu
            .set_power(&mut self.delay, SensorPower::LowNoise, SensorPower::LowNoise)
            .await;
        defmt::info!("[STAT] {=str} on: set_power took {=u64} ms", name, t.elapsed().as_millis());
        if let Err(e) = &r {
            defmt::error!("set_power error: {}", defmt::Debug2Format(e));
        }
        self.verdict(r.is_ok(), "set_power on Ok");
        self.first20(gyro).await;
        let m = self.measure(0).await;
        let d = sub(m.a, base.a);
        let dg = sub(m.g, base.g);
        defmt::info!(
            "[STAT] {=str} on, next {=u32} samples vs baseline: dAccel_mg={} dGyro_dps={} stalls={} errors={}",
            name,
            N,
            d,
            dg,
            m.stalls,
            m.errors
        );
        self.verdict(
            within(d, None, NOISE_A_MG) && within(dg, None, NOISE_G_DPS),
            "mean back at baseline within noise",
        );
        for k in 0..6u32 {
            let t = self.measure(0).await;
            defmt::info!(
                "[STAT] {=str} on, tail block {=u32} (0.5 s each, from about 0.6 s after power on): mean raw accel={} gyro={} temp_c={}",
                name,
                k + 1,
                t.a_raw,
                t.g_raw,
                t.temp_c
            );
        }
        self.verify("verify Ok after power on").await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut spi_config = spi::Config::default();
    spi_config.frequency = SPI_HZ;
    let dr = DataReady::new(p.PWM_SLICE7.reborrow(), p.PIN_15.reborrow());

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
    let cfg = Config::new().with_int1(Int1Config::new().with_data_ready(true));
    let mut c = Ctx {
        imu: Icm45686::new(device),
        dr,
        delay: Delay,
        failures: 0,
        cfg,
        t0: Instant::now(),
    };

    // 1. Baseline.
    defmt::info!("[INFO] check 1: baseline");
    c.init("init with the default configuration").await;
    c.verify("verify Ok after init").await;
    let cold = c.measure(DISCARD).await;
    defmt::info!("[STAT] just after init: temp_c={} accel_mg={} gyro_dps={}", cold.temp_c, cold.a, cold.g);
    Timer::after(Duration::from_secs(15)).await;
    let base = c.measure(DISCARD).await;
    defmt::info!("[STAT] after 15 s warm-up: temp_c={}", base.temp_c);
    defmt::info!(
        "[STAT] baseline accel_mg={} gyro_dps={} accel_sd_mg={} gyro_sd_dps={} stalls={} errors={}",
        base.a,
        base.g,
        base.a_sd,
        base.g_sd,
        base.stalls,
        base.errors
    );
    c.verdict(base.stalls == 0 && base.errors == 0, "baseline: no stalls, no read errors");
    // The last measurement with all offsets zero; checks 2 to 4 judge each step against it.
    let mut zero = base;

    // 2. Accel offsets.
    defmt::info!("[INFO] check 2: accel offsets");
    for axis in 0..3 {
        if c.time_left() {
            c.axis(&mut zero, false, axis, 800, 800.0 * ACCEL_MG_PER_LSB).await;
        } else {
            c.skipped("check 2");
        }
    }

    // 3. Gyro offsets.
    defmt::info!("[INFO] check 3: gyro offsets");
    for axis in 0..3 {
        if c.time_left() {
            c.axis(&mut zero, true, axis, 1333, 1333.0 * GYRO_DPS_PER_LSB).await;
        } else {
            c.skipped("check 3");
        }
    }

    // 4. Two's complement and the field edges.
    defmt::info!("[INFO] check 4: field edges");
    for gyro in [false, true] {
        if c.time_left() {
            c.extremes(&base, &mut zero, gyro).await;
        } else {
            c.skipped("check 4");
        }
    }

    // 5. configure keeps the offsets, init clears them.
    defmt::info!("[INFO] check 5: configure and init");
    if c.time_left() {
        let o = Offsets::new().with_accel([800, 0, 0]).with_gyro([1333, 0, 0]);
        let m1 = c.set(&o, "set_offsets Ok").await;
        let cfg = c.cfg;
        let r = c.imu.configure(&mut c.delay, &cfg).await;
        if let Err(e) = &r {
            defmt::error!("configure error: {}", defmt::Debug2Format(e));
        }
        c.verdict(r.is_ok(), "configure Ok");
        c.verdict(c.imu.offsets() == Some(o), "offsets() unchanged after configure");
        c.verify("verify Ok after configure").await;
        let m2 = c.measure(DISCARD).await;
        let (da, dg) = (sub(m2.a, m1.a), sub(m2.g, m1.g));
        let (aa, ag) = (sub(m2.a, base.a), sub(m2.g, base.g));
        defmt::info!(
            "[STAT] after configure vs before: dAccel_mg={} dGyro_dps={}; vs baseline: dAccel_mg={} dGyro_dps={}",
            da,
            dg,
            aa,
            ag
        );
        c.verdict(
            within(da, None, NOISE_A_MG) && within(dg, None, NOISE_G_DPS),
            "mean unchanged by configure (offset still acts)",
        );
        c.verdict((aa[0].abs() / 100.0 - 1.0).abs() <= 0.10, "accel X offset still acts after configure");
        c.init("init Ok").await;
        c.verdict(c.imu.offsets() == Some(Offsets::new()), "offsets() zero after init");
        c.verify("verify Ok after init").await;
        let m3 = c.measure(DISCARD).await;
        let (ba, bg) = (sub(m3.a, base.a), sub(m3.g, base.g));
        defmt::info!("[STAT] after init vs baseline: dAccel_mg={} dGyro_dps={}", ba, bg);
        c.verdict(
            within(ba, None, NOISE_A_MG) && within(bg, None, NOISE_G_DPS),
            "mean back at baseline after init",
        );
    } else {
        c.skipped("check 5");
    }

    // 6. set_power.
    defmt::info!("[INFO] check 6: set_power");
    if c.time_left() {
        c.power_cycle(&base, true).await;
    } else {
        c.skipped("check 6 gyro");
    }
    if c.time_left() {
        c.power_cycle(&base, false).await;
    } else {
        c.skipped("check 6 accel");
    }
    if c.time_left() {
        let r = c
            .imu
            .set_power(&mut c.delay, SensorPower::Standby, SensorPower::LowNoise)
            .await;
        defmt::info!("[STAT] set_power(accel Standby) returned {}", defmt::Debug2Format(&r));
        c.verdict(
            matches!(r, Err(Error::InvalidConfig(ConfigError::AccelStandby))),
            "invalid request returns InvalidConfig(AccelStandby)",
        );
        let cfg = c.cfg;
        c.verdict(c.imu.config() == Some(&cfg), "driver still configured with the same Config");
        let ok = c.imu.read().await.is_ok();
        c.verdict(ok, "read Ok after the invalid request");
        c.verify("verify Ok after the invalid request").await;
    } else {
        c.skipped("check 6 invalid");
    }

    // 7. soft_reset.
    defmt::info!("[INFO] check 7: soft_reset");
    if c.time_left() {
        let o = Offsets::new().with_accel([800, 0, 0]).with_gyro([1333, 0, 0]);
        let r = c.imu.set_offsets(&mut c.delay, &o).await;
        c.verdict(r.is_ok(), "set_offsets Ok before soft_reset");
        let r = c.imu.soft_reset(&mut c.delay).await;
        if let Err(e) = &r {
            defmt::error!("soft_reset error: {}", defmt::Debug2Format(e));
        }
        c.verdict(r.is_ok(), "soft_reset Ok");
        let nc = matches!(c.imu.read().await, Err(Error::NotConfigured));
        c.verdict(nc, "read returns NotConfigured after soft_reset");
        c.verdict(c.imu.offsets().is_none(), "offsets() is None after soft_reset");
        // Read-only look at what the device holds after the reset.
        let mut raw = [0u8; 4];
        for (k, a) in [
            icm45686::reg::ACCEL_X_OFFUSER,
            icm45686::reg::ACCEL_X_OFFUSER + 1,
            icm45686::reg::GYRO_X_OFFUSER,
            icm45686::reg::GYRO_X_OFFUSER + 1,
        ]
        .into_iter()
        .enumerate()
        {
            if let Ok(v) = c.imu.ireg_read(&mut c.delay, a).await {
                raw[k] = v;
            } else {
                raw[k] = 0xEE;
            }
        }
        defmt::info!(
            "[STAT] device offset registers right after soft_reset (accel X lo,hi gyro X lo,hi; set before: 0x20,0x03 0x35,0x05): {=[u8]:x}",
            raw
        );
        c.verdict(raw == [0; 4], "soft reset cleared the offset registers on the device");
        c.init("init works after soft_reset").await;
        c.verify("verify Ok after init").await;
        c.verdict(c.imu.offsets() == Some(Offsets::new()), "offsets() zero after init");
    } else {
        c.skipped("check 7");
    }

    // 8. recover.
    defmt::info!("[INFO] check 8: recover");
    if c.time_left() {
        let t = Instant::now();
        let r = c.imu.recover(&mut c.delay).await;
        defmt::info!("[STAT] recover on a healthy bus took {=u64} us", t.elapsed().as_micros());
        if let Err(e) = &r {
            defmt::error!("recover error: {}", defmt::Debug2Format(e));
        }
        c.verdict(r.is_ok(), "recover Ok on a healthy device");
        let r = c.imu.soft_reset(&mut c.delay).await;
        c.verdict(r.is_ok(), "soft_reset Ok");
        let t = Instant::now();
        let r = c.imu.recover(&mut c.delay).await;
        defmt::info!("[STAT] recover when not configured took {=u64} us", t.elapsed().as_micros());
        c.verdict(
            matches!(r, Err(Error::NotConfigured)),
            "recover returns NotConfigured when not configured",
        );
        c.init("init Ok after recover").await;
        c.verify("verify Ok after init").await;
    } else {
        c.skipped("check 8");
    }

    // 9. Extra: does the offset scale follow the full scale? (Information only, no verdict.)
    defmt::info!("[INFO] check 9: offset scale at accel 16 g and gyro 250 dps");
    if c.time_left() {
        let cfg1 = c.cfg;
        let cfg2 = cfg1
            .with_accel_fs(icm45686::AccelFullScale::G16)
            .with_gyro_fs(icm45686::GyroFullScale::Dps250);
        c.cfg = cfg2;
        let r = c.imu.configure(&mut c.delay, &cfg2).await;
        c.verdict(r.is_ok(), "configure to 16 g / 250 dps Ok");
        let b2 = c.measure(DISCARD).await;
        let o = Offsets::new().with_accel([2000, 0, 0]).with_gyro([2000, 0, 0]);
        let m = c.set(&o, "set_offsets Ok at 16 g / 250 dps").await;
        let (da, dg) = (sub(m.a, b2.a), sub(m.g, b2.g));
        defmt::info!(
            "[STAT] at 16 g / 250 dps, offsets +2000 on X: dAccel_mg={} dGyro_dps={} => accel {=f32} ug per LSB, gyro {=f32} mdps per LSB (at 4 g / 2000 dps: 122.1 ug, 7.63 mdps)",
            da,
            dg,
            da[0] / 2.0,
            dg[0] / 2.0
        );
        let _ = c.set(&Offsets::new(), "set_offsets back to 0 Ok").await;
        c.cfg = cfg1;
    } else {
        c.skipped("check 9");
    }

    // Cleanup on every path: init writes the offsets to zero.
    defmt::info!("[INFO] cleanup");
    c.init("cleanup init Ok").await;
    c.verdict(c.imu.offsets() == Some(Offsets::new()), "cleanup: offsets zero");
    c.verify("cleanup: verify Ok").await;
    defmt::info!("[STAT] run took {=u64} ms", c.t0.elapsed().as_millis());
    hil_common::exit(c.failures)
}
