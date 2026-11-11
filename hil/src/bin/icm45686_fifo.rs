//! On-board check of the icm45686 FIFO and `verify()` on the MARV V2 Rev 1 board (U21, SPI0, INT1 on
//! GPIO15). Settles the facts the host tests could only assume. No SPI transfer is ever wrapped in a
//! timeout or cancelled: only the wait for an INT1 edge has a timeout.
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
use icm45686::fifo::{
    hires_accel_ms2, hires_gyro_rads, temperature_c_8bit, temperature_c_16bit, timestamp_us,
};
use icm45686::{
    AccelFullScale, ClockSource, Config, Error, FifoConfig, FifoDepth, FifoFormat, FifoFrame,
    FifoMode, GyroFullScale, Icm45686, Int1Config, Odr, RawSample, Space, fifo_frames,
};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const PERIOD_US: u32 = 1250; // 800 Hz
const G: f32 = 9.80665;
const RAD_TO_DEG: f32 = 57.295_78;

/// INT1 used twice: a PWM slice counts every rising edge in hardware, the GPIO edge interrupt wakes the
/// reader (same as icm45686_smoke).
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

    /// Waits until the hardware edge count exceeds `seen`; returns the count, or `None` after `timeout`.
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
    defmt::error!(
        "[FAIL] {=str}: {}",
        what,
        defmt::Debug2Format(e)
    );
}

fn cfg_a() -> Config {
    Config::new()
}

fn cfg_b() -> Config {
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_gyro_fs(GyroFullScale::Dps250)
        .with_int1(Int1Config::new().with_data_ready(true))
        .with_accel_odr(Odr::Hz200)
        .with_gyro_odr(Odr::Hz200)
        .with_clock(ClockSource::ExternalClkin)
}

fn cfg_c(depth: FifoDepth, format: FifoFormat) -> Config {
    Config::new()
        .with_accel_fs(AccelFullScale::G4)
        .with_gyro_fs(GyroFullScale::Dps250)
        .with_int1(Int1Config::new().with_fifo_threshold(true))
        .with_fifo(
            FifoConfig::new()
                .with_mode(FifoMode::Stream)
                .with_format(format)
                .with_accel(true)
                .with_gyro(true)
                .with_timestamp(true)
                .with_watermark(8)
                .with_depth(depth),
        )
}

async fn apply<SPI: SpiDevice>(
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
    match r {
        Ok(_) => true,
        Err(e) => {
            bus_fail(failures, name, &e);
            false
        }
    }
}

async fn verify_ok<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32, name: &str) {
    match imu.verify(&mut Delay).await {
        Ok(()) => verdict(failures, true, name),
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
            verdict(failures, false, name);
        }
        Err(e) => {
            bus_fail(failures, name, &e);
        }
    }
}

/// Result of decoding one FIFO read.
struct Dec {
    n: u32,
    valid: u32,
    other: u32,
    dn: u32,
    dmin: u32,
    dmax: u32,
    dsum: u64,
    bad_delta: u32,
    first_bad: i32,
    a2_min: f32,
    a2_max: f32,
    a2_sum: f32,
    gmax_dps: f32,
    temp_sum: f32,
    last_ts: Option<u16>,
}

/// Decodes `buf` against `config`; `prev` is the timestamp of the frame before the first one.
fn analyse(buf: &[u8], config: &Config, prev: Option<u16>) -> Dec {
    let mut d = Dec {
        n: 0,
        valid: 0,
        other: 0,
        dn: 0,
        dmin: u32::MAX,
        dmax: 0,
        dsum: 0,
        bad_delta: 0,
        first_bad: -1,
        a2_min: f32::MAX,
        a2_max: 0.0,
        a2_sum: 0.0,
        gmax_dps: 0.0,
        temp_sum: 0.0,
        last_ts: prev,
    };
    let res = config.fifo.timestamp_resolution;
    for fr in fifo_frames(buf, &config.fifo) {
        d.n += 1;
        let (ts, a, g, t) = match fr {
            FifoFrame::AccelGyro(f) => {
                let s = RawSample {
                    accel: f.accel,
                    gyro: f.gyro,
                    temp: 0,
                };
                (
                    f.timestamp,
                    s.accel_ms2(config.accel_fs),
                    s.gyro_rads(config.gyro_fs),
                    temperature_c_8bit(f.temp),
                )
            }
            FifoFrame::HighRes(f) => (
                f.timestamp,
                hires_accel_ms2(f.accel),
                hires_gyro_rads(f.gyro),
                temperature_c_16bit(f.temp),
            ),
            _ => {
                d.other += 1;
                continue;
            }
        };
        d.valid += 1;
        if let Some(p) = d.last_ts {
            let us = timestamp_us(ts.wrapping_sub(p), res);
            d.dn += 1;
            d.dmin = d.dmin.min(us);
            d.dmax = d.dmax.max(us);
            d.dsum += u64::from(us);
            if us * 100 < PERIOD_US * 95 || us * 100 > PERIOD_US * 105 {
                d.bad_delta += 1;
                if d.first_bad < 0 {
                    d.first_bad = d.n as i32 - 1;
                }
            }
        }
        d.last_ts = Some(ts);
        let a2 = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (G * G);
        d.a2_min = d.a2_min.min(a2);
        d.a2_max = d.a2_max.max(a2);
        d.a2_sum += a2;
        for v in g {
            d.gmax_dps = d.gmax_dps.max(v.abs() * RAD_TO_DEG);
        }
        d.temp_sum += t;
    }
    d
}

impl Dec {
    fn mean_delta_us(&self) -> f32 {
        self.dsum as f32 / self.dn.max(1) as f32
    }

    fn mean_temp(&self) -> f32 {
        self.temp_sum / self.valid.max(1) as f32
    }

    fn print(&self, name: &str) {
        defmt::info!(
            "[STAT] {=str} frames={} valid={} other={} deltas={} delta_us min={} max={} mean={} bad_deltas={} first_bad_frame={} accel_g2 min={} max={} mean={} gyro_max_dps={} fifo_temp_c_mean={}",
            name,
            self.n,
            self.valid,
            self.other,
            self.dn,
            if self.dn == 0 { 0 } else { self.dmin },
            self.dmax,
            self.mean_delta_us(),
            self.bad_delta,
            self.first_bad,
            self.a2_min,
            self.a2_max,
            self.a2_sum / self.valid.max(1) as f32,
            self.gmax_dps,
            self.mean_temp()
        );
    }

    /// The common PASS/FAIL set of checks 4 and 5 (frames, gravity, gyro).
    fn judge(&self, failures: &mut u32, prefix: &str, min_frames: u32) {
        let _ = prefix;
        verdict(failures, self.n >= min_frames, "enough frames read");
        verdict(
            failures,
            self.n > 0 && self.valid == self.n && self.other == 0,
            "every frame is a valid accel + gyro frame",
        );
        verdict(
            failures,
            self.dn > 0 && self.bad_delta == 0,
            "timestamp deltas equal the output period within 5 %",
        );
        verdict(
            failures,
            self.a2_min > 0.81 && self.a2_max < 1.21,
            "accel magnitude is 1 g within 10 %",
        );
        verdict(failures, self.gmax_dps <= 5.0, "gyro near zero at rest");
    }
}

async fn reg_temp_c<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32) -> f32 {
    match imu.read().await {
        Ok(s) => s.temperature_c(),
        Err(e) => {
            bus_fail(failures, "register read()", &e);
            f32::NAN
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Check 1 and 2: verify()
// ---------------------------------------------------------------------------------------------

async fn check1<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32) {
    defmt::info!("[INFO] check 1: verify() on the real chip");
    let a = cfg_a();
    let b = cfg_b();
    let c = cfg_c(FifoDepth::Kib2, FifoFormat::Standard);
    for (name, config, reset) in [
        ("init a default -> verify Ok", &a, true),
        ("init b external clock -> verify Ok", &b, true),
        ("init c fifo -> verify Ok", &c, true),
        ("configure c->a -> verify Ok", &a, false),
        ("configure a->b -> verify Ok", &b, false),
        ("configure b->c -> verify Ok", &c, false),
        ("configure c->a (2) -> verify Ok", &a, false),
        ("configure a->c -> verify Ok", &c, false),
    ] {
        if apply(imu, failures, name, config, reset).await {
            verify_ok(imu, failures, name).await;
        }
    }
}

async fn check2<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32) {
    defmt::info!("[INFO] check 2: positive control for verify()");
    let a = cfg_a();
    if !apply(imu, failures, "check 2 init a", &a, true).await {
        return;
    }
    let orig = match imu.read_register(icm45686::reg::GYRO_CONFIG0).await {
        Ok(v) => v,
        Err(e) => return bus_fail(failures, "read GYRO_CONFIG0", &e),
    };
    let changed = orig ^ 0x10;
    if let Err(e) = imu
        .write_register(icm45686::reg::GYRO_CONFIG0, changed)
        .await
    {
        return bus_fail(failures, "write GYRO_CONFIG0", &e);
    }
    defmt::info!(
        "[STAT] GYRO_CONFIG0 original={=u8:#x} written={=u8:#x}",
        orig,
        changed
    );
    match imu.verify(&mut Delay).await {
        Err(Error::ConfigMismatch(m)) => {
            defmt::info!(
                "[STAT] verify reported space={} register={=u16:#x} mask={=u8:#x} expected={=u8:#x} found={=u8:#x}",
                defmt::Debug2Format(&m.space),
                m.register,
                m.mask,
                m.expected,
                m.found
            );
            verdict(
                failures,
                matches!(m.space, Space::Direct)
                    && m.register == u16::from(icm45686::reg::GYRO_CONFIG0)
                    && m.found == changed
                    && m.expected == orig & m.mask,
                "verify reports exactly GYRO_CONFIG0",
            );
        }
        Ok(()) => verdict(failures, false, "verify reports exactly GYRO_CONFIG0 (got Ok)"),
        Err(e) => bus_fail(failures, "verify after raw write", &e),
    }
    if apply(imu, failures, "check 2 configure a", &a, false).await {
        verify_ok(imu, failures, "verify Ok again after configure").await;
    }
}

// ---------------------------------------------------------------------------------------------
// Checks 3 and 4: count and decode (8 KB FIFO)
// ---------------------------------------------------------------------------------------------

async fn check3_4<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    buf: &mut [u8],
) {
    defmt::info!("[INFO] check 3: FIFO count, 8 KB depth, 400 ms at 800 Hz");
    let c8 = cfg_c(FifoDepth::Kib8, FifoFormat::Standard);
    if !apply(imu, failures, "check 3 configure c8", &c8, false).await {
        return;
    }
    if let Err(e) = imu.fifo_flush(&mut Delay).await {
        return bus_fail(failures, "fifo_flush", &e);
    }
    let t0 = Instant::now();
    Timer::after_millis(400).await;
    let count = match imu.fifo_count().await {
        Ok(v) => v,
        Err(e) => return bus_fail(failures, "fifo_count", &e),
    };
    let el1 = t0.elapsed().as_micros();
    let mut raw = [0u8; 2];
    if let Err(e) = imu
        .read_registers(icm45686::reg::FIFO_COUNT_0, &mut raw)
        .await
    {
        return bus_fail(failures, "raw FIFO_COUNT read", &e);
    }
    let el2 = t0.elapsed().as_micros();
    let expect1 = el1 as f32 * 800.0 / 1.0e6;
    let expect2 = el2 as f32 * 800.0 / 1.0e6;
    let le = u16::from_le_bytes(raw);
    let be = u16::from_be_bytes(raw);
    defmt::info!(
        "[STAT] fifo_count={} elapsed_us={} expected_frames={} ratio={} | raw bytes [0x12]={=u8:#x} [0x13]={=u8:#x} le={} be={} expected_at_raw={}",
        count,
        el1,
        expect1,
        count as f32 / expect1,
        raw[0],
        raw[1],
        le,
        be,
        expect2
    );
    verdict(
        failures,
        (count as f32 - expect1).abs() <= 0.1 * expect1 && (le as f32 - expect2).abs() <= 0.1 * expect2,
        "FIFO count is frames, little endian, within 10 % of elapsed x 800",
    );

    defmt::info!("[INFO] check 4: fifo_read and decode");
    let reg_t = reg_temp_c(imu, failures).await;
    let r = match imu.fifo_read(buf).await {
        Ok(r) => r,
        Err(e) => return bus_fail(failures, "fifo_read", &e),
    };
    defmt::info!(
        "[STAT] fifo_read bytes={} frames={} left={} overflow={}",
        r.bytes,
        r.frames,
        r.left,
        r.overflow
    );
    let d = analyse(buf.get(..r.bytes).unwrap_or(&[]), &c8, None);
    d.print("check 4 16-byte frames");
    d.judge(failures, "check 4", 100);
    defmt::info!(
        "[STAT] register temp_c={} fifo temp_c_mean={}",
        reg_t,
        d.mean_temp()
    );
    verdict(
        failures,
        (d.mean_temp() - reg_t).abs() <= 3.0,
        "FIFO temperature within 3 C of the register read",
    );
}

// ---------------------------------------------------------------------------------------------
// Check 5: high resolution
// ---------------------------------------------------------------------------------------------

async fn check5<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32, buf: &mut [u8]) {
    defmt::info!("[INFO] check 5: high-resolution 20-byte frames, 8 KB depth");
    let ch = cfg_c(FifoDepth::Kib8, FifoFormat::HighResolution);
    if !apply(imu, failures, "check 5 configure hires", &ch, false).await {
        return;
    }
    if let Err(e) = imu.fifo_flush(&mut Delay).await {
        return bus_fail(failures, "fifo_flush", &e);
    }
    Timer::after_millis(400).await;
    let reg_t = reg_temp_c(imu, failures).await;
    let r = match imu.fifo_read(buf).await {
        Ok(r) => r,
        Err(e) => return bus_fail(failures, "fifo_read hires", &e),
    };
    defmt::info!(
        "[STAT] fifo_read hires bytes={} frames={} left={} overflow={}",
        r.bytes,
        r.frames,
        r.left,
        r.overflow
    );
    let data = buf.get(..r.bytes).unwrap_or(&[]);
    let d = analyse(data, &ch, None);
    d.print("check 5 20-byte frames");
    d.judge(failures, "check 5", 200);
    defmt::info!(
        "[STAT] register temp_c={} fifo temp_c_mean={}",
        reg_t,
        d.mean_temp()
    );
    verdict(
        failures,
        (d.mean_temp() - reg_t).abs() <= 3.0,
        "hires FIFO temperature within 3 C of the register read",
    );
    // Nibble placement: bytes 17..19 of every 20-byte frame.
    let mut bit4 = [0u32; 3];
    let mut bit0 = [0u32; 3];
    let mut frames = 0u32;
    for chunk in data.chunks_exact(20) {
        frames += 1;
        for i in 0..3 {
            let b = chunk.get(17 + i).copied().unwrap_or(0);
            bit4[i] += u32::from(b >> 4 & 1);
            bit0[i] += u32::from(b & 1);
        }
    }
    defmt::info!(
        "[STAT] nibble bytes over {} frames: bit4 set x={} y={} z={} | bit0 set x={} y={} z={} (decoder: accel lowest = bit4, gyro lowest = bit0)",
        frames,
        bit4[0],
        bit4[1],
        bit4[2],
        bit0[0],
        bit0[1],
        bit0[2]
    );
    verdict(
        failures,
        frames >= 200 && bit4 == [0, 0, 0],
        "accel lowest bit (bit 4 of the nibble bytes) is never set",
    );
    verdict(
        failures,
        frames >= 200 && bit0.iter().all(|&n| n > 0 && n < frames),
        "gyro lowest bit (bit 0 of the nibble bytes) toggles",
    );
}

// ---------------------------------------------------------------------------------------------
// Check 6: overflow and flush
// ---------------------------------------------------------------------------------------------

async fn check6<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32, buf: &mut [u8]) {
    defmt::info!("[INFO] check 6: overflow and flush, 2 KB depth");
    let c = cfg_c(FifoDepth::Kib2, FifoFormat::Standard).with_int1(
        Int1Config::new()
            .with_fifo_threshold(true)
            .with_fifo_full(true),
    );
    if !apply(imu, failures, "check 6 configure c", &c, false).await {
        return;
    }
    if let Err(e) = imu.fifo_flush(&mut Delay).await {
        return bus_fail(failures, "fifo_flush", &e);
    }
    let _ = imu.interrupt_status().await; // clear stale flags
    let s0 = *imu.stats();
    Timer::after_millis(300).await;
    // The status is read BEFORE the draining fifo_read: it describes the moment of the read.
    let sa = match imu.interrupt_status().await {
        Ok(st) => {
            defmt::info!(
                "[STAT] interrupt_status before the drain: status0={=u8:#x} status1={=u8:#x} fifo_full={} fifo_threshold={} (FIFO-full source enabled on INT1)",
                st.status0,
                st.status1,
                st.fifo_full,
                st.fifo_threshold
            );
            verdict(failures, st.fifo_full, "interrupt_status reports fifo_full");
            *imu.stats()
        }
        Err(e) => return bus_fail(failures, "interrupt_status", &e),
    };
    let r = match imu.fifo_read(buf).await {
        Ok(r) => r,
        Err(e) => return bus_fail(failures, "fifo_read after overflow", &e),
    };
    defmt::info!(
        "[STAT] fifo_read bytes={} frames={} left={} overflow={} count read={} (expected capacity + 2 = 130)",
        r.bytes,
        r.frames,
        r.left,
        r.overflow,
        r.frames + usize::from(r.left.unwrap_or(0))
    );
    verdict(
        failures,
        r.overflow == Some(true),
        "fifo_read reports overflow",
    );
    let d = analyse(buf.get(..r.bytes).unwrap_or(&[]), &c, None);
    d.print("check 6 frames read after overflow");
    let s1 = *imu.stats();
    defmt::info!(
        "[STAT] stats overflows before={} after interrupt_status={} after fifo_read={} frames_read={}",
        s0.fifo_overflows,
        sa.fifo_overflows,
        s1.fifo_overflows,
        s1.fifo_frames_read
    );
    verdict(
        failures,
        s1.fifo_overflows == sa.fifo_overflows + 1,
        "stats() counted the fifo_read overflow",
    );
    let t = Instant::now();
    let fl = imu.fifo_flush(&mut Delay).await;
    let us = t.elapsed().as_micros();
    defmt::info!("[STAT] fifo_flush took {} us", us);
    if let Err(e) = fl {
        return bus_fail(failures, "fifo_flush", &e);
    }
    match imu.fifo_count().await {
        Ok(n) => {
            defmt::info!("[STAT] fifo_count right after flush = {}", n);
            verdict(failures, n <= 2, "fifo_count right after flush is at most 2");
        }
        Err(e) => bus_fail(failures, "fifo_count after flush", &e),
    }
    Timer::after_millis(30).await;
    match imu.fifo_read(buf).await {
        Ok(r) => {
            let d = analyse(buf.get(..r.bytes).unwrap_or(&[]), &c, None);
            d.print("check 6 frames after flush");
            verdict(
                failures,
                r.frames > 0 && d.valid == d.n && d.other == 0 && d.bad_delta == 0,
                "frames after flush decode cleanly and contiguously",
            );
        }
        Err(e) => bus_fail(failures, "fifo_read after flush", &e),
    }
}


/// Not one of the eight checks: what the device does when the FIFO overflows (capacity, the full flag
/// with and without its INT1 enable, the frames that survive).
async fn overflow_probe<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    buf: &mut [u8],
    int_full: bool,
    depth: FifoDepth,
    wait_ms: u64,
) {
    let c = cfg_c(depth, FifoFormat::Standard).with_int1(
        Int1Config::new()
            .with_fifo_threshold(true)
            .with_fifo_full(int_full),
    );
    defmt::info!("[INFO] overflow probe, INT1 fifo_full enabled = {} depth_bytes={} wait_ms={}", int_full, depth.bytes(), wait_ms);
    if !apply(imu, failures, "probe configure", &c, false).await {
        return;
    }
    if imu.fifo_flush(&mut Delay).await.is_err() {
        return;
    }
    let _ = imu.interrupt_status().await;
    Timer::after_millis(wait_ms).await;
    let sa = imu.interrupt_status().await.ok();
    let sb = imu.interrupt_status().await.ok();
    let c1 = imu.fifo_count().await.ok();
    Timer::after_millis(100).await;
    let c2 = imu.fifo_count().await.ok();
    defmt::info!(
        "[STAT] probe status0 first read={} second read={} (bit0 full, bit1 threshold) count after 300 ms={} after 400 ms={}",
        sa.map(|s| s.status0),
        sb.map(|s| s.status0),
        c1,
        c2
    );
    if let Ok(r) = imu.fifo_read(buf).await {
        defmt::info!(
            "[STAT] probe fifo_read frames={} left={} overflow={}",
            r.frames,
            r.left,
            r.overflow
        );
        analyse(buf.get(..r.bytes).unwrap_or(&[]), &c, None).print("probe frames after overflow");
    }
}

// ---------------------------------------------------------------------------------------------
// Check 7: watermark interrupt path
// ---------------------------------------------------------------------------------------------

async fn check7<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    failures: &mut u32,
    dr: &mut DataReady<'_>,
    buf: &mut [u8],
) {
    defmt::info!("[INFO] check 7: watermark 8 interrupt path, 200 edges");
    let c = cfg_c(FifoDepth::Kib2, FifoFormat::Standard);
    if !apply(imu, failures, "check 7 configure c", &c, false).await {
        return;
    }
    if let Err(e) = imu.fifo_flush(&mut Delay).await {
        return bus_fail(failures, "fifo_flush", &e);
    }
    let _ = imu.interrupt_status().await;
    let mut seen = dr.count();
    let mut prev: Option<u16> = None;
    let (mut valid, mut other, mut bad, mut total) = (0u32, 0u32, 0u32, 0u32);
    let (mut sum_us, mut worst_us) = (0u64, 0u32);
    let mut edges = 0u32;
    let mut max_backlog = 0u64;
    let mut timeout = false;
    let mut read_err = false;
    let (mut tick_sum, mut tick_n) = (0u64, 0u32);
    let mut t_first = Instant::now();
    let mut t_last = t_first;
    for _ in 0..200 {
        let Some(c_now) = dr.wait_beyond(seen, Duration::from_millis(100)).await else {
            timeout = true;
            break;
        };
        seen += 1;
        edges += 1;
        t_last = Instant::now();
        if edges == 1 {
            t_first = t_last;
        }
        max_backlog = max_backlog.max(c_now - seen);
        let t = Instant::now();
        let r = imu.fifo_read_exact(buf, 8).await;
        let us = t.elapsed().as_micros() as u32;
        sum_us += u64::from(us);
        worst_us = worst_us.max(us);
        let Ok(r) = r else {
            read_err = true;
            break;
        };
        let d = analyse(buf.get(..r.bytes).unwrap_or(&[]), &c, prev);
        prev = d.last_ts;
        tick_sum += d.dsum;
        tick_n += d.dn;
        total += d.n;
        valid += d.valid;
        other += d.other;
        bad += d.bad_delta;
    }
    defmt::info!(
        "[STAT] edges handled={} frames={} valid={} other={} bad_deltas={} max_edge_backlog={} timeout={} read_error={} fifo_read_exact_us mean={} worst={}",
        edges,
        total,
        valid,
        other,
        bad,
        max_backlog,
        timeout,
        read_err,
        sum_us as f32 / edges.max(1) as f32,
        worst_us
    );
    defmt::info!(
        "[STAT] timestamp tick vs wall clock: mean delta {} us over {} deltas; wall clock first to last edge {} us over {} frame periods = {} us per frame",
        tick_sum as f32 / tick_n.max(1) as f32,
        tick_n,
        (t_last - t_first).as_micros(),
        8 * edges.saturating_sub(1),
        (t_last - t_first).as_micros() as f32 / (8 * edges.saturating_sub(1)).max(1) as f32
    );
    match imu.fifo_count().await {
        Ok(n) => defmt::info!("[STAT] fifo_count after the loop = {}", n),
        Err(e) => bus_fail(failures, "fifo_count after loop", &e),
    }
    verdict(failures, !timeout && !read_err, "200 INT1 edges arrived, none timed out");
    verdict(
        failures,
        valid == 1600 && total == 1600 && other == 0 && bad == 0,
        "all 1600 frames valid and contiguous",
    );
}

// ---------------------------------------------------------------------------------------------
// Check 8: errata statistics
// ---------------------------------------------------------------------------------------------

async fn check8<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, failures: &mut u32, buf: &mut [u8]) {
    defmt::info!("[INFO] check 8: vendor errata statistics (no PASS/FAIL)");
    let c = cfg_c(FifoDepth::Kib2, FifoFormat::Standard);
    if let Err(e) = imu.fifo_flush(&mut Delay).await {
        return bus_fail(failures, "fifo_flush", &e);
    }
    let (mut done, mut differ, mut gt1, mut first_larger, mut maxdiff, mut errs) =
        (0u32, 0u32, 0u32, 0u32, 0u32, 0u32);
    let mut drained = 0u32;
    for _ in 0..2000 {
        let mut a = [0u8; 2];
        let mut b = [0u8; 2];
        if imu
            .read_registers(icm45686::reg::FIFO_COUNT_0, &mut a)
            .await
            .is_err()
            || imu
                .read_registers(icm45686::reg::FIFO_COUNT_0, &mut b)
                .await
                .is_err()
        {
            errs += 1;
            break;
        }
        done += 1;
        let (x, y) = (u16::from_le_bytes(a), u16::from_le_bytes(b));
        let diff = u32::from(x.abs_diff(y));
        if diff != 0 {
            differ += 1;
            if x > y {
                first_larger += 1;
            }
        }
        if diff > 1 {
            gt1 += 1;
        }
        maxdiff = maxdiff.max(diff);
        if y > 100 {
            // keep the FIFO from overflowing during the run
            let _ = imu.fifo_read_exact(buf, 64).await;
            drained += 1;
        }
    }
    defmt::info!(
        "[STAT] count read twice: pairs={} differ={} differ_by_more_than_1={} first_larger={} max_diff={} bus_errors={} drains={}",
        done,
        differ,
        gt1,
        first_larger,
        maxdiff,
        errs,
        drained
    );

    // Read ALL frames the count reports.
    let _ = imu.fifo_flush(&mut Delay).await;
    Timer::after_millis(12).await;
    let n = match imu.fifo_count().await {
        Ok(n) => n,
        Err(e) => return bus_fail(failures, "fifo_count", &e),
    };
    let n_use = usize::from(n).clamp(1, 100);
    let r = match imu.fifo_read_exact(buf, n_use).await {
        Ok(r) => r,
        Err(e) => return bus_fail(failures, "fifo_read_exact all", &e),
    };
    let after = imu.fifo_count().await.ok();
    let d1 = analyse(buf.get(..r.bytes).unwrap_or(&[]), &c, None);
    d1.print("check 8 drain read (all frames the count reported)");
    defmt::info!(
        "[STAT] drain: count was {} read {} frames, fifo_count right after = {}",
        n,
        n_use,
        after
    );
    Timer::after_millis(25).await;
    let n2 = match imu.fifo_count().await {
        Ok(n) => n,
        Err(e) => return bus_fail(failures, "fifo_count 2", &e),
    };
    let take = usize::from(n2).min(16);
    if take == 0 {
        defmt::info!("[STAT] after drain: fifo_count = 0 after 25 ms (FIFO stopped?)");
        return;
    }
    match imu.fifo_read_exact(buf, take).await {
        Ok(r) => {
            let d2 = analyse(buf.get(..r.bytes).unwrap_or(&[]), &c, d1.last_ts);
            d2.print("check 8 next frames after a full drain");
            defmt::info!(
                "[STAT] after drain: asked {} frames, clean={} contiguous_with_drained={}",
                take,
                d2.valid == d2.n && d2.other == 0,
                d2.bad_delta == 0
            );
        }
        Err(e) => bus_fail(failures, "fifo_read_exact after drain", &e),
    }

    // Reading one frame beyond what the FIFO holds.
    let _ = imu.fifo_flush(&mut Delay).await;
    Timer::after_millis(12).await;
    let n3 = imu.fifo_count().await.ok().unwrap_or(0);
    let n3u = usize::from(n3).min(100);
    match imu.fifo_read_exact(buf, n3u + 1).await {
        Ok(r) => {
            let good = n3u * 16;
            let extra = buf.get(good..good + 16).unwrap_or(&[]);
            defmt::info!(
                "[STAT] over-read: count was {}, read {} frames; the extra frame bytes = {=[u8]:x}",
                n3,
                n3u + 1,
                extra
            );
            let d3 = analyse(buf.get(..r.bytes).unwrap_or(&[]), &c, None);
            d3.print("check 8 over-read by one frame");
            let after = imu.fifo_count().await.ok();
            Timer::after_millis(25).await;
            let n4 = imu.fifo_count().await.ok();
            let take = usize::from(n4.unwrap_or(0)).min(16);
            if take > 0 {
                if let Ok(r) = imu.fifo_read_exact(buf, take).await {
                    let last_good = buf.get(..0).map(|_| ());
                    let _ = last_good;
                    let d4 = analyse(buf.get(..r.bytes).unwrap_or(&[]), &c, None);
                    d4.print("check 8 frames after the over-read");
                }
            }
            defmt::info!(
                "[STAT] over-read: fifo_count right after = {} and 25 ms later = {}",
                after,
                n4
            );
        }
        Err(e) => bus_fail(failures, "over-read", &e),
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
    let mut buf = [0u8; 8192];

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

        check1(&mut imu, &mut failures).await;
        check2(&mut imu, &mut failures).await;
        check3_4(&mut imu, &mut failures, &mut buf).await;
        check5(&mut imu, &mut failures, &mut buf).await;
        check6(&mut imu, &mut failures, &mut buf).await;
        overflow_probe(&mut imu, &mut failures, &mut buf, false, FifoDepth::Kib2, 300).await;
        overflow_probe(&mut imu, &mut failures, &mut buf, true, FifoDepth::Kib2, 300).await;
        overflow_probe(&mut imu, &mut failures, &mut buf, true, FifoDepth::Kib8, 800).await;
        check7(&mut imu, &mut failures, &mut dr, &mut buf).await;
        check8(&mut imu, &mut failures, &mut buf).await;
    }
    defmt::info!("[INFO] run took {} ms", started.elapsed().as_millis());

    hil_common::exit(failures)
}
