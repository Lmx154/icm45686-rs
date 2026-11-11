//! Contract rule A4d on the board (U21, SPI0, INT1 on GPIO15): cancel the driver's `read()` part-way,
//! call `recover()` after every cancel, prove no wrong read and no register change follow.
#![no_std]
#![no_main]

use core::fmt::Debug;

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use icm45686::{Config, Error, Icm45686, Int1Config, RawSample};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>, dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const BLOCK: u32 = 25;
const CONTROL_CANCELS: u64 = 100;
const DR_CANCELS: u32 = 500;
const MAX_CANCELS: u32 = 2500;
const CANCEL_PHASES_END: Duration = Duration::from_secs(45);
const EDGE_WAIT: Duration = Duration::from_millis(100);

enum Stop {
    /// The sensor or the bus misbehaved: repair attempt, then FAIL.
    Sensor,
    /// The program's own time budget ran out: no repair, nothing wrong with the sensor.
    Budget,
}

struct Limits {
    mean: [i32; 7],
    tol: [i32; 7],
}

fn values(s: &RawSample) -> [i32; 7] {
    [
        i32::from(s.accel[0]),
        i32::from(s.accel[1]),
        i32::from(s.accel[2]),
        i32::from(s.gyro[0]),
        i32::from(s.gyro[1]),
        i32::from(s.gyro[2]),
        i32::from(s.temp),
    ]
}

fn is_wrong<E>(r: &Result<RawSample, Error<E>>, lim: &Limits) -> bool {
    match r {
        Err(_) => true,
        Ok(s) => {
            let v = values(s);
            (0..7).any(|i| (v[i] - lim.mean[i]).abs() > lim.tol[i])
        }
    }
}

fn show_wrong<E: Debug>(r: &Result<RawSample, Error<E>>, lim: &Limits) {
    match r {
        Err(e) => defmt::error!("wrong read: error {}", defmt::Debug2Format(e)),
        Ok(s) => {
            let v = values(s);
            defmt::error!(
                "wrong read: got {} reference {} tolerance {}",
                v,
                lim.mean,
                lim.tol
            );
        }
    }
}

fn describe<E: Debug>(what: &str, e: &Error<E>) {
    match e {
        Error::ConfigMismatch(m) => defmt::error!(
            "{=str}: MISMATCH space={} register={=u16:#x} mask={=u8:#x} expected={=u8:#x} found={=u8:#x}",
            what,
            defmt::Debug2Format(&m.space),
            m.register,
            m.mask,
            m.expected,
            m.found
        ),
        other => defmt::error!("{=str}: {}", what, defmt::Debug2Format(other)),
    }
}

#[derive(Default)]
struct Stats {
    phase: &'static str,
    d: u64,
    attempts: u32,
    cancelled: u32,
    completed: u32,
    recover_fail: u32,
    wrong: u32,
    mismatches: u32,
    rec_n: u32,
    rec_sum_us: u64,
    rec_worst_us: u32,
}

impl Stats {
    fn new(phase: &'static str) -> Self {
        Self {
            phase,
            ..Self::default()
        }
    }

    fn print(&self) {
        let mean = self.rec_sum_us as f32 / self.rec_n.max(1) as f32;
        defmt::info!(
            "[STAT] {=str} attempts={} cancelled={} completed={} recover_failures={} wrong_after_recover={} verify_mismatches={} recover_mean_us={} recover_worst_us={} last_delay_us={}",
            self.phase,
            self.attempts,
            self.cancelled,
            self.completed,
            self.recover_fail,
            self.wrong,
            self.mismatches,
            mean,
            self.rec_worst_us,
            self.d
        );
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

async fn do_recover<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    delay: &mut Delay,
    st: &mut Stats,
) -> Result<(), Stop> {
    let t = Instant::now();
    let r = imu.recover(delay).await;
    let us = t.elapsed().as_micros() as u32;
    st.rec_n += 1;
    st.rec_sum_us += u64::from(us);
    st.rec_worst_us = st.rec_worst_us.max(us);
    match r {
        Ok(()) => Ok(()),
        Err(e) => {
            st.recover_fail += 1;
            describe("recover() did not return Ok", &e);
            Err(Stop::Sensor)
        }
    }
}

async fn do_verify<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    delay: &mut Delay,
    st: &mut Stats,
) -> Result<(), Stop> {
    match imu.verify(delay).await {
        Ok(()) => Ok(()),
        Err(e) => {
            st.mismatches += 1;
            describe("verify() failed", &e);
            Err(Stop::Sensor)
        }
    }
}

async fn checked_reads<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    lim: &Limits,
    st: &mut Stats,
) -> Result<(), Stop> {
    for _ in 0..2 {
        let r = imu.read().await;
        if is_wrong(&r, lim) {
            st.wrong += 1;
            show_wrong(&r, lim);
            return Err(Stop::Sensor);
        }
    }
    Ok(())
}

/// One cancel attempt of `read()` after `d` microseconds, with the recovery and the checks of phases 2 and 3.
async fn attempt<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    delay: &mut Delay,
    lim: &Limits,
    st: &mut Stats,
    d: u64,
    deadline: Instant,
) -> Result<(), Stop> {
    if Instant::now() > deadline {
        return Err(Stop::Budget);
    }
    st.d = d;
    st.attempts += 1;
    match with_timeout(Duration::from_micros(d), imu.read()).await {
        Ok(r) => {
            st.completed += 1;
            if is_wrong(&r, lim) {
                st.wrong += 1;
                show_wrong(&r, lim);
                return Err(Stop::Sensor);
            }
        }
        Err(_) => {
            st.cancelled += 1;
            do_recover(imu, delay, st).await?;
            checked_reads(imu, lim, st).await?;
        }
    }
    Ok(())
}

fn finish_phase(failures: &mut u32, st: &Stats, min_cancelled: u32, name: &str) {
    st.print();
    verdict(
        failures,
        st.recover_fail == 0
            && st.wrong == 0
            && st.mismatches == 0
            && st.cancelled >= min_cancelled,
        name,
    );
}

async fn phase1<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    delay: &mut Delay,
    lim: &Limits,
    t_us: u64,
    failures: &mut u32,
    deadline: Instant,
) -> Result<(), Stop> {
    let mut st = Stats::new("phase 1 control");
    let mut followed = 0u32;
    let mut worst = 0u32;
    let mut unsettled = 0u32;
    let r: Result<(), Stop> = async {
        for i in 0..CONTROL_CANCELS {
            if Instant::now() > deadline {
                return Err(Stop::Budget);
            }
            let d = i * (t_us + 10) / (CONTROL_CANCELS - 1);
            st.d = d;
            st.attempts += 1;
            match with_timeout(Duration::from_micros(d), imu.read()).await {
                Ok(_) => st.completed += 1,
                Err(_) => {
                    st.cancelled += 1;
                    let mut wrong_here = 0u32;
                    let mut run = 0u32;
                    for _ in 0..8 {
                        if is_wrong(&imu.read().await, lim) {
                            wrong_here += 1;
                            run = 0;
                        } else {
                            run += 1;
                            if run == 2 {
                                break;
                            }
                        }
                    }
                    if run < 2 {
                        unsettled += 1;
                    }
                    if wrong_here > 0 {
                        followed += 1;
                    }
                    worst = worst.max(wrong_here);
                }
            }
            do_recover(imu, delay, &mut st).await?;
        }
        do_verify(imu, delay, &mut st).await
    }
    .await;
    defmt::info!(
        "[STAT] phase 1 control really_cancelled={} completed_first={} cancels_followed_by_wrong_read={} worst_wrong_reads_after_one_cancel={} unsettled_in_8={}",
        st.cancelled,
        st.completed,
        followed,
        worst,
        unsettled
    );
    st.print();
    match &r {
        Err(Stop::Budget) => defmt::error!("[FAIL] phase 1 control: time budget"),
        _ => verdict(
            failures,
            r.is_ok() && st.recover_fail == 0 && st.mismatches == 0,
            "phase 1 control: every recover() Ok and verify() Ok",
        ),
    }
    r
}

async fn phase2<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    delay: &mut Delay,
    lim: &Limits,
    t_us: u64,
    failures: &mut u32,
    deadline: Instant,
) -> Result<(), Stop> {
    let mut st = Stats::new("phase 2 sweep");
    let r: Result<(), Stop> = async {
        for d in 0..=(t_us + 10) {
            for _ in 0..BLOCK {
                attempt(imu, delay, lim, &mut st, d, deadline).await?;
            }
            do_verify(imu, delay, &mut st).await?;
        }
        Ok(())
    }
    .await;
    if r.is_err() {
        defmt::error!("[FAIL] phase 2 sweep stopped early");
        *failures += 1;
    }
    finish_phase(failures, &st, 300, "phase 2 sweep");
    r
}

async fn phase3<SPI: SpiDevice>(
    imu: &mut Icm45686<SPI>,
    delay: &mut Delay,
    lim: &Limits,
    cfg: &Config,
    edge: &mut Input<'_>,
    t_us: u64,
    failures: &mut u32,
    deadline: Instant,
) -> Result<(), Stop> {
    let mut st = Stats::new("phase 3 data-ready");
    let mut stalls = 0u32;
    let r: Result<(), Stop> = async {
        if let Err(e) = imu.configure(delay, cfg).await {
            describe("phase 3 configure failed", &e);
            return Err(Stop::Sensor);
        }
        do_verify(imu, delay, &mut st).await?;
        for k in 0..DR_CANCELS {
            if with_timeout(EDGE_WAIT, edge.wait_for_rising_edge())
                .await
                .is_err()
            {
                stalls += 1;
                if stalls >= 3 {
                    defmt::error!("no data-ready edge within 100 ms, three times");
                    return Err(Stop::Budget);
                }
                continue;
            }
            attempt(
                imu,
                delay,
                lim,
                &mut st,
                u64::from(k) % (t_us + 11),
                deadline,
            )
            .await?;
            if (k + 1) % BLOCK == 0 {
                do_verify(imu, delay, &mut st).await?;
            }
        }
        do_verify(imu, delay, &mut st).await
    }
    .await;
    defmt::info!("[STAT] phase 3 edge_wait_stalls={}", stalls);
    if r.is_err() {
        defmt::error!("[FAIL] phase 3 data-ready stopped early");
        *failures += 1;
    }
    finish_phase(failures, &st, 150, "phase 3 data-ready");
    r
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let t_boot = Instant::now();
    let deadline = t_boot + CANCEL_PHASES_END;
    let mut failures = 0u32;
    let mut spi_config = spi::Config::default();
    spi_config.frequency = SPI_HZ;
    let mut edge = Input::new(p.PIN_15.reborrow(), Pull::None);

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
    let mut delay = Delay;

    let default_cfg = Config::new();
    let dr_cfg = default_cfg.with_int1(Int1Config::new().with_data_ready(true));
    let mut cur_cfg = default_cfg;

    let mut lim = Limits {
        mean: [0; 7],
        tol: [0; 7],
    };
    let accel_tol = (0.060 * default_cfg.accel_fs.lsb_per_g()) as i32;
    let gyro_tol = (3.0 * default_cfg.gyro_fs.lsb_per_dps()) as i32;
    lim.tol = [
        accel_tol, accel_tol, accel_tol, gyro_tol, gyro_tol, gyro_tol, 384,
    ];
    defmt::info!(
        "[STAT] limits: accel {} counts, gyro {} counts, temp 384 counts",
        accel_tol,
        gyro_tol
    );

    // Phase 0.
    let mut t_us = 0u64;
    let mut go = false;
    'p0: {
        if let Err(e) = imu.init(&mut delay, &default_cfg).await {
            describe("phase 0 init failed", &e);
            verdict(&mut failures, false, "phase 0 reference");
            break 'p0;
        }
        if let Err(e) = imu.verify(&mut delay).await {
            describe("phase 0 verify failed", &e);
            verdict(&mut failures, false, "phase 0 reference");
            break 'p0;
        }
        let mut durations = [0u16; 1000];
        let mut timing_errors = 0u32;
        for slot in durations.iter_mut() {
            let t = Instant::now();
            let r = imu.read().await;
            *slot = t.elapsed().as_micros().min(u64::from(u16::MAX)) as u16;
            if r.is_err() {
                timing_errors += 1;
            }
        }
        durations.sort_unstable();
        let median = u64::from(durations[500]);
        let worst = u64::from(durations[999]);
        t_us = median;
        defmt::info!(
            "[STAT] phase 0 read duration T median_us={} worst_us={} read_errors={}",
            median,
            worst,
            timing_errors
        );
        let mut sum = [0i32; 7];
        let mut n = 0i32;
        for _ in 0..400 {
            if let Ok(s) = imu.read().await {
                let v = values(&s);
                for i in 0..7 {
                    sum[i] += v[i];
                }
                n += 1;
            }
        }
        if n < 400 {
            defmt::error!("reference reads failed: {} of 400 ok", n);
            verdict(&mut failures, false, "phase 0 reference");
            break 'p0;
        }
        for i in 0..7 {
            lim.mean[i] = sum[i] / n;
        }
        defmt::info!("[STAT] phase 0 reference means (counts) {}", lim.mean);
        let mut wrong = 0u32;
        for _ in 0..1000 {
            let r = imu.read().await;
            if is_wrong(&r, &lim) {
                wrong += 1;
            }
        }
        defmt::info!("[STAT] phase 0 wrong_of_1000={}", wrong);
        verdict(
            &mut failures,
            wrong == 0 && timing_errors == 0,
            "phase 0 reference",
        );
        if wrong != 0 || timing_errors != 0 {
            break 'p0;
        }
        if u32::try_from(t_us + 11).map_or(true, |nd| {
            nd * BLOCK + CONTROL_CANCELS as u32 + DR_CANCELS > MAX_CANCELS
        }) {
            defmt::error!("T too large for the 2500-cancel bound");
            verdict(&mut failures, false, "phase 0 cancel bound");
            break 'p0;
        }
        go = true;
    }

    let mut stopped = false;
    if go {
        match phase1(&mut imu, &mut delay, &lim, t_us, &mut failures, deadline).await {
            Ok(()) => {}
            Err(Stop::Sensor) => stopped = true,
            Err(Stop::Budget) => go = false,
        }
    }
    if go && !stopped {
        match phase2(&mut imu, &mut delay, &lim, t_us, &mut failures, deadline).await {
            Ok(()) => {}
            Err(Stop::Sensor) => stopped = true,
            Err(Stop::Budget) => go = false,
        }
    }
    if go && !stopped {
        cur_cfg = dr_cfg;
        match phase3(
            &mut imu,
            &mut delay,
            &lim,
            &dr_cfg,
            &mut edge,
            t_us,
            &mut failures,
            deadline,
        )
        .await
        {
            Ok(()) => {}
            Err(Stop::Sensor) => stopped = true,
            Err(Stop::Budget) => {}
        }
    }

    let mut bus_allows_end = true;
    if stopped {
        defmt::error!("STOPPED on a sensor fault; one repair attempt");
        Timer::after_millis(1).await;
        let mut ok = false;
        match imu.recover(&mut delay).await {
            Ok(()) => match imu.configure(&mut delay, &cur_cfg).await {
                Ok(_) => match imu.verify(&mut delay).await {
                    Ok(()) => ok = true,
                    Err(e) => describe("repair: verify after configure", &e),
                },
                Err(e) => describe("repair: configure", &e),
            },
            Err(e) => describe("repair: recover", &e),
        }
        defmt::info!("[STAT] repair outcome ok={}", ok);
        failures += 1;
        if !ok {
            bus_allows_end = false;
            let id = imu.read_register(icm45686::reg::WHO_AM_I).await;
            defmt::error!("WHO_AM_I after failed repair: {}", defmt::Debug2Format(&id));
        }
    }

    // End state.
    let mut normal = false;
    if bus_allows_end {
        'end: {
            if let Err(e) = imu.init(&mut delay, &default_cfg).await {
                describe("end init failed", &e);
                break 'end;
            }
            if let Err(e) = imu.verify(&mut delay).await {
                describe("end verify failed", &e);
                break 'end;
            }
            let mut bad = 0u32;
            for _ in 0..200 {
                if is_wrong(&imu.read().await, &lim) {
                    bad += 1;
                }
            }
            defmt::info!("[STAT] end state implausible_of_200={}", bad);
            normal = bad == 0 && lim.tol[0] != 0 && lim.mean != [0; 7];
        }
    }
    verdict(&mut failures, normal, "sensor normal at the end");

    hil_common::exit(failures)
}
