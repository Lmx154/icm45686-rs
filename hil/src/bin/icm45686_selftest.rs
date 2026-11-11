//! On-board check of `Icm45686::self_test` on the MARV V2 Rev 1 board (U21, SPI0). The board must
//! lie still. Every self-test run is followed by the health check: `init` with the default
//! configuration, `verify`, and 200 samples at rest (1 g within 10 %, rotation under 5 dps). A
//! failed health check or a self-test error ends the runs; the program always ends with the verdict.
#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::dma;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1};
use embassy_rp::spi::{self, Spi};
use embassy_time::{Delay, Instant, Timer};
use embedded_hal_async::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use icm45686::{AxisVerdict, Config, Icm45686, SelfTestConfig, SelfTestStatus};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
const SAMPLES: u32 = 200;

fn verdict(failures: &mut u32, ok: bool, name: &str) {
    if ok {
        defmt::info!("[PASS] {=str}", name);
    } else {
        *failures += 1;
        defmt::error!("[FAIL] {=str}", name);
    }
}

fn axes(v: Option<AxisVerdict>) -> (u8, u8, u8) {
    // 1 pass, 0 fail, 9 not tested.
    v.map_or((9, 9, 9), |a| (u8::from(a.x), u8::from(a.y), u8::from(a.z)))
}

/// `init` with the default configuration, `verify`, then 200 samples 2 ms apart at rest.
async fn healthy<SPI: SpiDevice>(imu: &mut Icm45686<SPI>, name: &str) -> bool {
    let config = Config::new();
    let mut delay = Delay;
    if let Err(e) = imu.init(&mut delay, &config).await {
        defmt::error!("[STAT] {=str} health: init failed {}", name, defmt::Debug2Format(&e));
        return false;
    }
    if let Err(e) = imu.verify(&mut delay).await {
        defmt::error!("[STAT] {=str} health: verify failed {}", name, defmt::Debug2Format(&e));
        return false;
    }
    let gyro_max = 5.0_f32.to_radians();
    let (mut errors, mut bad) = (0u32, 0u32);
    let (mut a2_min, mut a2_max, mut g_worst) = (f32::MAX, 0.0_f32, 0.0_f32);
    for _ in 0..SAMPLES {
        Timer::after_millis(2).await;
        match imu.read().await {
            Ok(s) => {
                let a = s.accel_ms2(config.accel_fs);
                let g = s.gyro_rads(config.gyro_fs);
                let a2 = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / (9.80665 * 9.80665);
                let gw = g.iter().fold(0.0_f32, |m, v| m.max(v.abs()));
                a2_min = a2_min.min(a2);
                a2_max = a2_max.max(a2);
                g_worst = g_worst.max(gw);
                if !(a2 > 0.81 && a2 < 1.21 && gw <= gyro_max) {
                    bad += 1;
                }
            }
            Err(_) => errors += 1,
        }
    }
    defmt::info!(
        "[STAT] {=str} health: samples={} read_errors={} out_of_limits={} accel_g^2 min={} max={} (limit 0.81..1.21) gyro_worst_rad_s={} (limit {})",
        name,
        SAMPLES,
        errors,
        bad,
        a2_min,
        a2_max,
        g_worst,
        gyro_max
    );
    errors == 0 && bad == 0
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut failures = 0u32;
    let mut spi_config = spi::Config::default();
    spi_config.frequency = SPI_HZ;
    let bus = Spi::new(
        p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, p.DMA_CH0, p.DMA_CH1, Irqs, spi_config,
    );
    let cs = Output::new(p.PIN_20, Level::High);
    let device = ExclusiveDevice::new_no_delay(bus, cs).unwrap();
    let mut imu = Icm45686::new(device);
    let mut delay = Delay;

    let before = healthy(&mut imu, "before").await;
    verdict(&mut failures, before, "healthy before any self-test");

    let default = SelfTestConfig::new();
    let runs: [(&str, SelfTestConfig, bool); 7] = [
        ("default 1", default, true),
        ("default 2", default, true),
        ("default 3", default, true),
        ("accel only", default.with_gyro(false), false),
        ("gyro only", default.with_accel(false), false),
        (
            "tightest 5%",
            default.with_accel_tolerance_pct(5).with_gyro_tolerance_pct(5),
            false,
        ),
        (
            "loosest 50%",
            default.with_accel_tolerance_pct(50).with_gyro_tolerance_pct(50),
            false,
        ),
    ];
    if before {
        for (name, config, judged) in runs {
            let word = config.param_word().unwrap_or(0);
            let t0 = Instant::now();
            let result = imu.self_test(&mut delay, &config).await;
            let ms = t0.elapsed().as_millis();
            let mut stop = false;
            match result {
                Ok(r) => {
                    let (ax, ay, az) = axes(r.accel);
                    let (gx, gy, gz) = axes(r.gyro);
                    defmt::info!(
                        "[STAT] {=str} param=0x{=u16:04X} result=0x{=u8:02X} status={} accel_xyz={}{}{} gyro_xyz={}{}{} (1 pass, 0 fail, 9 not run) passed={} ms={}",
                        name,
                        word,
                        r.raw,
                        r.status,
                        ax,
                        ay,
                        az,
                        gx,
                        gy,
                        gz,
                        r.passed(),
                        ms
                    );
                    if judged {
                        let ok = r.status == SelfTestStatus::Done
                            && r.passed()
                            && r.accel.is_some()
                            && r.gyro.is_some();
                        verdict(&mut failures, ok, "default run: done, six axes pass");
                    }
                }
                Err(e) => {
                    failures += 1;
                    defmt::error!(
                        "[FAIL] {=str} self_test error {} after ms={}",
                        name,
                        defmt::Debug2Format(&e),
                        ms
                    );
                    stop = true;
                }
            }
            let ok = healthy(&mut imu, name).await;
            verdict(&mut failures, ok, "healthy after the run");
            if !ok || stop {
                defmt::error!("[FAIL] stopping the runs after {=str}", name);
                break;
            }
        }
    }
    hil_common::exit(failures)
}
