# icm45686

Driver for the TDK InvenSense ICM-45686 six-axis IMU over SPI. It is `no_std` and written against the `embedded-hal` 1.0 and `embedded-hal-async` 1.0 traits only (plus optional `defmt`); it owns no pin, timer, executor or HAL. The crate root is the async driver; the `blocking` module runs the same code on the blocking `embedded-hal` traits. It was written for the MARV V2 Rev 1 flight controller and has been run on that one board only.

Status: version 0.1.0, prepared for crates.io. The API may still change. Nothing here is a stability or production-readiness claim.

## Works with

The driver works with any HAL and any MCU whose SPI device and delay implement `embedded-hal-async` 1.0 (async driver) or `embedded-hal` 1.0 (`blocking` module). No embassy crate is needed.

## Quick start

The configuration structs are `#[non_exhaustive]`: build them with `Config::new()` and the `with_*` setters, not with a struct literal.

```rust,no_run
# use core::convert::Infallible;
# use embedded_hal::spi::{ErrorType, Operation};
# use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
# struct Spi;
# impl ErrorType for Spi { type Error = Infallible; }
# impl SpiDevice for Spi {
#     async fn transaction(&mut self, _ops: &mut [Operation<'_, u8>]) -> Result<(), Infallible> { Ok(()) }
# }
# struct Delay;
# impl DelayNs for Delay { async fn delay_ns(&mut self, _ns: u32) {} }
use icm45686::{
    AccelFullScale, Config, FifoConfig, FifoFrame, FifoMode, GyroFullScale, Icm45686, Int1Config,
    fifo_frames,
};

# async fn single(spi: Spi, mut delay: Delay) -> Result<(), icm45686::Error<Infallible>> {
let config = Config::new()
    .with_accel_fs(AccelFullScale::G4)
    .with_gyro_fs(GyroFullScale::Dps250)
    .with_int1(Int1Config::new().with_data_ready(true));
let mut imu = Icm45686::new(spi);
imu.init(&mut delay, &config).await?;
loop {
    // Wait for the INT1 edge here: the application owns the pin (and any timeout).
    let sample = imu.read().await?;
    let accel = sample.accel_ms2(config.accel_fs); // [f32; 3], m/s^2
    let gyro = sample.gyro_rads(config.gyro_fs); // [f32; 3], rad/s
    let temperature = sample.temperature_c(); // degrees Celsius
#   let _ = (accel, gyro, temperature);
}
# }
// FIFO: a watermark of 32 frames on INT1, read into a caller buffer, decoded in place.
# async fn batch(spi: Spi, mut delay: Delay) -> Result<(), icm45686::Error<Infallible>> {
let fifo = FifoConfig::new().with_mode(FifoMode::Stream).with_watermark(32).with_timestamp(true);
let config = Config::new().with_fifo(fifo).with_int1(Int1Config::new().with_fifo_threshold(true));
let mut imu = Icm45686::new(spi);
imu.init(&mut delay, &config).await?;
let mut buf = [0u8; 16 * 40];
// ... wait for INT1 (watermark) ...
let read = imu.fifo_read(&mut buf).await?;
for frame in fifo_frames(&buf[..read.bytes], &config.fifo) {
    if let FifoFrame::AccelGyro(f) = frame {
#       let _ = (f.accel, f.gyro, f.timestamp);
    }
}
# Ok(())
# }
```

`icm45686::blocking::Icm45686` has the same methods without `async`, on a blocking `embedded_hal::spi::SpiDevice` and `embedded_hal::delay::DelayNs`.

## What the application provides

- An SPI device: `embedded_hal_async::spi::SpiDevice` (async) or `embedded_hal::spi::SpiDevice` (`blocking`). The driver documents SPI mode 0 or 3 and a maximum clock of 24 MHz (datasheet p49, as cited in `docs/device.md`).
- A `DelayNs` for `init`, `configure`, `verify`, `recover`, `soft_reset`, `set_power`, `set_offsets`, `fifo_flush` and `self_test`.
- The INT1 pin and the waiting on it. `Config::int1` selects which events drive INT1 (data-ready, FIFO threshold, FIFO full, AGC ready) and the mode, polarity, drive and pulse width; the default is an 8 us push-pull active-high pulse with no event enabled.
- Optionally the 32 kHz clock input on pin 9: `Config::with_clock(ClockSource::ExternalClkin)`. The board must provide the oscillator; the driver only switches the device to it.

## Rules for correct use

- Never cancel a bus transfer in flight. Do not put a timeout, `select` or any other drop around a driver call that is on the bus; put the timeout around the wait for data-ready. On embassy-rp 0.10 a cancelled SPI transfer leaves stale bytes in the receive FIFO, and with a shared-bus wrapper cancelled transfers wrote stray registers into this sensor, which stopped answering until the board was power-cycled (`hil/src/bin/spi_cancel.rs`, header).
- After any doubt, call `recover()`: it re-frames the bus, checks WHO_AM_I and runs `verify()`. `verify()` is the health check: it reads back every register the applied configuration owns and returns `Error::ConfigMismatch` if one differs on two reads; it changes nothing except the indirect address registers.
- Methods return `Error::NotConfigured` until `init` or `configure` has completed. `soft_reset`, `self_test`, `set_power` and a failed or dropped `init`, `configure` or `set_power` clear that state again.
- After a `fifo_read` that reports `overflow == Some(true)` in stream mode, discard the first two frames returned (or `fifo_flush` and discard everything read). In stop-on-full mode the frames returned are the oldest.
- FIFO timestamps space frames relative to each other; they are not real time (the tick ran about 0.44 % fast on this board, also with the external clock). `fifo::timestamp_us_with_period` applies a tick period you calibrated yourself.
- After `self_test` the driver is NOT configured: call `init` or `configure` before reading again.
- `init` returns after the sensors have started up, so that the first `read()` afterwards is valid.
- After `set_power` switches the gyroscope back on, its first sample is valid but the output wanders by up to about 0.6 dps for at least 100 ms (measured on the board, 2026-11-10); discard at least the first 0.2 s after the call returns if you need settled gyroscope data.

## What the driver does

- Settings (`Config`, `Config::validate`): accel and gyro full scale and rate set separately (6.4 kHz down to 12.5 Hz; 6.25 Hz and below need low-power mode and are rejected), UI low-pass bandwidth, anti-alias FIR/interpolator, per-sensor power (`SensorPower`), clock source and RTC alignment, SPI pad slew, INT1 options.
- Single sample: `read()` returns a `RawSample` in one transaction; `accel_ms2`, `gyro_rads` and `temperature_c` convert it.
- FIFO (`FifoConfig`): bypass, stream, stop-on-full; 2 KB or 8 KB; 8-byte (one sensor), 16-byte and 20-byte high-resolution frames; watermark; decimation; 1 us or 16 us timestamps. `fifo_read`, `fifo_read_exact`, `fifo_count`, `fifo_flush`, the in-place decoder `fifo_frames`, and the unit helpers in `fifo`.
- `interrupt_status()` (INT1_STATUS0/1, both clear on read) and `stats()` (overflow, frames-read and bus-error counters).
- User offsets: `set_offsets`, `offsets`, conversion helpers in `offsets`. `init` zeroes them; `configure` writes the stored value back.
- `set_power`, `soft_reset`, and `self_test` (runs on the chip's processor; sequence from TDK's public driver; `SelfTestConfig`, `SelfTestReport`).
- `verify`, `recover`, `info`, `config`, `release`. `verify_steps` and `verify_step(delay, index)` split a `verify` into one step per owned register (one direct read, or one indirect read with its two 10 us waits; a register that differs is read once more), with the same comparison: all steps in order send what `verify` sends and end the same way, so an application can spread the check over time.
- Raw access: `read_register`, `read_registers`, `write_register`, `write_registers`, `ireg_read`, `ireg_write`, and the register map in `reg`.
- The blocking front-end `blocking::Icm45686`, one method per async method.

## Not covered

Left out on purpose (checked against `src/`):

- Low-power and ultra-low-power modes: `SensorPower` has off, standby and low-noise only, and `validate` rejects accel standby and rates of 6.25 Hz and below.
- FIFO compression and extended-header frames (they decode as `FifoFrame::Invalid`).
- INT2, I2C, I3C, AUX1/OIS, wake-on-motion and the APEX functions. FSYNC is not offered (`ClockSource` docs); a received FSYNC tag is only reported in `FifoFlags`.

In the API but never set explicitly by any program in `hil/src/bin`, so not exercised on the real sensor (established by grepping those programs for each setter and enum):

- `with_accel_bandwidth`, `with_gyro_bandwidth` (`UiBandwidth`).
- `with_accel_anti_alias`, `with_gyro_anti_alias` (`AntiAlias`). The default choice with `ExternalClkin`, which `icm45686_smoke` and `icm45686_modes` use, does write the FIR and interpolator.
- `with_rtc_align`, `with_spi_slew` (`SpiSlew`).
- Full scales other than 4 g and 16 g (accel) and 250 dps (gyro) are not set explicitly. Only `icm45686_soak` sets SPI mode 3 and a clock other than 8 MHz (18.75 MHz); the `spi_cancel` experiment is excepted.

## Measured on the MARV V2 Rev 1 board

All figures: MARV V2 Rev 1 board, RP2354B at 150 MHz, embassy-rp 0.10, 2026-11-11. They are observations, not guarantees.

- One `read()` (15 bytes on the wire): median 27 us at 7.5 MHz in SPI mode 0; 16.3 us at 18.75 MHz in SPI mode 3.
- Clock sweep 7.5 to 18.75 MHz in mode 0: 10 000 reads per point, 0 wrong. Mode 3 at 7.5 and at 18.75 MHz: 0 wrong of 10 000 each.
- On this MCU (RP2350, PL022 SPI block) a burst costs 9.5 bit times per byte in mode 0 and 8.0 in mode 3 (1.267 against 1.067 us per byte at 7.5 MHz): mode 3 saves about 16 % of bus time.
- Transfers are carried by DMA: the time of a 2040-byte burst did not grow (0.2 %) while another task kept the CPU busy.
- `fifo_frames` decode: 0.50 / 0.56 / 0.77 us per 8 / 16 / 20-byte frame.
- Driver CPU cost on top of the bare HAL for the same transaction: under 1 us in the builds where it could be measured cleanly. This figure moved by several microseconds with code placement in flash.
- Reading on every data-ready edge at 6.4 kHz for 2 s, from an interrupt executor and from the second core, while a thread-mode task spun: no edge missed, no read failed.
- Output rate at a nominal 200 Hz: 200.895 Hz on the internal oscillator, 200.000 Hz on the external 32 kHz clock (exact up to 3.2 kHz, 6388.7 Hz at the 6.4 kHz setting).
- Decimation, FIFO timestamps (1.0044 times real time, also on the external clock), overflow behaviour, mixed rates and the FIFO-full flag: see the rustdoc (`Decimation`, `TimestampResolution`, `FifoRead`, `Config::accel_odr`) and `docs/device.md`.

## Tests

Host tests, no hardware: `cargo test --locked`.

The on-board programs run through a debug probe: `tools/run.sh <name>` builds `hil/`, flashes and runs one (a lock serialises probe use). Exit status 0 means every check passed; logs go to `hil/logs/`. All of them except the `spi_cancel` experiment and `icm45686_soak` (SPI mode 3, 18.75 MHz) run SPI0 at 8 MHz; pins are in `docs/device.md`.

| Program | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `icm45686_smoke` | identity, 6.4 kHz data-ready rate, time per read, values at rest, the blocking front-end |
| `icm45686_fifo` | `verify` (with a positive control), FIFO count, `fifo_read` and decode, 20-byte high-resolution frames (20-bit samples), overflow and flush, watermark path |
| `icm45686_modes` | FIFO bypass, stop-on-full against stream, decimation, timestamp resolutions, different rates per sensor, 8-byte frames |
| `icm45686_cal` | user offsets, `set_power`, `soft_reset`, `recover` |
| `icm45686_cancel` | cancelling `read()` part-way, `recover()` after each cancel |
| `icm45686_selftest` | `self_test`, each run followed by a health check |
| `icm45686_int1` | the INT1 pin options seen on the pin: polarity, 8 us and 100 us pulse width, pulse and latched mode, push-pull and open-drain, AGC-ready source |

`spi_cancel` is an experiment and must NOT be run: its shared-bus sweep wrote stray registers into the sensor, which then needed a power cycle. It is kept as the record behind the cancel rule above.

## Cargo features and layout

- `defmt` (off by default): `defmt::Format` for the public data and error types.

```text
src/            the driver (config, plan, reg, fifo, selftest, offsets, blocking, ...)
hil/            on-board programs; its own workspace, the only user of embassy-rp
tools/run.sh    build, flash and run one on-board program
docs/device.md  register, timing and measured facts, pins, datasheet link
```

## License

MIT, see `LICENSE`. The driver is written from the datasheet linked in `docs/device.md`; the self-test sequence follows TDK's public driver (facts only, see `src/selftest.rs`).
