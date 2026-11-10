# icm45686

Driver for the TDK InvenSense ICM-45686 six-axis IMU over SPI, for the MARV V2 Rev 1 flight controller (RP2354B). It is built on the `embedded-hal` 1.0 traits only: no executor, no HAL, no pin, no timer. The crate root is the async driver. The `blocking` module is the same driver for blocking buses. It wraps the bus in an adapter that never pends and runs the same code, so nothing is written twice. All device knowledge (register map, configuration codes, decoding, units) is pure code tested on the host.

The shape is: `new(bus)`, `init(&mut delay, &config)`, `read()` (one bus transaction, integers only), unit conversion as separate pure functions, public register access, `release()`.

## Usage

Async, for example in an embassy task:

```rust
let mut imu = icm45686::Icm45686::new(spi_device);
let config = icm45686::Config { int1_data_ready: true, ..Default::default() };
imu.init(&mut delay, &config).await?;
loop {
    int1.wait_for_rising_edge().await;
    let sample = imu.read().await?;
    let accel = sample.accel_ms2(config.accel_fs);
}
```

Blocking, on any embedded-hal bus:

```rust
let mut imu = icm45686::blocking::Icm45686::new(spi_device);
imu.init(&mut delay, &icm45686::Config::default())?;
let sample = imu.read()?;
```

Units: `RawSample::accel_ms2` is m/s^2, `RawSample::gyro_rads` is rad/s, `RawSample::temperature_c` is degrees Celsius.

## What the application provides

- An SPI device: `embedded_hal_async::spi::SpiDevice` for the async driver, the blocking `embedded-hal` `SpiDevice` trait for `blocking`. SPI mode 0 or 3, up to 24 MHz.
- A `DelayNs` for `init`.
- The interrupt pin and the waiting on it. The driver owns no interrupt pin, timer or task. With `Config::int1_data_ready`, `init` routes data-ready to INT1 as an 8 us push-pull, active-high pulse; the application waits on the pin. Every sample signalled by data-ready after `init` returns is valid.

Dependencies: `embedded-hal` 1.0, `embedded-hal-async` 1.0 and, optionally, `defmt`. No embassy crate, no HAL, no other driver.

## Cargo features

All features are off by default. Without them the crate is hardware-independent `no_std` code.

| Feature | What it adds |
|---|---|
| `defmt` | `defmt::Format` for the public data and error types |

## Measured on the MARV V2 Rev 1 board

- 6.4 kHz with no missed sample.
- 26 us per read at 8 MHz SPI.
- Output rate with the internal oscillator: 200.895 Hz for a nominal 200 Hz.
- Output rate with the external clock (`ClockSource::ExternalClkin`, 32.000 kHz oscillator X1 on pin 9): 200.000 Hz. It is exact up to 3.2 kHz but 6388.7 Hz at the 6.4 kHz setting.

## Tests

On the host:

    cargo test

On the board, through a debug probe (the script takes a lock, so runs never overlap):

    tools/run.sh alive
    tools/run.sh icm45686_smoke

Used here: Rust 1.99.1 with the `thumbv8m.main-none-eabihf` target, probe-rs 0.31.0, and a debug probe on the SWD header J10.

| Binary | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `icm45686_smoke` | identity, data-ready rate with missed-sample count, time per read, plausible values at rest, the blocking front-end |

The exit status is 0 when every check passed. Full logs are kept under `hil/logs/`.

Pins the test uses: SPI0 SCK on GPIO18, MOSI on GPIO19, MISO on GPIO16, chip select on GPIO20 (a plain GPIO, 10k pull-up R48), INT1 on GPIO15. The test runs async SPI0 at 8 MHz with data-ready on GPIO15.

## Status

Version 0.1.0, as tested on the Rev 1 board.

- Host: `cargo test` passes. Clippy, rustdoc and rustfmt are clean for the driver crate, and every public item is documented.
- Board: `alive` and `icm45686_smoke` exited 0 when last run.
- On-board test program: `cargo clippy` in `hil/` reports a style finding in `icm45686_smoke`. It does not change any result.

Not done for this device:

- FIFO and batch reads, self-test, motion and embedded functions, calibration of the installed sensor.

## Layout

```
Cargo.toml       the driver crate (its own workspace root)
src/             the driver
hil/             the on-board test: one package, its own workspace; the only place that uses embassy-rp
  src/lib.rs     reports the verdict to the host
  src/bin/       alive.rs and icm45686_smoke.rs
  build.rs       links memory.x
tools/run.sh     builds, flashes and runs one on-board test
docs/device.md   register, timing and measured facts, pins, datasheet
LICENSE          MIT
```

## License

MIT, see `LICENSE`. The driver is written from the datasheet listed in `docs/device.md`.
