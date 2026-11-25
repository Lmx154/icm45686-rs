# hil

Part of [icm45686](../README.md)

On-board test programs for the MARV V2 Rev 1 board. This folder is its own workspace and the only user of embassy-rp. `tools/run.sh <name> [timeout_s]` builds one program, flashes it through a debug probe (probe-rs, chip RP235x) under a lock that serialises probe use, and runs it. Exit status 0 means every check passed; logs go to `hil/logs/`. All programs except `spi_cancel` and `icm45686_soak` (SPI mode 3, 18.75 MHz) run SPI0 at 8 MHz; pins are in [device](../docs/device.md).

| Program | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `icm45686_smoke` | identity, 6.4 kHz data-ready rate, time per read, values at rest, the blocking front-end |
| `icm45686_fifo` | `verify` (with a positive control), FIFO count, `fifo_read` and decode, 20-byte high-resolution frames, overflow and flush, watermark path |
| `icm45686_modes` | FIFO bypass, stop-on-full against stream, decimation, timestamp resolutions, different rates per sensor, 8-byte frames |
| `icm45686_cal` | user offsets, `set_power`, `soft_reset`, `recover` |
| `icm45686_cancel` | cancelling `read()` part-way, `recover()` after each cancel |
| `icm45686_selftest` | `self_test`, each run followed by a health check |
| `icm45686_int1` | INT1 options seen on the pin: polarity, 8 us and 100 us pulse width, pulse and latched mode, push-pull and open-drain, AGC-ready source |
| `icm45686_soak` | data integrity over a long run in SPI mode 3 at 18.75 MHz, driver in an interrupt-executor task under CPU and DMA load |

`spi_cancel` is an experiment and must not be run: its shared-bus sweep wrote stray registers into the sensor, which then needed a power cycle. It is kept as the record behind the cancel rule in [driver](../docs/driver.md).

## Related

[driver](../docs/driver.md) · [device](../docs/device.md)
