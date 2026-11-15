//! Driver for the TDK InvenSense ICM-45686 six-axis IMU over SPI.
//!
//! Built on the `embedded-hal` 1.0 traits only: it works with any executor or none. The crate root
//! is the async driver ([`Icm45686`], `embedded-hal-async`); [`blocking`] is the same driver on the
//! blocking traits. The device knowledge (registers, configuration, decoding, unit conversion) is
//! pure code in [`reg`], [`config`], [`validate`], [`image`], [`plan`] and [`sample`].
//!
//! The driver owns no interrupt pin and no timer: `init` can route data-ready to INT1 and the
//! application waits on the pin.
//!
//! Units: [`RawSample::accel_ms2`] is m/s^2, [`RawSample::gyro_rads`] is rad/s,
//! [`RawSample::temperature_c`] is degrees Celsius.
//!
//! For a complete example, see README.md, whose example is compiled by `cargo test`.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

mod fifo_io;
mod status;

pub mod blocking;
pub mod config;
pub mod fifo;
pub mod image;
pub mod offsets;
pub mod owned;
pub mod plan;
pub mod reg;
pub mod sample;
pub mod selftest;
pub mod validate;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::{Operation, SpiDevice};
use owned::{OwnedRegister, owned_registers};

pub use config::{
    AccelFullScale, AntiAlias, ClockSource, Config, Decimation, FifoConfig, FifoDepth, FifoFormat,
    FifoMode, GyroFullScale, Int1Config, Int1Drive, Int1Mode, Int1Polarity, Odr, PulseWidth,
    SensorPower, SpiSlew, TimestampResolution, UiBandwidth,
};
pub use fifo::{
    AccelFrame, AccelGyroFrame, FifoFlags, FifoFrame, FifoFrames, GyroFrame, HighResFrame,
    INVALID_SAMPLE_I16, fifo_frames,
};
pub use offsets::{OffsetAxis, OffsetError, Offsets};
pub use owned::{Mismatch, Space};
pub use plan::{Plan, Step};
pub use sample::RawSample;
pub use selftest::{
    AxisVerdict, SelfTestConfig, SelfTestConfigError, SelfTestReport, SelfTestStatus,
};
pub use status::{FifoRead, InterruptStatus, Stats};
pub use validate::ConfigError;

const READ: u8 = 0x80;
/// Wait between two indirect-register accesses: 10 us. The vendor's minimum is 4 us (TDK's Linux
/// driver; the datasheet's minimum gap is also 4 us, 13.3, p55); 10 us is the value that has run on
/// the board for weeks.
const IREG_WAIT_US: u32 = 10;
/// Soft-reset polling: first wait, then up to `RESET_POLLS` polls this far apart.
const RESET_FIRST_WAIT_US: u32 = 1_000;
const RESET_POLL_US: u32 = 500;
const RESET_POLLS: u32 = 100;

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<E> {
    /// SPI bus error.
    Bus(E),
    /// WHO_AM_I did not read 0xE9.
    WrongId {
        /// Value read.
        found: u8,
    },
    /// RESET_DONE did not appear after a soft reset.
    ResetTimeout,
    /// No configuration is applied: `init` or `configure` has not completed since the driver was
    /// created or since the last one that failed or was dropped.
    NotConfigured,
    /// The [`Config`] cannot be applied. Found before any bus access.
    InvalidConfig(ConfigError),
    /// `verify` read a register that does not hold the applied configuration, twice.
    ConfigMismatch(Mismatch),
    /// A FIFO method was called but the FIFO is off in the applied [`Config`]. No bus access.
    FifoOff,
    /// The buffer is shorter than one frame (`fifo_read`) or than the requested frames
    /// (`fifo_read_exact`). No bus access.
    BufferTooSmall,
    /// `fifo_read_exact` was asked for 0 frames. No bus access.
    ZeroFrames,
    /// FIFO_FLUSH did not clear within the polling limit of `fifo_flush`.
    FlushTimeout,
    /// An [`Offsets`] value is outside -8192..=8191. Found before any bus access.
    InvalidOffsets(OffsetError),
    /// The [`SelfTestConfig`] cannot be run. Found before any bus access.
    InvalidSelfTestConfig(SelfTestConfigError),
    /// The self-test-done bit did not appear within 3 s (see [`selftest`]).
    SelfTestTimeout {
        /// The last INT_APEX_STATUS1 value read.
        apex_status: u8,
    },
}

/// What `init` and `configure` learned and applied.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Info {
    /// WHO_AM_I as read (0xE9 for this chip).
    pub who_am_i: u8,
    /// Accelerometer output rate in millihertz, nominal from the [`Config`]; 0 unless the
    /// accelerometer is in low-noise mode.
    pub accel_odr_mhz: u32,
    /// Gyroscope output rate in millihertz, nominal from the [`Config`]; 0 unless the gyroscope
    /// is in low-noise mode.
    pub gyro_odr_mhz: u32,
    /// FIFO frame size in bytes; 0 when the FIFO is off.
    pub fifo_frame_bytes: u8,
}

/// Nominal output rate in millihertz (1.5625 Hz rounds to 1563; low-noise mode rejects it).
const fn odr_mhz(odr: Odr) -> u32 {
    match odr {
        Odr::Hz6400 => 6_400_000,
        Odr::Hz3200 => 3_200_000,
        Odr::Hz1600 => 1_600_000,
        Odr::Hz800 => 800_000,
        Odr::Hz400 => 400_000,
        Odr::Hz200 => 200_000,
        Odr::Hz100 => 100_000,
        Odr::Hz50 => 50_000,
        Odr::Hz25 => 25_000,
        Odr::Hz12_5 => 12_500,
        Odr::Hz6_25 => 6_250,
        Odr::Hz3_125 => 3_125,
        Odr::Hz1_5625 => 1_563,
    }
}

const fn running_mhz(power: SensorPower, odr: Odr) -> u32 {
    match power {
        SensorPower::LowNoise => odr_mhz(odr),
        _ => 0,
    }
}

/// What `apply` does with the user offsets after the plan.
#[derive(Clone, Copy)]
enum Calibration {
    /// Nothing: the plan did not touch the offset registers and the stored value stays.
    Keep,
    /// The plan wrote the twelve zeros: store zero when marking the driver configured.
    Zeroed,
    /// Write the twelve registers from the stored value; the stored value stays.
    Restore,
}

/// ICM-45686 on an async SPI device (mode 0 or 3, up to 24 MHz).
///
/// The bus must be SPI mode 0 or 3 (AP_SPI_MODE, datasheet p90) and run at 24 MHz at most (p49).
///
/// Never cancel a bus transfer in flight: do not put a timeout or `select` around a driver call
/// that is on the bus; put the timeout around the wait for data-ready. On embassy-rp 0.10 a
/// cancelled SPI transfer leaves stale bytes in the receive FIFO, and with a shared-bus wrapper
/// cancelled transfers wrote stray registers into this sensor, which stopped answering until the
/// board was power-cycled. [`recover`](Self::recover) is the way back.
///
/// # Validity
///
/// The driver is "configured" only between the successful end of [`init`](Self::init),
/// [`configure`](Self::configure) or [`set_power`](Self::set_power) and the start of the next one of
/// them or of [`soft_reset`](Self::soft_reset). Each clears it before its first await, so a failure
/// or a dropped future leaves the driver not configured, and [`read`](Self::read) returns
/// [`Error::NotConfigured`] until a call completes. [`set_offsets`](Self::set_offsets) does not
/// change it. The raw register methods
/// ([`read_register`](Self::read_register), [`read_registers`](Self::read_registers),
/// [`write_register`](Self::write_register), [`write_registers`](Self::write_registers),
/// [`ireg_read`](Self::ireg_read), [`ireg_write`](Self::ireg_write)) are outside this: they neither
/// check nor change the state.
///
/// # Transactions
///
/// Every register read is one SPI transaction `[Write(address with read bit), Read(n bytes)]`,
/// never an equal-length transfer. [`read`](Self::read) is one transaction (14 bytes).
pub struct Icm45686<SPI> {
    spi: SPI,
    applied: Option<Config>,
    /// The user offsets the driver has written and the device holds. Changed only after the device
    /// was written: set to zero by `init` when it marks the driver configured, replaced by
    /// `set_offsets` after its last write. `configure` writes this value to the registers and leaves
    /// it. A failed or dropped call leaves it as it was. Meaningful while `applied` is `Some`.
    offsets: Offsets,
    info: Option<Info>,
    stats: Stats,
}

impl<SPI: SpiDevice> Icm45686<SPI> {
    /// Wraps the SPI device. Nothing is sent. The driver starts not configured.
    pub const fn new(spi: SPI) -> Self {
        Self {
            spi,
            applied: None,
            offsets: Offsets::new(),
            info: None,
            stats: Stats::new(),
        }
    }

    /// Returns the SPI device.
    pub fn release(self) -> SPI {
        self.spi
    }

    /// The applied configuration; `None` while the driver is not configured.
    pub fn config(&self) -> Option<&Config> {
        self.applied.as_ref()
    }

    /// The user offsets the driver has written and the device holds; `None` while the driver is not
    /// configured. No bus access.
    ///
    /// The stored value changes only when the device has been written: it is zero after
    /// [`init`](Self::init) (set when `init` marks the driver configured), replaced by
    /// [`set_offsets`](Self::set_offsets) after its last write, and written to the device again by
    /// [`configure`](Self::configure) without changing it. [`soft_reset`](Self::soft_reset),
    /// [`set_power`](Self::set_power), [`self_test`](Self::self_test) and any failed or dropped call
    /// leave it as it was.
    pub fn offsets(&self) -> Option<Offsets> {
        self.applied.map(|_| self.offsets)
    }

    /// Soft reset, identity check, then the whole [`Config`] through [`Plan::after_reset`]. Returns
    /// after the sensors have started up, so that the first `read` is valid.
    ///
    /// The driver is not configured from the start of this call until it returns `Ok`.
    ///
    /// The user offsets are zero afterwards: the plan writes the twelve offset registers to zero, 12
    /// more transactions than a plan without them. The device clears them itself on a soft reset
    /// (measured on the board, 2026-11-10: they read 0 right after one), but the datasheet gives only
    /// their reset value (0x00, 20.1-21.6, p189-p194) and does not say that a soft reset restores
    /// it, so `init` writes the zeros and the result never depends on it.
    ///
    /// The stored [`Offsets`] change only when `init` marks the driver configured, after the zero
    /// writes. An `init` that fails or is dropped (an invalid [`Config`] included) leaves them as
    /// they were, even when the device was already reset, and [`configure`](Self::configure) then
    /// writes them back.
    ///
    /// Transactions: 22 for the default [`Config`] and 37 with the external clock, the twelve zero
    /// writes included (the status read, the reset write, one reset poll and WHO_AM_I make 4 of
    /// them). Awaits: 36 and 63.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<SPI::Error>> {
        self.applied = None;
        self.info = None;
        let plan = Plan::after_reset(config).map_err(Error::InvalidConfig)?;
        self.soft_reset(delay).await?;
        let who_am_i = self.check_identity().await?;
        self.apply(delay, config, &plan, who_am_i, Calibration::Zeroed)
            .await
    }

    /// Applies `config` to a device that is not reset ([`Plan::reconfigure`]: sensors, INT1 events
    /// and FIFO off first, then every register this crate controls). WHO_AM_I is checked first.
    ///
    /// The driver is not configured from the start of this call until it returns `Ok`.
    ///
    /// The user offsets are calibration, not configuration, and this restores them: after the plan
    /// and before it marks the driver configured it writes all twelve offset registers from the
    /// stored [`Offsets`] (the writes of [`set_offsets`](Self::set_offsets)), 12 more transactions
    /// than the plan and the identity check (53 transactions and 89 awaits for the default
    /// [`Config`], 57 and 96 with the external clock), and leaves the stored value as it is. So after any
    /// successful `configure` the device holds what the driver stores, whatever came before: an
    /// `init` that failed, a `set_offsets` that was cut off, a driver created with [`new`](Self::new)
    /// on a device that another program calibrated (it stores zero, so the registers are cleared).
    /// To change the offsets, call [`set_offsets`](Self::set_offsets).
    pub async fn configure(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<SPI::Error>> {
        self.applied = None;
        self.info = None;
        let plan = Plan::reconfigure(config).map_err(Error::InvalidConfig)?;
        let who_am_i = self.check_identity().await?;
        self.apply(delay, config, &plan, who_am_i, Calibration::Restore)
            .await
    }

    /// Reads back the registers the applied configuration owns (see [`crate::owned`]) and compares
    /// each under its mask. [`Error::NotConfigured`] while the driver is not configured, with no
    /// bus access.
    ///
    /// Order: the direct registers by ascending address, then the indirect registers by ascending
    /// address. A direct register is one transaction `[Write(address with read bit), Read(1)]`; an
    /// indirect register is [`ireg_read`](Self::ireg_read): one address write, a 10 us wait, one data
    /// read, a 10 us wait. A register that differs is read once more, and only if it differs again
    /// the method returns [`Error::ConfigMismatch`] (`found` is the whole byte of the second read,
    /// not masked).
    ///
    /// The twelve user-offset registers are owned under the stored [`Offsets`] (the high registers
    /// under bits 5:0).
    ///
    /// Transactions when nothing differs: 15 direct + 2 per indirect register. That is 51 for the
    /// default [`Config`] (15 + 18 indirect) and 53 with the external clock (15 + 19 indirect); each
    /// difference adds 1 (direct) or 2 (indirect). Awaits: 1 per direct register, 4 per indirect.
    ///
    /// Writes only the indirect address registers (as an indirect read does). Never changes the
    /// validity state or the stored [`Config`]; a mismatch is for the application to act on
    /// (read on, [`configure`](Self::configure), or switch sensors).
    pub async fn verify(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        let Some(config) = self.applied else {
            return Err(Error::NotConfigured);
        };
        let owned = owned_registers(&config, &self.offsets);
        for item in owned.registers() {
            if self.read_owned(delay, item).await? & item.mask == item.expected {
                continue;
            }
            let found = self.read_owned(delay, item).await?;
            if found & item.mask != item.expected {
                return Err(Error::ConfigMismatch(Mismatch {
                    space: item.space,
                    register: item.register,
                    mask: item.mask,
                    expected: item.expected,
                    found,
                }));
            }
        }
        Ok(())
    }

    /// The number of [`verify_step`](Self::verify_step) indices: one per register
    /// [`verify`](Self::verify) reads (33 for the default [`Config`], 34 with the external clock);
    /// 0 when not configured.
    pub fn verify_steps(&self) -> usize {
        self.applied.map_or(0, |config| {
            owned_registers(&config, &self.offsets).registers().len()
        })
    }

    /// Checks the `index`-th register that [`verify`](Self::verify) reads, with the same comparison:
    /// a register that differs is read once more, and only if it differs again the method returns
    /// [`Error::ConfigMismatch`]. Every index from 0 to [`verify_steps`](Self::verify_steps) - 1, in
    /// order, sends what one `verify` sends and ends the same way, so an application can spread a
    /// `verify` over time. A direct register costs one transaction; an indirect one two transactions
    /// and two 10 us waits; a register that differs is read once more. An index at or past
    /// `verify_steps()` sends nothing and returns `Ok`. [`Error::NotConfigured`] when not configured.
    pub async fn verify_step(
        &mut self,
        delay: &mut impl DelayNs,
        index: usize,
    ) -> Result<(), Error<SPI::Error>> {
        let Some(config) = self.applied else {
            return Err(Error::NotConfigured);
        };
        let owned = owned_registers(&config, &self.offsets);
        let Some(item) = owned.registers().get(index) else {
            return Ok(());
        };
        if self.read_owned(delay, item).await? & item.mask == item.expected {
            return Ok(());
        }
        let found = self.read_owned(delay, item).await?;
        if found & item.mask != item.expected {
            return Err(Error::ConfigMismatch(Mismatch {
                space: item.space,
                register: item.register,
                mask: item.mask,
                expected: item.expected,
                found,
            }));
        }
        Ok(())
    }

    /// Re-frames the bus, checks the identity and, when configured, runs [`verify`](Self::verify).
    ///
    /// 1. One transaction holding a single `Operation::Write` of one byte, the read address of
    ///    WHO_AM_I (`0xF2`) and nothing else. On embassy-rp 0.10 it drains the receive FIFO that a
    ///    cancelled transfer left behind (measured on the board).
    /// 2. The identity check, one read; [`Error::WrongId`] on a mismatch.
    /// 3. [`verify`](Self::verify); [`Error::NotConfigured`] when the driver is not configured.
    ///
    /// Transactions with no mismatch: 2 + the count of `verify` (53 for the default [`Config`], 55
    /// with the external clock); 2 when not configured. No device register is written other than
    /// the indirect address registers that an indirect read needs. The validity state and the stored
    /// [`Config`] are never changed. Cancelling between transactions is safe; call it again.
    pub async fn recover(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        let r = self.spi.write(&[reg::WHO_AM_I | READ]).await;
        self.bus(r)?;
        self.check_identity().await?;
        self.verify(delay).await
    }

    /// One owned register, as read (not masked).
    async fn read_owned(
        &mut self,
        delay: &mut impl DelayNs,
        item: &OwnedRegister,
    ) -> Result<u8, Error<SPI::Error>> {
        match item.space {
            Space::Direct => self.read_register(item.register as u8).await,
            Space::Indirect => self.ireg_read(delay, item.register).await,
        }
    }

    /// Reads the latest sample in one SPI transaction: `[Write(1 byte), Read(14 bytes)]`.
    /// [`Error::NotConfigured`] while the driver is not configured.
    pub async fn read(&mut self) -> Result<RawSample, Error<SPI::Error>> {
        if self.applied.is_none() {
            return Err(Error::NotConfigured);
        }
        let mut data = [0u8; 14];
        self.read_registers(reg::ACCEL_DATA_X1_UI, &mut data)
            .await?;
        Ok(RawSample::decode(&data))
    }

    /// Reads one register (one transaction). Outside the validity state.
    pub async fn read_register(&mut self, addr: u8) -> Result<u8, Error<SPI::Error>> {
        let mut b = [0u8; 1];
        self.read_registers(addr, &mut b).await?;
        Ok(b[0])
    }

    /// Reads `buf.len()` consecutive registers starting at `addr` in one transaction
    /// `[Write(address with read bit), Read(buf)]`. Outside the validity state.
    pub async fn read_registers(
        &mut self,
        addr: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<SPI::Error>> {
        let cmd = [addr | READ];
        let r = self
            .spi
            .transaction(&mut [Operation::Write(&cmd), Operation::Read(buf)])
            .await;
        self.bus(r)
    }

    /// Writes one register (one `Operation::Write`). Outside the validity state.
    pub async fn write_register(&mut self, addr: u8, value: u8) -> Result<(), Error<SPI::Error>> {
        let r = self.spi.write(&[addr & !READ, value]).await;
        self.bus(r)
    }

    /// Writes consecutive registers starting at `addr` in one transaction. Outside the validity
    /// state.
    pub async fn write_registers(
        &mut self,
        addr: u8,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        let cmd = [addr & !READ];
        let r = self
            .spi
            .transaction(&mut [Operation::Write(&cmd), Operation::Write(data)])
            .await;
        self.bus(r)
    }

    /// Reads an indirect register (`reg::IPREG_*` base plus offset): the address write, a 10 us wait,
    /// the data read, a 10 us wait. Outside the validity state.
    pub async fn ireg_read(
        &mut self,
        delay: &mut impl DelayNs,
        addr: u16,
    ) -> Result<u8, Error<SPI::Error>> {
        let [hi, lo] = addr.to_be_bytes();
        let r = self.spi.write(&[reg::IREG_ADDR_15_8, hi, lo]).await;
        self.bus(r)?;
        delay.delay_us(IREG_WAIT_US).await;
        let value = self.read_register(reg::IREG_DATA).await?;
        delay.delay_us(IREG_WAIT_US).await;
        Ok(value)
    }

    /// Writes an indirect register (`reg::IPREG_*` base plus offset): address and data in one write,
    /// then a 10 us wait. Outside the validity state.
    pub async fn ireg_write(
        &mut self,
        delay: &mut impl DelayNs,
        addr: u16,
        value: u8,
    ) -> Result<(), Error<SPI::Error>> {
        let [hi, lo] = addr.to_be_bytes();
        let r = self.spi.write(&[reg::IREG_ADDR_15_8, hi, lo, value]).await;
        self.bus(r)?;
        delay.delay_us(IREG_WAIT_US).await;
        Ok(())
    }

    async fn ireg_modify(
        &mut self,
        delay: &mut impl DelayNs,
        addr: u16,
        mask: u8,
        value: u8,
    ) -> Result<(), Error<SPI::Error>> {
        let old = self.ireg_read(delay, addr).await?;
        self.ireg_write(delay, addr, (old & !mask) | (value & mask))
            .await
    }

    /// Soft reset: reads INT1_STATUS0 to clear it, writes SOFT_RST to REG_MISC2 (16.84, p109), waits
    /// 1 ms, then reads INT1_STATUS0 until RESET_DONE shows (16.24, p76), up to 100 reads 500 us
    /// apart; [`Error::ResetTimeout`] if it never does. It is the reset `init` uses.
    ///
    /// Afterwards the driver is NOT configured: [`read`](Self::read) returns [`Error::NotConfigured`]
    /// until [`init`](Self::init) or [`configure`](Self::configure), and [`offsets`](Self::offsets)
    /// returns `None`. The validity state is cleared before the first await. WHO_AM_I is not read.
    ///
    /// The stored [`Offsets`] are not touched: they change only when the device is written. The
    /// device clears its offset registers on a soft reset (measured on the board, 2026-11-10: they
    /// read 0 right after one), so after this call the stored value may differ from the device until
    /// `init` (which writes zeros) or `configure` (which writes the stored value) completes.
    ///
    /// Transactions: 3 when RESET_DONE shows at the first poll (read, write, read), plus 1 per
    /// further poll; 102 on a timeout. Awaits: 1 per transaction and 1 per wait.
    pub async fn soft_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        self.applied = None;
        self.info = None;
        self.read_register(reg::INT1_STATUS0).await?;
        self.write_register(reg::REG_MISC2, reg::SOFT_RST).await?;
        delay.delay_us(RESET_FIRST_WAIT_US).await;
        for _ in 0..RESET_POLLS {
            let status = self.read_register(reg::INT1_STATUS0).await?;
            if status != 0xFF && status & reg::RESET_DONE != 0 {
                return Ok(());
            }
            delay.delay_us(RESET_POLL_US).await;
        }
        Err(Error::ResetTimeout)
    }

    /// Writes the twelve offset registers (low byte, then high register, gyro X, Y, Z, accel X, Y,
    /// Z) with [`ireg_write`](Self::ireg_write), then stores `offsets` as applied. The driver must be
    /// configured ([`Error::NotConfigured`], no bus access) and every value in -8192..=8191
    /// ([`Error::InvalidOffsets`], no bus access). Raw values are two's complement in 14 bits; a
    /// positive value raises the output (measured on the board, 2026-11-10); the conversion helpers
    /// are in [`offsets`].
    ///
    /// Transactions: 12 writes (one each, 24 awaits with the 10 us waits). A 14-bit value is two
    /// registers, so a read between the two writes of one value sees a half-written offset.
    ///
    /// The validity state is not changed. The stored offsets change only after the last write
    /// succeeded. If a write fails or the future is dropped part-way, the stored offsets stay as they
    /// were while some registers already hold the new values: [`verify`](Self::verify) then reports
    /// the first register that changed ([`Error::ConfigMismatch`]); calling `set_offsets` again with
    /// the same values writes all twelve and puts it right, and so does [`configure`](Self::configure),
    /// which writes the old stored values back.
    pub async fn set_offsets(
        &mut self,
        delay: &mut impl DelayNs,
        offsets: &Offsets,
    ) -> Result<(), Error<SPI::Error>> {
        if self.applied.is_none() {
            return Err(Error::NotConfigured);
        }
        offsets.validate().map_err(Error::InvalidOffsets)?;
        self.write_offset_registers(delay, offsets).await?;
        self.offsets = *offsets;
        Ok(())
    }

    /// The twelve writes of [`set_offsets`](Self::set_offsets); stores nothing.
    async fn write_offset_registers(
        &mut self,
        delay: &mut impl DelayNs,
        offsets: &Offsets,
    ) -> Result<(), Error<SPI::Error>> {
        for (addr, value, _) in offsets.registers() {
            self.ireg_write(delay, addr, value).await?;
        }
        Ok(())
    }

    /// Changes only the power state of the two sensors of a configured driver and returns the new
    /// [`Info`]. [`Error::NotConfigured`] when not configured; [`Error::InvalidConfig`] when the
    /// applied [`Config`] with these states is invalid (for example a FIFO frame that holds data of a
    /// sensor that would not run), found before any bus access and with the driver unchanged.
    ///
    /// The steps are [`Plan::power_change`]: one write of PWR_MGMT0 (16.17, p70), with the gyroscope
    /// stop wait of 150 ms before it when the gyroscope goes from off to on, and after it the
    /// start-up wait of 60 ms when a sensor goes up a state (off, standby, low-noise; the gyroscope's
    /// wait when it does, else the accelerometer's). No other register is written: the offset
    /// registers are not written and the stored [`Offsets`] are left as they are.
    ///
    /// A sensor that is powered off reads 0x8000 (-32768) on all three of its axes while the other
    /// sensor keeps producing data (measured on the board, 2026-11-10). After the gyroscope is powered
    /// on again the first sample is valid but the output wanders by up to about 0.6 dps for at least
    /// 100 ms (measured on the board, 2026-11-10); an application that needs settled gyroscope data
    /// should discard at least the first 0.2 s after this call returns.
    ///
    /// Transactions: 1 for every transition, also when nothing changes (no wait then). Awaits: the
    /// transaction plus the waits above, so 1 (no sensor turned on), 2 (a sensor turned on), or 3
    /// (gyroscope off to on).
    ///
    /// The driver is not configured from the start of the bus access until the call returns `Ok`;
    /// after a failure or a dropped future it is not configured (the register may or may not have been
    /// written), so call [`configure`](Self::configure).
    pub async fn set_power(
        &mut self,
        delay: &mut impl DelayNs,
        accel: SensorPower,
        gyro: SensorPower,
    ) -> Result<Info, Error<SPI::Error>> {
        let Some(old) = self.applied else {
            return Err(Error::NotConfigured);
        };
        let new = old.with_accel_power(accel).with_gyro_power(gyro);
        let plan = Plan::power_change(&old, &new).map_err(Error::InvalidConfig)?;
        let who_am_i = self.info.map_or(reg::WHO_AM_I_VALUE, |i| i.who_am_i);
        self.applied = None;
        self.info = None;
        self.apply(delay, &new, &plan, who_am_i, Calibration::Keep)
            .await
    }

    async fn check_identity(&mut self) -> Result<u8, Error<SPI::Error>> {
        let found = self.read_register(reg::WHO_AM_I).await?;
        if found != reg::WHO_AM_I_VALUE {
            return Err(Error::WrongId { found });
        }
        Ok(found)
    }

    /// Runs `plan`, then `calibration`, stores `config` and only then marks the driver configured.
    async fn apply(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
        plan: &Plan,
        who_am_i: u8,
        calibration: Calibration,
    ) -> Result<Info, Error<SPI::Error>> {
        for step in plan {
            match *step {
                Step::Write { reg, value } => self.write_register(reg, value).await?,
                Step::Modify { reg, mask, value } => {
                    let old = self.read_register(reg).await?;
                    self.write_register(reg, (old & !mask) | (value & mask))
                        .await?;
                }
                Step::IregWrite { addr, value } => self.ireg_write(delay, addr, value).await?,
                Step::IregModify { addr, mask, value } => {
                    self.ireg_modify(delay, addr, mask, value).await?;
                }
                Step::DelayUs(us) => delay.delay_us(us).await,
            }
        }
        match calibration {
            Calibration::Keep => {}
            Calibration::Zeroed => self.offsets = Offsets::new(),
            Calibration::Restore => {
                let stored = self.offsets;
                self.write_offset_registers(delay, &stored).await?;
            }
        }
        self.applied = Some(*config);
        let info = Info {
            who_am_i,
            accel_odr_mhz: running_mhz(config.accel_power, config.accel_odr),
            gyro_odr_mhz: running_mhz(config.gyro_power, config.gyro_odr),
            fifo_frame_bytes: if config.fifo.enabled() {
                config.fifo.frame_bytes()
            } else {
                0
            },
        };
        self.info = Some(info);
        Ok(info)
    }

    fn bus<T>(&mut self, result: Result<T, SPI::Error>) -> Result<T, Error<SPI::Error>> {
        result.map_err(|e| {
            self.stats.bus_errors = self.stats.bus_errors.saturating_add(1);
            Error::Bus(e)
        })
    }
}

#[cfg(test)]
mod driver_tests;
#[cfg(test)]
mod fifo_tests;
#[cfg(test)]
mod power_tests;
#[cfg(test)]
mod selftest_tests;
#[cfg(test)]
mod testbus;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod verify_tests;

#[cfg(doctest)]
#[doc = include_str!("../README.md")]
pub struct ReadmeDoctests;
