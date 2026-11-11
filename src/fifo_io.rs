//! The FIFO methods, `interrupt_status`, `stats` and `info` of [`Icm45686`].
//!
//! Transactions: [`fifo_read`](Icm45686::fifo_read) 3 (2 when it reads nothing),
//! [`fifo_read_exact`](Icm45686::fifo_read_exact) 1, [`fifo_count`](Icm45686::fifo_count) 2,
//! [`fifo_flush`](Icm45686::fifo_flush) 2 on a device that clears the flush bit at once (1 write, then
//! 1 read per poll), [`interrupt_status`](Icm45686::interrupt_status) 1.
//!
//! A frame whose read failed or was cancelled: the device pops bytes as the transfer clocks them out,
//! so what a failed or dropped data transaction already clocked out is gone and the driver does not
//! know how much that was. Nothing is counted in [`Stats`] for it. After such an error the safe
//! course is [`fifo_flush`](Icm45686::fifo_flush) (a frame boundary is lost if the transfer stopped
//! inside a frame), then carry on. A bus error or a cancel before the data transaction starts loses
//! nothing: the frames are still in the FIFO and the same call returns them. The host fake in the
//! tests fails or cancels before a transaction takes effect, so its tests show the second case.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

use crate::{Error, FifoRead, Icm45686, Info, InterruptStatus, Stats, image, reg};

/// Wait between two polls of the flush bit: 100 us (vendor: TDK's driver polls every 100 us).
const FLUSH_POLL_US: u32 = 100;
/// Polls of the flush bit before giving up: 10 000 of 100 us, the vendor's 1 s limit.
const FLUSH_POLLS: u32 = 10_000;

impl<SPI: SpiDevice> Icm45686<SPI> {
    /// Frame size in bytes and capacity in frames of the applied configuration.
    /// [`Error::NotConfigured`] while not configured, [`Error::FifoOff`] when the FIFO is off.
    fn fifo_geometry(&self) -> Result<(usize, u16), Error<SPI::Error>> {
        let Some(config) = self.applied.as_ref() else {
            return Err(Error::NotConfigured);
        };
        let frame = usize::from(config.fifo.frame_bytes());
        if !config.fifo.enabled() || frame == 0 {
            return Err(Error::FifoOff);
        }
        Ok((frame, config.fifo.capacity_frames()))
    }

    /// FIFO_COUNT read twice in two transactions; the second value is returned (vendor erratum
    /// AN-000364 section 2.2: "read FIFO_COUNT value twice, and use the second value"; it did not
    /// reproduce on the board, 2026-11-10: of 2000 pairs 15 differed, never by more than one frame;
    /// it stays implemented, it costs one transaction). The count is
    /// in frames (vendor; the datasheet says "packets" in 16.18, p71, and "bytes" in 5, p39). The
    /// two registers are read in one transaction as a LITTLE-endian word: little endian is the
    /// device default for sensor data, FIFO data and the FIFO count (14, p57; this crate never sets
    /// SREG_DATA_ENDIAN_SEL), so 0x12 holds the low byte. The register map (16.18 and 16.19, p71)
    /// labels 0x12 as bits 15:8 because it is drawn for big-endian mode, as it is for the sensor
    /// data registers. Count unit and byte order confirmed on the board, 2026-11-10 (322 read as
    /// 0x42 0x01).
    async fn count_twice(&mut self) -> Result<u16, Error<SPI::Error>> {
        let mut raw = [0u8; 2];
        self.read_registers(reg::FIFO_COUNT_0, &mut raw).await?;
        self.read_registers(reg::FIFO_COUNT_0, &mut raw).await?;
        Ok(u16::from_le_bytes(raw))
    }

    fn count_frames(&mut self, frames: usize) {
        self.stats.fifo_frames_read = self
            .stats
            .fifo_frames_read
            .saturating_add(u64::try_from(frames).unwrap_or(u64::MAX));
    }

    /// The number of frames in the FIFO: FIFO_COUNT read twice, the second value used (vendor
    /// erratum). Two transactions, `[Write(0x92), Read(2)]` each.
    ///
    /// [`Error::NotConfigured`] while not configured and [`Error::FifoOff`] when the FIFO is off,
    /// both with no bus access. Nothing is lost when it fails or is cancelled.
    pub async fn fifo_count(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.fifo_geometry()?;
        self.count_twice().await
    }

    /// Reads whole frames from the FIFO into the start of `buf`.
    ///
    /// Transactions: 3 (2 when nothing is read). 1 and 2: FIFO_COUNT, `[Write(0x92), Read(2)]`
    /// each; the second value is the count (vendor erratum AN-000364 section 2.2). 3: one burst
    /// `[Write(0x94), Read(n)]` from FIFO_DATA of `n = frame size * frames` bytes, where `frames` is
    /// the smaller of `count - 1` and what fits in `buf` (vendor erratum: in stream mode the FIFO is
    /// never drained completely, so at least one frame stays; reading every frame the count reports,
    /// even one too many, did no harm on the board, 2026-11-10, but the driver keeps the erratum).
    /// With a count of 0 or 1 there is no third transaction. The frame size is that of the applied
    /// [`Config`](crate::Config).
    ///
    /// Only whole frames are read, so a frame is never split. [`FifoRead::overflow`] is
    /// `Some(true)` when the count is at or above the nominal capacity of the FIFO in frames. In
    /// stream mode the count saturates two frames above the nominal capacity (130 with the 2 KB depth
    /// and 16-byte frames, 514 with 8 KB; measured on the board, 2026-11-10), so a buffer for a whole
    /// FIFO must hold capacity + 2 frames. After an overflow the stream has lost data and the first
    /// two frames returned are stale (a timestamp jump follows them): call
    /// [`fifo_flush`](Self::fifo_flush) and discard what was read. Recovery is the application's,
    /// never automatic. A returned overflow is added once to [`Stats::fifo_overflows`], the frames
    /// to [`Stats::fifo_frames_read`].
    ///
    /// Errors, with no bus access: [`Error::NotConfigured`], [`Error::FifoOff`],
    /// [`Error::BufferTooSmall`] when `buf` is shorter than one frame. A bus error or a cancel loses
    /// no frame unless it hits the data transaction itself; see the module notes on what a failed
    /// or dropped data transfer costs.
    pub async fn fifo_read(&mut self, buf: &mut [u8]) -> Result<FifoRead, Error<SPI::Error>> {
        let (frame, capacity) = self.fifo_geometry()?;
        if buf.len() < frame {
            return Err(Error::BufferTooSmall);
        }
        let count = self.count_twice().await?;
        let frames = usize::from(count.saturating_sub(1)).min(buf.len() / frame);
        let bytes = frames * frame;
        if let Some(dst) = buf.get_mut(..bytes).filter(|d| !d.is_empty()) {
            self.read_registers(reg::FIFO_DATA, dst).await?;
        }
        let overflow = count >= capacity;
        if overflow {
            self.stats.fifo_overflows = self.stats.fifo_overflows.saturating_add(1);
        }
        self.count_frames(frames);
        Ok(FifoRead {
            bytes,
            frames,
            left: Some(count.saturating_sub(u16::try_from(frames).unwrap_or(u16::MAX))),
            overflow: Some(overflow),
        })
    }

    /// Reads exactly `frames` whole frames into the start of `buf` in ONE transaction,
    /// `[Write(0x94), Read(frame size * frames)]` from FIFO_DATA, with no count read. This is the
    /// call for the watermark interrupt: the driver programs watermark N + 1 for a configured
    /// watermark of N, so reading N frames when the interrupt fires always leaves one frame in the
    /// FIFO, as the vendor erratum requires.
    ///
    /// The caller must know that `frames` frames are in the FIFO. If fewer are, the device returns
    /// what its data port returns when empty and the driver cannot tell; decode with
    /// [`fifo_frames`](crate::fifo_frames), which reports such bytes as malformed frames. The
    /// returned [`FifoRead`] has `left` and `overflow` as `None`: nothing was counted, so nothing
    /// is known about either.
    ///
    /// Errors, with no bus access: [`Error::NotConfigured`], [`Error::FifoOff`],
    /// [`Error::ZeroFrames`] when `frames` is 0, [`Error::BufferTooSmall`] when `frames` frames do
    /// not fit in `buf`. See the module notes on what a failed or dropped transfer costs.
    pub async fn fifo_read_exact(
        &mut self,
        buf: &mut [u8],
        frames: usize,
    ) -> Result<FifoRead, Error<SPI::Error>> {
        let (frame, _) = self.fifo_geometry()?;
        if frames == 0 {
            return Err(Error::ZeroFrames);
        }
        let bytes = frames
            .checked_mul(frame)
            .filter(|b| *b <= buf.len())
            .ok_or(Error::BufferTooSmall)?;
        let dst = buf.get_mut(..bytes).ok_or(Error::BufferTooSmall)?;
        self.read_registers(reg::FIFO_DATA, dst).await?;
        self.count_frames(frames);
        Ok(FifoRead {
            bytes,
            frames,
            left: None,
            overflow: None,
        })
    }

    /// Empties the FIFO: FIFO_FLUSH (bit 7 of FIFO_CONFIG2) is set with the rest of the register at
    /// its applied value (16.31, p81), then FIFO_CONFIG2 is polled every 100 us until the bit reads
    /// 0, at most 10 000 times (vendor: TDK's driver polls every 100 us with a 1 s limit); the bit
    /// is a command and clears itself. [`Error::FlushTimeout`] when it does not.
    ///
    /// Transactions: 1 write, then 1 read per poll; 2 when the bit is already clear at the first
    /// poll. The first poll comes after one 100 us wait. Frames in the FIFO are discarded; nothing
    /// is returned for them. Cancelling after the write still flushes; call it again if in doubt.
    ///
    /// [`Error::NotConfigured`] while not configured and [`Error::FifoOff`] when the FIFO is off,
    /// with no bus access.
    pub async fn fifo_flush(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        self.fifo_geometry()?;
        let Some(config) = self.applied else {
            return Err(Error::NotConfigured);
        };
        let value = image::fifo_config2(&config) | reg::FIFO_FLUSH;
        self.write_register(reg::FIFO_CONFIG2, value).await?;
        for _ in 0..FLUSH_POLLS {
            delay.delay_us(FLUSH_POLL_US).await;
            if self.read_register(reg::FIFO_CONFIG2).await? & reg::FIFO_FLUSH == 0 {
                return Ok(());
            }
        }
        Err(Error::FlushTimeout)
    }

    /// Reads INT1_STATUS0 and INT1_STATUS1 in one transaction, `[Write(0x99), Read(2)]` (the
    /// registers are adjacent, 0x19 and 0x1A). Both clear on read (R/C, 16.24, p76; 16.25, p77), so
    /// every flag the application wants is in the returned value and nowhere else. The flags are the
    /// state at the moment of the read: after the FIFO has been drained they read 0, so read the
    /// status BEFORE draining (measured on the board, 2026-11-10). The `fifo_full` flag exists only
    /// when [`Int1Config::fifo_full`](crate::Int1Config::fifo_full) is enabled. A `fifo_full`
    /// flag is added once to [`Stats::fifo_overflows`]. Works whether or not the driver is
    /// configured and does not change the validity state.
    pub async fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<SPI::Error>> {
        let mut raw = [0u8; 2];
        self.read_registers(reg::INT1_STATUS0, &mut raw).await?;
        let [status0, status1] = raw;
        let status = InterruptStatus::decode(status0, status1);
        if status.fifo_full {
            self.stats.fifo_overflows = self.stats.fifo_overflows.saturating_add(1);
        }
        Ok(status)
    }

    /// The counters; nothing clears them.
    pub const fn stats(&self) -> &Stats {
        &self.stats
    }

    /// What the last successful `init` or `configure` computed; `None` while the driver is not
    /// configured.
    pub const fn info(&self) -> Option<Info> {
        self.info
    }

    #[cfg(test)]
    pub(crate) fn set_stats(&mut self, stats: Stats) {
        self.stats = stats;
    }
}
