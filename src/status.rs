//! Values the FIFO and interrupt methods return: [`FifoRead`], [`InterruptStatus`] and the driver's
//! saturating counters [`Stats`]. Plain data, no bus access.

use crate::reg;

/// What one `fifo_read` or `fifo_read_exact` call did.
///
/// The fields that need a FIFO count are `Option`s: `fifo_read_exact` reads no count, so it returns
/// `None` there, which cannot be mistaken for "0 frames left" or "no overflow". `Default` is "nothing
/// read, nothing known".
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct FifoRead {
    /// Bytes written to the start of the caller's buffer: `frames` times the frame size.
    pub bytes: usize,
    /// Whole frames read.
    pub frames: usize,
    /// Frames left in the device after this call, as far as the count read at the start of the call
    /// knows (frames that arrived since are not included). `None` when no count was read
    /// (`fifo_read_exact`).
    pub left: Option<u16>,
    /// `Some(true)` when the count was at or above the nominal capacity of the FIFO in frames for
    /// the applied configuration, which is a FIFO that is full and, in stream mode, is overwriting
    /// its oldest frames; `Some(false)` when it was not. In stream mode the count saturates two
    /// frames above the nominal capacity (measured on the board, 2026-11-10: 130 with the 2 KB depth
    /// and 16-byte frames, 514 with 8 KB). After an overflow in stream mode the stream has lost data
    /// and the first two frames returned are stale (measured on the MARV V2 board, 2026-11-11: a
    /// timestamp jump follows them and the rest are contiguous up to the time of the read): discard
    /// those two, or call `fifo_flush` and discard what was read. In stop-on-full mode the count stops
    /// at the capacity and the frames read are the oldest, with no jump; later frames are lost.
    /// `None` when no count was read (`fifo_read_exact`): then nothing is known about an overflow.
    pub overflow: Option<bool>,
}

impl FifoRead {
    /// The empty value, the same as `Default`.
    pub const fn new() -> Self {
        Self {
            bytes: 0,
            frames: 0,
            left: None,
            overflow: None,
        }
    }
}

/// INT1_STATUS0 and INT1_STATUS1 as read. Both registers clear on read (R/C, 16.24, p76; 16.25,
/// p77): a flag that is not looked at in the value this returns is gone. Both flags are the state
/// at the moment of the read and the read clears them: after the FIFO has been drained they read 0,
/// so read the status BEFORE draining the FIFO. The `fifo_full` flag: with only the threshold source
/// enabled it stayed 0 through an overflow (measured on the board, 2026-11-10); with its INT1 source
/// ([`Int1Config::fifo_full`](crate::Int1Config::fifo_full)) enabled it was set after an overflow
/// (measured on the MARV V2 board, 2026-11-11). To see FIFO full here, enable that source;
/// [`FifoRead::overflow`] reports it either way.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct InterruptStatus {
    /// INT1_STATUS_DRDY, bit 2 of INT1_STATUS0.
    pub data_ready: bool,
    /// INT1_STATUS_FIFO_THS, bit 1: FIFO count at or above the threshold.
    pub fifo_threshold: bool,
    /// INT1_STATUS_FIFO_FULL, bit 0.
    pub fifo_full: bool,
    /// INT1_STATUS_AP_AGC_RDY, bit 5.
    pub agc_ready: bool,
    /// INT1_STATUS_RESET_DONE, bit 7.
    pub reset_done: bool,
    /// INT1_STATUS0 as read.
    pub status0: u8,
    /// INT1_STATUS1 as read.
    pub status1: u8,
}

impl InterruptStatus {
    /// Decodes the two bytes.
    pub const fn decode(status0: u8, status1: u8) -> Self {
        Self {
            data_ready: status0 & reg::DRDY != 0,
            fifo_threshold: status0 & reg::FIFO_THS != 0,
            fifo_full: status0 & reg::FIFO_FULL != 0,
            agc_ready: status0 & reg::AP_AGC_RDY != 0,
            reset_done: status0 & reg::RESET_DONE != 0,
            status0,
            status1,
        }
    }
}

/// Counters of what the driver saw. They saturate at their maximum and nothing clears them: not
/// `init`, not `configure`, not an error. Read them with `stats()`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Stats {
    /// Calls whose returned result showed a FIFO overflow: a `fifo_read` with `overflow` set or an
    /// `interrupt_status` with `fifo_full` set. One per call.
    pub fifo_overflows: u32,
    /// Frames returned by `fifo_read` and `fifo_read_exact`.
    pub fifo_frames_read: u64,
    /// Every [`Error::Bus`](crate::Error::Bus) the driver returns, whichever method returns it:
    /// the raw register methods (including the indirect-register ones), `init`, `configure`,
    /// `verify`, `recover`, `read` and the FIFO and interrupt methods. Each bus error is counted
    /// once, where it happens.
    pub bus_errors: u32,
}

impl Stats {
    /// All counters zero.
    pub const fn new() -> Self {
        Self {
            fifo_overflows: 0,
            fifo_frames_read: 0,
            bus_errors: 0,
        }
    }
}
