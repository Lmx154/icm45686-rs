//! The self-test, which runs inside the chip's embedded processor (eDMP).
//!
//! The datasheet (DS-000489 Rev 1.1) describes the registers but not the sequence (4.13, p37 only
//! defines the self-test response). The sequence below comes from TDK's public driver ("vendor"):
//! github.com/tdk-invn-oss/zephyr.hal_tdk, commit fa54cb6, folder `icm456xx/imu`, BSD-3-Clause;
//! facts only, no code. File names below are in that folder.
//!
//! # Sequence, as [`Icm45686::self_test`] runs it
//!
//! Every indirect access is [`Icm45686::ireg_write`] or [`Icm45686::ireg_read`] (one address
//! write per byte, 10 us after each transaction; the vendor waits 4 us, inv_imu_transport.c:97-154).
//!
//! 1. Soft reset ([`Icm45686::soft_reset`]). Vendor: icm456xx_adv_device_reset,
//!    inv_imu_selftest.c:41, inv_imu_driver_advanced.c:51, icm456xx_soft_reset inv_imu_driver.c:16.
//! 2. Wait 10 ms. Vendor: inv_imu_selftest.c:43.
//! 3. eDMP start addresses (icm456xx_edmp_configure, inv_imu_edmp.c:795-811; values from
//!    inv_imu_edmp.c:10-12 and inv_imu_edmp_defs.h:15,17; the vendor writes the three 16-bit
//!    addresses low byte first): EDMP_PRGRM_IRQ0_0..IRQ2_1 (0xA24F..0xA254, 19.40-19.45,
//!    p176-p177) = 0x00, 0x40, 0x04, 0x40, 0x08, 0x40 (IRQ0 at 0x4000, IRQ1 at 0x4004, IRQ2 at
//!    0x4008, the eDMP ROM); EDMP_SP_START_ADDR (0xA255, 19.46, p178) = 0x05 (stack end 0x500,
//!    upper byte).
//! 4. FIFO_SRAM_SLEEP (0xA2A7, 19.69, p188) bits 1:0 = 3, read-modify-write: SRAM awake. Vendor:
//!    icm456xx_adv_power_up_sram, inv_imu_selftest.c:68, inv_imu_driver_advanced.c:1291-1300.
//! 5. The parameter word ([`SelfTestConfig::param_word`]) to IMEM_SRAM 0x38 (low byte,
//!    IMEM_SRAM_REG_56, 17.13, p113) and 0x39 (high byte, IMEM_SRAM_REG_57, 17.14, p113-p114), then
//!    0 to 0x3A and 0x3B, the upper half of the vendor's 32-bit word. Vendor: inv_imu_selftest.c:65,
//!    70-77, EDMP_STC_CONFIGPARAMS inv_imu_edmp_memmap.h:579-590.
//! 6. IMEM_SRAM 0x40 (ST_DEBUG_EN, IMEM_SRAM_REG_64, 17.15, p114) = 0: "must be set to 0 whenever
//!    self-test is requested"; then 0 to 0x41..=0x43, the rest of the vendor's 32-bit debug word.
//!    Vendor: inv_imu_selftest.c:79-80, EDMP_STC_DEBUG_EN inv_imu_edmp_memmap.h:603-608.
//! 7. IMEM_SRAM 0x3C..=0x3F (EDMP_STC_PATCH_EN, "enable SRAM patching") = 0, the vendor's default
//!    patch setting. Vendor: inv_imu_selftest.c:28, 82, inv_imu_edmp_memmap.h:592-601.
//!
//!    The nine bytes 0x3A, 0x3B, 0x3C..=0x3F and 0x41..=0x43 are not in the datasheet (its map,
//!    15.2, p60, lists 0x38, 0x39, 0x40 and 0x44). They are written anyway (lead's decision,
//!    2026-11-10) because they are volatile RAM of the chip's processor, not interface or pad
//!    registers; the vendor's own driver writes exactly these zeros; on the board they held random
//!    values after a reset, including set bits in the patch-enable word, so leaving them unwritten
//!    is the unsafe choice; and the sequence ends in a soft reset.
//! 8. REG_HOST_MSG (0x73, 16.80, p108) bit 0 TESTOPENABLE = 1, read-modify-write. Vendor:
//!    inv_imu_selftest.c:94-96.
//! 9. INT_APEX_CONFIG1 (0x3A, 16.53, p96) bit 2 = 0 (self-test-done interrupt enabled),
//!    read-modify-write. Vendor: inv_imu_selftest.c:99-101.
//! 10. STATUS_MASK_PIN_16_23 (0xA273, 19.61, p185) bit 5 INT_ON_DEMAND_PIN_2_DIS = 0,
//!     read-modify-write. Vendor: inv_imu_selftest.c:104, inv_imu_edmp.c:818, 774-793,
//!     inv_imu_defs.h:592.
//! 11. EDMP_APEX_EN1 (0x2A, 16.41, p88) bit 6 EDMP_ENABLE = 1, read-modify-write. Vendor:
//!     inv_imu_edmp.c:820, 368-378.
//! 12. REG_HOST_MSG bit 5 EDMP_ON_DEMAND_EN = 1, read-modify-write: the eDMP runs once (the bit
//!     clears itself, p108). Vendor: inv_imu_edmp.c:822-824.
//! 13. Poll INT_APEX_STATUS1 (0x3C, 16.55, p97; read clears) for bit 2 INT_STATUS_SELFTEST_DONE:
//!     one read, then 100 us, at most 30 000 reads (3 s). Vendor: inv_imu_selftest.c:92, 107-118.
//!     No bit: [`Error::SelfTestTimeout`] with the last value read, and the reset of step 16.
//! 14. INT_APEX_CONFIG1 bit 2 = 1 again, read-modify-write. Vendor: inv_imu_selftest.c:121-123.
//! 15. Read the result byte, IMEM_SRAM 0x44 (IMEM_SRAM_REG_68, 17.16, p114). The vendor reads 4
//!     bytes (inv_imu_selftest.c:135, EDMP_STC_RESULTS inv_imu_edmp_memmap.h:565-577); every
//!     field either source defines is in the first. Decoded by [`SelfTestReport::decode`].
//! 16. Soft reset ([`Icm45686::soft_reset`]), also after a failure in steps 1-15 (as far as the bus
//!     allows). Vendor: inv_imu_selftest.c:54.
//!
//! Not done, unlike the vendor: its soft reset saves DRIVE_CONFIG0 and INTF_CONFIG1_OVRD before
//! the reset and writes them back afterwards (inv_imu_driver.c:22-41); this crate's soft reset,
//! the one `init` uses, writes no interface or pad register.
//!
//! # Parameter word (16 bits; the vendor's word is 32 bits with the upper 16 zero)
//!
//! Bit 0 STC_INIT_EN (set whenever a sensor is tested), bit 1 accelerometer, bit 2 gyroscope,
//! bits 9:7 averaging time (0..=5: 10, 20, 40, 80, 160, 320 ms; 6 and 7 reserved), bits 12:10
//! accelerometer tolerance and bits 15:13 gyroscope tolerance against the factory trim (0..=7: 5,
//! 10, 15, 20, 25, 30, 40, 50 %). Datasheet 17.13-17.14, p113-p114; vendor inv_imu_defs.h:742-787
//! and inv_imu_edmp_memmap.h:579-588 agree. Vendor defaults (icm456xx_selftest_init_params,
//! inv_imu_selftest.c:19-31): both sensors, 320 ms, 50 % for both: word 0xFE87. The vendor refuses
//! a request with neither sensor (inv_imu_selftest.c:38-39).
//!
//! # Result byte (IMEM_SRAM 0x44)
//!
//! Bits 7:6 status: 0 done, 1 in progress, 2 error, 3 reserved (17.16, p114; vendor
//! inv_imu_edmp_memmap.h:565-575). Bits 0..=5 are AX, AY, AZ, GX, GY, GZ. The vendor ignores the
//! bits of a sensor it did not test, and passes a sensor only if its three bits and the status are
//! all 0.
//!
//! ## The sense of the six axis bits (read this before trusting a verdict)
//!
//! The two sources contradict each other. The datasheet's register description names the six bits
//! `*_ST_PASS` and says 1 means the self-test passed (17.16, p114). The vendor's header says
//! "0:pass 1:fail" (inv_imu_edmp_memmap.h:568-573) and the vendor's code reports an axis as failed
//! when its bit is set (inv_imu_selftest.c:143-178).
//!
//! **This driver uses the vendor's sense: a set bit is a FAILED axis.** The reason is the board
//! (MARV V2 Rev 1, 2026-11-10): the self-test ran seven times on a healthy, stationary sensor
//! (default configuration three times, accelerometer only, gyroscope only, 5 % tolerance, 50 %
//! tolerance) and every run returned result byte 0x00 with status "done" after 748 ms, while the
//! data stayed at 1 g and near zero rate. Only the vendor's reading makes a healthy sensor pass;
//! read the datasheet's way, those 0x00 results would be six failed axes in every run.
//!
//! **No failing result has ever been observed on hardware.** No run has returned a set axis bit,
//! and neither source documents a way to force a failure. The "failed axis" path
//! ([`SelfTestReport::passed`] returning `false` with status "done") has been exercised on the host
//! only, with made-up result bytes. A `false` from `passed()` has never been seen on a real device,
//! so what a real failure looks like is a hypothesis taken from the vendor's header and code.
//!
//! **Under the datasheet's wording the sense would be the opposite**: a set bit would be a passed
//! axis and the board's 0x00 would mean every axis failed. If a later measurement (a sensor that
//! really fails, or documentation from the vendor) shows that, only the decoding here and its tests
//! change; the bus traffic does not.
//!
//! # Conditions
//!
//! The device must be stationary: there are no absolute limits, the eDMP compares the self-test
//! response with the factory trim under the tolerance. The vendor sets no clock, power or sensor
//! state beforehand: the sequence starts with a soft reset, so the device's previous configuration
//! does not matter, and it ends with one, so the driver is NOT configured afterwards.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

use crate::{Error, Icm45686, reg};

/// Step 2: wait after the first soft reset, 10 ms (vendor, inv_imu_selftest.c:43).
pub(crate) const RESET_WAIT_US: u32 = 10_000;
/// Step 13: wait between two polls, 100 us (vendor, inv_imu_selftest.c:113).
pub(crate) const POLL_US: u32 = 100;
/// Step 13: polls before [`Error::SelfTestTimeout`], 30 000 = 3 s / 100 us (vendor,
/// inv_imu_selftest.c:92, 113-117).
pub(crate) const POLLS: u32 = 30_000;

/// Step 3: the eDMP start registers and values. IRQ0, IRQ1, IRQ2 start at the eDMP ROM base 0x4000
/// plus 0, 4, 8 (vendor, inv_imu_edmp.c:10-12, inv_imu_edmp_defs.h:15), low byte first (vendor,
/// inv_imu_edmp.c:798-805); the stack pointer register takes the upper byte of 0x500 (vendor,
/// inv_imu_edmp.c:801, inv_imu_edmp_defs.h:17). Registers: 19.40-19.46, p176-p178.
pub(crate) const EDMP_START: [(u16, u8); 7] = [
    (reg::EDMP_PRGRM_IRQ0_0, 0x00), // EDMP_PRGRM_IRQ0_0, 0xA24F (19.40, p176)
    (reg::EDMP_PRGRM_IRQ0_0 + 1, 0x40), // EDMP_PRGRM_IRQ0_1, 0xA250 (19.41, p176)
    (reg::EDMP_PRGRM_IRQ0_0 + 2, 0x04), // EDMP_PRGRM_IRQ1_0, 0xA251 (19.42, p177)
    (reg::EDMP_PRGRM_IRQ0_0 + 3, 0x40), // EDMP_PRGRM_IRQ1_1, 0xA252 (19.43, p177)
    (reg::EDMP_PRGRM_IRQ0_0 + 4, 0x08), // EDMP_PRGRM_IRQ2_0, 0xA253 (19.44, p177)
    (reg::EDMP_PRGRM_IRQ0_0 + 5, 0x40), // EDMP_PRGRM_IRQ2_1, 0xA254 (19.45, p177)
    (reg::EDMP_SP_START_ADDR, 0x05), // EDMP_SP_START_ADDR, 0xA255 (19.46, p178)
];

/// Step 5: upper half of the vendor's 32-bit parameter word, written 0.
pub(crate) const PARAM_HIGH_ZERO: [u16; 2] = [
    reg::IMEM_SRAM + 0x3A, // vendor, not in the datasheet: inv_imu_selftest.c:65, 77; memmap.h:589-590
    reg::IMEM_SRAM + 0x3B, // vendor, not in the datasheet: inv_imu_selftest.c:65, 77; memmap.h:589-590
];

/// Step 6: upper bytes of the vendor's 32-bit debug word, written 0.
pub(crate) const DEBUG_HIGH_ZERO: [u16; 3] = [
    reg::IMEM_SRAM + 0x41, // vendor, not in the datasheet: inv_imu_selftest.c:79-80; memmap.h:607-608
    reg::IMEM_SRAM + 0x42, // vendor, not in the datasheet: inv_imu_selftest.c:79-80; memmap.h:607-608
    reg::IMEM_SRAM + 0x43, // vendor, not in the datasheet: inv_imu_selftest.c:79-80; memmap.h:607-608
];

/// Step 7: EDMP_STC_PATCH_EN, the vendor's 32-bit patch-enable word, written 0 (its default).
pub(crate) const PATCH_ZERO: [u16; 4] = [
    reg::IMEM_SRAM + 0x3C, // vendor, not in the datasheet: inv_imu_selftest.c:28, 82; memmap.h:600-601
    reg::IMEM_SRAM + 0x3D, // vendor, not in the datasheet: inv_imu_selftest.c:28, 82; memmap.h:600-601
    reg::IMEM_SRAM + 0x3E, // vendor, not in the datasheet: inv_imu_selftest.c:28, 82; memmap.h:600-601
    reg::IMEM_SRAM + 0x3F, // vendor, not in the datasheet: inv_imu_selftest.c:28, 82; memmap.h:600-601
];

/// What to test and how strictly. Vendor defaults: both sensors, 320 ms, 50 %.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SelfTestConfig {
    /// Test the accelerometer.
    pub accel: bool,
    /// Test the gyroscope.
    pub gyro: bool,
    /// Averaging time in ms: 10, 20, 40, 80, 160 or 320.
    pub averaging_ms: u16,
    /// Accelerometer tolerance against the factory trim, in %: 5, 10, 15, 20, 25, 30, 40 or 50.
    pub accel_tolerance_pct: u8,
    /// Gyroscope tolerance against the factory trim, in %: 5, 10, 15, 20, 25, 30, 40 or 50.
    pub gyro_tolerance_pct: u8,
}

impl Default for SelfTestConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Why a [`SelfTestConfig`] cannot be run. Found before any bus access.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum SelfTestConfigError {
    /// Neither sensor is selected (the vendor refuses this too, inv_imu_selftest.c:38-39).
    NoSensor,
    /// The averaging time is not 10, 20, 40, 80, 160 or 320 ms.
    AveragingMs(u16),
    /// The accelerometer tolerance is not one of 5, 10, 15, 20, 25, 30, 40, 50 %.
    AccelTolerancePct(u8),
    /// The gyroscope tolerance is not one of 5, 10, 15, 20, 25, 30, 40, 50 %.
    GyroTolerancePct(u8),
}

/// Tolerance code 0..=7 (17.14, p113).
const fn tolerance_code(pct: u8) -> Option<u16> {
    match pct {
        5 => Some(0),
        10 => Some(1),
        15 => Some(2),
        20 => Some(3),
        25 => Some(4),
        30 => Some(5),
        40 => Some(6),
        50 => Some(7),
        _ => None,
    }
}

impl SelfTestConfig {
    /// The vendor's defaults: both sensors, 320 ms, 50 % for both.
    pub const fn new() -> Self {
        Self {
            accel: true,
            gyro: true,
            averaging_ms: 320,
            accel_tolerance_pct: 50,
            gyro_tolerance_pct: 50,
        }
    }

    /// Sets [`accel`](Self::accel).
    pub const fn with_accel(mut self, on: bool) -> Self {
        self.accel = on;
        self
    }

    /// Sets [`gyro`](Self::gyro).
    pub const fn with_gyro(mut self, on: bool) -> Self {
        self.gyro = on;
        self
    }

    /// Sets [`averaging_ms`](Self::averaging_ms).
    pub const fn with_averaging_ms(mut self, ms: u16) -> Self {
        self.averaging_ms = ms;
        self
    }

    /// Sets [`accel_tolerance_pct`](Self::accel_tolerance_pct).
    pub const fn with_accel_tolerance_pct(mut self, pct: u8) -> Self {
        self.accel_tolerance_pct = pct;
        self
    }

    /// Sets [`gyro_tolerance_pct`](Self::gyro_tolerance_pct).
    pub const fn with_gyro_tolerance_pct(mut self, pct: u8) -> Self {
        self.gyro_tolerance_pct = pct;
        self
    }

    /// The parameter word (module documentation): low byte to IMEM_SRAM 0x38, high byte to 0x39.
    /// The defaults give 0xFE87.
    pub const fn param_word(&self) -> Result<u16, SelfTestConfigError> {
        if !self.accel && !self.gyro {
            return Err(SelfTestConfigError::NoSensor);
        }
        let avg: u16 = match self.averaging_ms {
            10 => 0,
            20 => 1,
            40 => 2,
            80 => 3,
            160 => 4,
            320 => 5,
            other => return Err(SelfTestConfigError::AveragingMs(other)),
        };
        let Some(accel_tol) = tolerance_code(self.accel_tolerance_pct) else {
            return Err(SelfTestConfigError::AccelTolerancePct(
                self.accel_tolerance_pct,
            ));
        };
        let Some(gyro_tol) = tolerance_code(self.gyro_tolerance_pct) else {
            return Err(SelfTestConfigError::GyroTolerancePct(
                self.gyro_tolerance_pct,
            ));
        };
        let mut word = reg::STC_INIT_EN as u16;
        if self.accel {
            word |= reg::ST_ACCEL_EN as u16;
        }
        if self.gyro {
            word |= reg::ST_GYRO_EN as u16;
        }
        Ok(word | (avg << 7) | (accel_tol << 10) | (gyro_tol << 13))
    }
}

/// Bits 7:6 of the result byte (17.16, p114).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SelfTestStatus {
    /// 0: done.
    #[default]
    Done,
    /// 1: in progress.
    InProgress,
    /// 2: error.
    Error,
    /// 3: reserved.
    Reserved,
}

/// Verdict per axis of one sensor; `true` is a pass.
///
/// A pass is a result bit that is 0: the vendor's sense, which the board supports and the
/// datasheet's wording ("`*_ST_PASS`, 1 = passed") contradicts. A healthy, still sensor returned
/// result byte 0x00 in all seven runs on the board (2026-11-10), so with the datasheet's sense every
/// axis would have failed. No result with a set axis bit has ever been observed on hardware and no
/// source offers a way to produce one: a `false` here has never been seen on a real device, only on
/// the host. Under the datasheet's wording the sense would be the opposite. See the
/// [module documentation](crate::selftest) ("The sense of the six axis bits").
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AxisVerdict {
    /// X axis passed.
    pub x: bool,
    /// Y axis passed.
    pub y: bool,
    /// Z axis passed.
    pub z: bool,
}

impl AxisVerdict {
    /// All three axes passed.
    pub const fn all(&self) -> bool {
        self.x && self.y && self.z
    }
}

/// The decoded result byte.
///
/// The six axis bits are read in the vendor's sense: a set bit is a FAILED axis. The datasheet's
/// register description says the opposite ("`*_ST_PASS`, 1 = passed"); the board returned 0x00 with
/// status "done" in all seven runs on a healthy, stationary sensor (2026-11-10), which only the
/// vendor's reading turns into a pass. No result with a set axis bit has ever been observed on
/// hardware and neither source documents a way to force one, so [`passed`](Self::passed) returning
/// `false` for a failed axis has never been seen on a real device; that path is tested on the host
/// only. Under the datasheet's wording the sense would be the opposite. See the
/// [module documentation](crate::selftest) ("The sense of the six axis bits").
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SelfTestReport {
    /// Accelerometer axes; `None` when it was not tested.
    pub accel: Option<AxisVerdict>,
    /// Gyroscope axes; `None` when it was not tested.
    pub gyro: Option<AxisVerdict>,
    /// The status field.
    pub status: SelfTestStatus,
    /// The result byte as read (IMEM_SRAM 0x44).
    pub raw: u8,
}

/// An axis passed when its bit is 0: the vendor's sense, which the board's 0x00 results on a healthy
/// sensor support and the datasheet's wording contradicts (module documentation).
const fn axes(raw: u8, shift: u8) -> AxisVerdict {
    AxisVerdict {
        x: raw & (1 << shift) == 0,
        y: raw & (2 << shift) == 0,
        z: raw & (4 << shift) == 0,
    }
}

impl SelfTestReport {
    /// Decodes the result byte for the sensors `config` tested; the bits of a sensor it did not test
    /// are ignored, as the vendor does.
    pub const fn decode(raw: u8, config: &SelfTestConfig) -> Self {
        let status = match (raw & reg::ST_STATUS_MASK) >> 6 {
            0 => SelfTestStatus::Done,
            1 => SelfTestStatus::InProgress,
            2 => SelfTestStatus::Error,
            _ => SelfTestStatus::Reserved,
        };
        Self {
            accel: if config.accel {
                Some(axes(raw, 0))
            } else {
                None
            },
            gyro: if config.gyro {
                Some(axes(raw, 3))
            } else {
                None
            },
            status,
            raw,
        }
    }

    /// The status is "done", at least one sensor was tested, and every tested axis passed.
    ///
    /// "Passed" means every tested axis bit is 0 (the vendor's sense; the datasheet's wording would
    /// invert it). `true` has been seen on a real device (result byte 0x00, seven runs, 2026-11-10);
    /// `false` has never been: no failing result has ever been observed on hardware and no source
    /// offers a way to produce one, so that path is verified on the host only.
    pub const fn passed(&self) -> bool {
        let accel = match self.accel {
            Some(a) => a.all(),
            None => true,
        };
        let gyro = match self.gyro {
            Some(g) => g.all(),
            None => true,
        };
        matches!(self.status, SelfTestStatus::Done)
            && (self.accel.is_some() || self.gyro.is_some())
            && accel
            && gyro
    }
}

impl<SPI: SpiDevice> Icm45686<SPI> {
    /// Runs the chip's self-test (module documentation: [`crate::selftest`]) and returns the
    /// decoded result. The device must lie still.
    ///
    /// The sequence starts and ends with a soft reset, so whatever was configured before does not
    /// matter, and afterwards the driver is NOT configured: call [`init`](Self::init) (or
    /// [`configure`](Self::configure)) before reading data again. The stored [`crate::Offsets`] are
    /// not touched, whatever the outcome (the device itself clears its offset registers on the soft
    /// resets); `init` stores zero and `configure` writes the stored value back. The validity state
    /// is cleared before the first await. An invalid `config` returns [`Error::InvalidSelfTestConfig`] before any bus
    /// access, with the driver unchanged.
    ///
    /// A self-test that completes with a failed axis returns `Ok` with
    /// [`SelfTestReport::passed`] `false`. Errors: [`Error::SelfTestTimeout`] when the done bit does
    /// not appear within 3 s; a bus error or [`Error::ResetTimeout`]. After any failure in steps
    /// 1-15 the final soft reset is still attempted; the first error is returned.
    ///
    /// Transactions: 44 when the done bit shows at the first poll, plus 1 per further poll. Awaits:
    /// 74, plus 2 per further poll. A dropped future can leave the eDMP running: [`init`](Self::init)
    /// starts with a soft reset.
    pub async fn self_test(
        &mut self,
        delay: &mut impl DelayNs,
        config: &SelfTestConfig,
    ) -> Result<SelfTestReport, Error<SPI::Error>> {
        let word = config.param_word().map_err(Error::InvalidSelfTestConfig)?;
        self.applied = None;
        self.info = None;
        let outcome = self.self_test_steps(delay, word).await;
        let reset = self.soft_reset(delay).await;
        let raw = outcome?;
        reset?;
        Ok(SelfTestReport::decode(raw, config))
    }

    /// Steps 1-15; returns the result byte.
    async fn self_test_steps(
        &mut self,
        delay: &mut impl DelayNs,
        word: u16,
    ) -> Result<u8, Error<SPI::Error>> {
        self.soft_reset(delay).await?;
        delay.delay_us(RESET_WAIT_US).await;
        for (addr, value) in EDMP_START {
            self.ireg_write(delay, addr, value).await?;
        }
        self.ireg_modify(
            delay,
            reg::FIFO_SRAM_SLEEP,
            reg::FIFO_GSLEEP_SHARED_SRAM,
            reg::FIFO_GSLEEP_SHARED_SRAM,
        )
        .await?;
        let [low, high] = word.to_le_bytes();
        self.ireg_write(delay, reg::IMEM_SRAM_REG_56, low).await?;
        self.ireg_write(delay, reg::IMEM_SRAM_REG_57, high).await?;
        for addr in PARAM_HIGH_ZERO {
            self.ireg_write(delay, addr, 0).await?;
        }
        self.ireg_write(delay, reg::IMEM_SRAM_REG_64, 0).await?;
        for addr in DEBUG_HIGH_ZERO {
            self.ireg_write(delay, addr, 0).await?;
        }
        for addr in PATCH_ZERO {
            self.ireg_write(delay, addr, 0).await?;
        }
        self.modify(reg::REG_HOST_MSG, reg::TESTOPENABLE, reg::TESTOPENABLE)
            .await?;
        self.modify(
            reg::INT_APEX_CONFIG1,
            reg::INT_STATUS_MASK_PIN_SELFTEST_DONE,
            0,
        )
        .await?;
        self.ireg_modify(
            delay,
            reg::STATUS_MASK_PIN_16_23,
            reg::INT_ON_DEMAND_PIN_2_DIS,
            0,
        )
        .await?;
        self.modify(reg::EDMP_APEX_EN1, reg::EDMP_ENABLE, reg::EDMP_ENABLE)
            .await?;
        self.modify(
            reg::REG_HOST_MSG,
            reg::EDMP_ON_DEMAND_EN,
            reg::EDMP_ON_DEMAND_EN,
        )
        .await?;
        let mut last = 0;
        let mut done = false;
        for _ in 0..POLLS {
            last = self.read_register(reg::INT_APEX_STATUS1).await?;
            if last & reg::INT_STATUS_SELFTEST_DONE != 0 {
                done = true;
                break;
            }
            delay.delay_us(POLL_US).await;
        }
        if !done {
            return Err(Error::SelfTestTimeout { apex_status: last });
        }
        self.modify(
            reg::INT_APEX_CONFIG1,
            reg::INT_STATUS_MASK_PIN_SELFTEST_DONE,
            reg::INT_STATUS_MASK_PIN_SELFTEST_DONE,
        )
        .await?;
        self.ireg_read(delay, reg::IMEM_SRAM_REG_68).await
    }

    /// Read-modify-write of a direct register: two transactions.
    async fn modify(&mut self, addr: u8, mask: u8, value: u8) -> Result<(), Error<SPI::Error>> {
        let old = self.read_register(addr).await?;
        self.write_register(addr, (old & !mask) | (value & mask))
            .await
    }
}
