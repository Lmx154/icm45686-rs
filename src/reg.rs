//! Register map, names as in the datasheet (DS-000489 Rev 1.1).
//!
//! Every entry cites the datasheet section and the printed page ("p" is the number in the page
//! footer, "Page N of 199"). A fact the datasheet lacks is marked "vendor": it comes from TDK's
//! public reference driver. Reset values are the "Reset value" lines of the register sections.
//! Fields the datasheet marks Reserved must keep their reset value (p66, text before section 16),
//! so each register has a `*_FIELDS` mask of the bits this crate may change.
//!
//! Three address spaces exist. Direct registers are `u8` (bank 0, sections 15.1 and 16, p58-p109).
//! Indirect registers are `u16`: the bank base below plus the offset printed in the datasheet
//! (section 13, p55-p56). A burst access is not allowed on them; the gap between two accesses is
//! at least 4 us (section 13.3, p55).

// ---- Direct registers (bank 0) ----------------------------------------------------------------

/// First accelerometer data register, ACCEL_DATA_X1_UI (16.1, p67). The 14 bytes from here are
/// accel X, Y, Z, gyro X, Y, Z, temperature, each a little-endian `i16`.
pub const ACCEL_DATA_X1_UI: u8 = 0x00;
/// First gyroscope data register, GYRO_DATA_X1_UI (16.7, p68).
pub const GYRO_DATA_X1_UI: u8 = 0x06;
/// First temperature data register, TEMP_DATA1_UI (16.13, p69).
pub const TEMP_DATA1_UI: u8 = 0x0C;
/// Time from the FSYNC edge to the latest ODR, high byte, TMST_FSYNCH (16.15, p70).
pub const TMST_FSYNCH: u8 = 0x0E;
/// Time from the FSYNC edge to the latest ODR, low byte, TMST_FSYNCL (16.16, p70).
pub const TMST_FSYNCL: u8 = 0x0F;

/// Power management (16.17, p70): gyro mode in bits 3:2, accel mode in bits 1:0. Reset 0x00.
pub const PWR_MGMT0: u8 = 0x10;
/// PWR_MGMT0 bits this crate may change (GYRO_MODE, ACCEL_MODE).
pub const PWR_MGMT0_FIELDS: u8 = 0x0F;
/// PWR_MGMT0 reset value.
pub const PWR_MGMT0_RESET: u8 = 0x00;
/// PWR_MGMT0 GYRO_MODE shift.
pub const PWR_MGMT0_GYRO_SHIFT: u8 = 2;
/// PWR_MGMT0 ACCEL_MODE shift.
pub const PWR_MGMT0_ACCEL_SHIFT: u8 = 0;
/// GYRO_MODE / ACCEL_MODE code 0: sensor off (p70).
pub const POWER_OFF: u8 = 0;
/// GYRO_MODE code 1: standby, gyro drive on (p70). ACCEL_MODE code 1 also means off.
pub const POWER_STANDBY: u8 = 1;
/// GYRO_MODE / ACCEL_MODE code 2: low-power mode (p70). Outside this crate's scope.
pub const POWER_LOW_POWER: u8 = 2;
/// GYRO_MODE / ACCEL_MODE code 3: low-noise mode (p70).
pub const POWER_LOW_NOISE: u8 = 3;
/// PWR_MGMT0 with accel and gyro in low-noise mode.
pub const PWR_MGMT0_LOW_NOISE: u8 = 0x0F;

/// FIFO count, first byte (16.18, p71). Read only, reset 0x00. The register map labels it
/// FIFO_DATA_CNT\[15:8\], which is its big-endian view; in the default little-endian mode (14, p57:
/// little endian "for data in Sensor Data Registers and FIFO, and for FIFO Count") it holds bits 7:0,
/// as the sensor data registers do. The register section says "packets", section 5 (p39) says
/// "bytes"; vendor sources count frames.
pub const FIFO_COUNT_0: u8 = 0x12;
/// FIFO count, second byte (16.19, p71): bits 15:8 in the default little-endian mode. Read only,
/// reset 0x00.
pub const FIFO_COUNT_1: u8 = 0x13;
/// FIFO data port (16.20, p71). Read only, reset 0x7F.
pub const FIFO_DATA: u8 = 0x14;

/// INT1 status enables (16.21, p72-p73). Reset 0x80 (reset-done enabled). Each bit enables the
/// matching flag of INT1_STATUS0 and the pin event; "can be changed by UI interface".
pub const INT1_CONFIG0: u8 = 0x16;
/// INT1_CONFIG0 reset value.
pub const INT1_CONFIG0_RESET: u8 = 0x80;
/// INT1_CONFIG0 bits this crate may change: all eight.
pub const INT1_CONFIG0_FIELDS: u8 = 0xFF;
/// INT1_CONFIG0 / INT1_STATUS0 bit 7: reset done.
pub const RESET_DONE: u8 = 0x80;
/// INT1_CONFIG0 / INT1_STATUS0 bit 6: AUX1 AGC ready (AUX1 is out of scope).
pub const AUX1_AGC_RDY: u8 = 0x40;
/// INT1_CONFIG0 / INT1_STATUS0 bit 5: UI AGC ready.
pub const AP_AGC_RDY: u8 = 0x20;
/// INT1_CONFIG0 / INT1_STATUS0 bit 4: UI FSYNC event.
pub const AP_FSYNC: u8 = 0x10;
/// INT1_CONFIG0 / INT1_STATUS0 bit 3: AUX1 data ready (AUX1 is out of scope).
pub const AUX1_DRDY: u8 = 0x08;
/// INT1_CONFIG0 / INT1_STATUS0 bit 2: UI data ready.
pub const DRDY: u8 = 0x04;
/// INT1_CONFIG0 / INT1_STATUS0 bit 1: FIFO count at or above the watermark.
pub const FIFO_THS: u8 = 0x02;
/// INT1_CONFIG0 / INT1_STATUS0 bit 0: FIFO full.
pub const FIFO_FULL: u8 = 0x01;

/// INT1 second enable register (16.22, p74-p75): APEX, I2C master, I3C error, wake-on-motion X/Y/Z
/// and PLL ready; bit 7 is reserved. Reset 0x00. Out of scope; this is not where the pulse width
/// is, see [`INT_PULSE_MIN_ON_INTF0`].
pub const INT1_CONFIG1: u8 = 0x17;
/// INT1_CONFIG1 reset value.
pub const INT1_CONFIG1_RESET: u8 = 0x00;

/// INT1 pin (16.23, p75): bit 2 drive (1 open-drain), bit 1 mode (1 latched), bit 0 polarity
/// (1 active high). Reset 0x04 (open-drain, pulse, active low). Mode and polarity "can be changed
/// only when all interrupts of the corresponding serial interface are disabled".
pub const INT1_CONFIG2: u8 = 0x18;
/// INT1_CONFIG2 reset value.
pub const INT1_CONFIG2_RESET: u8 = 0x04;
/// INT1_CONFIG2 bits this crate may change (INT1_DRIVE, INT1_MODE, INT1_POLARITY).
pub const INT1_CONFIG2_FIELDS: u8 = 0x07;
/// INT1_CONFIG2 bit 2, INT1_DRIVE: open-drain when set.
pub const INT1_DRIVE_OPEN_DRAIN: u8 = 0x04;
/// INT1_CONFIG2 bit 1, INT1_MODE: latched when set.
pub const INT1_MODE_LATCHED: u8 = 0x02;
/// INT1_CONFIG2 bit 0, INT1_POLARITY: active high when set.
pub const INT1_POLARITY_HIGH: u8 = 0x01;
/// INT1_CONFIG2: push-pull, pulsed, active high.
pub const INT1_CONFIG2_PUSH_PULL_PULSE_HIGH: u8 = 0x01;

/// INT1 status flags, cleared on read (16.24, p76); bit layout as INT1_CONFIG0. Reset 0x00.
pub const INT1_STATUS0: u8 = 0x19;
/// INT1 second status register, cleared on read (16.25, p77): bit 7 reserved, 6 APEX, 5 I2C master
/// done, 4 I3C protocol error, 3:1 wake-on-motion Z/Y/X, 0 PLL ready. Reset 0x00.
pub const INT1_STATUS1: u8 = 0x1A;

/// Accelerometer full scale (bits 6:4) and output data rate (bits 3:0) (16.26, p78). Reset 0x06.
pub const ACCEL_CONFIG0: u8 = 0x1B;
/// ACCEL_CONFIG0 reset value: +-32 g, 800 Hz.
pub const ACCEL_CONFIG0_RESET: u8 = 0x06;
/// ACCEL_CONFIG0 bits this crate may change (ACCEL_UI_FS_SEL, ACCEL_ODR).
pub const ACCEL_CONFIG0_FIELDS: u8 = 0x7F;
/// Gyroscope full scale (bits 7:4) and output data rate (bits 3:0) (16.27, p79). Reset 0x06.
pub const GYRO_CONFIG0: u8 = 0x1C;
/// GYRO_CONFIG0 reset value: +-4000 dps, 800 Hz.
pub const GYRO_CONFIG0_RESET: u8 = 0x06;
/// GYRO_CONFIG0 bits this crate may change (GYRO_UI_FS_SEL, GYRO_ODR).
pub const GYRO_CONFIG0_FIELDS: u8 = 0xFF;

/// FIFO mode (bits 7:6) and depth (bits 5:0) (16.28, p80). Reset 0x00. Depth "can be changed when
/// FIFO is disabled (Bypass mode)".
pub const FIFO_CONFIG0: u8 = 0x1D;
/// FIFO_CONFIG0 reset value.
pub const FIFO_CONFIG0_RESET: u8 = 0x00;
/// FIFO_CONFIG0 bits this crate may change.
pub const FIFO_CONFIG0_FIELDS: u8 = 0xFF;
/// FIFO_MODE shift.
pub const FIFO_MODE_SHIFT: u8 = 6;
/// FIFO_MODE code 0: bypass (disabled) (p80).
pub const FIFO_MODE_BYPASS: u8 = 0;
/// FIFO_MODE code 1: stream; frames are overwritten when full (p80).
pub const FIFO_MODE_STREAM: u8 = 1;
/// FIFO_MODE code 2: stop on full; frames are dropped when full (p80).
pub const FIFO_MODE_STOP_ON_FULL: u8 = 2;
/// FIFO_DEPTH code 000111: 2 KB, "recommended setting" (p80).
pub const FIFO_DEPTH_2K: u8 = 0x07;
/// FIFO_DEPTH code 011111: 8 KB, valid when all APEX features are disabled (p80).
pub const FIFO_DEPTH_8K: u8 = 0x1F;

/// Watermark threshold, low byte (16.29, p80). Write this byte first; the new value takes effect
/// when the high byte is written. 0 disables the watermark. Reset 0x00.
pub const FIFO_CONFIG1_0: u8 = 0x1E;
/// Watermark threshold, high byte (16.30, p81). Reset 0x00.
pub const FIFO_CONFIG1_1: u8 = 0x1F;

/// FIFO flush (bit 7), watermark condition (bit 3) (16.31, p81). Reset 0x20: bits 6:4 and 2:0 are
/// reserved and bit 5 reads 1 at reset, so it must be kept. The watermark condition "can be changed
/// when FIFO is disabled".
pub const FIFO_CONFIG2: u8 = 0x20;
/// FIFO_CONFIG2 reset value.
pub const FIFO_CONFIG2_RESET: u8 = 0x20;
/// FIFO_CONFIG2 bits this crate may change (FIFO_FLUSH, FIFO_WR_WM_GT_TH).
pub const FIFO_CONFIG2_FIELDS: u8 = 0x88;
/// FIFO_CONFIG2 bit 7, FIFO_FLUSH: flush command.
pub const FIFO_FLUSH: u8 = 0x80;
/// FIFO_CONFIG2 bit 3, FIFO_WR_WM_GT_TH: 1 raises the watermark event at count >= threshold, 0 only
/// at count == threshold.
pub const FIFO_WR_WM_GT_TH: u8 = 0x08;

/// FIFO frame content (16.32, p82). Reset 0x00. Bits 5:4 are external sensors (out of scope).
pub const FIFO_CONFIG3: u8 = 0x21;
/// FIFO_CONFIG3 reset value.
pub const FIFO_CONFIG3_RESET: u8 = 0x00;
/// FIFO_CONFIG3 bits this crate may change (FIFO_HIRES_EN, FIFO_GYRO_EN, FIFO_ACCEL_EN, FIFO_IF_EN).
pub const FIFO_CONFIG3_FIELDS: u8 = 0x0F;
/// FIFO_CONFIG3 bit 3, FIFO_HIRES_EN: 20-byte high-resolution frames.
pub const FIFO_HIRES_EN: u8 = 0x08;
/// FIFO_CONFIG3 bit 2, FIFO_GYRO_EN.
pub const FIFO_GYRO_EN: u8 = 0x04;
/// FIFO_CONFIG3 bit 1, FIFO_ACCEL_EN.
pub const FIFO_ACCEL_EN: u8 = 0x02;
/// FIFO_CONFIG3 bit 0, FIFO_IF_EN: sensor registers write into the FIFO. Enable after the FIFO
/// mode, disable before it (p82).
pub const FIFO_IF_EN: u8 = 0x01;

/// FIFO compression, timestamp and external sensor 0 width (16.33, p83). Reset 0x00.
pub const FIFO_CONFIG4: u8 = 0x22;
/// FIFO_CONFIG4 reset value.
pub const FIFO_CONFIG4_RESET: u8 = 0x00;
/// FIFO_CONFIG4 bits this crate may change (FIFO_TMST_FSYNC_EN).
pub const FIFO_CONFIG4_FIELDS: u8 = 0x02;
/// FIFO_CONFIG4 bit 1, FIFO_TMST_FSYNC_EN: timestamp or FSYNC delay in the frame (p83).
pub const FIFO_TMST_FSYNC_EN: u8 = 0x02;

/// Timestamp configuration and wake-on-motion (16.34, p84). Reset 0x00.
pub const TMST_WOM_CONFIG: u8 = 0x23;
/// TMST_WOM_CONFIG reset value.
pub const TMST_WOM_CONFIG_RESET: u8 = 0x00;
/// TMST_WOM_CONFIG bits this crate may change (TMST_RESOL).
pub const TMST_WOM_CONFIG_FIELDS: u8 = 0x20;
/// TMST_WOM_CONFIG bit 5, TMST_RESOL: 0 is 1 us, 1 is 16 us per LSB (p84).
pub const TMST_RESOL_16US: u8 = 0x20;

/// FSYNC for the UI path (16.35, p85): bit 3 flag clear policy, bits 2:0 tag target. Reset 0x00.
pub const FSYNC_CONFIG0: u8 = 0x24;
/// FSYNC_CONFIG0 reset value.
pub const FSYNC_CONFIG0_RESET: u8 = 0x00;
/// FSYNC_CONFIG0 bits this crate may change (AP_FSYNC_FLAG_CLEAR_SEL, AP_FSYNC_SEL).
pub const FSYNC_CONFIG0_FIELDS: u8 = 0x0F;
/// FSYNC_CONFIG0 bit 3: flag cleared when the tagged register LSB is read (0: when it is updated).
pub const AP_FSYNC_FLAG_CLEAR_ON_READ: u8 = 0x08;
/// FSYNC for the AUX1 path (16.36, p85). Out of scope; reset 0x00.
pub const FSYNC_CONFIG1: u8 = 0x25;

/// Clock (16.37, p86): bit 6 RTC_ALIGN (a write of 1 is a re-align command), bit 5 RTC_MODE (1 uses
/// the external clock on CLKIN). Reset 0x03: bits 1:0 are reserved and read 1, keep them. RTC mode
/// works only with ACCEL_LP_CLK_SEL = 1 (see [`SMC_CONTROL_0`]).
pub const RTC_CONFIG: u8 = 0x26;
/// RTC_CONFIG reset value.
pub const RTC_CONFIG_RESET: u8 = 0x03;
/// RTC_CONFIG bits this crate may change (RTC_ALIGN, RTC_MODE).
pub const RTC_CONFIG_FIELDS: u8 = 0x60;
/// RTC_CONFIG bit 6, RTC_ALIGN.
pub const RTC_ALIGN: u8 = 0x40;
/// RTC_CONFIG bit 5, RTC_MODE.
pub const RTC_MODE: u8 = 0x20;

/// FIFO decimation (16.39, p87): gyro in bits 7:4, accel in bits 3:0; code n is a factor of 2^n,
/// n = 0..=12. Reset 0x00. Sensors must be off when it is changed (section 5, p39).
pub const ODR_DECIMATE_CONFIG: u8 = 0x28;
/// ODR_DECIMATE_CONFIG reset value.
pub const ODR_DECIMATE_CONFIG_RESET: u8 = 0x00;
/// ODR_DECIMATE_CONFIG bits this crate may change.
pub const ODR_DECIMATE_CONFIG_FIELDS: u8 = 0xFF;

/// Pad drive (16.48, p93): bits 3:1 PADS_SPI_SLEW. Reset 0x6C. Bits 6:4 PADS_I2C_SLEW read 6, a
/// code the table does not list, so they are kept. Takes effect 1.5 us after the write.
pub const DRIVE_CONFIG0: u8 = 0x32;
/// DRIVE_CONFIG0 reset value.
pub const DRIVE_CONFIG0_RESET: u8 = 0x6C;
/// DRIVE_CONFIG0 bits this crate may change (PADS_SPI_SLEW).
pub const DRIVE_CONFIG0_FIELDS: u8 = 0x0E;
/// PADS_SPI_SLEW shift.
pub const PADS_SPI_SLEW_SHIFT: u8 = 1;

/// Device identity (16.79, p107), reads [`WHO_AM_I_VALUE`].
pub const WHO_AM_I: u8 = 0x72;
/// Expected WHO_AM_I value (16.79, p107).
pub const WHO_AM_I_VALUE: u8 = 0xE9;
/// Indirect register address, bits 15:8 (16.81, p108). Reset 0xAF.
pub const IREG_ADDR_15_8: u8 = 0x7C;
/// Indirect register address, bits 7:0 (16.82, p108). Reset 0x06.
pub const IREG_ADDR_7_0: u8 = 0x7D;
/// Indirect register data (16.83, p108). Reset 0x02.
pub const IREG_DATA: u8 = 0x7E;
/// Bit 1 SOFT_RST (self-clearing), bit 0 IREG_DONE (16.84, p109). Reset 0x01.
pub const REG_MISC2: u8 = 0x7F;
/// REG_MISC2 value that starts a soft reset (16.84, p109).
pub const SOFT_RST: u8 = 0x02;
/// Not in the datasheet (vendor): pad override that routes pin 9 to CLKIN; the vendor driver writes
/// [`IOC_PAD_SCENARIO_OVRD_CLKIN`].
pub const IOC_PAD_SCENARIO_OVRD: u8 = 0x31;
/// Vendor value of [`IOC_PAD_SCENARIO_OVRD`] that selects CLKIN on pin 9.
pub const IOC_PAD_SCENARIO_OVRD_CLKIN: u8 = 0x06;

// ---- Indirect registers -----------------------------------------------------------------------

/// IMEM_SRAM base (section 13.4, p55). Contents are undefined until the eDMP init procedure ran
/// (section 17 "Reset value", p110 ff.); the datasheet text does not describe that procedure.
pub const IMEM_SRAM: u16 = 0x0000;
/// IPREG_TOP1 base (section 13.4, p55).
pub const IPREG_TOP1: u16 = 0xA200;
/// IPREG_SYS1 base (section 13.4, p55).
pub const IPREG_SYS1: u16 = 0xA400;
/// IPREG_SYS2 base (section 13.4, p55).
pub const IPREG_SYS2: u16 = 0xA500;

/// IPREG_TOP1 0x58 SMC_CONTROL_0 (19.47, p178-p179). Reset 0x60 (bits 7:5 reserved, keep). Bit 4
/// ACCEL_LP_CLK_SEL must be 1 for RTC mode; bit 3 TEMP_DIS; bit 2 TMST_FORCE_AUX_FINE_EN; bit 1
/// TMST_FSYNC_EN; bit 0 TMST_EN.
pub const SMC_CONTROL_0: u16 = IPREG_TOP1 + 0x58;
/// SMC_CONTROL_0 reset value.
pub const SMC_CONTROL_0_RESET: u8 = 0x60;
/// SMC_CONTROL_0 bit 4, ACCEL_LP_CLK_SEL.
pub const ACCEL_LP_CLK_SEL: u8 = 0x10;
/// SMC_CONTROL_0 bit 3, TEMP_DIS.
pub const TEMP_DIS: u8 = 0x08;
/// SMC_CONTROL_0 bit 1, TMST_FSYNC_EN: the FSYNC delay registers capture.
pub const TMST_FSYNC_EN: u8 = 0x02;
/// SMC_CONTROL_0 bit 0, TMST_EN: timestamp enabled.
pub const TMST_EN: u8 = 0x01;
/// IPREG_TOP1 0x67 SREG_CTRL (19.50, p180). Reset 0x00. Bit 1 SREG_DATA_ENDIAN_SEL: 0 little
/// endian (reset), 1 big endian, for sensor registers, FIFO data and FIFO count.
pub const SREG_CTRL: u16 = IPREG_TOP1 + 0x67;
/// SREG_CTRL bit 1, SREG_DATA_ENDIAN_SEL.
pub const SREG_DATA_ENDIAN_SEL: u8 = 0x02;
/// IPREG_TOP1 0x68 SIFS_I3C_STC_CFG (19.51, p180). Reset 0x23. Bit 2 I3C_STC_MODE must be 0 for
/// RTC mode to take effect (16.37, p86).
pub const SIFS_I3C_STC_CFG: u16 = IPREG_TOP1 + 0x68;
/// SIFS_I3C_STC_CFG reset value.
pub const SIFS_I3C_STC_CFG_RESET: u8 = 0x23;
/// SIFS_I3C_STC_CFG bit 2, I3C_STC_MODE.
pub const I3C_STC_MODE: u8 = 0x04;
/// IPREG_TOP1 0x69 INT_PULSE_MIN_ON_INTF0 (19.52, p181). Reset 0x01. Bits 2:0 INT0_TPULSE_DURATION:
/// 0 is 100 us (use only if ODR < 4 kHz), 1 is 8 us (required if ODR >= 4 kHz). Applies in pulse
/// mode. This register sets the width of the pulse on the INT1 pin, measured on the MARV V2 board
/// (2026-11-11): 0x69 = 0 gave 103.9 us and 1 gave 8.4 us on INT1, and the datasheet's INT0 / INT1 of
/// 19.52 / 19.53 are numbered from zero, so they are the INT1 / INT2 pins. 0x6A (19.53,
/// INT1_TPULSE_DURATION) is the INT2 pin's and has no effect on INT1; this crate does not touch it.
pub const INT_PULSE_MIN_ON_INTF0: u16 = IPREG_TOP1 + 0x69;
/// INT_PULSE_MIN_ON_INTF0 reset value: 8 us.
pub const INT_PULSE_MIN_ON_INTF0_RESET: u8 = 0x01;
/// INT_PULSE_MIN_ON_INTF0 bits this crate may change (INT0_TPULSE_DURATION).
pub const INT_PULSE_MIN_ON_INTF0_FIELDS: u8 = 0x07;
/// IPREG_TOP1 0x6C INT_PULSE_MIN_OFF_INTF1 (19.55, p181). Reset 0x01. Bits 2:0
/// INT1_TDEASSERT_DISABLE: 0 is 100 us, 1 is 8 us minimum de-assertion time.
pub const INT_PULSE_MIN_OFF_INTF1: u16 = IPREG_TOP1 + 0x6C;
/// INT_PULSE_MIN_OFF_INTF1 reset value: 8 us.
pub const INT_PULSE_MIN_OFF_INTF1_RESET: u8 = 0x01;
/// IPREG_TOP1 0x90 SELFTEST (19.66, p187). Reset 0x00. Bits 5:3 EN_GZ/GY/GX_ST, bits 2:0
/// EN_AZ/AY/AX_ST enable the self-test of one axis; "can be changed on-the-fly".
pub const SELFTEST: u16 = IPREG_TOP1 + 0x90;
/// SELFTEST bits that carry a field (bits 7:6 are reserved).
pub const SELFTEST_FIELDS: u8 = 0x3F;
/// SELFTEST bit 0, EN_AX_ST.
pub const EN_AX_ST: u8 = 0x01;
/// SELFTEST bit 1, EN_AY_ST.
pub const EN_AY_ST: u8 = 0x02;
/// SELFTEST bit 2, EN_AZ_ST.
pub const EN_AZ_ST: u8 = 0x04;
/// SELFTEST bit 3, EN_GX_ST.
pub const EN_GX_ST: u8 = 0x08;
/// SELFTEST bit 4, EN_GY_ST.
pub const EN_GY_ST: u8 = 0x10;
/// SELFTEST bit 5, EN_GZ_ST.
pub const EN_GZ_ST: u8 = 0x20;

/// IMEM_SRAM 0x00 GYRO_X_STR_FT (17.1-17.2, p110): self-test response of the gyro, 16 bit, low byte
/// first, in kdps with a 30 mdps LSB. Y is at +2 (17.3-17.4), Z at +4 (17.5-17.6, p111).
pub const GYRO_X_STR_FT: u16 = IMEM_SRAM;
/// IMEM_SRAM 0x02, GYRO_Y_STR_FT (17.3-17.4, p110).
pub const GYRO_Y_STR_FT: u16 = IMEM_SRAM + 0x02;
/// IMEM_SRAM 0x04, GYRO_Z_STR_FT (17.5-17.6, p111).
pub const GYRO_Z_STR_FT: u16 = IMEM_SRAM + 0x04;
/// IMEM_SRAM 0x38 IMEM_SRAM_REG_56 (17.13, p113): bit 7 ST_AVG_TIME\[0\], bit 2 ST_GYRO_EN, bit 1
/// ST_ACCEL_EN, bit 0 STC_INIT_EN.
pub const IMEM_SRAM_REG_56: u16 = IMEM_SRAM + 0x38;
/// IMEM_SRAM 0x39 IMEM_SRAM_REG_57 (17.14, p113-p114): bits 7:5 ST_GYRO_LIMIT, bits 4:2
/// ST_ACCEL_LIMIT (tolerance codes 0..=7: 5, 10, 15, 20, 25, 30, 40, 50 %), bits 1:0
/// ST_AVG_TIME\[2:1\].
pub const IMEM_SRAM_REG_57: u16 = IMEM_SRAM + 0x39;
/// IMEM_SRAM 0x40 IMEM_SRAM_REG_64 (17.15, p114): ST_DEBUG_EN, must be 0 when self-test is run.
pub const IMEM_SRAM_REG_64: u16 = IMEM_SRAM + 0x40;
/// IMEM_SRAM 0x44 IMEM_SRAM_REG_68 (17.16, p114), read only: bits 7:6 ST_STATUS (0 done, 1 in
/// progress, 2 error), bits 5:0 GZ, GY, GX, AZ, AY, AX. The datasheet names the six bits
/// `*_ST_PASS` and describes a set bit as "self-test passed"; the vendor's memory map says "0:pass
/// 1:fail" and its code reports an axis as failed when its bit is set. The driver uses the vendor's
/// sense: a set bit is a FAILED axis (a healthy, still sensor returned 0x00 in all seven runs on the
/// board, 2026-11-10; no failing result has ever been observed).
pub const IMEM_SRAM_REG_68: u16 = IMEM_SRAM + 0x44;
/// IMEM_SRAM_REG_56 bit 2, ST_GYRO_EN.
pub const ST_GYRO_EN: u8 = 0x04;
/// IMEM_SRAM_REG_56 bit 1, ST_ACCEL_EN.
pub const ST_ACCEL_EN: u8 = 0x02;
/// IMEM_SRAM_REG_56 bit 0, STC_INIT_EN.
pub const STC_INIT_EN: u8 = 0x01;
/// IMEM_SRAM_REG_68 ST_STATUS mask.
pub const ST_STATUS_MASK: u8 = 0xC0;
/// IMEM_SRAM_REG_68 ST_STATUS value 0: done.
pub const ST_STATUS_DONE: u8 = 0x00;
/// IMEM_SRAM_REG_68 ST_STATUS value 1: in progress.
pub const ST_STATUS_IN_PROGRESS: u8 = 0x40;
/// IMEM_SRAM_REG_68 ST_STATUS value 2: error.
pub const ST_STATUS_ERROR: u8 = 0x80;
/// IMEM_SRAM_REG_68 result bits; a set bit is a failed axis (vendor; confirmed by the board's 0x00 on
/// a healthy sensor). One per axis (AX is bit 0 .. GZ is bit 5). The datasheet calls them pass bits
/// (`*_ST_PASS`, 1 = passed) and the constant keeps that name, but the driver does not read them that
/// way; no failing result has ever been observed on hardware.
pub const ST_PASS_MASK: u8 = 0x3F;

// ---- Registers the eDMP self-test writes (crate::selftest). "vendor" = TDK's public driver,
// github.com/tdk-invn-oss/zephyr.hal_tdk, commit fa54cb6, folder icm456xx/imu. Each address below is
// the same in the vendor header and in the datasheet (DS-000489 Rev 1.1), except IMEM_SRAM 0x3A,
// 0x3B, 0x3C..=0x3F and 0x41..=0x43 (the nine bytes in crate::selftest): the datasheet does not list
// them, they are vendor-only and written with zero.

/// EDMP_APEX_EN1 (16.41, p88). Reset 0x00. Bit 6 EDMP_ENABLE. Vendor: inv_imu_regmap_le.h:214-224;
/// set by icm456xx_edmp_enable, inv_imu_edmp.c:368-378.
pub const EDMP_APEX_EN1: u8 = 0x2A;
/// EDMP_APEX_EN1 bit 6, EDMP_ENABLE: "Set 1 to enable eDMP" (p88).
pub const EDMP_ENABLE: u8 = 0x40;
/// INT_APEX_CONFIG1 (16.53, p96). Reset 0x00. Bit 2 INT_STATUS_MASK_PIN_SELFTEST_DONE (0 enables the
/// self-test-done interrupt). Vendor: inv_imu_regmap_le.h:326-335; inv_imu_selftest.c:99-101, 121-123.
pub const INT_APEX_CONFIG1: u8 = 0x3A;
/// INT_APEX_CONFIG1 bit 2, INT_STATUS_MASK_PIN_SELFTEST_DONE.
pub const INT_STATUS_MASK_PIN_SELFTEST_DONE: u8 = 0x04;
/// INT_APEX_STATUS1 (16.55, p97), read clears (R/C). Reset 0x00. Bit 2 INT_STATUS_SELFTEST_DONE.
/// Vendor: inv_imu_regmap_le.h:349-358; polled by inv_imu_selftest.c:107-118. Read only here.
pub const INT_APEX_STATUS1: u8 = 0x3C;
/// INT_APEX_STATUS1 bit 2, INT_STATUS_SELFTEST_DONE.
pub const INT_STATUS_SELFTEST_DONE: u8 = 0x04;
/// REG_HOST_MSG (16.80, p108). Reset 0x00. Bit 5 EDMP_ON_DEMAND_EN ("the eDMP runs once", clears
/// itself), bit 0 TESTOPENABLE. Vendor: inv_imu_regmap_le.h:517-523; inv_imu_selftest.c:94-96 and
/// inv_imu_edmp.c:822-824.
pub const REG_HOST_MSG: u8 = 0x73;
/// REG_HOST_MSG bit 5, EDMP_ON_DEMAND_EN.
pub const EDMP_ON_DEMAND_EN: u8 = 0x20;
/// REG_HOST_MSG bit 0, TESTOPENABLE: "1: Enable test operation".
pub const TESTOPENABLE: u8 = 0x01;
/// IPREG_TOP1 0x4F EDMP_PRGRM_IRQ0_0 (19.40, p176): PRGRM_STRT_ADDR_IRQ_0\[7:0\], reset 0x00. The
/// next five registers hold IRQ_0\[15:8\] (0x50, 19.41, p176), IRQ_1\[7:0\] (0x51, 19.42, p177),
/// IRQ_1\[15:8\] (0x52, 19.43, p177), IRQ_2\[7:0\] (0x53, 19.44, p177), IRQ_2\[15:8\] (0x54, 19.45,
/// p177). Vendor: inv_imu_regmap_le.h:774-779; written by icm456xx_edmp_configure,
/// inv_imu_edmp.c:795-811.
pub const EDMP_PRGRM_IRQ0_0: u16 = IPREG_TOP1 + 0x4F;
/// IPREG_TOP1 0x55 EDMP_SP_START_ADDR (19.46, p178): the eDMP stack address, reset 0x00. Vendor:
/// inv_imu_regmap_le.h:780; inv_imu_edmp.c:801, 808.
pub const EDMP_SP_START_ADDR: u16 = IPREG_TOP1 + 0x55;
/// IPREG_TOP1 0x73 STATUS_MASK_PIN_16_23 (19.61, p185). Reset 0x3F. Bit 5 INT_ON_DEMAND_PIN_2_DIS:
/// 0 lets EDMP_ON_DEMAND_EN run the eDMP through IRQ2. Vendor: inv_imu_regmap_le.h:897-905 (the
/// on-demand mask 0x20 is inv_imu_defs.h:592); cleared by icm456xx_edmp_unmask_int_src,
/// inv_imu_edmp.c:774-793, called with IRQ2 from inv_imu_edmp.c:818 via inv_imu_selftest.c:104.
pub const STATUS_MASK_PIN_16_23: u16 = IPREG_TOP1 + 0x73;
/// STATUS_MASK_PIN_16_23 bit 5, INT_ON_DEMAND_PIN_2_DIS.
pub const INT_ON_DEMAND_PIN_2_DIS: u8 = 0x20;
/// IPREG_TOP1 0xA7 FIFO_SRAM_SLEEP (19.69, p188). Reset 0x00. Bits 1:0 FIFO_GSLEEP_SHARED_SRAM: a 1
/// keeps that SRAM bank awake. Vendor: inv_imu_regmap_le.h:952-956; set to 3 by
/// icm456xx_adv_power_up_sram, inv_imu_driver_advanced.c:1291-1300.
pub const FIFO_SRAM_SLEEP: u16 = IPREG_TOP1 + 0xA7;
/// FIFO_SRAM_SLEEP bits 1:0, FIFO_GSLEEP_SHARED_SRAM.
pub const FIFO_GSLEEP_SHARED_SRAM: u8 = 0x03;

/// IPREG_SYS1 0x2A GYRO_X_OFFUSER\[7:0\] (20.1, p189); the high bits are at +1 (20.2, bits 5:0). The
/// offset is a 14-bit two's complement value, reset 0; a positive value raises the output, and the
/// scale is 62.5 dps / 8192 = 7.629 mdps per LSB (the datasheet's 7.5 mdps is rounded; measured on
/// the board, 2026-11-10).
pub const GYRO_X_OFFUSER: u16 = IPREG_SYS1 + 0x2A;
/// IPREG_SYS1 0x38 GYRO_Y_OFFUSER\[7:0\] (20.3, p189); high bits at +1 (20.4).
pub const GYRO_Y_OFFUSER: u16 = IPREG_SYS1 + 0x38;
/// IPREG_SYS1 0x46 GYRO_Z_OFFUSER\[7:0\] (20.5, p190); high bits at +1 (20.6). The register map
/// (15.5, p65) prints these two as IPREG_SYS1_REG_56 and _57; the descriptions say _70 and _71.
pub const GYRO_Z_OFFUSER: u16 = IPREG_SYS1 + 0x46;
/// IPREG_SYS1 0xA6 IPREG_SYS1_REG_166 (20.7, p190). Reset 0x1B: bit 7 and bits 4:0 are reserved
/// and read 0x1B, keep them. Bits 6:5 GYRO_SRC_CTRL: 0 interpolator and FIR off, 1 FIR on, 2
/// interpolator and FIR on, 3 reserved.
pub const IPREG_SYS1_REG_166: u16 = IPREG_SYS1 + 0xA6;
/// IPREG_SYS1_REG_166 reset value.
pub const IPREG_SYS1_REG_166_RESET: u8 = 0x1B;
/// IPREG_SYS1_REG_166 bits this crate may change (GYRO_SRC_CTRL).
pub const GYRO_SRC_CTRL_MASK: u8 = 0x60;
/// GYRO_SRC_CTRL shift.
pub const GYRO_SRC_CTRL_SHIFT: u8 = 5;
/// IPREG_SYS1 0xAC IPREG_SYS1_REG_172 (20.11, p192). Reset 0x80: bit 7 is GYRO_OIS_HPF1_BYP (AUX1
/// path, not touched). Bits 2:0 GYRO_UI_LPFBW_SEL: 0 bypass, 1..=6 ODR/4 .. ODR/128, 7 ODR/128.
pub const IPREG_SYS1_REG_172: u16 = IPREG_SYS1 + 0xAC;
/// IPREG_SYS1_REG_172 reset value.
pub const IPREG_SYS1_REG_172_RESET: u8 = 0x80;
/// IPREG_SYS1_REG_172 bits this crate may change (GYRO_UI_LPFBW_SEL).
pub const GYRO_UI_LPFBW_MASK: u8 = 0x07;

/// IPREG_SYS2 0x18 ACCEL_X_OFFUSER\[7:0\] (21.1, p193); high bits at +1 (21.2, bits 5:0). The offset
/// is a 14-bit two's complement value, reset 0; a positive value raises the output, and the scale
/// is 1 g / 8192 = 122.07 micro-g per LSB (the datasheet's 0.125 mg is rounded; measured on the
/// board, 2026-11-10).
pub const ACCEL_X_OFFUSER: u16 = IPREG_SYS2 + 0x18;
/// IPREG_SYS2 0x20 ACCEL_Y_OFFUSER\[7:0\] (21.3, p193); high bits at +1 (21.4).
pub const ACCEL_Y_OFFUSER: u16 = IPREG_SYS2 + 0x20;
/// IPREG_SYS2 0x28 ACCEL_Z_OFFUSER\[7:0\] (21.5, p194); high bits at +1 (21.6).
pub const ACCEL_Z_OFFUSER: u16 = IPREG_SYS2 + 0x28;
/// Mask of the OFFUSER high-byte registers' field (bits 5:0); bits 7:6 are reserved.
pub const OFFUSER_HIGH_MASK: u8 = 0x3F;
/// IPREG_SYS2 0x7B IPREG_SYS2_REG_123 (21.7, p194). Reset 0x14: bits 7:2 are reserved and read
/// 0x14, keep them. Bits 1:0 ACCEL_SRC_CTRL: same codes as GYRO_SRC_CTRL.
pub const IPREG_SYS2_REG_123: u16 = IPREG_SYS2 + 0x7B;
/// IPREG_SYS2_REG_123 reset value.
pub const IPREG_SYS2_REG_123_RESET: u8 = 0x14;
/// IPREG_SYS2_REG_123 bits this crate may change (ACCEL_SRC_CTRL).
pub const ACCEL_SRC_CTRL_MASK: u8 = 0x03;
/// ACCEL_SRC_CTRL shift.
pub const ACCEL_SRC_CTRL_SHIFT: u8 = 0;
/// IPREG_SYS2 0x83 IPREG_SYS2_REG_131 (21.10, p196). Reset 0x00. Bits 2:0 ACCEL_UI_LPFBW_SEL, codes
/// as GYRO_UI_LPFBW_SEL.
pub const IPREG_SYS2_REG_131: u16 = IPREG_SYS2 + 0x83;
/// IPREG_SYS2_REG_131 reset value.
pub const IPREG_SYS2_REG_131_RESET: u8 = 0x00;
/// IPREG_SYS2_REG_131 bits this crate may change (ACCEL_UI_LPFBW_SEL).
pub const ACCEL_UI_LPFBW_MASK: u8 = 0x07;
