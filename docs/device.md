# ICM-45686 device facts (MARV V2 Rev 1)

Copied from the MARV V2 firmware repository's `docs/devices.md` and `docs/pinout.md`. Page numbers are PDF pages of the datasheet listed at the end. Items marked UNKNOWN or "unverified" have not been checked against the datasheet or on hardware. "Measured" means measured on this board.

## ICM-45686 (TDK InvenSense, IMU on SPI0)

Pins: see "Pins and bus" below.

- SPI: first byte bit7 = 1 for read, bits 6:0 address; MSB first; burst read supported; max 24 MHz (p49).
- WHO_AM_I 0x72 = 0xE9 (p107).
- PWR_MGMT0 0x10: bits 3:2 GYRO_MODE, bits 1:0 ACCEL_MODE; 11 = low noise. 6-axis low noise = 0x0F (p70).
- ACCEL_CONFIG0 0x1B: bits 6:4 FS (000 = +-32 g, 001 = +-16 g, 010 = +-8 g, 011 = +-4 g, 100 = +-2 g); bits 3:0 ODR
  (0111 = 400 Hz, 1000 = 200 Hz, 1001 = 100 Hz). Reset 0x06 = +-32 g, 800 Hz (p78).
- GYRO_CONFIG0 0x1C: bits 7:4 FS (0000 = +-4000, 0001 = +-2000, 0010 = +-1000, 0011 = +-500, 0100 = +-250 dps);
  bits 3:0 ODR, same codes. Reset 0x06 (p79).
- Data: accel X 0x00/0x01, Y 0x02/0x03, Z 0x04/0x05; gyro X 0x06/0x07, Y 0x08/0x09, Z 0x0A/0x0B; temperature
  0x0C/0x0D (p67-69). Byte order: the datasheet contradicts itself (p57 vs p64). Measured: data is little-endian.
- Accel sensitivity LSB/g: +-32 g 1024, +-16 g 2048, +-8 g 4096, +-4 g 8192, +-2 g 16384 (p19).
  Gyro sensitivity: table on p18; measured: +-250 dps is 131 LSB/dps. Other full-scale values: UNKNOWN here.
- Temperature: degC = raw/128 + 25 (p69).
- INT1_CONFIG0 0x16: bit2 = data-ready enable on INT1; reset 0x80 (p72).
- INT1_CONFIG2 0x18: bit2 drive (0 push-pull), bit1 mode (0 pulse), bit0 polarity (1 active high); reset 0x04.
  Push-pull, pulse, active high = 0x01. Write it while interrupts are disabled (p75).
- Indirect registers (p55-56, p108-109): IREG_ADDR_15_8 0x7C, IREG_ADDR_7_0 0x7D, IREG_DATA 0x7E.
  Bases: IPREG_BAR 0xA000, IPREG_TOP1 0xA200, IPREG_SYS1 0xA400, IPREG_SYS2 0xA500. Write = one burst of
  {addr_hi, addr_lo, data} starting at 0x7C. Read = write the address, CS high, wait >= 4 us, read 0x7E.
  Wait >= 4 us between accesses.
- REG_MISC2 0x7F: bit1 SOFT_RST (self-clearing), bit0 IREG_DONE (p109). Delay after reset: UNKNOWN.
  Measured: RESET_DONE (INT1_STATUS0 bit 7) appears about 1.06 ms after a soft reset.
- Register access is allowed 1 ms after power-up; accel start-up about 10 ms (p19, p21).
- External clock (pin 9 is wired to the 32.000 kHz oscillator X1, see `pinout.md`): RTC_CONFIG 0x26 bit5
  RTC_MODE = 1; it only works if ACCEL_LP_CLK_SEL = 1, which is bit 4 of SMC_CONTROL_0, IREG address 0xA258
  (p178; bit 3 is TEMP_DIS). CLKIN accepted range 20-40 kHz (p37). The datasheet does not say how pin 9 becomes
  CLKIN; the sequence that works on this board, taken from TDK's reference driver, is: IOC_PAD_SCENARIO_OVRD 0x31 =
  0x06; SIFS_I3C_STC_CFG 0xA268 bit 2 = 0; IPREG_SYS2_REG_123 0xA57B ACCEL_SRC_CTRL = 2; IPREG_SYS1_REG_166 0xA4A6
  GYRO_SRC_CTRL = 2; SMC_CONTROL_0 bit 4 = 1; RTC_CONFIG bit 5 = 1. Output data rate scales as CLKIN/32 kHz (TDK
  application note, not the datasheet): 200.000 Hz nominal with the 32.000 kHz X1.
- Measured output rate: internal oscillator 200.895 Hz for a nominal 200 Hz; external clock 200.000 Hz. On the
  external clock the rate is exact up to 3.2 kHz but 6388.7 Hz at the 6.4 kHz setting. The external-clock set-up
  also changes the ICM's filter path, so compare noise only with the filter path held equal.
- FIFO timestamp tick (measured on the MARV V2 board, 2026-11-11, external clock): frames at 200 Hz were 5022.5 us
  apart in timestamp units at both 1 us and 16 us per tick, the MCU clock measured 5000.5 us per frame, and the output
  rate on that clock was exact; the tick runs about 0.44 % fast and does not follow the external clock. Timestamps
  space frames relative to each other; real time needs a calibration of the tick against the application's clock.
- Decimation (same board and day, FIFO on at 800 Hz, factors 1, 2, 8): INT1 data-ready 800, 400, 100 per second,
  data registers changed at the same rates, FIFO frame rate followed. With the FIFO off the driver does not write the
  decimation, and with factor 8 set then INT1 and both registers stayed at 800 per second.
- FIFO overflow (2 KB, 16-byte frames, 800 Hz, 350 ms unread): stream mode counts up to capacity + 2 (130); the first
  two frames of the read are stale, then a timestamp jump, then the newest 127 contiguous. Stop-on-full counts to the
  capacity (128) and returns the oldest frames. The INT1 FIFO-full flag was set with its INT1 source enabled
  (2026-11-11) and stayed 0 with only the threshold source enabled (2026-11-10); enable the source to see it.
- Different accel and gyro rates, FIFO off: INT1 follows the faster sensor, each sensor's registers change at its own
  rate. With the FIFO on both must be equal after decimation; frames then arrive at the common rate.

## Pins and bus (from pinout.md)

All GPIO are 3.3 V logic and are not 5 V tolerant.

| GPIO | Net | Function used | Status |
|---|---|---|---|
| 15 | ICM_INT1 | GPIO input | ICM-45686 INT1 |
| 16 | ICM_MISO | SPI0 RX (F1) | ICM-45686 |
| 18 | ICM_SCK | SPI0 SCK (F1) | ICM-45686 |
| 19 | ICM_MOSI | SPI0 TX (F1) | ICM-45686 |
| 20 | ICM_CS | GPIO (chip select) | ICM-45686; 10k pull-up R48 |

| Peripheral | GPIOs | Device |
|---|---|---|
| SPI0 | SCK 18, MOSI 19, MISO 16; CS 20 (plain GPIO) | ICM-45686 only (U21), 4-wire |

| Item | Value |
|---|---|
| ICM-45686 CS (GPIO20) | 10k pull-up R48 to V3V3_ANA |
| Sensor SPI clock and data | no pull resistors on either bus |
| ICM-45686 pin 9 (INT2/FSYNC/CLKIN) | used as CLKIN, driven by the 32.000 kHz oscillator X1 (HCI 1521H-32K000J18DTSL, net ICM_CLKIN; Vcc and E/D on V3V3_ANA, 100 nF on Vcc); not connected to any MCU pin |
| ICM-45686 interrupts | INT1 only reaches the MCU (GPIO15); there is no INT2 on the MCU |

Rails: V3V3_ANA (sensors, their pull-ups, X1, ADC_AVDD) is enabled from V3V3_SYS, so it comes up after V3V3_SYS.

## Datasheet

| Device | URL |
|---|---|
| ICM-45686 | https://www.lcsc.com/datasheet/lcsc_datasheet_2411220643_TDK-InvenSense-ICM-45686_C22459454.pdf |
