//! A model of the device's register files (direct, indirect), and a recording SPI device on top of
//! it. Used to run a plan, and the real `init`, against the same reset state.

use std::collections::BTreeMap;
use std::convert::Infallible;

use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

use crate::plan::Step;
use crate::reg;

/// One register write seen on the bus, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Wr {
    Direct(u8, u8),
    Ireg(u16, u8),
}

/// Direct registers this crate manages: address, mask of the bits it may change, reset value.
pub const DIRECT: [(u8, u8, u8); 14] = [
    (reg::PWR_MGMT0, reg::PWR_MGMT0_FIELDS, reg::PWR_MGMT0_RESET),
    (
        reg::INT1_CONFIG0,
        reg::INT1_CONFIG0_FIELDS,
        reg::INT1_CONFIG0_RESET,
    ),
    (
        reg::INT1_CONFIG2,
        reg::INT1_CONFIG2_FIELDS,
        reg::INT1_CONFIG2_RESET,
    ),
    (
        reg::ACCEL_CONFIG0,
        reg::ACCEL_CONFIG0_FIELDS,
        reg::ACCEL_CONFIG0_RESET,
    ),
    (
        reg::GYRO_CONFIG0,
        reg::GYRO_CONFIG0_FIELDS,
        reg::GYRO_CONFIG0_RESET,
    ),
    (
        reg::FIFO_CONFIG0,
        reg::FIFO_CONFIG0_FIELDS,
        reg::FIFO_CONFIG0_RESET,
    ),
    (reg::FIFO_CONFIG1_0, 0xFF, 0x00),
    (reg::FIFO_CONFIG1_1, 0xFF, 0x00),
    (
        reg::FIFO_CONFIG2,
        reg::FIFO_CONFIG2_FIELDS,
        reg::FIFO_CONFIG2_RESET,
    ),
    (
        reg::FIFO_CONFIG3,
        reg::FIFO_CONFIG3_FIELDS,
        reg::FIFO_CONFIG3_RESET,
    ),
    (
        reg::FIFO_CONFIG4,
        reg::FIFO_CONFIG4_FIELDS,
        reg::FIFO_CONFIG4_RESET,
    ),
    (
        reg::TMST_WOM_CONFIG,
        reg::TMST_WOM_CONFIG_FIELDS,
        reg::TMST_WOM_CONFIG_RESET,
    ),
    (
        reg::ODR_DECIMATE_CONFIG,
        reg::ODR_DECIMATE_CONFIG_FIELDS,
        reg::ODR_DECIMATE_CONFIG_RESET,
    ),
    (
        reg::DRIVE_CONFIG0,
        reg::DRIVE_CONFIG0_FIELDS,
        reg::DRIVE_CONFIG0_RESET,
    ),
];

/// RTC_CONFIG: managed apart because RTC_ALIGN is a command, not state.
pub const RTC: (u8, u8, u8) = (reg::RTC_CONFIG, reg::RTC_MODE, reg::RTC_CONFIG_RESET);

/// Indirect registers this crate manages: address, mask, reset value.
pub const IREG: [(u16, u8, u8); 7] = [
    (
        reg::SMC_CONTROL_0,
        reg::ACCEL_LP_CLK_SEL | reg::TMST_EN,
        reg::SMC_CONTROL_0_RESET,
    ),
    (
        reg::SIFS_I3C_STC_CFG,
        reg::I3C_STC_MODE,
        reg::SIFS_I3C_STC_CFG_RESET,
    ),
    (
        reg::INT_PULSE_MIN_ON_INTF0,
        reg::INT_PULSE_MIN_ON_INTF0_FIELDS,
        reg::INT_PULSE_MIN_ON_INTF0_RESET,
    ),
    (
        reg::IPREG_SYS1_REG_166,
        reg::GYRO_SRC_CTRL_MASK,
        reg::IPREG_SYS1_REG_166_RESET,
    ),
    (
        reg::IPREG_SYS2_REG_123,
        reg::ACCEL_SRC_CTRL_MASK,
        reg::IPREG_SYS2_REG_123_RESET,
    ),
    (
        reg::IPREG_SYS2_REG_131,
        reg::ACCEL_UI_LPFBW_MASK,
        reg::IPREG_SYS2_REG_131_RESET,
    ),
    (
        reg::IPREG_SYS1_REG_172,
        reg::GYRO_UI_LPFBW_MASK,
        reg::IPREG_SYS1_REG_172_RESET,
    ),
];

#[derive(Clone, Debug)]
pub struct Device {
    pub direct: [u8; 256],
    pub ireg: BTreeMap<u16, u8>,
    pub addr: u16,
    pub log: Vec<Wr>,
}

impl Device {
    /// The state after a soft reset, with identity and reset-done readable.
    pub fn reset() -> Self {
        let mut direct = [0u8; 256];
        for (a, _, r) in DIRECT.into_iter().chain([RTC]) {
            direct[a as usize] = r;
        }
        direct[reg::WHO_AM_I as usize] = reg::WHO_AM_I_VALUE;
        direct[reg::INT1_STATUS0 as usize] = reg::RESET_DONE;
        let ireg = IREG.into_iter().map(|(a, _, r)| (a, r)).collect();
        Self {
            direct,
            ireg,
            addr: 0,
            log: Vec::new(),
        }
    }

    pub fn write_direct(&mut self, a: u8, v: u8) {
        self.direct[a as usize] = v;
        self.log.push(Wr::Direct(a, v));
    }

    pub fn write_ireg(&mut self, a: u16, v: u8) {
        self.ireg.insert(a, v);
        self.log.push(Wr::Ireg(a, v));
    }

    pub fn read_ireg(&self, a: u16) -> u8 {
        self.ireg.get(&a).copied().unwrap_or(0)
    }

    /// Executes a plan step the way a driver would.
    pub fn exec(&mut self, step: &Step) {
        match *step {
            Step::Write { reg, value } => self.write_direct(reg, value),
            Step::Modify { reg, mask, value } => {
                let old = self.direct[reg as usize];
                self.write_direct(reg, (old & !mask) | value);
            }
            Step::IregWrite { addr, value } => self.write_ireg(addr, value),
            Step::IregModify { addr, mask, value } => {
                let old = self.read_ireg(addr);
                self.write_ireg(addr, (old & !mask) | value);
            }
            Step::DelayUs(_) => {}
        }
    }
}

/// An SPI device over a [`Device`], following the bus protocol of the real driver: burst writes,
/// the indirect address registers 0x7C/0x7D and the data register 0x7E.
pub struct LogSpi(pub Device);

impl ErrorType for LogSpi {
    type Error = Infallible;
}

impl SpiDevice for LogSpi {
    fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), Infallible> {
        let d = &mut self.0;
        let mut cmd: Option<u8> = None;
        let mut step = |d: &mut Device, byte: &mut u8, input: bool| match cmd {
            None => cmd = Some(*byte),
            Some(c) => {
                let a = c & 0x7F;
                if c & 0x80 != 0 {
                    *byte = if a == reg::IREG_DATA {
                        d.read_ireg(d.addr)
                    } else {
                        d.direct[a as usize]
                    };
                } else if input {
                    match a {
                        reg::IREG_ADDR_15_8 => d.addr = (d.addr & 0x00FF) | (u16::from(*byte) << 8),
                        reg::IREG_ADDR_7_0 => d.addr = (d.addr & 0xFF00) | u16::from(*byte),
                        reg::IREG_DATA => d.write_ireg(d.addr, *byte),
                        _ => d.write_direct(a, *byte),
                    }
                }
                cmd = Some(c.wrapping_add(1));
            }
        };
        for op in ops {
            match op {
                Operation::Write(w) => w.iter().for_each(|b| step(d, &mut b.clone(), true)),
                Operation::Read(r) => r.iter_mut().for_each(|b| step(d, b, false)),
                Operation::TransferInPlace(t) => t.iter_mut().for_each(|b| step(d, b, true)),
                _ => {}
            }
        }
        Ok(())
    }
}
