//! The register write plan: from a validated [`Config`], the ordered steps that apply it. Pure
//! code, no bus and no allocation; a later `init` or `configure` only executes the steps.
//!
//! Two plans exist. [`Plan::after_reset`] is for `init`: the device has just been soft-reset, so a
//! register that already holds the wanted value is not written, except the ones `init` has always
//! written. [`Plan::reconfigure`] is for a running device: it stops the sensors, the INT1 events and
//! the FIFO first and then writes every register this crate controls.
//!
//! Order, with the datasheet reason of each rule:
//! - Sensors are off before any field that "must not be changed on-the-fly if sensor is on" is
//!   written (text before section 16, p66), and the FIFO decimation "must" be written with sensors
//!   disabled (5, p39). After a reset they are off; `reconfigure` writes PWR_MGMT0 = 0 first.
//! - INT1 events are disabled before INT1_CONFIG2 (mode and polarity "can be changed only when all
//!   interrupts ... are disabled", 16.23, p75) and enabled last but one.
//! - The clock sequence runs before the rates (RTC mode needs ACCEL_LP_CLK_SEL = 1, 16.37, p86).
//! - The FIFO is configured in bypass (depth only changes there, 16.28, p80), then the mode is
//!   set, then FIFO_IF_EN (16.32, p82); it is disabled in the opposite order.
//! - The watermark is written low byte first (16.29, p80).
//! - PWR_MGMT0 comes last, followed by the start-up wait.
//! - `reconfigure` waits after turning the sensors off, before any step that turns the gyroscope on
//!   again (vendor: TDK's Linux driver waits 150 ms after stopping the gyroscope).
//!
//! - [`Plan::after_reset`] writes the twelve user-offset registers to zero (the six offsets of
//!   [`crate::Offsets`], reset value 0x00, 20.1-21.6, p189-p194). The datasheet gives their reset
//!   value but does not say that a soft reset restores it (16.84, p109 only says that SOFT_RST
//!   "triggers soft reset operation"), so `init` does not rely on it: calibration left in the
//!   registers would be silently applied to the data (the device does clear them on a soft reset,
//!   measured on the board, 2026-11-10). [`Plan::reconfigure`] leaves them alone (they are
//!   calibration, not configuration): `configure` writes the stored offsets after the plan.
//! - [`Plan::power_change`] is the single write of PWR_MGMT0 that `set_power` executes, with the same
//!   waits as the plans above.
//!
//! Waits between two indirect register accesses (at least 4 us, 13.3, p55) belong to the executor of
//! an [`Step::IregWrite`] or [`Step::IregModify`]; they are not listed as steps.

use crate::config::{ClockSource, Config, FifoMode, SensorPower};
use crate::image;
use crate::offsets::Offsets;
use crate::reg;
use crate::validate::ConfigError;

/// One step of a plan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Step {
    /// Write `value` to the direct register `reg`.
    Write {
        /// Direct register address.
        reg: u8,
        /// Value to write.
        value: u8,
    },
    /// Read the direct register, replace the bits under `mask` with `value`, write it back.
    Modify {
        /// Direct register address.
        reg: u8,
        /// Bits to replace.
        mask: u8,
        /// New bits; zero outside `mask`.
        value: u8,
    },
    /// Write `value` to the indirect register `addr` (bank base plus offset).
    IregWrite {
        /// Indirect register address.
        addr: u16,
        /// Value to write.
        value: u8,
    },
    /// Read the indirect register, replace the bits under `mask` with `value`, write it back.
    IregModify {
        /// Indirect register address.
        addr: u16,
        /// Bits to replace.
        mask: u8,
        /// New bits; zero outside `mask`.
        value: u8,
    },
    /// Wait this many microseconds.
    DelayUs(u32),
}

/// Room for the longest plan (a reconfiguration with every option on); a test checks that no plan
/// reaches it. A step that does not fit is an error ([`ConfigError::PlanOverflow`]), never dropped.
pub const PLAN_CAPACITY: usize = 48;

/// Wait after PWR_MGMT0 when the gyroscope is on: 60 ms (vendor: TDK's Linux driver; the datasheet
/// gives 35 ms start-up, Table 1, p18).
pub const GYRO_START_UP_US: u32 = 60_000;
/// Wait after PWR_MGMT0 when only the accelerometer is on: 60 ms (vendor: TDK's Linux driver; the
/// datasheet gives 10 ms start-up, Table 2, p19).
pub const ACCEL_START_UP_US: u32 = 60_000;
/// Wait in the in-place plan after the sensors are turned off and before the gyroscope is turned on
/// again: 150 ms (vendor: TDK's Linux driver, wait after stopping the gyroscope).
pub const GYRO_STOP_US: u32 = 150_000;
/// Wait after DRIVE_CONFIG0: the slew rate "takes effect 1.5 us after the register is programmed"
/// (16.48, p93).
pub const SLEW_SETTLE_US: u32 = 2;

const IDLE: Step = Step::DelayUs(0);

/// A sensor that goes up a state (off, standby, low-noise): it has to start.
fn turned_on(from: SensorPower, to: SensorPower) -> bool {
    to.code() > from.code()
}

/// Wait after PWR_MGMT0 for sensors that are starting: the gyroscope's wait when it is, else the
/// accelerometer's, else none.
const fn start_up_wait(gyro_starting: bool, accel_starting: bool) -> u32 {
    if gyro_starting {
        GYRO_START_UP_US
    } else if accel_starting {
        ACCEL_START_UP_US
    } else {
        0
    }
}

/// An ordered, bounded list of [`Step`]s.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    steps: [Step; PLAN_CAPACITY],
    len: usize,
    full: bool,
    overflow: bool,
    limit: usize,
}

impl Plan {
    /// The plan `init` executes right after a soft reset. Fails if the configuration is invalid.
    pub fn after_reset(config: &Config) -> Result<Self, ConfigError> {
        Self::with_capacity(config, false, PLAN_CAPACITY)
    }

    /// The plan `configure` executes on a device that may be running. Fails if the configuration is
    /// invalid.
    pub fn reconfigure(config: &Config) -> Result<Self, ConfigError> {
        Self::with_capacity(config, true, PLAN_CAPACITY)
    }

    /// The plan `set_power` executes: one write of PWR_MGMT0 that takes the sensors from the power
    /// states of `from` to those of `to`, the rest of `to` being the same as `from`'s (16.17, p70).
    /// Fails if `to` is invalid.
    ///
    /// - A gyroscope that is off in `from` and on in `to` is preceded by the gyroscope stop wait
    ///   [`GYRO_STOP_US`]: the driver cannot know how long the gyroscope has been off.
    /// - After the write the start-up wait follows the rule of the other plans, applied to the
    ///   sensors that go up a state (off, standby, low-noise): [`GYRO_START_UP_US`] when the
    ///   gyroscope does, else [`ACCEL_START_UP_US`] when the accelerometer does, else no wait.
    ///
    /// The write is made even when the states do not change.
    pub fn power_change(from: &Config, to: &Config) -> Result<Self, ConfigError> {
        to.validate()?;
        let mut p = Self::new(false, PLAN_CAPACITY);
        let gyro_on = turned_on(from.gyro_power, to.gyro_power);
        if from.gyro_power == SensorPower::Off && gyro_on {
            p.push(Step::DelayUs(GYRO_STOP_US));
        }
        p.write(reg::PWR_MGMT0, image::pwr_mgmt0(to));
        let wait = start_up_wait(gyro_on, turned_on(from.accel_power, to.accel_power));
        if wait != 0 {
            p.push(Step::DelayUs(wait));
        }
        Ok(p)
    }

    /// The public entries, with the room for steps as a parameter so that a test can make the plan
    /// overflow.
    fn with_capacity(config: &Config, full: bool, limit: usize) -> Result<Self, ConfigError> {
        config.validate()?;
        Self::build(config, full, limit)
    }

    /// The steps, in execution order.
    pub fn steps(&self) -> &[Step] {
        self.steps.get(..self.len).unwrap_or_default()
    }

    /// Number of steps.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Whether the plan has no step.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    fn push(&mut self, step: Step) {
        match self.steps.get_mut(self.len) {
            Some(slot) if self.len < self.limit => {
                *slot = step;
                self.len += 1;
            }
            _ => self.overflow = true,
        }
    }

    fn write(&mut self, reg: u8, value: u8) {
        self.push(Step::Write { reg, value });
    }

    /// A direct-register write that a post-reset plan skips when the register already holds it.
    fn write_unless_reset(&mut self, reg: u8, value: u8, reset: u8) {
        if self.full || value != reset {
            self.write(reg, value);
        }
    }

    fn modify(&mut self, reg: u8, mask: u8, value: u8) {
        self.push(Step::Modify { reg, mask, value });
    }

    /// An indirect field write that a post-reset plan skips when the field already holds it.
    fn iregs_unless_reset(&mut self, addr: u16, mask: u8, value: u8, reset: u8) {
        if self.full || (reset & mask) != value {
            self.push(Step::IregModify { addr, mask, value });
        }
    }

    fn new(full: bool, limit: usize) -> Self {
        Self {
            steps: [IDLE; PLAN_CAPACITY],
            len: 0,
            full,
            overflow: false,
            limit,
        }
    }

    fn build(c: &Config, full: bool, limit: usize) -> Result<Self, ConfigError> {
        let mut p = Self::new(full, limit);
        let clkin = c.clock == ClockSource::ExternalClkin;

        if full {
            p.write(reg::PWR_MGMT0, reg::POWER_OFF);
            if image::pwr_mgmt0(c) >> reg::PWR_MGMT0_GYRO_SHIFT != 0 {
                p.push(Step::DelayUs(GYRO_STOP_US));
            }
        }
        p.write(reg::INT1_CONFIG0, 0);
        if full {
            p.write(reg::FIFO_CONFIG3, reg::FIFO_CONFIG3_RESET);
            p.write(reg::FIFO_CONFIG0, reg::FIFO_CONFIG0_RESET);
        }
        p.write(reg::INT1_CONFIG2, image::int1_config2(c));
        p.iregs_unless_reset(
            reg::INT_PULSE_MIN_ON_INTF1,
            reg::INT_PULSE_MIN_ON_INTF1_FIELDS,
            image::int1_pulse_width(c),
            reg::INT_PULSE_MIN_ON_INTF1_RESET,
        );

        if !p.full {
            for (addr, _, _) in Offsets::new().registers() {
                p.push(Step::IregWrite { addr, value: 0 });
            }
        }

        if clkin {
            p.write(reg::IOC_PAD_SCENARIO_OVRD, reg::IOC_PAD_SCENARIO_OVRD_CLKIN);
            p.push(Step::IregModify {
                addr: reg::SIFS_I3C_STC_CFG,
                mask: reg::I3C_STC_MODE,
                value: 0,
            });
        }
        if clkin || p.full || image::accel_src_ctrl(c) != 0 {
            p.push(Step::IregModify {
                addr: reg::IPREG_SYS2_REG_123,
                mask: reg::ACCEL_SRC_CTRL_MASK,
                value: image::accel_src_ctrl(c),
            });
        }
        if clkin || p.full || image::gyro_src_ctrl(c) != 0 {
            p.push(Step::IregModify {
                addr: reg::IPREG_SYS1_REG_166,
                mask: reg::GYRO_SRC_CTRL_MASK,
                value: image::gyro_src_ctrl(c),
            });
        }
        if clkin || p.full {
            p.push(Step::IregModify {
                addr: reg::SMC_CONTROL_0,
                mask: reg::ACCEL_LP_CLK_SEL,
                value: image::accel_lp_clk_sel(c),
            });
            p.modify(reg::RTC_CONFIG, reg::RTC_MODE, image::rtc_mode(c));
        }
        if clkin && c.rtc_align {
            p.modify(reg::RTC_CONFIG, reg::RTC_ALIGN, reg::RTC_ALIGN);
        }

        p.iregs_unless_reset(
            reg::IPREG_SYS2_REG_131,
            reg::ACCEL_UI_LPFBW_MASK,
            image::accel_ui_lpfbw(c),
            reg::IPREG_SYS2_REG_131_RESET,
        );
        p.iregs_unless_reset(
            reg::IPREG_SYS1_REG_172,
            reg::GYRO_UI_LPFBW_MASK,
            image::gyro_ui_lpfbw(c),
            reg::IPREG_SYS1_REG_172_RESET,
        );

        let slew = image::drive_config0(c);
        if p.full || slew != reg::DRIVE_CONFIG0_RESET {
            p.write(reg::DRIVE_CONFIG0, slew);
            p.push(Step::DelayUs(SLEW_SETTLE_US));
        }

        p.write_unless_reset(
            reg::TMST_WOM_CONFIG,
            image::tmst_wom_config(c),
            reg::TMST_WOM_CONFIG_RESET,
        );
        p.iregs_unless_reset(
            reg::SMC_CONTROL_0,
            reg::TMST_EN,
            image::smc_timestamp(c),
            reg::SMC_CONTROL_0_RESET,
        );
        p.write_unless_reset(
            reg::ODR_DECIMATE_CONFIG,
            image::odr_decimate_config(c),
            reg::ODR_DECIMATE_CONFIG_RESET,
        );

        p.build_fifo_setup(c);

        p.write(reg::ACCEL_CONFIG0, image::accel_config0(c));
        p.write(reg::GYRO_CONFIG0, image::gyro_config0(c));

        if c.fifo.mode != FifoMode::Off {
            p.write(reg::FIFO_CONFIG0, image::fifo_config0(c));
            p.write(reg::FIFO_CONFIG3, image::fifo_config3(c, true));
        }

        p.write(reg::INT1_CONFIG0, image::int1_config0(c));
        p.write(reg::PWR_MGMT0, image::pwr_mgmt0(c));
        let wait = start_up_wait(
            c.gyro_power != SensorPower::Off,
            c.accel_power != SensorPower::Off,
        );
        if wait != 0 {
            p.push(Step::DelayUs(wait));
        }
        if p.overflow {
            return Err(ConfigError::PlanOverflow);
        }
        Ok(p)
    }

    /// FIFO registers up to, not including, the switch-on (depth, watermark, flush, content,
    /// timestamp).
    fn build_fifo_setup(&mut self, c: &Config) {
        let on = c.fifo.enabled();
        if on {
            self.write(reg::FIFO_CONFIG0, image::fifo_config0_bypass(c));
        }
        let wm = image::fifo_watermark(c);
        if self.full || wm != [0, 0] {
            self.write(reg::FIFO_CONFIG1_0, wm[0]);
            self.write(reg::FIFO_CONFIG1_1, wm[1]);
        }
        let config2 = image::fifo_config2(c);
        if self.full {
            self.write(reg::FIFO_CONFIG2, config2 | reg::FIFO_FLUSH);
            self.write(reg::FIFO_CONFIG2, config2);
        } else {
            self.write_unless_reset(reg::FIFO_CONFIG2, config2, reg::FIFO_CONFIG2_RESET);
        }
        if on {
            self.write(reg::FIFO_CONFIG3, image::fifo_config3(c, false));
        }
        self.write_unless_reset(
            reg::FIFO_CONFIG4,
            image::fifo_config4(c),
            reg::FIFO_CONFIG4_RESET,
        );
    }
}

impl<'a> IntoIterator for &'a Plan {
    type Item = &'a Step;
    type IntoIter = core::slice::Iter<'a, Step>;

    fn into_iter(self) -> Self::IntoIter {
        self.steps().iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_step_past_the_capacity_is_an_error_not_a_drop() {
        let mut p = Plan::new(false, PLAN_CAPACITY);
        for _ in 0..PLAN_CAPACITY {
            p.push(Step::Write { reg: 1, value: 2 });
        }
        assert!(!p.overflow);
        assert_eq!(p.len(), PLAN_CAPACITY);
        p.push(Step::Write { reg: 3, value: 4 });
        assert!(p.overflow);
        assert_eq!(p.len(), PLAN_CAPACITY);
        assert_eq!(p.steps().last(), Some(&Step::Write { reg: 1, value: 2 }));
    }

    #[test]
    fn building_a_plan_that_does_not_fit_is_plan_overflow_through_both_entries() {
        for full in [false, true] {
            let c = Config::new();
            let n = Plan::with_capacity(&c, full, PLAN_CAPACITY).unwrap().len();
            assert!(n > 3);
            assert_eq!(Plan::with_capacity(&c, full, n).map(|p| p.len()), Ok(n));
            assert_eq!(
                Plan::with_capacity(&c, full, n - 1),
                Err(ConfigError::PlanOverflow)
            );
            assert_eq!(
                Plan::with_capacity(&c, full, 0),
                Err(ConfigError::PlanOverflow)
            );
        }
        let bad = Config::new().with_accel_power(crate::SensorPower::Standby);
        assert!(matches!(
            Plan::with_capacity(&bad, false, 0),
            Err(e) if e != ConfigError::PlanOverflow
        ));
        assert_eq!(Plan::after_reset(&Config::new()).map(|p| p.len()), Ok(19));
    }
}
