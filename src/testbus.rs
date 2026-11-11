//! Host test harness: a fake SPI device over a 256-byte direct register file plus the indirect
//! register file behind IREG_ADDR / IREG_DATA, a fake delay, a counter of await points, and a
//! minimal executor. One copy per driver repository, on purpose.
//!
//! The chip, the bus and the delay share one state so that a test can inspect the traffic after a
//! future was dropped. Awaits are counted in one sequence across bus and delay; a pend at await
//! `k` returns `Poll::Pending` once, before the operation takes effect, and completes afterwards.

use core::future::Future;
use core::pin::{Pin, pin};
use core::task::{Context, Poll, Waker};
use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex, MutexGuard};
use std::vec::Vec;

use embedded_hal::spi::{ErrorKind, ErrorType, Operation};

use crate::reg;

const READ_BIT: u8 = 0x80;

/// The bus error the fake raises.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BusFault;

impl embedded_hal::spi::Error for BusFault {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// What one operation of a transaction put on the bus and got back.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Op {
    /// `Operation::Write`: the bytes sent.
    Write(Vec<u8>),
    /// `Operation::Read`: the bytes returned.
    Read(Vec<u8>),
    /// `Operation::TransferInPlace`: the bytes sent and the bytes returned.
    InPlace(Vec<u8>, Vec<u8>),
}

/// One logged event, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Event {
    /// A completed transaction.
    Spi(Vec<Op>),
    /// A delay, in nanoseconds.
    Delay(u64),
}

/// Whether an indirect address is one of the twelve user-offset registers.
fn is_offset_register(a: u16) -> bool {
    crate::Offsets::new()
        .registers()
        .iter()
        .any(|(addr, _, _)| *addr == a)
}

pub(crate) struct Chip {
    pub regs: [u8; 256],
    /// The indirect register file, addressed by the 16-bit IREG address.
    pub ireg: BTreeMap<u16, u8>,
    pub log: Vec<Event>,
    /// Transactions started since the last `arm` (failed ones included).
    pub transactions: usize,
    /// Awaits reached since the last `arm`.
    pub awaits: u32,
    /// Fail this transaction (1-based) with `BusFault`; it has no effect on the register file.
    pub fail_at: Option<usize>,
    /// Return `Pending` once at this await (1-based).
    pub pend_at: Option<u32>,
    pub pended: bool,
    /// Bus rules the driver broke: a read without a preceding read address byte, an equal-length
    /// transfer, `Operation::DelayNs`.
    pub violations: usize,
    /// RESET_DONE never appears after a soft reset.
    pub reset_stuck: bool,
    /// Corrupt (invert) the first byte of the read operation of this transaction (1-based), once.
    /// A transaction without a read leaves it armed.
    pub glitch_at: Option<usize>,
    /// The FIFO: queued bytes, whole frames pushed with `push_frame`.
    pub fifo: VecDeque<u8>,
    /// FIFO_COUNT values the next count reads return instead of the true one (one per read of
    /// FIFO_COUNT_0), for the stale-first-read erratum.
    pub count_script: VecDeque<u16>,
    /// The count latched by the last read of FIFO_COUNT_0; FIFO_COUNT_1 returns its high byte
    /// (little endian, the device default: 14, p57).
    pub count_latch: u16,
    /// The flush bit never clears.
    pub flush_stuck: bool,
    /// A soft reset leaves the twelve user-offset registers as they are (the datasheet does not say
    /// that it clears them).
    pub reset_keeps_offsets: bool,
    /// Self-test model (crate::selftest): reads of INT_APEX_STATUS1 after the start until the
    /// done bit shows (1 = at the first poll).
    pub selftest_polls: u32,
    /// The done bit never shows.
    pub selftest_never: bool,
    /// The result byte the eDMP leaves at IMEM_SRAM 0x44.
    pub selftest_result: u8,
    /// Polls left of a running self-test.
    pub selftest_left: Option<u32>,
    /// Self-tests started.
    pub selftest_starts: u32,
    /// Bytes the IMEM_SRAM holds after a reset ("random", 17, p110); absent bytes read 0.
    pub sram_after_reset: Vec<(u16, u8)>,
}

impl Chip {
    fn new() -> Self {
        let mut chip = Self {
            regs: [0u8; 256],
            ireg: BTreeMap::new(),
            log: Vec::new(),
            transactions: 0,
            awaits: 0,
            fail_at: None,
            pend_at: None,
            pended: false,
            violations: 0,
            reset_stuck: false,
            glitch_at: None,
            fifo: VecDeque::new(),
            count_script: VecDeque::new(),
            count_latch: 0,
            flush_stuck: false,
            reset_keeps_offsets: false,
            selftest_polls: 1,
            selftest_never: false,
            selftest_result: 0x00,
            selftest_left: None,
            selftest_starts: 0,
            sram_after_reset: Vec::new(),
        };
        chip.regs[reg::WHO_AM_I as usize] = reg::WHO_AM_I_VALUE;
        // Distinct non-trivial bytes over the data block (accel, gyro, temperature).
        for (i, r) in chip.regs[..14].iter_mut().enumerate() {
            *r = (i as u8).wrapping_mul(37).wrapping_add(11);
        }
        chip.reset_state();
        chip
    }

    /// The registers this crate writes, back to their reset values (16.x, reg.rs).
    fn reset_state(&mut self) {
        for (a, v) in [
            (reg::INT1_CONFIG0, reg::INT1_CONFIG0_RESET),
            (reg::INT1_CONFIG2, reg::INT1_CONFIG2_RESET),
            (reg::ACCEL_CONFIG0, reg::ACCEL_CONFIG0_RESET),
            (reg::GYRO_CONFIG0, reg::GYRO_CONFIG0_RESET),
            (reg::FIFO_CONFIG2, reg::FIFO_CONFIG2_RESET),
            (reg::RTC_CONFIG, reg::RTC_CONFIG_RESET),
            (reg::DRIVE_CONFIG0, reg::DRIVE_CONFIG0_RESET),
            (reg::PWR_MGMT0, reg::PWR_MGMT0_RESET),
            (reg::EDMP_APEX_EN1, 0x00),    // 16.41, p88
            (reg::INT_APEX_CONFIG1, 0x00), // 16.53, p96
            (reg::INT_APEX_STATUS1, 0x00), // 16.55, p97
            (reg::REG_HOST_MSG, 0x00),     // 16.80, p108
        ] {
            self.regs[a as usize] = v;
        }
        self.fifo.clear();
        self.regs[reg::IREG_ADDR_15_8 as usize] = 0xAF;
        self.regs[reg::IREG_ADDR_7_0 as usize] = 0x06;
        self.regs[reg::IREG_DATA as usize] = 0x02;
        self.regs[reg::REG_MISC2 as usize] = 0x01;
        let kept: Vec<(u16, u8)> = if self.reset_keeps_offsets {
            self.ireg
                .iter()
                .filter(|(a, _)| is_offset_register(**a))
                .map(|(a, v)| (*a, *v))
                .collect()
        } else {
            Vec::new()
        };
        self.ireg = BTreeMap::from([
            (reg::SMC_CONTROL_0, reg::SMC_CONTROL_0_RESET),
            (reg::SIFS_I3C_STC_CFG, reg::SIFS_I3C_STC_CFG_RESET),
            (
                reg::INT_PULSE_MIN_ON_INTF1,
                reg::INT_PULSE_MIN_ON_INTF1_RESET,
            ),
            (reg::IPREG_SYS1_REG_166, reg::IPREG_SYS1_REG_166_RESET),
            (reg::IPREG_SYS1_REG_172, reg::IPREG_SYS1_REG_172_RESET),
            (reg::IPREG_SYS2_REG_123, reg::IPREG_SYS2_REG_123_RESET),
            (reg::IPREG_SYS2_REG_131, reg::IPREG_SYS2_REG_131_RESET),
        ]);
        self.ireg.extend(kept);
        self.ireg.insert(reg::STATUS_MASK_PIN_16_23, 0x3F); // 19.61, p185
        self.ireg.extend(self.sram_after_reset.iter().copied());
        self.selftest_left = None;
    }

    /// REG_HOST_MSG written: EDMP_ON_DEMAND_EN with TESTOPENABLE starts the self-test when the eDMP
    /// is enabled, its on-demand source unmasked, STC_INIT_EN set, the debug byte 0 and the vendor's
    /// zeros in IMEM_SRAM 0x3A..=0x3F and 0x41..=0x43. The on-demand bit clears itself (16.80, p108).
    fn host_msg(&mut self, byte: u8) {
        let ready = self.regs[reg::EDMP_APEX_EN1 as usize] & reg::EDMP_ENABLE != 0
            && self
                .ireg
                .get(&reg::STATUS_MASK_PIN_16_23)
                .copied()
                .unwrap_or(0)
                & reg::INT_ON_DEMAND_PIN_2_DIS
                == 0
            && self.ireg.get(&reg::IMEM_SRAM_REG_56).copied().unwrap_or(0) & reg::STC_INIT_EN != 0
            && (0x3A..=0x43u16)
                .filter(|a| *a != 0x40)
                .all(|a| self.ireg.get(&(reg::IMEM_SRAM + a)).copied().unwrap_or(0) == 0)
            && self.ireg.get(&reg::IMEM_SRAM_REG_64).copied().unwrap_or(0) == 0;
        let start = reg::EDMP_ON_DEMAND_EN | reg::TESTOPENABLE;
        if byte & start == start && ready {
            self.selftest_starts += 1;
            self.selftest_left = Some(self.selftest_polls);
            self.ireg
                .insert(reg::IMEM_SRAM_REG_68, reg::ST_STATUS_IN_PROGRESS);
        }
        self.regs[reg::REG_HOST_MSG as usize] = byte & !reg::EDMP_ON_DEMAND_EN;
    }

    /// INT_APEX_STATUS1 read (clears on read, 16.55, p97); a running self-test counts down.
    fn apex_status1(&mut self) -> u8 {
        if let Some(left) = self.selftest_left {
            let left = if self.selftest_never {
                left
            } else {
                left.saturating_sub(1)
            };
            if left == 0 {
                self.selftest_left = None;
                self.ireg
                    .insert(reg::IMEM_SRAM_REG_68, self.selftest_result);
                self.regs[reg::INT_APEX_STATUS1 as usize] |= reg::INT_STATUS_SELFTEST_DONE;
            } else {
                self.selftest_left = Some(left);
            }
        }
        core::mem::take(&mut self.regs[reg::INT_APEX_STATUS1 as usize])
    }

    /// Frame size from FIFO_CONFIG3 (16.32, p82): high resolution 20, accel and gyro 16, else 8.
    pub fn frame_bytes(&self) -> usize {
        let c3 = self.regs[reg::FIFO_CONFIG3 as usize];
        if c3 & reg::FIFO_HIRES_EN != 0 {
            20
        } else if c3 & (reg::FIFO_ACCEL_EN | reg::FIFO_GYRO_EN)
            == reg::FIFO_ACCEL_EN | reg::FIFO_GYRO_EN
        {
            16
        } else {
            8
        }
    }

    /// Frames the FIFO holds (16.28, p80: 2 KB, or 8 KB for depth code 0x1F).
    pub fn fifo_capacity(&self) -> usize {
        let depth = if self.regs[reg::FIFO_CONFIG0 as usize] & 0x3F == reg::FIFO_DEPTH_8K {
            8192
        } else {
            2048
        };
        depth / self.frame_bytes()
    }

    pub fn fifo_frames(&self) -> usize {
        self.fifo.len() / self.frame_bytes()
    }

    /// Queues one frame. Like the device (measured on the board, 2026-11-10), the count saturates
    /// two frames above the nominal capacity: a FIFO holding that many drops its oldest frame
    /// (stream mode). FIFO_FULL is set in INT1_STATUS0 when the queue reaches the nominal capacity.
    pub fn push_frame(&mut self, frame: &[u8]) {
        assert_eq!(frame.len(), self.frame_bytes(), "frame size");
        if self.fifo_frames() >= self.fifo_capacity() + 2 {
            self.fifo.drain(..frame.len());
        }
        self.fifo.extend(frame.iter().copied());
        if self.fifo_frames() >= self.fifo_capacity() {
            self.regs[reg::INT1_STATUS0 as usize] |= reg::FIFO_FULL;
        }
    }

    fn ireg_addr(&self) -> u16 {
        u16::from_be_bytes([
            self.regs[reg::IREG_ADDR_15_8 as usize],
            self.regs[reg::IREG_ADDR_7_0 as usize],
        ])
    }

    fn read_byte(&mut self, a: u8) -> u8 {
        match a {
            reg::IREG_DATA => self.ireg.get(&self.ireg_addr()).copied().unwrap_or(0),
            reg::INT1_STATUS0 | reg::INT1_STATUS1 => core::mem::take(&mut self.regs[a as usize]),
            reg::FIFO_COUNT_0 => {
                self.count_latch = self
                    .count_script
                    .pop_front()
                    .unwrap_or(self.fifo_frames() as u16);
                self.count_latch.to_le_bytes()[0]
            }
            reg::FIFO_COUNT_1 => self.count_latch.to_le_bytes()[1],
            reg::FIFO_DATA => self.fifo.pop_front().unwrap_or(0x7F),
            reg::INT_APEX_STATUS1 => self.apex_status1(),
            _ => self.regs[a as usize],
        }
    }

    fn write_byte(&mut self, a: u8, byte: u8) {
        match a {
            reg::IREG_DATA => {
                let at = self.ireg_addr();
                self.ireg.insert(at, byte);
            }
            reg::REG_MISC2 if byte & reg::SOFT_RST != 0 => {
                self.reset_state();
                if !self.reset_stuck {
                    self.regs[reg::INT1_STATUS0 as usize] = reg::RESET_DONE;
                }
            }
            reg::REG_HOST_MSG => self.host_msg(byte),
            reg::FIFO_CONFIG2 if byte & reg::FIFO_FLUSH != 0 => {
                self.fifo.clear();
                self.regs[a as usize] = if self.flush_stuck {
                    byte
                } else {
                    byte & !reg::FIFO_FLUSH
                };
            }
            _ => self.regs[a as usize] = byte,
        }
    }

    /// The chip's framing: bit 7 of the first byte is read, the rest is the address, and the
    /// address increments after every data byte.
    fn clock(&mut self, header: &mut Option<u8>, addr: &mut u8, byte: u8) -> u8 {
        let Some(h) = *header else {
            *header = Some(byte);
            *addr = byte & 0x7F;
            return 0;
        };
        let a = *addr;
        // The FIFO data port does not advance the register address.
        if a != reg::FIFO_DATA {
            *addr = addr.wrapping_add(1);
        }
        if h & READ_BIT != 0 {
            self.read_byte(a)
        } else {
            self.write_byte(a, byte);
            0
        }
    }

    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), BusFault> {
        self.transactions += 1;
        if self.fail_at == Some(self.transactions) {
            return Err(BusFault);
        }
        let (mut header, mut addr) = (None, 0u8);
        let mut ops = Vec::new();
        for op in operations {
            match op {
                Operation::Write(w) => {
                    for &b in w.iter() {
                        self.clock(&mut header, &mut addr, b);
                    }
                    ops.push(Op::Write(w.to_vec()));
                }
                Operation::Read(r) => {
                    if !matches!(header, Some(h) if h & READ_BIT != 0) {
                        self.violations += 1;
                    }
                    for b in r.iter_mut() {
                        *b = self.clock(&mut header, &mut addr, 0);
                    }
                    if self.glitch_at == Some(self.transactions) {
                        self.glitch_at = None;
                        if let Some(first) = r.first_mut() {
                            *first = !*first;
                        }
                    }
                    ops.push(Op::Read(r.to_vec()));
                }
                Operation::TransferInPlace(t) => {
                    self.violations += 1;
                    let sent = t.to_vec();
                    for b in t.iter_mut() {
                        *b = self.clock(&mut header, &mut addr, *b);
                    }
                    ops.push(Op::InPlace(sent, t.to_vec()));
                }
                Operation::Transfer(r, w) => {
                    self.violations += 1;
                    for (o, i) in r.iter_mut().zip(w.iter()) {
                        *o = self.clock(&mut header, &mut addr, *i);
                    }
                }
                Operation::DelayNs(_) => self.violations += 1,
            }
        }
        self.log.push(Event::Spi(ops));
        Ok(())
    }
}

/// The shared state; clone it to keep a handle after the bus has moved into the driver.
#[derive(Clone)]
pub(crate) struct Shared(Arc<Mutex<Chip>>);

impl Shared {
    pub fn new() -> Self {
        Self(Arc::new(Mutex::new(Chip::new())))
    }

    pub fn chip(&self) -> MutexGuard<'_, Chip> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn bus(&self) -> TestBus {
        TestBus(self.clone())
    }

    pub fn delay(&self) -> TestDelay {
        TestDelay(self.clone())
    }

    /// Clears the log and the counters and sets the injections; the register file stays.
    pub fn arm(&self, fail_at: Option<usize>, pend_at: Option<u32>) {
        let mut c = self.chip();
        c.log.clear();
        c.transactions = 0;
        c.awaits = 0;
        c.fail_at = fail_at;
        c.pend_at = pend_at;
        c.pended = false;
        c.violations = 0;
        c.glitch_at = None;
    }

    pub fn log(&self) -> Vec<Event> {
        self.chip().log.clone()
    }

    pub fn awaits(&self) -> u32 {
        self.chip().awaits
    }

    pub fn transactions(&self) -> usize {
        self.chip().transactions
    }

    pub fn violations(&self) -> usize {
        self.chip().violations
    }
}

/// An await point: counts itself, and pends once if it is the chosen one.
struct Tick {
    shared: Shared,
    counted: bool,
}

impl Future for Tick {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if self.counted {
            return Poll::Ready(());
        }
        self.counted = true;
        let mut c = self.shared.chip();
        c.awaits += 1;
        if c.pend_at == Some(c.awaits) && !c.pended {
            c.pended = true;
            return Poll::Pending;
        }
        Poll::Ready(())
    }
}

fn tick(shared: &Shared) -> Tick {
    Tick {
        shared: shared.clone(),
        counted: false,
    }
}

/// The fake SPI device. Both the async and the blocking trait are implemented over one state.
pub(crate) struct TestBus(Shared);

impl ErrorType for TestBus {
    type Error = BusFault;
}

impl embedded_hal_async::spi::SpiDevice for TestBus {
    async fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), BusFault> {
        tick(&self.0).await;
        self.0.chip().transaction(operations)
    }
}

impl embedded_hal::spi::SpiDevice for TestBus {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), BusFault> {
        self.0.chip().transaction(operations)
    }
}

/// The fake delay; it records and does not wait.
pub(crate) struct TestDelay(Shared);

impl embedded_hal_async::delay::DelayNs for TestDelay {
    async fn delay_ns(&mut self, ns: u32) {
        tick(&self.0).await;
        self.0.chip().log.push(Event::Delay(u64::from(ns)));
    }
}

impl embedded_hal::delay::DelayNs for TestDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.0.chip().log.push(Event::Delay(u64::from(ns)));
    }
}

/// Runs a future to completion with a no-op waker, polling again after every `Pending`.
pub(crate) fn run<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(v) = future.as_mut().poll(&mut cx) {
            return v;
        }
    }
}

/// How a future that was polled until its first `Pending` ended.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Polled<T> {
    /// It completed without ever pending: the chosen await is beyond its last.
    Done(T),
    /// It returned `Pending` and was dropped there.
    Cancelled,
}

/// Polls until the first `Pending`, then drops the future.
pub(crate) fn poll_then_drop<F: Future>(future: F) -> Polled<F::Output> {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    match future.as_mut().poll(&mut cx) {
        Poll::Ready(v) => Polled::Done(v),
        Poll::Pending => Polled::Cancelled,
    }
}
