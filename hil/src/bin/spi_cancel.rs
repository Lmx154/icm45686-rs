//! DO NOT RUN THIS PROGRAM. On 2026-11-10 its shared-bus sweep at 1 MHz wrote stray registers into
//! the ICM-45686, which then stopped answering until the board was power-cycled (this board cannot
//! switch the sensor supply from firmware). It is kept only as the record of the measurement behind
//! the rule "never cancel an SPI transfer in flight", and is to be removed before 1.0.
//!
//! Contract rule A4 on the MARV V2 Rev 1 board: cancel safety of SPI register reads on the real
//! embassy-rp stack (U21 ICM-45686 on SPI0, chip select on GPIO20), and the cost of the read shapes.
//!
//! For each bus wrapper (embassy-embedded-hal shared-bus `SpiDevice`, embedded-hal-bus
//! `ExclusiveDevice`) and each read shape (one equal-length `transfer_in_place`, or
//! `transaction([Write(addr), Read(buf)])`), a 15-byte register read (the length of the driver's
//! sample read) is raced against a timer stepped in 1 us steps across the transfer, at 8 MHz and at
//! 1 MHz (where the transfer is long against the poll latency of the executor). After every
//! cancel, WHO_AM_I and a block of static configuration registers are read back with the same
//! wrapper and shape and compared with a reference taken after a soft reset; then the register file
//! is checked for changes, and the bus and device are put back to a known state. A sweep stops at
//! the first register change; the test ends if a soft reset no longer restores the register file.
//! Recovery actions are tried the same way. The cost section times 10,000 uncancelled reads per shape with the DWT
//! cycle counter, at 8 MHz and at the highest SPI clock within the datasheet limit that reads
//! correctly.
//!
//! Verdict: the recommended combination must show no wrong read and no register change at either
//! clock, after at least `MIN_CANCELS` cancels that landed mid-transfer. The other combinations are
//! reported only.
#![no_std]
#![no_main]

use cortex_m::peripheral::DWT;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice as SharedDevice;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::{clk_peri_freq, clk_sys_freq};
use embassy_rp::dma;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, SPI0};
use embassy_rp::spi::{self, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer, with_timeout};
use embedded_hal_async::spi::{Operation, SpiBus, SpiDevice};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use icm45686::{Icm45686, reg};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const SPI_HZ: u32 = 8_000_000;
/// Second sweep clock: at 1 MHz the 15-byte transfer (120 us) is long against the executor's poll
/// latency, so the timer lands everywhere in the transfer, and bytes still being clocked out after
/// a cancel overlap the next transaction.
const SLOW_HZ: u32 = 1_000_000;
const SWEEP_CLOCKS: [u32; 2] = [SPI_HZ, SLOW_HZ];
/// ICM-45686 datasheet maximum SPI clock.
const DATASHEET_MAX_HZ: u32 = 24_000_000;
const CS_PIN: u32 = 20;
const READ: u8 = 0x80;

/// Static configuration registers 0x26..=0x33: unchanged by soft reset and `init`, and varied
/// enough that a shifted read is recognisable. 14 data bytes, 15 on the wire, like `Icm45686::read`.
const BLOCK_ADDR: u8 = 0x26;
const BLOCK_LEN: usize = 14;
/// Register file compared after every cancel, as (first register, length). Skips FIFO_DATA (0x13),
/// the clear-on-read INT1_STATUS0/1 (0x19, 0x1A) and 0x71, IREG_DATA (0x7E) and REG_MISC2 (0x7F).
/// With the sensors off after a soft reset, everything else here is static.
const STATE_RANGES: [(u8, usize); 4] = [(0x00, 19), (0x14, 5), (0x1B, 86), (0x72, 12)];
const STATE_LEN: usize = 122;

/// Cancels that must land mid-transfer per combination and sweep clock.
const MIN_CANCELS: u32 = 1000;
const MAX_ROUNDS: u32 = 40;
/// Read pairs (WHO_AM_I, static block) after each cancel.
const CHECK_PAIRS: u32 = 10;
const CHECKS: u32 = 2 * CHECK_PAIRS;
/// The sweep runs this far past the longest uncancelled read.
const MARGIN_US: u64 = 5;
const RECOVERY_ROUNDS: u32 = 3;
const COST_READS: usize = 10_000;
const CLOCK_READS: u32 = 1000;
const MAX_SHIFT: usize = 8;

type Bus<'d> = Spi<'d, SPI0, spi::Async>;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Shape {
    /// One `transfer_in_place` of address plus data, as `Icm45686::read` does today.
    TransferInPlace,
    /// `transaction([Write(addr), Read(data)])`, as `Icm45686::read_registers` does.
    WriteRead,
}

impl Shape {
    fn name(self) -> &'static str {
        match self {
            Shape::TransferInPlace => "transfer_in_place",
            Shape::WriteRead => "write_read",
        }
    }
}

const SHAPES: [Shape; 2] = [Shape::WriteRead, Shape::TransferInPlace];

const SHARED_NAME: &str = "SharedSpiDevice";

/// The combination the driver and the on-board tests should use; the verdict is taken on it.
const RECOMMENDED_WRAPPER: &str = SHARED_NAME;
const RECOMMENDED_SHAPE: Shape = Shape::WriteRead;

#[derive(Clone, Copy)]
enum Recovery {
    /// `SpiBus::flush` on the bus.
    Flush,
    /// An empty transaction: chip select low, bus flush, chip select high.
    CsToggle,
    /// A transaction with one `Write`: the address byte of WHO_AM_I and nothing else.
    DummyWrite,
    /// Wait until SPI0 is idle, then read its receive FIFO empty (registers, below the HAL).
    Drain,
}

impl Recovery {
    fn name(self) -> &'static str {
        match self {
            Recovery::Flush => "flush",
            Recovery::CsToggle => "cs_toggle",
            Recovery::DummyWrite => "dummy_write",
            Recovery::Drain => "drain_rx_fifo",
        }
    }
}

const RECOVERIES: [Recovery; 4] = [
    Recovery::Flush,
    Recovery::CsToggle,
    Recovery::DummyWrite,
    Recovery::Drain,
];

/// A bus wrapper under test, with access to the bus beneath it.
trait Wrapper {
    const NAME: &'static str;
    type Dev: SpiDevice;
    fn dev(&mut self) -> &mut Self::Dev;
    async fn flush(&mut self);
    async fn set_frequency(&mut self, hz: u32);
}

struct Exclusive<'d>(ExclusiveDevice<Bus<'d>, Output<'d>, NoDelay>);

impl<'d> Wrapper for Exclusive<'d> {
    const NAME: &'static str = "ExclusiveDevice";
    type Dev = ExclusiveDevice<Bus<'d>, Output<'d>, NoDelay>;

    fn dev(&mut self) -> &mut Self::Dev {
        &mut self.0
    }

    async fn flush(&mut self) {
        let _ = SpiBus::flush(self.0.bus_mut()).await;
    }

    async fn set_frequency(&mut self, hz: u32) {
        self.0.bus_mut().set_frequency(hz);
    }
}

struct Shared<'a, 'd> {
    bus: &'a Mutex<NoopRawMutex, Bus<'d>>,
    dev: SharedDevice<'a, NoopRawMutex, Bus<'d>, Output<'d>>,
}

impl<'a, 'd> Wrapper for Shared<'a, 'd> {
    const NAME: &'static str = SHARED_NAME;
    type Dev = SharedDevice<'a, NoopRawMutex, Bus<'d>, Output<'d>>;

    fn dev(&mut self) -> &mut Self::Dev {
        &mut self.dev
    }

    async fn flush(&mut self) {
        let _ = SpiBus::flush(&mut *self.bus.lock().await).await;
    }

    async fn set_frequency(&mut self, hz: u32) {
        self.bus.lock().await.set_frequency(hz);
    }
}

struct Reference {
    who: [u8; 1],
    block: [u8; BLOCK_LEN],
    state: [u8; STATE_LEN],
}

/// Reads `out.len()` registers from `addr` (at most `BLOCK_LEN`) in the given shape.
async fn read_regs<D: SpiDevice>(
    dev: &mut D,
    shape: Shape,
    addr: u8,
    out: &mut [u8],
) -> Result<(), D::Error> {
    match shape {
        Shape::TransferInPlace => {
            let mut wire = [0u8; BLOCK_LEN + 1];
            let wire = &mut wire[..=out.len()];
            wire[0] = addr | READ;
            dev.transfer_in_place(wire).await?;
            out.copy_from_slice(&wire[1..]);
            Ok(())
        }
        Shape::WriteRead => {
            dev.transaction(&mut [Operation::Write(&[addr | READ]), Operation::Read(out)])
                .await
        }
    }
}

/// Reads the register file of `STATE_RANGES` with the driver's `read_registers`.
async fn read_state<D: SpiDevice>(dev: &mut D, out: &mut [u8; STATE_LEN]) -> bool {
    let mut imu = Icm45686::new(dev);
    let mut at = 0;
    for (addr, len) in STATE_RANGES {
        if imu
            .read_registers(addr, &mut out[at..at + len])
            .await
            .is_err()
        {
            return false;
        }
        at += len;
    }
    true
}

/// The first register that differs, as (register, expected, found).
fn first_difference(found: &[u8; STATE_LEN], want: &[u8; STATE_LEN]) -> (u8, u8, u8) {
    let mut at = 0;
    for (addr, len) in STATE_RANGES {
        for i in 0..len {
            if found[at + i] != want[at + i] {
                return (addr + i as u8, want[at + i], found[at + i]);
            }
        }
        at += len;
    }
    (0, 0, 0)
}

/// Soft reset through the driver's register access; true once RESET_DONE is seen.
async fn soft_reset<D: SpiDevice>(dev: &mut D) -> bool {
    let mut imu = Icm45686::new(dev);
    if imu
        .write_register(reg::REG_MISC2, reg::SOFT_RST)
        .await
        .is_err()
    {
        return false;
    }
    Timer::after_millis(1).await;
    for _ in 0..100 {
        if let Ok(status) = imu.read_register(reg::INT1_STATUS0).await
            && status != 0xFF
            && status & reg::RESET_DONE != 0
        {
            return true;
        }
        Timer::after_micros(500).await;
    }
    false
}

/// Waits until SPI0 is idle and reads its receive FIFO empty; returns the bytes found and whether
/// the FIFO had overrun. Clears the overrun flag.
fn drain_rx() -> (u32, bool) {
    let p = pac::SPI0;
    while p.sr().read().bsy() {}
    let mut n = 0;
    while p.sr().read().rne() {
        let _ = p.dr().read();
        n += 1;
    }
    let overrun = p.ris().read().rorris();
    p.icr().write(|w| w.set_roric(true));
    (n, overrun)
}

/// The SPI0 clock actually set, from the prescaler registers.
fn spi_hz() -> u32 {
    let p = pac::SPI0;
    let cpsdvsr = u32::from(p.cpsr().read().cpsdvsr());
    let scr = u32::from(p.cr0().read().scr());
    clk_peri_freq() / (cpsdvsr * (scr + 1))
}

fn cycles_to_us(cycles: u32) -> f32 {
    cycles as f32 / (clk_sys_freq() as f32 / 1_000_000.0)
}

/// `found` equal to `want` delayed by `k` bytes (positive) or advanced by `k` bytes (negative).
fn shift_of(found: &[u8], want: &[u8]) -> Option<i8> {
    let n = found.len();
    (1..=MAX_SHIFT.min(n - 1)).find_map(|k| {
        if found[k..] == want[..n - k] {
            Some(k as i8)
        } else if found[..n - k] == want[k..] {
            Some(-(k as i8))
        } else {
            None
        }
    })
}

/// State right after a cancelled read.
struct Cancel {
    /// SPI0 still shifting, or bytes left in its FIFOs.
    mid_transfer: bool,
    cs_high: bool,
    cycles: u32,
}

/// Races one static-block read against a timer of `d_us`; `None` if the read completed first.
async fn race<W: Wrapper>(w: &mut W, shape: Shape, d_us: u64) -> Option<Cancel> {
    let mut buf = [0u8; BLOCK_LEN];
    let t0 = DWT::cycle_count();
    let done = with_timeout(
        Duration::from_micros(d_us),
        read_regs(w.dev(), shape, BLOCK_ADDR, &mut buf),
    )
    .await;
    // A cancelled read future has been dropped by now.
    let sr = pac::SPI0.sr().read();
    let cs_high = pac::SIO.gpio_in(0).read() & (1 << CS_PIN) != 0;
    let cycles = DWT::cycle_count().wrapping_sub(t0);
    match done {
        Ok(_) => None,
        Err(_) => Some(Cancel {
            mid_transfer: sr.bsy() || sr.rne() || !sr.tfe(),
            cs_high,
            cycles,
        }),
    }
}

#[derive(Clone, Copy)]
struct WrongRead {
    /// Check read number after the cancel, from 0; even numbers are WHO_AM_I, odd the block.
    index: u32,
    found: [u8; BLOCK_LEN],
    shift: Option<i8>,
}

#[derive(Default)]
struct Checked {
    wrong_reads: u32,
    wrong_bytes: u32,
    first_index: Option<u32>,
    first_block: Option<WrongRead>,
    last_wrong: bool,
}

/// `CHECK_PAIRS` reads of WHO_AM_I and the static block in the given shape, against the reference.
async fn check<D: SpiDevice>(dev: &mut D, shape: Shape, r: &Reference) -> Checked {
    let mut c = Checked::default();
    for index in 0..CHECKS {
        let is_block = index % 2 == 1;
        let (addr, want): (u8, &[u8]) = if is_block {
            (BLOCK_ADDR, &r.block[..])
        } else {
            (reg::WHO_AM_I, &r.who[..])
        };
        let mut found = [0u8; BLOCK_LEN];
        let ok = read_regs(dev, shape, addr, &mut found[..want.len()])
            .await
            .is_ok();
        let bad = found.iter().zip(want).filter(|(f, w)| f != w).count() as u32;
        let wrong = !ok || bad > 0;
        if wrong {
            c.wrong_reads += 1;
            c.wrong_bytes += bad;
            c.first_index.get_or_insert(index);
            if is_block && c.first_block.is_none() {
                c.first_block = Some(WrongRead {
                    index,
                    found,
                    shift: shift_of(&found, want),
                });
            }
        }
        c.last_wrong = wrong;
    }
    c
}

struct Cleaned {
    rx_left: u32,
    changed: Option<(u8, u8, u8)>,
}

/// Ends any frame left open, empties the receive FIFO, then compares the register file with the
/// reference and soft-resets the device if it changed. Ends the test if the soft reset does not
/// restore it: the device then needs a power cycle, and more cancels would only add damage.
async fn clean<W: Wrapper>(w: &mut W, r: &Reference) -> Cleaned {
    let _ = w.dev().transaction(&mut []).await;
    let (rx_left, _) = drain_rx();
    let mut state = [0u8; STATE_LEN];
    if read_state(w.dev(), &mut state).await && state == r.state {
        return Cleaned {
            rx_left,
            changed: None,
        };
    }
    let (addr, want, found) = first_difference(&state, &r.state);
    if !(soft_reset(w.dev()).await && read_state(w.dev(), &mut state).await && state == r.state) {
        defmt::error!(
            "[FAIL] register 0x{=u8:02x} changed {=u8:02x} -> {=u8:02x} and a soft reset did not restore the register file: power-cycle the board",
            addr,
            want,
            found
        );
        hil_common::exit(1)
    }
    Cleaned {
        rx_left,
        changed: Some((addr, want, found)),
    }
}

/// Longest of 32 uncancelled static-block reads, in whole microseconds.
async fn transfer_us<W: Wrapper>(w: &mut W, shape: Shape) -> u64 {
    let mut buf = [0u8; BLOCK_LEN];
    let mut worst = 0;
    for _ in 0..32 {
        let t0 = DWT::cycle_count();
        let _ = read_regs(w.dev(), shape, BLOCK_ADDR, &mut buf).await;
        worst = worst.max(DWT::cycle_count().wrapping_sub(t0));
    }
    u64::from(worst.div_ceil(clk_sys_freq() / 1_000_000))
}

struct Sweep {
    d_max: u64,
    rounds: u32,
    attempts: u32,
    cancels: u32,
    mid_transfer: u32,
    cancel_min: u32,
    cancel_max: u32,
    cs_low: u32,
    wrong_reads: u32,
    wrong_bytes: u32,
    with_wrong: u32,
    only_first: u32,
    wrong_at_end: u32,
    device_changed: u32,
    control_checks: u32,
    control_wrong: u32,
    /// Receive-FIFO bytes left after the check reads, 0..=8 and more.
    rx_left: [u32; 10],
    /// Shift of the first wrong block read after each cancel, -8..=8 and other.
    first_shift: [u32; 2 * MAX_SHIFT + 2],
    first_wrong: Option<(u64, WrongRead)>,
    first_change: Option<(u8, u8, u8)>,
}

impl Sweep {
    fn new(d_max: u64) -> Self {
        Self {
            d_max,
            rounds: 0,
            attempts: 0,
            cancels: 0,
            mid_transfer: 0,
            cancel_min: u32::MAX,
            cancel_max: 0,
            cs_low: 0,
            wrong_reads: 0,
            wrong_bytes: 0,
            with_wrong: 0,
            only_first: 0,
            wrong_at_end: 0,
            device_changed: 0,
            control_checks: 0,
            control_wrong: 0,
            rx_left: [0; 10],
            first_shift: [0; 2 * MAX_SHIFT + 2],
            first_wrong: None,
            first_change: None,
        }
    }
}

fn shift_bin(shift: Option<i8>) -> usize {
    match shift {
        Some(k) => (k as isize + MAX_SHIFT as isize) as usize,
        None => 2 * MAX_SHIFT + 1,
    }
}

/// Cancel sweep: d = 0..=d_max us, repeated until `MIN_CANCELS` cancels landed mid-transfer.
/// Stops at the first change of the register file.
async fn sweep<W: Wrapper>(w: &mut W, shape: Shape, r: &Reference) -> Sweep {
    clean(w, r).await;
    let mut s = Sweep::new(transfer_us(w, shape).await + MARGIN_US);
    'sweep: while s.mid_transfer < MIN_CANCELS && s.rounds < MAX_ROUNDS {
        s.rounds += 1;
        for d in 0..=s.d_max {
            s.attempts += 1;
            let cancel = race(w, shape, d).await;
            let c = check(w.dev(), shape, r).await;
            match cancel {
                None => {
                    s.control_checks += CHECKS;
                    s.control_wrong += c.wrong_reads;
                }
                Some(x) => {
                    s.cancels += 1;
                    s.mid_transfer += u32::from(x.mid_transfer);
                    s.cancel_min = s.cancel_min.min(x.cycles);
                    s.cancel_max = s.cancel_max.max(x.cycles);
                    s.cs_low += u32::from(!x.cs_high);
                    s.wrong_reads += c.wrong_reads;
                    s.wrong_bytes += c.wrong_bytes;
                    s.with_wrong += u32::from(c.wrong_reads > 0);
                    s.only_first += u32::from(c.wrong_reads == 1 && c.first_index == Some(0));
                    s.wrong_at_end += u32::from(c.last_wrong);
                    if let Some(f) = c.first_block {
                        s.first_shift[shift_bin(f.shift)] += 1;
                        s.first_wrong.get_or_insert((d, f));
                    }
                }
            }
            let cl = clean(w, r).await;
            s.rx_left[(cl.rx_left as usize).min(9)] += 1;
            if let Some(change) = cl.changed {
                s.device_changed += 1;
                s.first_change = Some(change);
                break 'sweep;
            }
        }
    }
    s
}

struct RecoveryResult {
    cancels: u32,
    mid_transfer: u32,
    wrong_reads: u32,
    device_changed: u32,
    /// For `Drain`: receive-FIFO bytes found right after the cancel, 0..=8 and more.
    rx_after_cancel: [u32; 10],
    overruns: u32,
}

/// Cancels as in the sweep, applies `action`, then checks the reads. Stops at the first change of
/// the register file.
async fn recovery<W: Wrapper>(
    w: &mut W,
    shape: Shape,
    r: &Reference,
    d_max: u64,
    action: Recovery,
) -> RecoveryResult {
    let mut out = RecoveryResult {
        cancels: 0,
        mid_transfer: 0,
        wrong_reads: 0,
        device_changed: 0,
        rx_after_cancel: [0; 10],
        overruns: 0,
    };
    'trials: for _ in 0..RECOVERY_ROUNDS {
        for d in 0..=d_max {
            let Some(x) = race(w, shape, d).await else {
                continue;
            };
            out.cancels += 1;
            out.mid_transfer += u32::from(x.mid_transfer);
            match action {
                Recovery::Flush => w.flush().await,
                Recovery::CsToggle => {
                    let _ = w.dev().transaction(&mut []).await;
                }
                Recovery::DummyWrite => {
                    let _ = w.dev().write(&[reg::WHO_AM_I | READ]).await;
                }
                Recovery::Drain => {
                    let (n, overrun) = drain_rx();
                    out.rx_after_cancel[(n as usize).min(9)] += 1;
                    out.overruns += u32::from(overrun);
                }
            }
            out.wrong_reads += check(w.dev(), shape, r).await.wrong_reads;
            if clean(w, r).await.changed.is_some() {
                out.device_changed += 1;
                break 'trials;
            }
        }
    }
    out
}

/// Times `COST_READS` uncancelled static-block reads; returns the number of wrong reads.
async fn cost<W: Wrapper>(
    w: &mut W,
    shape: Shape,
    r: &Reference,
    samples: &mut [u32; COST_READS],
) -> u32 {
    let mut buf = [0u8; BLOCK_LEN];
    let mut wrong = 0;
    for s in samples.iter_mut() {
        let t0 = DWT::cycle_count();
        let ok = read_regs(w.dev(), shape, BLOCK_ADDR, &mut buf)
            .await
            .is_ok();
        *s = DWT::cycle_count().wrapping_sub(t0);
        wrong += u32::from(!ok || buf != r.block);
    }
    samples.sort_unstable();
    let hz = spi_hz();
    let floor_us = (BLOCK_LEN + 1) as f32 * 8.0 * 1_000_000.0 / hz as f32;
    defmt::info!(
        "[STAT] cost {=str} {=str} spi_hz={} clk_sys_hz={} n={} median_us={} p99_us={} max_us={} floor_us={} wrong={}",
        W::NAME,
        shape.name(),
        hz,
        clk_sys_freq(),
        COST_READS,
        cycles_to_us(samples[COST_READS / 2]),
        cycles_to_us(samples[COST_READS * 99 / 100]),
        cycles_to_us(samples[COST_READS - 1]),
        floor_us,
        wrong
    );
    wrong
}

fn verdict(failures: &mut u32, ok: bool, name: &str) {
    if ok {
        defmt::info!("[PASS] {=str}", name);
    } else {
        *failures += 1;
        defmt::error!("[FAIL] {=str}", name);
    }
}

/// Soft reset, then WHO_AM_I, the static block and the register file; the block and WHO_AM_I must
/// read the same 16 times through both shapes.
async fn take_reference<W: Wrapper>(w: &mut W) -> Option<Reference> {
    drain_rx();
    if !soft_reset(w.dev()).await {
        return None;
    }
    let mut r = Reference {
        who: [0],
        block: [0; BLOCK_LEN],
        state: [0; STATE_LEN],
    };
    let mut again = [0u8; STATE_LEN];
    if !(read_state(w.dev(), &mut r.state).await && read_state(w.dev(), &mut again).await) {
        return None;
    }
    let dev = w.dev();
    read_regs(dev, Shape::WriteRead, reg::WHO_AM_I, &mut r.who)
        .await
        .ok()?;
    read_regs(dev, Shape::WriteRead, BLOCK_ADDR, &mut r.block)
        .await
        .ok()?;
    let mut stable = r.state == again && r.who[0] == reg::WHO_AM_I_VALUE;
    for shape in SHAPES {
        for _ in 0..16 {
            let mut who = [0u8; 1];
            let mut block = [0u8; BLOCK_LEN];
            stable &= read_regs(dev, shape, reg::WHO_AM_I, &mut who).await.is_ok()
                && read_regs(dev, shape, BLOCK_ADDR, &mut block).await.is_ok()
                && who == r.who
                && block == r.block;
        }
    }
    defmt::info!(
        "[INFO] reference after soft reset: WHO_AM_I={=u8:02x} block 0x{=u8:02x}..: {=[u8]:02x} stable={}",
        r.who[0],
        BLOCK_ADDR,
        r.block,
        stable
    );
    stable.then_some(r)
}

/// Steps the SPI clock up through clk_peri / (2 n); returns the highest clock within the datasheet
/// limit at which both shapes read WHO_AM_I and the static block correctly `CLOCK_READS` times.
async fn clock_search<W: Wrapper>(w: &mut W, r: &Reference) -> u32 {
    let clk = clk_peri_freq();
    let mut best = 0;
    for n in (2..=10).rev() {
        let hz = clk.div_ceil(2 * n);
        w.set_frequency(hz).await;
        let actual = spi_hz();
        let mut wrong = [0u32; 2];
        let mut first: Option<(Shape, u8, [u8; BLOCK_LEN])> = None;
        for (i, shape) in SHAPES.into_iter().enumerate() {
            for _ in 0..CLOCK_READS {
                for (addr, want) in [(reg::WHO_AM_I, &r.who[..]), (BLOCK_ADDR, &r.block[..])] {
                    let mut found = [0u8; BLOCK_LEN];
                    let ok = read_regs(w.dev(), shape, addr, &mut found[..want.len()])
                        .await
                        .is_ok();
                    if !ok || found[..want.len()] != *want {
                        wrong[i] += 1;
                        first.get_or_insert((shape, addr, found));
                    }
                }
            }
        }
        defmt::info!(
            "[STAT] clock spi_hz={} in_spec={} {=str}_wrong={}/{} {=str}_wrong={}/{}",
            actual,
            actual <= DATASHEET_MAX_HZ,
            SHAPES[0].name(),
            wrong[0],
            2 * CLOCK_READS,
            SHAPES[1].name(),
            wrong[1],
            2 * CLOCK_READS
        );
        if let Some((shape, addr, found)) = first {
            defmt::info!(
                "[INFO] first wrong read at spi_hz={}: {=str} register 0x{=u8:02x} found {=[u8]:02x}",
                actual,
                shape.name(),
                addr,
                found
            );
            break;
        }
        if actual <= DATASHEET_MAX_HZ {
            best = hz;
        }
    }
    w.set_frequency(SPI_HZ).await;
    if let Some((addr, want, found)) = clean(w, r).await.changed {
        defmt::info!(
            "[INFO] clock search changed register 0x{=u8:02x}: {=u8:02x} -> {=u8:02x}",
            addr,
            want,
            found
        );
    }
    best
}

fn report_sweep<W: Wrapper>(shape: Shape, s: &Sweep) {
    defmt::info!(
        "[STAT] cancel {=str} {=str} spi_hz={} d=0..={} us rounds={} attempts={} cancels={} mid_transfer={} cancel_at_us={}..{} cs_low_after_cancel={} wrong_reads={}/{} wrong_bytes={} cancels_with_wrong={} only_first_read_wrong={} last_read_wrong={} device_changed={} control_wrong={}/{}",
        W::NAME,
        shape.name(),
        spi_hz(),
        s.d_max,
        s.rounds,
        s.attempts,
        s.cancels,
        s.mid_transfer,
        cycles_to_us(s.cancel_min),
        cycles_to_us(s.cancel_max),
        s.cs_low,
        s.wrong_reads,
        s.cancels * CHECKS,
        s.wrong_bytes,
        s.with_wrong,
        s.only_first,
        s.wrong_at_end,
        s.device_changed,
        s.control_wrong,
        s.control_checks
    );
    defmt::info!(
        "[STAT] pattern {=str} {=str} spi_hz={} first_wrong_block_shift[-8..=8,other]={} rx_fifo_left_after_checks[0..=8,more]={}",
        W::NAME,
        shape.name(),
        spi_hz(),
        s.first_shift,
        s.rx_left
    );
    if let Some((d, f)) = s.first_wrong {
        defmt::info!(
            "[INFO] first wrong block read {=str} {=str} spi_hz={}: d={} us, check read {}, found {=[u8]:02x}, shift {}",
            W::NAME,
            shape.name(),
            spi_hz(),
            d,
            f.index,
            f.found,
            f.shift
        );
    }
    if let Some((addr, want, found)) = s.first_change {
        defmt::info!(
            "[INFO] first register changed by a cancel {=str} {=str} spi_hz={}: 0x{=u8:02x} {=u8:02x} -> {=u8:02x}",
            W::NAME,
            shape.name(),
            spi_hz(),
            addr,
            want,
            found
        );
    }
}

/// Sweeps at both clocks, recovery at the slow clock, and cost, for both shapes on one wrapper.
async fn run<W: Wrapper>(
    w: &mut W,
    r: &Reference,
    fast_hz: u32,
    samples: &mut [u32; COST_READS],
    failures: &mut u32,
) {
    for shape in SHAPES {
        let mut mid_transfer = 0;
        let mut clean_reads = true;
        for hz in SWEEP_CLOCKS {
            w.set_frequency(hz).await;
            let s = sweep(w, shape, r).await;
            report_sweep::<W>(shape, &s);
            mid_transfer += s.mid_transfer;
            clean_reads &= s.wrong_reads == 0 && s.device_changed == 0 && s.control_wrong == 0;
            if hz != SLOW_HZ {
                continue;
            }
            for action in RECOVERIES {
                let x = recovery(w, shape, r, s.d_max, action).await;
                defmt::info!(
                    "[STAT] recovery {=str} {=str} spi_hz={} {=str}: cancels={} mid_transfer={} wrong_reads={}/{} device_changed={}",
                    W::NAME,
                    shape.name(),
                    spi_hz(),
                    action.name(),
                    x.cancels,
                    x.mid_transfer,
                    x.wrong_reads,
                    x.cancels * CHECKS,
                    x.device_changed
                );
                if let Recovery::Drain = action {
                    defmt::info!(
                        "[STAT] rx_fifo_after_cancel {=str} {=str} spi_hz={} [0..=8,more]={} overruns={}",
                        W::NAME,
                        shape.name(),
                        spi_hz(),
                        x.rx_after_cancel,
                        x.overruns
                    );
                }
            }
        }
        w.set_frequency(SPI_HZ).await;
        if W::NAME == RECOMMENDED_WRAPPER && shape == RECOMMENDED_SHAPE {
            verdict(
                failures,
                mid_transfer >= MIN_CANCELS,
                "recommended combination: at least 1000 cancels landed mid-transfer",
            );
            verdict(
                failures,
                clean_reads,
                "recommended combination: no wrong read and no register change after any cancel",
            );
        }
    }
    for hz in [SPI_HZ, fast_hz].into_iter().filter(|&hz| hz > 0) {
        w.set_frequency(hz).await;
        clean(w, r).await;
        for shape in SHAPES {
            let wrong = cost(w, shape, r, samples).await;
            if W::NAME == RECOMMENDED_WRAPPER && shape == RECOMMENDED_SHAPE {
                verdict(
                    failures,
                    wrong == 0,
                    "recommended combination: cost reads correct",
                );
            }
        }
    }
    w.set_frequency(SPI_HZ).await;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = embassy_rp::init(Default::default());
    let mut cp = cortex_m::Peripherals::take().unwrap();
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    let mut failures = 0u32;
    let mut samples = [0u32; COST_READS];
    let mut spi_config = spi::Config::default();
    spi_config.frequency = SPI_HZ;
    defmt::info!(
        "[INFO] clk_sys_hz={} clk_peri_hz={} recommended={=str} {=str}",
        clk_sys_freq(),
        clk_peri_freq(),
        RECOMMENDED_WRAPPER,
        RECOMMENDED_SHAPE.name()
    );

    let (reference, fast_hz) = {
        let bus = Mutex::new(Spi::new(
            p.SPI0.reborrow(),
            p.PIN_18.reborrow(),
            p.PIN_19.reborrow(),
            p.PIN_16.reborrow(),
            p.DMA_CH0.reborrow(),
            p.DMA_CH1.reborrow(),
            Irqs,
            spi_config.clone(),
        ));
        let mut w = Shared {
            bus: &bus,
            dev: SharedDevice::new(&bus, Output::new(p.PIN_20.reborrow(), Level::High)),
        };
        let reference = take_reference(&mut w).await;
        verdict(
            &mut failures,
            reference.is_some(),
            "reference: WHO_AM_I == 0xE9, static block and register file stable through both shapes",
        );
        let Some(reference) = reference else {
            hil_common::exit(failures)
        };
        let fast_hz = clock_search(&mut w, &reference).await;
        verdict(
            &mut failures,
            fast_hz > 0,
            "both shapes read correctly at the 8 MHz setting",
        );
        run(&mut w, &reference, fast_hz, &mut samples, &mut failures).await;
        (reference, fast_hz)
    };

    {
        let bus = Spi::new(
            p.SPI0.reborrow(),
            p.PIN_18.reborrow(),
            p.PIN_19.reborrow(),
            p.PIN_16.reborrow(),
            p.DMA_CH0.reborrow(),
            p.DMA_CH1.reborrow(),
            Irqs,
            spi_config,
        );
        let cs = Output::new(p.PIN_20.reborrow(), Level::High);
        let mut w = Exclusive(ExclusiveDevice::new_no_delay(bus, cs).unwrap());
        run(&mut w, &reference, fast_hz, &mut samples, &mut failures).await;
    }

    hil_common::exit(failures)
}
