//! Blocking front-end: the async driver run to completion over an adapter whose futures never pend.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

use crate::{
    Config, Error, FifoRead, Info, InterruptStatus, Offsets, RawSample, SelfTestConfig,
    SelfTestReport, SensorPower, Stats,
};

/// Blocking bus presented as an async one.
struct Bus<SPI>(SPI);

impl<SPI: ErrorType> embedded_hal_async::spi::ErrorType for Bus<SPI> {
    type Error = SPI::Error;
}

impl<SPI: SpiDevice> embedded_hal_async::spi::SpiDevice for Bus<SPI> {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.0.transaction(operations)
    }
}

/// Blocking delay presented as an async one.
struct Wait<'a, D>(&'a mut D);

impl<D: DelayNs> embedded_hal_async::delay::DelayNs for Wait<'_, D> {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.delay_ns(ns);
    }

    async fn delay_us(&mut self, us: u32) {
        self.0.delay_us(us);
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.0.delay_ms(ms);
    }
}

fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
    }
}

/// ICM-45686 on a blocking SPI device. Same methods as [`crate::Icm45686`], without `async`.
pub struct Icm45686<SPI> {
    inner: crate::Icm45686<Bus<SPI>>,
}

impl<SPI: SpiDevice> Icm45686<SPI> {
    /// Wraps the SPI device. Nothing is sent until `init`.
    pub const fn new(spi: SPI) -> Self {
        Self {
            inner: crate::Icm45686::new(Bus(spi)),
        }
    }

    /// Returns the SPI device.
    pub fn release(self) -> SPI {
        self.inner.release().0
    }

    /// See [`crate::Icm45686::init`].
    pub fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<SPI::Error>> {
        block_on(self.inner.init(&mut Wait(delay), config))
    }

    /// See [`crate::Icm45686::configure`].
    pub fn configure(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<Info, Error<SPI::Error>> {
        block_on(self.inner.configure(&mut Wait(delay), config))
    }

    /// See [`crate::Icm45686::config`].
    pub fn config(&self) -> Option<&Config> {
        self.inner.config()
    }

    /// See [`crate::Icm45686::offsets`].
    pub fn offsets(&self) -> Option<Offsets> {
        self.inner.offsets()
    }

    /// See [`crate::Icm45686::set_offsets`].
    pub fn set_offsets(
        &mut self,
        delay: &mut impl DelayNs,
        offsets: &Offsets,
    ) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.set_offsets(&mut Wait(delay), offsets))
    }

    /// See [`crate::Icm45686::set_power`].
    pub fn set_power(
        &mut self,
        delay: &mut impl DelayNs,
        accel: SensorPower,
        gyro: SensorPower,
    ) -> Result<Info, Error<SPI::Error>> {
        block_on(self.inner.set_power(&mut Wait(delay), accel, gyro))
    }

    /// See [`crate::Icm45686::soft_reset`].
    pub fn soft_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.soft_reset(&mut Wait(delay)))
    }

    /// See [`crate::Icm45686::self_test`].
    pub fn self_test(
        &mut self,
        delay: &mut impl DelayNs,
        config: &SelfTestConfig,
    ) -> Result<SelfTestReport, Error<SPI::Error>> {
        block_on(self.inner.self_test(&mut Wait(delay), config))
    }

    /// See [`crate::Icm45686::verify`].
    pub fn verify(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.verify(&mut Wait(delay)))
    }

    /// See [`crate::Icm45686::recover`].
    pub fn recover(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.recover(&mut Wait(delay)))
    }

    /// See [`crate::Icm45686::read`].
    pub fn read(&mut self) -> Result<RawSample, Error<SPI::Error>> {
        block_on(self.inner.read())
    }

    /// See [`crate::Icm45686::fifo_read`].
    pub fn fifo_read(&mut self, buf: &mut [u8]) -> Result<FifoRead, Error<SPI::Error>> {
        block_on(self.inner.fifo_read(buf))
    }

    /// See [`crate::Icm45686::fifo_read_exact`].
    pub fn fifo_read_exact(
        &mut self,
        buf: &mut [u8],
        frames: usize,
    ) -> Result<FifoRead, Error<SPI::Error>> {
        block_on(self.inner.fifo_read_exact(buf, frames))
    }

    /// See [`crate::Icm45686::fifo_count`].
    pub fn fifo_count(&mut self) -> Result<u16, Error<SPI::Error>> {
        block_on(self.inner.fifo_count())
    }

    /// See [`crate::Icm45686::fifo_flush`].
    pub fn fifo_flush(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.fifo_flush(&mut Wait(delay)))
    }

    /// See [`crate::Icm45686::interrupt_status`].
    pub fn interrupt_status(&mut self) -> Result<InterruptStatus, Error<SPI::Error>> {
        block_on(self.inner.interrupt_status())
    }

    /// See [`crate::Icm45686::stats`].
    pub const fn stats(&self) -> &Stats {
        self.inner.stats()
    }

    /// See [`crate::Icm45686::info`].
    pub const fn info(&self) -> Option<Info> {
        self.inner.info()
    }

    /// See [`crate::Icm45686::read_register`].
    pub fn read_register(&mut self, addr: u8) -> Result<u8, Error<SPI::Error>> {
        block_on(self.inner.read_register(addr))
    }

    /// See [`crate::Icm45686::read_registers`].
    pub fn read_registers(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.read_registers(addr, buf))
    }

    /// See [`crate::Icm45686::write_register`].
    pub fn write_register(&mut self, addr: u8, value: u8) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.write_register(addr, value))
    }

    /// See [`crate::Icm45686::write_registers`].
    pub fn write_registers(&mut self, addr: u8, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.write_registers(addr, data))
    }

    /// See [`crate::Icm45686::ireg_read`].
    pub fn ireg_read(
        &mut self,
        delay: &mut impl DelayNs,
        addr: u16,
    ) -> Result<u8, Error<SPI::Error>> {
        block_on(self.inner.ireg_read(&mut Wait(delay), addr))
    }

    /// See [`crate::Icm45686::ireg_write`].
    pub fn ireg_write(
        &mut self,
        delay: &mut impl DelayNs,
        addr: u16,
        value: u8,
    ) -> Result<(), Error<SPI::Error>> {
        block_on(self.inner.ireg_write(&mut Wait(delay), addr, value))
    }
}
