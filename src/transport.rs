//! Transport layer abstractions for AD7124 driver
//!
//! Enhanced v3.0 with family-consistent error handling and SPI optimization

#[cfg(feature = "embedded-hal")]
use embedded_hal::digital::OutputPin;
#[cfg(feature = "embedded-hal")]
use embedded_hal::spi::{SpiBus, SpiDevice};
#[cfg(feature = "embedded-hal-async")]
use embedded_hal_async::spi::{SpiBus as AsyncSpiBus, SpiDevice as AsyncSpiDevice};

use crate::errors::{AD7124CoreError, AD7124Error};

/// SPI speed configuration for AD7124 family
pub mod spi_speeds {
    /// Maximum SPI clock frequency for AD7124 devices (Hz)
    pub const MAX_SPI_FREQ_HZ: u32 = 5_000_000; // 5 MHz
    /// Safe SPI frequency for all device variants (Hz)  
    pub const SAFE_SPI_FREQ_HZ: u32 = 1_000_000; // 1 MHz
    /// Minimum SPI frequency (Hz)
    pub const MIN_SPI_FREQ_HZ: u32 = 100_000; // 100 kHz
}

/// SPI speed validator for AD7124 devices
#[derive(Debug)]
pub struct SpiSpeedValidator;

impl SpiSpeedValidator {
    /// Validate SPI frequency against AD7124 device limits
    pub fn validate_frequency(freq_hz: u32) -> Result<(), AD7124CoreError> {
        if freq_hz > spi_speeds::MAX_SPI_FREQ_HZ {
            return Err(AD7124CoreError::InvalidParameter);
        }
        if freq_hz < spi_speeds::MIN_SPI_FREQ_HZ {
            return Err(AD7124CoreError::InvalidParameter);
        }
        Ok(())
    }
}

//=============================================================================
// Core Transport Traits (AD7124-Specific)
//=============================================================================

/// Synchronous SPI transport interface for AD7124
pub trait SyncSpiTransport {
    /// Error type returned by transport operations
    type Error;

    /// Write data to the device
    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Read data from the device
    fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error>;

    /// Transfer data (write then read)
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error>;

    /// Delay for specified milliseconds
    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error>;
}

/// Asynchronous SPI transport interface for AD7124
#[allow(async_fn_in_trait)]
pub trait AsyncSpiTransport {
    /// Error type returned by transport operations
    type Error;

    /// Write data to the device asynchronously
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Read data from the device asynchronously
    async fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error>;

    /// Transfer data asynchronously
    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error>;

    /// Delay for specified milliseconds asynchronously
    async fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error>;
}

//=============================================================================
// Embedded-HAL Implementations with AD7124-Consistent Error Types
//=============================================================================

/// SPI transport wrapper for bus + CS pin
#[cfg(feature = "embedded-hal")]
#[derive(Debug)]
pub struct SpiTransport<SPI, CS, DELAY> {
    /// SPI bus instance
    pub spi: SPI,
    /// Chip select pin
    pub cs: CS,
    /// Delay provider for timing requirements
    pub delay: DELAY,
}

#[cfg(feature = "embedded-hal")]
impl<SPI, CS, DELAY> SpiTransport<SPI, CS, DELAY> {
    /// Create new SPI transport with separate bus, CS pin, and delay provider
    pub fn new(spi: SPI, cs: CS, delay: DELAY) -> Self {
        Self { spi, cs, delay }
    }
}

/// Implementation for SPI bus with separate CS pin (synchronous)
#[cfg(feature = "embedded-hal")]
impl<SPI, CS, DELAY, E, PinE> SyncSpiTransport for SpiTransport<SPI, CS, DELAY>
where
    SPI: SpiBus<Error = E>,
    CS: OutputPin<Error = PinE>,
    DELAY: embedded_hal::delay::DelayNs,
{
    type Error = AD7124Error<E, PinE>;

    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|e| AD7124Error::Pin(e))?;
        let result = self.spi.write(data).map_err(|e| AD7124Error::Transport(e));
        self.cs.set_high().map_err(|e| AD7124Error::Pin(e))?;
        result
    }

    fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|e| AD7124Error::Pin(e))?;
        let result = self.spi.read(data).map_err(|e| AD7124Error::Transport(e));
        self.cs.set_high().map_err(|e| AD7124Error::Pin(e))?;
        result
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|e| AD7124Error::Pin(e))?;
        let result = self
            .spi
            .transfer(read, write)
            .map_err(|e| AD7124Error::Transport(e));
        self.cs.set_high().map_err(|e| AD7124Error::Pin(e))?;
        result
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.delay.delay_ms(ms);
        Ok(())
    }
}

/// Implementation for SPI bus with separate CS pin (asynchronous)
#[cfg(all(
    feature = "async",
    feature = "embedded-hal-async",
    feature = "embedded-hal"
))]
impl<SPI, CS, DELAY, E, PinE> AsyncSpiTransport for SpiTransport<SPI, CS, DELAY>
where
    SPI: AsyncSpiBus<Error = E>,
    CS: OutputPin<Error = PinE>,
    DELAY: embedded_hal_async::delay::DelayNs,
{
    type Error = AD7124Error<E, PinE>;

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|e| AD7124Error::Pin(e))?;
        let result = self
            .spi
            .write(data)
            .await
            .map_err(|e| AD7124Error::Transport(e));
        self.cs.set_high().map_err(|e| AD7124Error::Pin(e))?;
        result
    }

    async fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|e| AD7124Error::Pin(e))?;
        let result = self
            .spi
            .read(data)
            .await
            .map_err(|e| AD7124Error::Transport(e));
        self.cs.set_high().map_err(|e| AD7124Error::Pin(e))?;
        result
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|e| AD7124Error::Pin(e))?;
        let result = self
            .spi
            .transfer(read, write)
            .await
            .map_err(|e| AD7124Error::Transport(e));
        self.cs.set_high().map_err(|e| AD7124Error::Pin(e))?;
        result
    }

    async fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.delay.delay_ms(ms).await;
        Ok(())
    }
}

/// Implementation for SpiDevice (synchronous) - CS is handled automatically
#[cfg(feature = "embedded-hal")]
impl<SPI, DELAY, E> SyncSpiTransport for (SPI, DELAY)
where
    SPI: SpiDevice<Error = E>,
    DELAY: embedded_hal::delay::DelayNs,
{
    type Error = AD7124Error<E, ()>;

    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.0.write(data).map_err(|e| AD7124Error::Transport(e))
    }

    fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(data).map_err(|e| AD7124Error::Transport(e))
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.0
            .transfer(read, write)
            .map_err(|e| AD7124Error::Transport(e))
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.1.delay_ms(ms);
        Ok(())
    }
}

/// Implementation for SpiDevice (asynchronous) - CS is handled automatically
#[cfg(all(feature = "async", feature = "embedded-hal-async"))]
impl<SPI, DELAY, E> AsyncSpiTransport for (SPI, DELAY)
where
    SPI: AsyncSpiDevice<Error = E>,
    DELAY: embedded_hal_async::delay::DelayNs,
{
    type Error = AD7124Error<E, ()>;

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.0
            .write(data)
            .await
            .map_err(|e| AD7124Error::Transport(e))
    }

    async fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        self.0
            .read(data)
            .await
            .map_err(|e| AD7124Error::Transport(e))
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.0
            .transfer(read, write)
            .await
            .map_err(|e| AD7124Error::Transport(e))
    }

    async fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.1.delay_ms(ms).await;
        Ok(())
    }
}

//=============================================================================
// Utility Functions and Constants
//=============================================================================

/// Default timeout for AD7124 operations (milliseconds)
pub const DEFAULT_TIMEOUT_MS: u32 = 100;

/// Maximum number of retries for communication
pub const MAX_RETRIES: u32 = 3;
