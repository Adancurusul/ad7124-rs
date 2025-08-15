//! # AD7124 Driver
//!
//! A platform-independent driver for the AD7124 family (AD7124-4/AD7124-8) with comprehensive embedded-hal support.
//!
//! This v4.0 driver implements lessons learned from comprehensive architecture refactoring:
//! - **Unified Naming**: Family-consistent type naming throughout
//! - **Core-Transport Separation**: Business logic independent of transport layer
//! - **Embassy Ready**: Full async/await support with Embassy templates
//! - **Simple Construction**: Direct driver construction for both sync and async
//!
//! ## Features
//!
//! - **Multiple Device Support**: AD7124-4 (4-channel), AD7124-8 (8-channel)
//! - **Transport Layer**: SPI with embedded-hal integration
//! - **Sync & Async**: Both blocking and non-blocking APIs
//! - **24-bit Precision**: High-resolution ADC measurements
//! - **Programmable Gain**: 1x to 128x amplification
//! - **Multiple Reference Sources**: Internal 2.5V, external, or AVDD-AVSS
//! - **No-std Compatible**: Works in embedded environments
//! - **C FFI Support**: Static library generation for C integration
//!
//! ## Quick Start
//!
//! ### Synchronous Usage
//!
//! ```rust,ignore
//! use ad7124_rs::{AD7124Sync, DeviceType, SpiTransport};
//!
//! // Create SPI transport (example with embedded-hal)
//! let transport = SpiTransport::new(spi, cs, delay);
//!
//! // Create and initialize driver
//! let mut adc = AD7124Sync::new(transport, DeviceType::AD7124_8)?;
//! adc.init()?;
//!
//! // Setup single-ended measurement on channel 0
//! adc.setup_single_ended(0, ad7124_rs::ChannelInput::Ain0, 0)?;
//!
//! // Read voltage
//! let voltage = adc.read_voltage(0)?;
//! println!("Voltage: {:.3}V", voltage);
//! ```
//!
//! ### Asynchronous Usage
//!
//! ```rust,ignore
//! use ad7124_rs::{AD7124Async, DeviceType, SpiTransport};
//!
//! // Create SPI transport
//! let transport = SpiTransport::new(spi, cs, delay);
//!
//! // Create and initialize async driver
//! let mut adc = AD7124Async::new(transport, DeviceType::AD7124_8)?;
//! adc.init().await?;
//!
//! // Setup differential measurement
//! adc.setup_differential(0, ad7124_driver::ChannelInput::Ain0,
//!                       ad7124_driver::ChannelInput::Ain1, 0).await?;
//!
//! // Read voltage
//! let voltage = adc.read_voltage(0).await?;
//! ```
//!

#![no_std]
#![cfg_attr(not(feature = "capi"), deny(unsafe_code))]
#![warn(missing_docs)]
#![warn(
    missing_debug_implementations,
    rust_2018_idioms,
    trivial_casts,
    unused_qualifications
)]

// Core modules - always available
pub mod device_type;
pub mod errors;
pub mod registers;
pub mod transport;

// Driver core implementation
pub mod ad7124;

// C FFI interface (optional feature)
#[cfg(feature = "capi")]
pub mod ffi;

// Re-exports for convenience
pub use device_type::{DeviceCapabilities, DeviceFeature, DeviceType};
pub use errors::{AD7124CoreError, AD7124Error};
pub use registers::*;

// Transport re-exports
pub use transport::{spi_speeds, SpiSpeedValidator, SyncSpiTransport};

#[cfg(feature = "async")]
pub use transport::AsyncSpiTransport;

#[cfg(feature = "embedded-hal")]
pub use transport::SpiTransport;

// Driver re-exports from ad7124 module (unified API)
pub use ad7124::AD7124Sync;

#[cfg(feature = "async")]
pub use ad7124::AD7124Async;

// Configuration re-exports
pub use ad7124::{AD7124Config, ChannelConfig, FilterConfig, SetupConfig};

/// Library version information
pub mod version {
    /// Major version
    pub const MAJOR: u32 = 4;
    /// Minor version  
    pub const MINOR: u32 = 0;
    /// Patch version
    pub const PATCH: u32 = 0;
    /// Version string
    pub const VERSION: &str = concat!(
        env!("CARGO_PKG_VERSION_MAJOR"),
        ".",
        env!("CARGO_PKG_VERSION_MINOR"),
        ".",
        env!("CARGO_PKG_VERSION_PATCH")
    );
}

/// Common type aliases for convenience
pub mod types {
    use crate::AD7124Error;

    /// Standard result type for AD7124 operations
    pub type Result<T, TransportE = (), PinE = ()> =
        core::result::Result<T, AD7124Error<TransportE, PinE>>;

    /// Core error result type
    pub type CoreResult<T> = core::result::Result<T, crate::errors::AD7124CoreError>;

    /// Device capabilities type
    pub type Capabilities = crate::device_type::DeviceCapabilities;

    /// Configuration type
    pub type Config = crate::ad7124::AD7124Config;
}

/// Prelude module for common imports
pub mod prelude {
    pub use crate::ad7124::{AD7124Config, AD7124Sync, ChannelConfig, FilterConfig, SetupConfig};
    pub use crate::device_type::{DeviceCapabilities, DeviceType};
    pub use crate::errors::{AD7124CoreError, AD7124Error};
    pub use crate::registers::{
        ChannelInput, ClockSource, OperatingMode, PgaGain, PowerMode, ReferenceSource,
    };
    pub use crate::transport::SyncSpiTransport;
    pub use crate::types::{CoreResult, Result};

    #[cfg(feature = "async")]
    pub use crate::ad7124::AD7124Async;

    #[cfg(feature = "async")]
    pub use crate::transport::AsyncSpiTransport;

    #[cfg(feature = "embedded-hal")]
    pub use crate::transport::SpiTransport;
}

/// Compile-time feature validation
const _: () = {
    #[cfg(all(feature = "async", not(feature = "embedded-hal-async")))]
    compile_error!("async feature requires embedded-hal-async feature");

    #[cfg(all(
        feature = "full-async",
        not(all(feature = "async", feature = "embedded-hal-async"))
    ))]
    compile_error!("full-async feature requires both async and embedded-hal-async features");
};

// Panic handler for C FFI builds - applications should provide their own
#[cfg(all(not(test), feature = "capi"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    loop {}
}
