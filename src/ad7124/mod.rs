//! AD7124 Driver Core
//!
//! This module contains the complete implementation for the AD7124 family
//! with clear separation between core logic and transport layers.
#[cfg(feature = "async")]
pub mod r#async;

pub mod core;
pub mod sync; // Always include async module

// Re-export core types
pub use core::{
    AD7124Config, AD7124Core, ChannelConfig, CommandSequence, FilterConfig, SetupConfig,
};

#[cfg(feature = "async")]
pub use r#async::AD7124Async;

pub use sync::AD7124Sync;
