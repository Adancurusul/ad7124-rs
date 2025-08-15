//! C FFI (Foreign Function Interface) module for AD7124 driver
//!
//! This module provides a C-compatible interface for the AD7124 driver,
//! enabling integration with C/C++ applications while maintaining zero heap allocation
//! and no_std compatibility.

// Module structure
mod api;
mod transport;
mod types;

pub use api::*;
pub use types::*;
