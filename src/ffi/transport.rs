//! C FFI transport layer implementation for AD7124
//!
//! This module provides a transport layer that adapts C function pointers
//! to the Rust transport traits.

#![allow(dead_code)]

use super::types::{CAd7124Error, CAd7124SpiInterface};
use crate::transport::SyncSpiTransport;

/// Error type for FFI transport operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CFfiError {
    SpiWrite(i32),
    SpiRead(i32),
    SpiTransfer(i32),
    DelayMs(i32),
    NullPointer,
    InvalidParameter,
}

impl CFfiError {
    pub fn to_c_error(self) -> i32 {
        match self {
            CFfiError::SpiWrite(_) => CAd7124Error::SpiWrite as i32,
            CFfiError::SpiRead(_) => CAd7124Error::SpiRead as i32,
            CFfiError::SpiTransfer(_) => CAd7124Error::SpiTransfer as i32,
            CFfiError::DelayMs(_) => CAd7124Error::InvalidParameter as i32,
            CFfiError::NullPointer => CAd7124Error::NullPointer as i32,
            CFfiError::InvalidParameter => CAd7124Error::InvalidParameter as i32,
        }
    }
}

/// Transport implementation that bridges C function pointers to Rust traits
#[derive(Debug, Clone, Copy)]
pub struct CFfiTransport {
    interface: CAd7124SpiInterface,
}

impl CFfiTransport {
    /// Create new FFI transport from C interface
    pub fn new(interface: CAd7124SpiInterface) -> Result<Self, CFfiError> {
        // No context validation needed anymore
        Ok(Self { interface })
    }

    /// Get the underlying interface (for validation)
    pub fn interface(&self) -> &CAd7124SpiInterface {
        &self.interface
    }
}

impl SyncSpiTransport for CFfiTransport {
    type Error = CFfiError;

    fn transfer(&mut self, read_buf: &mut [u8], write_buf: &[u8]) -> Result<(), Self::Error> {
        if read_buf.len() != write_buf.len() {
            return Err(CFfiError::InvalidParameter);
        }

        let result =
            (self.interface.transfer)(read_buf.as_mut_ptr(), write_buf.as_ptr(), read_buf.len());

        if result == 0 {
            Ok(())
        } else {
            Err(CFfiError::SpiTransfer(result))
        }
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        let result = (self.interface.write)(buf.as_ptr(), buf.len());

        if result == 0 {
            Ok(())
        } else {
            Err(CFfiError::SpiWrite(result))
        }
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let result = (self.interface.read)(buf.as_mut_ptr(), buf.len());

        if result == 0 {
            Ok(())
        } else {
            Err(CFfiError::SpiRead(result))
        }
    }

    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        let result = (self.interface.delay_ms)(ms);

        if result == 0 {
            Ok(())
        } else {
            Err(CFfiError::DelayMs(result))
        }
    }
}
