//! C-compatible type definitions for AD7124 FFI
//!
//! This module defines C-compatible enums and structures that match
//! the definitions in the C header file.

/// C-compatible error codes
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124Error {
    /// Operation completed successfully
    Ok = 0,
    /// Null pointer was passed as argument
    NullPointer = -1,
    /// SPI write operation failed
    SpiWrite = -2,
    /// SPI read operation failed
    SpiRead = -3,
    /// SPI transfer operation failed
    SpiTransfer = -4,
    /// Invalid channel number specified
    InvalidChannel = -5,
    /// Invalid parameter value provided
    InvalidParameter = -6,
    /// Driver not initialized before use
    NotInitialized = -7,
    /// Device not responding to commands
    DeviceNotResponding = -8,
    /// Calibration procedure failed
    CalibrationFailed = -9,
    /// Conversion operation timed out
    ConversionTimeout = -10,
    /// Data length is invalid for operation
    InvalidDataLength = -11,
    /// Device ID does not match expected value
    InvalidDeviceId = -12,
    /// Operation timed out
    Timeout = -13,
    /// Invalid configuration specified
    InvalidConfiguration = -14,
}

/// C-compatible device types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124DeviceType {
    /// AD7124-4: 4-channel, 24-bit ADC
    AD7124_4 = 0,
    /// AD7124-8: 8-channel, 24-bit ADC
    AD7124_8 = 1,
    /// Unknown or unsupported device
    Unknown = 255,
}

/// C-compatible gain values
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124Gain {
    /// Gain of 1x
    Gain1 = 0,
    /// Gain of 2x
    Gain2 = 1,
    /// Gain of 4x
    Gain4 = 2,
    /// Gain of 8x
    Gain8 = 3,
    /// Gain of 16x
    Gain16 = 4,
    /// Gain of 32x
    Gain32 = 5,
    /// Gain of 64x
    Gain64 = 6,
    /// Gain of 128x
    Gain128 = 7,
}

/// C-compatible channel input values
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124ChannelInput {
    /// Analog input 0
    Ain0 = 0,
    /// Analog input 1
    Ain1 = 1,
    /// Analog input 2
    Ain2 = 2,
    /// Analog input 3
    Ain3 = 3,
    /// Analog input 4
    Ain4 = 4,
    /// Analog input 5
    Ain5 = 5,
    /// Analog input 6
    Ain6 = 6,
    /// Analog input 7
    Ain7 = 7,
    /// Internal temperature sensor
    TempSensor = 16,
    /// Internal reference
    IntRef = 17,
    /// Digital ground
    Dgnd = 18,
    /// (AVDD - AVSS) / 5
    AvddAvssDiv5 = 19,
}

/// C-compatible operating modes
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124OperatingMode {
    /// Continuous conversion mode
    Continuous = 0,
    /// Single conversion mode
    SingleConv = 1,
    /// Standby mode
    Standby = 2,
    /// Power down mode
    PowerDown = 3,
    /// Idle mode
    Idle = 4,
    /// Internal zero-scale calibration
    InternalZeroScale = 5,
    /// Internal full-scale calibration
    InternalFullScale = 6,
    /// System zero-scale calibration
    SystemZeroScale = 7,
    /// System full-scale calibration
    SystemFullScale = 8,
}

/// C-compatible power modes
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124PowerMode {
    /// Low power consumption mode
    LowPower = 0,
    /// Medium power consumption mode
    MidPower = 1,
    /// Full power mode (best performance)
    FullPower = 2,
}

/// C-compatible reference sources
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124ReferenceSource {
    /// External reference voltage
    External = 0,
    /// Internal reference voltage
    Internal = 1,
    /// AVDD - AVSS as reference
    AvddAvss = 2,
}

/// C-compatible burnout current sources
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124BurnoutCurrent {
    /// No burnout current
    Off = 0,
    /// 0.5 µA burnout current
    Current0_5uA = 1,
    /// 2 µA burnout current
    Current2uA = 2,
    /// 4 µA burnout current
    Current4uA = 3,
}

/// C-compatible filter types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CAd7124FilterType {
    /// SINC4 filter
    Sinc4 = 0,
    /// SINC3 filter
    Sinc3 = 5,
    /// Fast settling filter
    FastSettle = 4,
}

/// SPI write function pointer type
pub type CSpiWriteFn = extern "C" fn(data: *const u8, len: usize) -> i32;
/// SPI read function pointer type
pub type CSpiReadFn = extern "C" fn(data: *mut u8, len: usize) -> i32;
/// SPI transfer function pointer type
pub type CSpiTransferFn =
    extern "C" fn(read_data: *mut u8, write_data: *const u8, len: usize) -> i32;
/// Delay function pointer type
pub type CDelayMsFn = extern "C" fn(ms: u32) -> i32;

/// C-compatible SPI interface structure
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CAd7124SpiInterface {
    /// SPI write function pointer
    pub write: CSpiWriteFn,
    /// SPI read function pointer
    pub read: CSpiReadFn,
    /// SPI transfer function pointer
    pub transfer: CSpiTransferFn,
    /// Delay function pointer
    pub delay_ms: CDelayMsFn,
}

/// C-compatible AD7124 configuration structure
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CAd7124Config {
    /// ADC operating mode
    pub operating_mode: CAd7124OperatingMode,
    /// Device power consumption mode
    pub power_mode: CAd7124PowerMode,
    /// Reference voltage source
    pub reference_source: CAd7124ReferenceSource,
    /// Whether internal reference is enabled
    pub internal_ref_enabled: bool,
    /// Whether data ready output is enabled
    pub data_ready_output_enabled: bool,
}

/// C-compatible channel configuration structure
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CAd7124ChannelConfig {
    /// Whether this channel is enabled
    pub enabled: bool,
    /// Positive input selection
    pub positive_input: CAd7124ChannelInput,
    /// Negative input selection
    pub negative_input: CAd7124ChannelInput,
    /// Setup configuration index to use
    pub setup_index: u8,
}

/// C-compatible setup configuration structure
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CAd7124SetupConfig {
    /// Programmable gain amplifier setting
    pub pga_gain: CAd7124Gain,
    /// Reference voltage source
    pub reference_source: CAd7124ReferenceSource,
    /// Whether to use bipolar mode
    pub bipolar: bool,
    /// Whether reference buffers are enabled
    pub reference_buffers_enabled: bool,
    /// Whether input buffers are enabled
    pub input_buffers_enabled: bool,
    /// Burnout current source for sensor diagnostics
    pub burnout_current: CAd7124BurnoutCurrent,
}

/// C-compatible filter configuration structure
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CAd7124FilterConfig {
    /// Digital filter type
    pub filter_type: CAd7124FilterType,
    /// Output data rate (0-2047)
    pub output_data_rate: u16,
    /// Single cycle conversion mode
    pub single_cycle: bool,
    /// Enable 60Hz rejection
    pub reject_60hz: bool,
}

/// Opaque driver handle type
#[repr(C)]
#[derive(Debug)]
pub struct Ad7124Driver {
    /// Private data (zero-sized for opaque handle)
    _private: [u8; 0],
}
