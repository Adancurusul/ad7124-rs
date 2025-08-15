//! Register definitions and configuration structures for AD7124
//!
//! Provides type-safe register access and bit manipulation

/// AD7124 register addresses
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Register {
    /// Status register (read-only)
    Status = 0x00,
    /// ADC control register
    AdcCtrl = 0x01,
    /// Data register (read-only)
    Data = 0x02,
    /// IO control register 1
    IoCtrl1 = 0x03,
    /// IO control register 2
    IoCtrl2 = 0x04,
    /// Device ID register (read-only)
    Id = 0x05,
    /// Error register (read-only)
    Error = 0x06,
    /// Error enable register
    ErrorEn = 0x07,
    /// MCLK count register (read-only)
    MclkCount = 0x08,
    /// Channel 0 configuration
    Channel0 = 0x09,
    /// Channel 1 configuration
    Channel1 = 0x0A,
    /// Channel 2 configuration
    Channel2 = 0x0B,
    /// Channel 3 configuration
    Channel3 = 0x0C,
    /// Channel 4 configuration (AD7124-8 only)
    Channel4 = 0x0D,
    /// Channel 5 configuration (AD7124-8 only)
    Channel5 = 0x0E,
    /// Channel 6 configuration (AD7124-8 only)
    Channel6 = 0x0F,
    /// Channel 7 configuration (AD7124-8 only)
    Channel7 = 0x10,
    /// Configuration 0 register
    Config0 = 0x19,
    /// Configuration 1 register
    Config1 = 0x1A,
    /// Configuration 2 register
    Config2 = 0x1B,
    /// Configuration 3 register
    Config3 = 0x1C,
    /// Configuration 4 register
    Config4 = 0x1D,
    /// Configuration 5 register
    Config5 = 0x1E,
    /// Configuration 6 register
    Config6 = 0x1F,
    /// Configuration 7 register
    Config7 = 0x20,
    /// Filter 0 register
    Filter0 = 0x21,
    /// Filter 1 register
    Filter1 = 0x22,
    /// Filter 2 register
    Filter2 = 0x23,
    /// Filter 3 register
    Filter3 = 0x24,
    /// Filter 4 register
    Filter4 = 0x25,
    /// Filter 5 register
    Filter5 = 0x26,
    /// Filter 6 register
    Filter6 = 0x27,
    /// Filter 7 register
    Filter7 = 0x28,
    /// Offset 0 register
    Offset0 = 0x29,
    /// Offset 1 register
    Offset1 = 0x2A,
    /// Offset 2 register
    Offset2 = 0x2B,
    /// Offset 3 register
    Offset3 = 0x2C,
    /// Offset 4 register
    Offset4 = 0x2D,
    /// Offset 5 register
    Offset5 = 0x2E,
    /// Offset 6 register
    Offset6 = 0x2F,
    /// Offset 7 register
    Offset7 = 0x30,
    /// Gain 0 register
    Gain0 = 0x31,
    /// Gain 1 register
    Gain1 = 0x32,
    /// Gain 2 register
    Gain2 = 0x33,
    /// Gain 3 register
    Gain3 = 0x34,
    /// Gain 4 register
    Gain4 = 0x35,
    /// Gain 5 register
    Gain5 = 0x36,
    /// Gain 6 register
    Gain6 = 0x37,
    /// Gain 7 register
    Gain7 = 0x38,
}

impl Register {
    /// Get register address as u8
    pub fn addr(self) -> u8 {
        self as u8
    }

    /// Check if register is read-only
    pub fn is_readonly(self) -> bool {
        matches!(
            self,
            Register::Status
                | Register::Data
                | Register::Id
                | Register::Error
                | Register::MclkCount
        )
    }

    /// Get register size in bytes
    pub fn size(self) -> u8 {
        match self {
            Register::Status => 1,
            Register::AdcCtrl => 2,
            Register::Data => 3,
            Register::IoCtrl1 => 3,
            Register::IoCtrl2 => 2,
            Register::Id => 1,
            Register::Error => 3,
            Register::ErrorEn => 3,
            Register::MclkCount => 1,
            Register::Channel0
            | Register::Channel1
            | Register::Channel2
            | Register::Channel3
            | Register::Channel4
            | Register::Channel5
            | Register::Channel6
            | Register::Channel7 => 2,
            Register::Config0
            | Register::Config1
            | Register::Config2
            | Register::Config3
            | Register::Config4
            | Register::Config5
            | Register::Config6
            | Register::Config7 => 2,
            Register::Filter0
            | Register::Filter1
            | Register::Filter2
            | Register::Filter3
            | Register::Filter4
            | Register::Filter5
            | Register::Filter6
            | Register::Filter7 => 3,
            Register::Offset0
            | Register::Offset1
            | Register::Offset2
            | Register::Offset3
            | Register::Offset4
            | Register::Offset5
            | Register::Offset6
            | Register::Offset7 => 3,
            Register::Gain0
            | Register::Gain1
            | Register::Gain2
            | Register::Gain3
            | Register::Gain4
            | Register::Gain5
            | Register::Gain6
            | Register::Gain7 => 3,
        }
    }
}

/// Operating modes for AD7124
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OperatingMode {
    /// Continuous conversion mode
    Continuous = 0,
    /// Single conversion mode
    SingleConv = 1,
    /// Standby mode
    Standby = 2,
    /// Power-down mode
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

/// Power modes for AD7124
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    /// Low power mode
    LowPower = 0,
    /// Mid power mode
    MidPower = 1,
    /// Full power mode
    FullPower = 2,
}

/// Clock source selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ClockSource {
    /// Internal clock (default)
    Internal = 0,
    /// Internal clock with output
    InternalOutput = 1,
    /// External clock
    External = 2,
    /// External clock divided by 4
    ExternalDiv4 = 3,
}

/// Reference source selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ReferenceSource {
    /// External reference 1 (REFIN1)  
    External1 = 0,
    /// External reference 2 (REFIN2)
    External2 = 1,
    /// Internal 2.5V reference - matches C code REF_SEL=2
    Internal = 2,
    /// AVDD - AVSS
    Avdd = 3,
}

/// Programmable gain amplifier settings
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum PgaGain {
    /// Gain = 1
    Gain1 = 0,
    /// Gain = 2
    Gain2 = 1,
    /// Gain = 4
    Gain4 = 2,
    /// Gain = 8
    Gain8 = 3,
    /// Gain = 16
    Gain16 = 4,
    /// Gain = 32
    Gain32 = 5,
    /// Gain = 64
    Gain64 = 6,
    /// Gain = 128
    Gain128 = 7,
}

impl PgaGain {
    /// Create PgaGain from raw bits
    pub fn from_bits(bits: u8) -> Result<Self, crate::errors::AD7124CoreError> {
        match bits & 0x07 {
            0 => Ok(PgaGain::Gain1),
            1 => Ok(PgaGain::Gain2),
            2 => Ok(PgaGain::Gain4),
            3 => Ok(PgaGain::Gain8),
            4 => Ok(PgaGain::Gain16),
            5 => Ok(PgaGain::Gain32),
            6 => Ok(PgaGain::Gain64),
            7 => Ok(PgaGain::Gain128),
            _ => Err(crate::errors::AD7124CoreError::InvalidParameter),
        }
    }

    /// Get the numerical gain value
    pub fn value(self) -> u32 {
        match self {
            PgaGain::Gain1 => 1,
            PgaGain::Gain2 => 2,
            PgaGain::Gain4 => 4,
            PgaGain::Gain8 => 8,
            PgaGain::Gain16 => 16,
            PgaGain::Gain32 => 32,
            PgaGain::Gain64 => 64,
            PgaGain::Gain128 => 128,
        }
    }

    /// Get the numerical gain value as f32
    pub fn gain_value(self) -> f32 {
        self.value() as f32
    }
}

/// Burnout current sources
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BurnoutCurrent {
    /// No burnout current
    Off = 0,
    /// 0.5 µA burnout current
    Current0_5uA = 1,
    /// 2 µA burnout current
    Current2uA = 2,
    /// 4 µA burnout current
    Current4uA = 3,
}

/// Digital filter selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FilterType {
    /// SINC4 filter
    Sinc4 = 0,
    /// SINC3 filter
    Sinc3 = 5,
    /// Fast settling filter
    FastSettle = 4,
}

/// Channel input selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ChannelInput {
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
    /// Analog input 8
    Ain8 = 8,
    /// Analog input 9
    Ain9 = 9,
    /// Analog input 10
    Ain10 = 10,
    /// Analog input 11
    Ain11 = 11,
    /// Analog input 12
    Ain12 = 12,
    /// Analog input 13
    Ain13 = 13,
    /// Analog input 14
    Ain14 = 14,
    /// Analog input 15
    Ain15 = 15,
    /// Temperature sensor
    TempSensor = 16,
    /// Internal reference
    IntRef = 17,
    /// DGND
    Dgnd = 18,
    /// (AVDD - AVSS)/5
    AvddAvssDiv5 = 19,
}

/// Device status structure
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceStatus {
    /// Raw 8-bit status register value
    pub raw: u8,
}

impl DeviceStatus {
    /// Create status from raw register value
    pub fn new(raw: u8) -> Self {
        Self { raw }
    }

    /// Check if ADC is ready for data
    pub fn rdy(&self) -> bool {
        (self.raw & 0x80) == 0
    }

    /// Check for error condition
    pub fn error(&self) -> bool {
        (self.raw & 0x40) != 0
    }

    /// Check for power-on reset
    pub fn por_flag(&self) -> bool {
        (self.raw & 0x10) != 0
    }

    /// Get active channel
    pub fn channel(&self) -> u8 {
        self.raw & 0x0F
    }

    /// Get active channel as Option (None if no channel is active)
    pub fn active_channel(&self) -> Option<u8> {
        let ch = self.channel();
        if ch < 16 {
            Some(ch)
        } else {
            None
        }
    }

    /// Check if conversion is ready (non-blocking check)
    pub fn is_ready(&self) -> bool {
        self.rdy()
    }

    /// Check if there's an error
    pub fn has_error(&self) -> bool {
        self.error()
    }

    /// Check if specific channel is ready
    pub fn is_channel_ready(&self, channel: u8) -> bool {
        self.is_ready() && self.channel() == channel
    }
}

impl core::fmt::Display for DeviceStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Status(0x{:02X})", self.raw)?;
        if self.rdy() {
            write!(f, " RDY")?;
        }
        if self.error() {
            write!(f, " ERROR")?;
        }
        if self.por_flag() {
            write!(f, " POR")?;
        }
        write!(f, " CH{}", self.channel())?;
        Ok(())
    }
}

/// Bit field manipulation utilities
pub mod bit_fields {
    /// Extract bit field from register value
    pub fn extract_bits(value: u32, offset: u8, width: u8) -> u32 {
        let mask = (1u32 << width) - 1;
        (value >> offset) & mask
    }

    /// Insert bit field into register value
    pub fn insert_bits(value: u32, field: u32, offset: u8, width: u8) -> u32 {
        let mask = (1u32 << width) - 1;
        (value & !(mask << offset)) | ((field & mask) << offset)
    }

    /// Set single bit
    pub fn set_bit(value: u32, bit: u8) -> u32 {
        value | (1 << bit)
    }

    /// Clear single bit
    pub fn clear_bit(value: u32, bit: u8) -> u32 {
        value & !(1 << bit)
    }

    /// Check if bit is set
    pub fn is_bit_set(value: u32, bit: u8) -> bool {
        (value & (1 << bit)) != 0
    }
}
