//! Device type detection and capability management for AD7124 family
//!
//! Enhanced v3.0 with family-wide device support

/// Supported device variants in the AD7124 family
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceType {
    /// AD7124-4: 4-channel, 24-bit ADC
    AD7124_4,
    /// AD7124-8: 8-channel, 24-bit ADC  
    AD7124_8,
    /// Unknown or unsupported device
    Unknown,
}

impl DeviceType {
    /// Get device type from device ID register
    pub fn from_device_id(device_id: u8) -> Self {
        match device_id {
            0x04 => DeviceType::AD7124_4,
            0x12 => DeviceType::AD7124_8,
            _ => DeviceType::Unknown,
        }
    }

    /// Get expected device ID for this variant
    pub fn device_id(self) -> Option<u8> {
        match self {
            DeviceType::AD7124_4 => Some(0x04),
            DeviceType::AD7124_8 => Some(0x12),
            DeviceType::Unknown => None,
        }
    }

    /// Check if device supports specific feature
    pub fn supports_feature(self, feature: DeviceFeature) -> bool {
        use DeviceFeature::*;
        match (self, feature) {
            (_, BasicOperation) => true, // All devices support basic ADC operation
            (DeviceType::AD7124_4, MultiChannel) => true, // 4 channels
            (DeviceType::AD7124_8, MultiChannel) => true, // 8 channels
            (_, InternalReference) => true, // Both have internal 2.5V reference
            (_, ProgrammableGain) => true, // Both have PGA
            (_, SelfTest) => true,       // Both support internal test modes
            (_, LowPowerMode) => true,   // Both support power modes
            (_, HighPrecision) => true,  // Both are 24-bit precision
            (_, DiagnosticMode) => true, // Both support diagnostic features
            (_, BurnoutCurrent) => true, // Both support burnout current sources
            _ => false,
        }
    }

    /// Get maximum number of channels for this device
    pub fn max_channels(self) -> u8 {
        match self {
            DeviceType::AD7124_4 => 4,
            DeviceType::AD7124_8 => 8,
            DeviceType::Unknown => 0,
        }
    }

    /// Get maximum SPI frequency for this device
    pub fn max_spi_frequency_hz(self) -> u32 {
        match self {
            DeviceType::AD7124_4 | DeviceType::AD7124_8 => 5_000_000, // 5 MHz max
            DeviceType::Unknown => 1_000_000,                         // Conservative value
        }
    }

    /// Get device description string
    pub fn description(self) -> &'static str {
        match self {
            DeviceType::AD7124_4 => "AD7124-4: 4-channel, 24-bit Sigma-Delta ADC",
            DeviceType::AD7124_8 => "AD7124-8: 8-channel, 24-bit Sigma-Delta ADC",
            DeviceType::Unknown => "Unknown or unsupported AD7124 device",
        }
    }
}

/// Device feature enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceFeature {
    /// Basic ADC operation
    BasicOperation,
    /// Multiple channel support
    MultiChannel,
    /// Internal reference voltage
    InternalReference,
    /// Programmable gain amplifier
    ProgrammableGain,
    /// Built-in self-test
    SelfTest,
    /// Low-power operation modes
    LowPowerMode,
    /// High-precision measurements (24-bit)
    HighPrecision,
    /// Diagnostic mode
    DiagnosticMode,
    /// Burnout current sources
    BurnoutCurrent,
    /// Continuous conversion mode
    ContinuousMode,
    /// Single conversion mode
    SingleShotMode,
    /// Digital filtering
    DigitalFiltering,
}

/// Device capabilities structure (runtime capability detection)
#[derive(Debug, Clone, Copy)]
pub struct DeviceCapabilities {
    /// The specific device type (AD7124-4, AD7124-8, etc.)
    pub device_type: DeviceType,
    /// Maximum number of analog input channels supported
    pub max_channels: u8,
    /// Maximum SPI clock frequency supported in Hz
    pub max_spi_frequency_hz: u32,
    /// Whether device has built-in voltage reference
    pub internal_reference: bool,
    /// Whether device supports programmable gain amplification
    pub programmable_gain: bool,
    /// Whether device supports built-in self-test functionality
    pub self_test: bool,
    /// Whether device supports low-power operation modes
    pub low_power_mode: bool,
    /// Whether device provides high-precision (24-bit) measurements
    pub high_precision: bool,
    /// Whether device supports diagnostic and status monitoring
    pub diagnostic_mode: bool,
    /// Whether device supports burnout current sources for sensor diagnostics
    pub burnout_current: bool,
    /// Whether device supports continuous conversion mode
    pub continuous_mode: bool,
    /// Whether device supports single-shot conversion mode
    pub single_shot_mode: bool,
    /// Whether device supports digital filtering options
    pub digital_filtering: bool,
}

impl DeviceCapabilities {
    /// Create capabilities for detected device type
    pub fn for_device(device_type: DeviceType) -> Self {
        Self {
            device_type,
            max_channels: device_type.max_channels(),
            max_spi_frequency_hz: device_type.max_spi_frequency_hz(),
            internal_reference: device_type.supports_feature(DeviceFeature::InternalReference),
            programmable_gain: device_type.supports_feature(DeviceFeature::ProgrammableGain),
            self_test: device_type.supports_feature(DeviceFeature::SelfTest),
            low_power_mode: device_type.supports_feature(DeviceFeature::LowPowerMode),
            high_precision: device_type.supports_feature(DeviceFeature::HighPrecision),
            diagnostic_mode: device_type.supports_feature(DeviceFeature::DiagnosticMode),
            burnout_current: device_type.supports_feature(DeviceFeature::BurnoutCurrent),
            continuous_mode: device_type.supports_feature(DeviceFeature::ContinuousMode),
            single_shot_mode: device_type.supports_feature(DeviceFeature::SingleShotMode),
            digital_filtering: device_type.supports_feature(DeviceFeature::DigitalFiltering),
        }
    }

    /// Check if device supports a specific feature
    pub fn supports(&self, feature: DeviceFeature) -> bool {
        self.device_type.supports_feature(feature)
    }

    /// Validate channel number against device capabilities
    pub fn validate_channel(&self, channel: u8) -> bool {
        channel < self.max_channels
    }
}

impl Default for DeviceCapabilities {
    fn default() -> Self {
        Self::for_device(DeviceType::Unknown)
    }
}
