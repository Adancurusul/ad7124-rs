//! Core AD7124 logic (transport-layer independent)
//!
//! Contains all business logic without any transport dependencies

use crate::{
    device_type::{DeviceCapabilities, DeviceType},
    errors::*,
    registers::*,
};
use heapless::Vec;

/// Default timeout for ADC operations in milliseconds
pub const AD7124_DEFAULT_TIMEOUT_MS: u32 = 5000; // 5 seconds for first conversion

/// Core AD7124 logic (shared between sync and async)
#[derive(Debug)]
pub struct AD7124Core {
    /// Detected device type
    device_type: DeviceType,
    /// Device capabilities
    capabilities: DeviceCapabilities,
    /// Current configuration
    config: AD7124Config,
    /// Initialization state
    initialized: bool,
    /// Reference voltage (V)
    reference_voltage: f32,
    /// CRC enabled flag
    crc_enabled: bool,
    // Runtime state caching removed per final design - all status queries read hardware directly
    /// Channel configurations cache
    channel_configs: [ChannelConfig; 16],
    /// Setup configurations cache  
    setup_configs: [SetupConfig; 8],
}

/// Device configuration structure
#[derive(Debug, Clone, Copy)]
pub struct AD7124Config {
    /// ADC operating mode (continuous, single conversion, etc.)
    pub operating_mode: OperatingMode,
    /// Device power consumption mode
    pub power_mode: PowerMode,
    /// Clock source selection (internal/external)
    pub clock_source: ClockSource,
    /// Reference voltage source selection
    pub reference_source: ReferenceSource,
    /// Whether internal reference voltage is enabled
    pub internal_ref_enabled: bool,
    /// Whether data ready output pin is enabled
    pub data_ready_output_enabled: bool,
}

impl Default for AD7124Config {
    fn default() -> Self {
        Self {
            operating_mode: OperatingMode::Continuous,
            power_mode: PowerMode::FullPower,
            clock_source: ClockSource::Internal,
            reference_source: ReferenceSource::Internal,
            internal_ref_enabled: true,
            data_ready_output_enabled: true,
        }
    }
}

/// Channel configuration structure
#[derive(Debug, Clone, Copy)]
pub struct ChannelConfig {
    /// Whether this channel is enabled for conversion
    pub enabled: bool,
    /// Positive analog input selection
    pub positive_input: ChannelInput,
    /// Negative analog input selection (for differential measurements)
    pub negative_input: ChannelInput,
    /// Index of the setup configuration to use for this channel
    pub setup_index: u8,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            positive_input: ChannelInput::Ain0,
            negative_input: ChannelInput::Ain1,
            setup_index: 0,
        }
    }
}

/// Filter configuration structure
#[derive(Debug, Clone, Copy)]
pub struct FilterConfig {
    /// Digital filter type selection
    pub filter_type: FilterType,
    /// Output data rate in Hz
    pub output_data_rate: u16,
    /// Whether to use single cycle settling
    pub single_cycle: bool,
    /// Whether to enable 50/60Hz rejection
    pub reject_60hz: bool,
}

impl Default for FilterConfig {
    fn default() -> Self {
        Self {
            filter_type: FilterType::Sinc4,
            output_data_rate: 50, // 50 Hz
            single_cycle: false,
            reject_60hz: true,
        }
    }
}

/// Setup configuration structure
#[derive(Debug, Clone, Copy)]
pub struct SetupConfig {
    /// Programmable gain amplifier setting
    pub pga_gain: PgaGain,
    /// Reference voltage source for this setup
    pub reference_source: ReferenceSource,
    /// Whether to use bipolar (signed) or unipolar (unsigned) mode
    pub bipolar: bool,
    /// Burnout current source configuration for sensor diagnostics
    pub burnout_current: BurnoutCurrent,
    /// Whether reference voltage input buffers are enabled
    pub reference_buffers_enabled: bool,
    /// Whether analog input buffers are enabled
    pub input_buffers_enabled: bool,
}

impl Default for SetupConfig {
    fn default() -> Self {
        Self {
            pga_gain: PgaGain::Gain1,
            reference_source: ReferenceSource::Internal,
            bipolar: true,
            burnout_current: BurnoutCurrent::Off,
            reference_buffers_enabled: true,
            input_buffers_enabled: true,
        }
    }
}

/// Command sequence for batched operations
#[derive(Debug)]
pub struct CommandSequence {
    /// List of register write commands (register, value pairs)
    commands: Vec<(Register, u32), 16>, // Maximum 16 commands
}

impl CommandSequence {
    /// Create new empty command sequence
    pub fn new() -> Self {
        Self {
            commands: Vec::new(),
        }
    }

    /// Add register write command to sequence
    pub fn add_write(&mut self, register: Register, value: u32) -> Result<(), AD7124CoreError> {
        self.commands
            .push((register, value))
            .map_err(|_| AD7124CoreError::BufferOverflow)
    }

    /// Get commands as slice
    pub fn commands(&self) -> &[(Register, u32)] {
        &self.commands
    }

    /// Clear all commands
    pub fn clear(&mut self) {
        self.commands.clear();
    }
}

impl AD7124Core {
    /// Create new core instance
    pub fn new(device_type: DeviceType) -> Result<Self, AD7124CoreError> {
        let capabilities = DeviceCapabilities::for_device(device_type);

        Ok(Self {
            device_type,
            capabilities,
            config: AD7124Config::default(),
            initialized: false,
            reference_voltage: 2.5, // Internal reference voltage
            crc_enabled: false,
            // No runtime state variables in final design
            channel_configs: [ChannelConfig::default(); 16],
            setup_configs: [SetupConfig::default(); 8],
        })
    }

    /// Generate initialization sequence
    pub fn initialization_sequence(&self) -> Result<CommandSequence, AD7124CoreError> {
        let mut sequence = CommandSequence::new();

        // Reset the device
        sequence.add_write(Register::AdcCtrl, 0x8000)?; // Software reset

        // Configure ADC control register
        let adc_ctrl = self.build_adc_control_value()?;
        sequence.add_write(Register::AdcCtrl, adc_ctrl)?;

        // Configure IO_CTRL1 register - 使用默认值，REF_EN只在ADC_CTRL寄存器有效
        let io_ctrl1 = 0x000000; // 默认值，根据用户提供的寄存器表，IO_CTRL1没有REF_EN位
        sequence.add_write(Register::IoCtrl1, io_ctrl1)?;

        Ok(sequence)
    }

    /// Configure channel
    pub fn configure_channel(
        &self,
        channel: u8,
        config: ChannelConfig,
    ) -> Result<CommandSequence, AD7124CoreError> {
        if !self.capabilities.validate_channel(channel) {
            return Err(AD7124CoreError::InvalidParameter);
        }

        let mut sequence = CommandSequence::new();
        let register = match channel {
            0 => Register::Channel0,
            1 => Register::Channel1,
            2 => Register::Channel2,
            3 => Register::Channel3,
            4 => Register::Channel4,
            5 => Register::Channel5,
            6 => Register::Channel6,
            7 => Register::Channel7,
            _ => return Err(AD7124CoreError::InvalidParameter),
        };

        let channel_value = self.build_channel_value(config)?;
        sequence.add_write(register, channel_value)?;

        Ok(sequence)
    }

    /// Configure setup (PGA, reference, etc.)
    pub fn configure_setup(
        &self,
        setup_index: u8,
        config: SetupConfig,
    ) -> Result<CommandSequence, AD7124CoreError> {
        if setup_index > 7 {
            return Err(AD7124CoreError::InvalidParameter);
        }

        let mut sequence = CommandSequence::new();
        let register = match setup_index {
            0 => Register::Config0,
            1 => Register::Config1,
            2 => Register::Config2,
            3 => Register::Config3,
            4 => Register::Config4,
            5 => Register::Config5,
            6 => Register::Config6,
            7 => Register::Config7,
            _ => return Err(AD7124CoreError::InvalidParameter),
        };

        let config_value = self.build_setup_value(config)?;
        sequence.add_write(register, config_value)?;

        Ok(sequence)
    }

    /// Configure filter
    pub fn configure_filter(
        &self,
        setup_index: u8,
        config: FilterConfig,
    ) -> Result<CommandSequence, AD7124CoreError> {
        if setup_index > 7 {
            return Err(AD7124CoreError::InvalidParameter);
        }

        let mut sequence = CommandSequence::new();
        let register = match setup_index {
            0 => Register::Filter0,
            1 => Register::Filter1,
            2 => Register::Filter2,
            3 => Register::Filter3,
            4 => Register::Filter4,
            5 => Register::Filter5,
            6 => Register::Filter6,
            7 => Register::Filter7,
            _ => return Err(AD7124CoreError::InvalidParameter),
        };

        let filter_value = self.build_filter_value(config)?;
        sequence.add_write(register, filter_value)?;

        Ok(sequence)
    }

    /// Verify device ID
    pub fn verify_device_id(&self, id: u8) -> Result<DeviceType, AD7124CoreError> {
        let device_type = DeviceType::from_device_id(id);
        if device_type == DeviceType::Unknown {
            return Err(AD7124CoreError::InvalidDeviceId);
        }
        Ok(device_type)
    }

    /// Convert raw ADC reading to voltage
    pub fn raw_to_voltage(
        &self,
        raw_data: u32,
        pga_gain: PgaGain,
        bipolar: bool,
    ) -> Result<f32, AD7124CoreError> {
        if raw_data > 0xFFFFFF {
            // 24-bit max
            return Err(AD7124CoreError::ValueOutOfRange);
        }

        let gain = pga_gain.value() as f32;
        let full_scale = if bipolar {
            0x800000 as f32 // 2^23 for bipolar
        } else {
            0xFFFFFF as f32 // 2^24 - 1 for unipolar
        };

        let voltage = if bipolar {
            // Bipolar: -Vref to +Vref
            let signed_data = raw_data as i32 - 0x800000;
            (signed_data as f32 / full_scale) * self.reference_voltage / gain
        } else {
            // Unipolar: 0 to +Vref
            (raw_data as f32 / full_scale) * self.reference_voltage / gain
        };

        Ok(voltage)
    }

    /// Get current configuration
    pub fn config(&self) -> &AD7124Config {
        &self.config
    }

    /// Update configuration
    pub fn set_config(&mut self, config: AD7124Config) {
        self.config = config;
    }

    /// Get device capabilities
    pub fn capabilities(&self) -> &DeviceCapabilities {
        &self.capabilities
    }

    /// Get device type
    pub fn device_type(&self) -> DeviceType {
        self.device_type
    }

    /// Check if initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Mark as initialized (called after successful init sequence)
    pub fn set_initialized(&mut self) {
        self.initialized = true;
    }

    /// Set reference voltage
    pub fn set_reference_voltage(&mut self, voltage: f32) -> Result<(), AD7124CoreError> {
        if voltage <= 0.0 || voltage > 5.0 {
            return Err(AD7124CoreError::ValueOutOfRange);
        }
        self.reference_voltage = voltage;
        Ok(())
    }

    /// Get reference voltage
    pub fn reference_voltage(&self) -> f32 {
        self.reference_voltage
    }

    /// Enable/disable CRC
    pub fn set_crc_enabled(&mut self, enabled: bool) {
        self.crc_enabled = enabled;
    }

    /// Get CRC status
    pub fn is_crc_enabled(&self) -> bool {
        self.crc_enabled
    }

    // Private helper methods
    fn build_adc_control_value(&self) -> Result<u32, AD7124CoreError> {
        let mut value = 0u32;

        // Operating mode (bits 2:0)
        value |= (self.config.operating_mode as u32) & 0x07;

        // Power mode (bits 7:6)
        value |= ((self.config.power_mode as u32) & 0x03) << 6;

        // Clock source (bits 5:4)
        value |= ((self.config.clock_source as u32) & 0x03) << 4;

        // Internal reference enable (bit 8) - IMPORTANT!
        // Note: This bit must be set when using internal reference
        if self.config.internal_ref_enabled {
            value |= 1 << 8;
        }

        // Data ready output enable (bit 9)
        if self.config.data_ready_output_enabled {
            value |= 1 << 9;
        }

        Ok(value)
    }

    /// Public method to get ADC control value for configure_adc
    pub fn build_adc_control_value_public(&self) -> u32 {
        self.build_adc_control_value().unwrap_or(0)
    }

    fn build_channel_value(&self, config: ChannelConfig) -> Result<u32, AD7124CoreError> {
        let mut value = 0u32;

        // Channel enable (bit 15)
        if config.enabled {
            value |= 1 << 15;
        }

        // Setup selection (bits 14:12)
        value |= ((config.setup_index as u32) & 0x07) << 12;

        // Positive input (bits 9:5)
        value |= ((config.positive_input as u32) & 0x1F) << 5;

        // Negative input (bits 4:0)
        value |= (config.negative_input as u32) & 0x1F;

        Ok(value)
    }

    fn build_setup_value(&self, config: SetupConfig) -> Result<u32, AD7124CoreError> {
        let mut value = 0u32;

        // Bipolar (bit 11)
        if config.bipolar {
            value |= 1 << 11;
        }

        // Reference source (bits 5:4) - According to AD7124 datasheet
        // Internal reference = 2, requires setting bit 4 to 1
        if config.reference_source == ReferenceSource::Internal {
            value |= 0x0010; // Set bit 4 to 1, matching the correct implementation
        } else {
            // Other reference sources use the original bits 5:4 setting
            let ref_source_bits = ((config.reference_source as u32) & 0x03) << 4;
            value |= ref_source_bits;
        }

        // PGA gain (bits 2:0)
        value |= (config.pga_gain as u32) & 0x07;

        // Burnout current (bits 9:8)
        value |= ((config.burnout_current as u32) & 0x03) << 8;

        // Reference buffers (bits 7:6)
        if config.reference_buffers_enabled {
            value |= 0x03 << 6;
        }

        // Input buffers (bits 5:4)
        if config.input_buffers_enabled {
            value |= 0x03 << 4;
        }

        Ok(value)
    }

    fn build_filter_value(&self, config: FilterConfig) -> Result<u32, AD7124CoreError> {
        let mut value = 0u32;

        // Filter type (bits 23:21)
        value |= ((config.filter_type as u32) & 0x07) << 21;

        // Output data rate (bits 10:0)
        value |= (config.output_data_rate as u32) & 0x7FF;

        // Single cycle (bit 20)
        if config.single_cycle {
            value |= 1 << 20;
        }

        // 60Hz reject (bit 14)
        if config.reject_60hz {
            value |= 1 << 14;
        }

        // 50Hz reject (bit 13) - commonly enabled with 60Hz
        if config.reject_60hz {
            value |= 1 << 13;
        }

        Ok(value)
    }

    // ==================== Pure Logic Methods (No I/O) ====================

    /// Generate channel configuration command sequence (removed - logic moved to I/O layer)
    // This method removed per final design - channel enable/disable done directly in I/O layer

    /// Get channel register address (static calculation)
    pub fn get_channel_register(channel: u8) -> Result<Register, AD7124CoreError> {
        match channel {
            0 => Ok(Register::Channel0),
            1 => Ok(Register::Channel1),
            2 => Ok(Register::Channel2),
            3 => Ok(Register::Channel3),
            4 => Ok(Register::Channel4),
            5 => Ok(Register::Channel5),
            6 => Ok(Register::Channel6),
            7 => Ok(Register::Channel7),
            _ => Err(AD7124CoreError::InvalidParameter),
        }
    }

    /// Get setup configuration register address (static calculation)
    pub fn get_config_register(setup: u8) -> Result<Register, AD7124CoreError> {
        match setup {
            0 => Ok(Register::Config0),
            1 => Ok(Register::Config1),
            2 => Ok(Register::Config2),
            3 => Ok(Register::Config3),
            4 => Ok(Register::Config4),
            5 => Ok(Register::Config5),
            6 => Ok(Register::Config6),
            7 => Ok(Register::Config7),
            _ => Err(AD7124CoreError::InvalidParameter),
        }
    }

    /// Get filter configuration register address (static calculation)
    pub fn get_filter_register(setup: u8) -> Result<Register, AD7124CoreError> {
        match setup {
            0 => Ok(Register::Filter0),
            1 => Ok(Register::Filter1),
            2 => Ok(Register::Filter2),
            3 => Ok(Register::Filter3),
            4 => Ok(Register::Filter4),
            5 => Ok(Register::Filter5),
            6 => Ok(Register::Filter6),
            7 => Ok(Register::Filter7),
            _ => Err(AD7124CoreError::InvalidParameter),
        }
    }

    /// Get channel configuration
    pub fn get_channel_config(&self, channel: u8) -> Option<&ChannelConfig> {
        if channel >= 16 {
            return None;
        }
        Some(&self.channel_configs[channel as usize])
    }

    /// Update channel configuration (internal cache only)
    pub fn update_channel_config(
        &mut self,
        channel: u8,
        config: ChannelConfig,
    ) -> Result<(), AD7124CoreError> {
        if !self.capabilities.validate_channel(channel) {
            return Err(AD7124CoreError::InvalidParameter);
        }

        self.channel_configs[channel as usize] = config;
        Ok(())
    }

    /// Get setup configuration
    pub fn get_setup_config(&self, setup_index: u8) -> Option<&SetupConfig> {
        if setup_index >= 8 {
            return None;
        }
        Some(&self.setup_configs[setup_index as usize])
    }

    /// Update setup configuration (cached)
    pub fn update_setup_config(
        &mut self,
        setup_index: u8,
        config: SetupConfig,
    ) -> Result<(), AD7124CoreError> {
        if setup_index >= 8 {
            return Err(AD7124CoreError::InvalidParameter);
        }

        self.setup_configs[setup_index as usize] = config;
        Ok(())
    }

    /// Convert raw data to voltage with explicit setup parameters
    pub fn raw_to_voltage_with_setup(
        &self,
        raw_data: u32,
        _setup: u8,
        bipolar: bool,
        gain: PgaGain,
        vref: f32,
    ) -> f32 {
        let full_scale = if bipolar {
            0x800000 as f32 // 2^23 for bipolar
        } else {
            0xFFFFFF as f32 // 2^24 - 1 for unipolar
        };

        let gain_value = gain.gain_value();

        if bipolar {
            // Bipolar: -Vref to +Vref
            let signed_data = (raw_data as i32) - 0x800000; // Convert to signed
            (signed_data as f32 / full_scale) * vref / gain_value
        } else {
            // Unipolar: 0 to Vref
            (raw_data as f32 / full_scale) * vref / gain_value
        }
    }

    /// Get default timeout in milliseconds (static method)
    pub fn get_default_timeout() -> u32 {
        AD7124_DEFAULT_TIMEOUT_MS
    }

    /// Validate channel number for this device (pure logic)
    pub fn validate_channel(&self, channel: u8) -> bool {
        self.capabilities.validate_channel(channel)
    }

    /// Build ADC control register value (public interface)
    pub fn build_adc_control_register(&self) -> u32 {
        self.build_adc_control_value().unwrap_or(0)
    }

    /// Build channel configuration value (pure logic)
    pub fn build_channel_config_value(config: ChannelConfig) -> u32 {
        let mut value = 0u32;

        // Channel enable (bit 15)
        if config.enabled {
            value |= 1 << 15;
        }

        // Setup selection (bits 14:12)
        value |= ((config.setup_index as u32) & 0x07) << 12;

        // Positive input (bits 9:5)
        value |= ((config.positive_input as u32) & 0x1F) << 5;

        // Negative input (bits 4:0)
        value |= (config.negative_input as u32) & 0x1F;

        value
    }

    /// Build setup configuration value (pure logic)
    pub fn build_setup_config_value(config: SetupConfig) -> u32 {
        let mut value = 0u32;

        // Bipolar (bit 11)
        if config.bipolar {
            value |= 1 << 11;
        }

        // Reference source (bits 5:4) - According to AD7124 datasheet
        // Internal reference = 2, requires setting bit 4 to 1
        if config.reference_source == ReferenceSource::Internal {
            value |= 0x0010; // Set bit 4 to 1, matching the correct implementation
        } else {
            // Other reference sources use the original bits 5:4 setting
            let ref_source_bits = ((config.reference_source as u32) & 0x03) << 4;
            value |= ref_source_bits;
        }

        // PGA gain (bits 2:0)
        value |= (config.pga_gain as u32) & 0x07;

        // Burnout current (bits 9:8)
        value |= ((config.burnout_current as u32) & 0x03) << 8;

        // Reference buffers (bits 7:6)
        if config.reference_buffers_enabled {
            value |= 0x03 << 6;
        }

        // Input buffers (bits 5:4)
        if config.input_buffers_enabled {
            value |= 0x03 << 4;
        }

        value
    }

    /// Build filter configuration value (pure logic)
    pub fn build_filter_config_value(config: FilterConfig) -> u32 {
        let mut value = 0u32;

        // Filter type (bits 23:21)
        value |= ((config.filter_type as u32) & 0x07) << 21;

        // Output data rate (bits 10:0)
        value |= (config.output_data_rate as u32) & 0x7FF;

        // Single cycle (bit 20)
        if config.single_cycle {
            value |= 1 << 20;
        }

        // 60Hz reject (bit 14)
        if config.reject_60hz {
            value |= 1 << 14;
        }

        // 50Hz reject (bit 13) - commonly enabled with 60Hz
        if config.reject_60hz {
            value |= 1 << 13;
        }

        value
    }
}

impl Default for AD7124Core {
    fn default() -> Self {
        Self::new(DeviceType::AD7124_8).unwrap()
    }
}
