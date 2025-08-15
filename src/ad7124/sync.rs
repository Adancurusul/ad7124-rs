//! Synchronous AD7124 driver implementation
//!
//! Provides blocking API for AD7124 driver

use crate::ad7124::{AD7124Config, AD7124Core, ChannelConfig, FilterConfig, SetupConfig};
use crate::device_type::DeviceType;
use crate::errors::{AD7124CoreError, AD7124Error};
use crate::registers::{DeviceStatus, Register};
use crate::transport::SyncSpiTransport;

/// Synchronous AD7124 driver
#[derive(Debug)]
pub struct AD7124Sync<T> {
    /// SPI transport interface
    transport: T,
    /// Core driver logic
    core: AD7124Core,
}

impl<T> AD7124Sync<T>
where
    T: SyncSpiTransport,
{
    /// Create new AD7124 driver instance
    pub fn new(transport: T, device_type: DeviceType) -> Result<Self, AD7124Error<T::Error, ()>> {
        let core = AD7124Core::new(device_type).map_err(AD7124Error::Core)?;

        Ok(Self { transport, core })
    }

    /// Create new AD7124 driver with auto-detection
    pub fn new_with_detection(mut transport: T) -> Result<Self, AD7124Error<T::Error, ()>> {
        // Read device ID to detect type
        let device_id = Self::read_device_id_raw(&mut transport)?;
        let device_type = DeviceType::from_device_id(device_id);

        if device_type == DeviceType::Unknown {
            return Err(AD7124Error::Core(AD7124CoreError::InvalidDeviceId));
        }

        let core = AD7124Core::new(device_type).map_err(AD7124Error::Core)?;

        Ok(Self { transport, core })
    }

    /// Initialize the AD7124 device
    pub fn init(&mut self) -> Result<(), AD7124Error<T::Error, ()>> {
        if self.core.is_initialized() {
            return Err(AD7124Error::Core(AD7124CoreError::InvalidConfiguration));
        }

        // Verify device ID
        let device_id = self.read_device_id()?;
        self.core
            .verify_device_id(device_id)
            .map_err(AD7124Error::Core)?;

        // Execute initialization sequence
        let sequence = self
            .core
            .initialization_sequence()
            .map_err(AD7124Error::Core)?;

        self.execute_sequence(&sequence)?;

        // Wait for device to be ready
        self.transport
            .delay_ms(10)
            .map_err(AD7124Error::Transport)?;

        self.core.set_initialized();
        Ok(())
    }

    /// Read device ID register
    pub fn read_device_id(&mut self) -> Result<u8, AD7124Error<T::Error, ()>> {
        Self::read_device_id_raw(&mut self.transport)
    }

    /// Read device status
    pub fn read_status(&mut self) -> Result<DeviceStatus, AD7124Error<T::Error, ()>> {
        let raw_status = self.read_register(Register::Status)?;
        Ok(DeviceStatus::new(raw_status as u8))
    }

    /// Configure ADC control settings
    pub fn configure_adc(&mut self, config: AD7124Config) -> Result<(), AD7124Error<T::Error, ()>> {
        self.core.set_config(config);

        // Build ADC control value directly without reset
        let adc_ctrl_value = self.core.build_adc_control_value_public();

        // Write to ADC control register
        self.write_register(Register::AdcCtrl, adc_ctrl_value)?;

        // If internal reference is enabled, wait for it to stabilize
        if config.internal_ref_enabled {
            // Add a small delay for reference stabilization
            self.transport
                .delay_ms(10)
                .map_err(AD7124Error::Transport)?;
        }

        Ok(())
    }

    /// Configure channel
    pub fn configure_channel(
        &mut self,
        channel: u8,
        config: ChannelConfig,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        let sequence = self
            .core
            .configure_channel(channel, config)
            .map_err(AD7124Error::Core)?;

        self.execute_sequence(&sequence)
    }

    /// Configure setup (PGA, reference, etc.)
    pub fn configure_setup(
        &mut self,
        setup_index: u8,
        config: SetupConfig,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        let sequence = self
            .core
            .configure_setup(setup_index, config)
            .map_err(AD7124Error::Core)?;

        self.execute_sequence(&sequence)
    }

    /// Configure filter
    pub fn configure_filter(
        &mut self,
        setup_index: u8,
        config: FilterConfig,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        let sequence = self
            .core
            .configure_filter(setup_index, config)
            .map_err(AD7124Error::Core)?;

        self.execute_sequence(&sequence)
    }

    /// Read raw ADC data (24-bit)
    pub fn read_data(&mut self) -> Result<u32, AD7124Error<T::Error, ()>> {
        let raw_data = self.read_register(Register::Data)?;
        Ok(raw_data & 0xFFFFFF) // Mask to 24-bit
    }

    /// Read ADC data and convert to voltage
    pub fn read_voltage(&mut self, setup_index: u8) -> Result<f32, AD7124Error<T::Error, ()>> {
        let raw_data = self.read_data()?;

        // For simplicity, assume default setup configuration
        // In a real application, you would track the setup configuration
        let pga_gain = crate::registers::PgaGain::Gain1;
        let bipolar = true;
        let vref = 2.5; // Internal reference

        let voltage =
            self.core
                .raw_to_voltage_with_setup(raw_data, setup_index, bipolar, pga_gain, vref);
        Ok(voltage)
    }

    /// Wait for data ready
    pub fn wait_for_data_ready(
        &mut self,
        timeout_ms: u32,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        let start_time = 0; // In a real implementation, you'd use a proper timer

        loop {
            let status = self.read_status()?;
            if status.rdy() {
                return Ok(());
            }

            if start_time > timeout_ms {
                return Err(AD7124Error::Core(AD7124CoreError::Timeout));
            }

            self.transport.delay_ms(1).map_err(AD7124Error::Transport)?;
        }
    }

    // ==================== Enhanced Channel Management ====================

    /// Check if data is ready (non-blocking)
    pub fn is_data_ready(&mut self) -> Result<bool, AD7124Error<T::Error, ()>> {
        let status = self.read_status()?;
        Ok(status.is_ready())
    }

    /// Wait for conversion ready with optional timeout
    pub fn wait_conv_ready(
        &mut self,
        timeout_ms: Option<u32>,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        let timeout = timeout_ms.unwrap_or(AD7124Core::get_default_timeout());
        self.wait_for_data_ready(timeout)
    }

    /// Enable or disable a specific channel (requires I/O to read-modify-write register)
    pub fn enable_channel(
        &mut self,
        channel: u8,
        enable: bool,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        if !self.core.validate_channel(channel) {
            return Err(AD7124Error::Core(AD7124CoreError::InvalidParameter));
        }

        let channel_reg = AD7124Core::get_channel_register(channel).map_err(AD7124Error::Core)?;

        // Read current register value
        let mut current_value = self.read_register(channel_reg)?;

        // Modify enable bit (bit 15)
        if enable {
            current_value |= 1 << 15;
        } else {
            current_value &= !(1 << 15);
        }

        // Write back
        self.write_register(channel_reg, current_value)
    }

    /// Check if a channel is enabled (requires I/O to read register for accuracy)
    pub fn is_channel_enabled(&mut self, channel: u8) -> Result<bool, AD7124Error<T::Error, ()>> {
        if !self.core.validate_channel(channel) {
            return Ok(false);
        }

        let channel_reg = AD7124Core::get_channel_register(channel).map_err(AD7124Error::Core)?;

        let reg_value = self.read_register(channel_reg)?;
        Ok((reg_value & (1 << 15)) != 0)
    }

    // Cached methods removed per final design - all queries read hardware directly

    /// Get the currently active channel (requires I/O to read status register)
    pub fn get_active_channel(&mut self) -> Result<u8, AD7124Error<T::Error, ()>> {
        let status_value = self.read_register(Register::Status)?;
        Ok((status_value as u8) & 0x0F) // Bits 3:0 contain channel
    }

    /// Get current channel from status register directly
    pub fn current_channel(&mut self) -> Result<u8, AD7124Error<T::Error, ()>> {
        let status = self.read_status()?;
        Ok(status.channel())
    }

    // ==================== Enhanced Data Reading ====================

    /// Read data from a specific channel
    pub fn read_channel_data(&mut self, channel: u8) -> Result<u32, AD7124Error<T::Error, ()>> {
        // 1. Validate channel
        if !self.core.validate_channel(channel) {
            return Err(AD7124Error::Core(AD7124CoreError::InvalidParameter));
        }

        // 2. Check if channel is enabled
        if !self.is_channel_enabled(channel)? {
            return Err(AD7124Error::Core(AD7124CoreError::ChannelNotEnabled));
        }

        // 3. Check current active channel
        let current_channel = self.get_active_channel()?;

        // 4. If need to switch channel
        if current_channel != channel {
            // Disable current channel
            self.enable_channel(current_channel, false)?;

            // Enable target channel
            self.enable_channel(channel, true)?;

            // Wait for first conversion with extended timeout
            self.wait_conv_ready(Some(AD7124Core::get_default_timeout() * 2))?;
        } else {
            // Same channel, wait for data ready
            self.wait_conv_ready(None)?;
        }

        // 5. Read the data
        self.read_data()
    }

    /// Read voltage from a specific channel
    pub fn read_channel_voltage(&mut self, channel: u8) -> Result<f32, AD7124Error<T::Error, ()>> {
        let raw_data = self.read_channel_data(channel)?;

        // Get channel configuration to determine setup
        let channel_reg = AD7124Core::get_channel_register(channel).map_err(AD7124Error::Core)?;
        let channel_value = self.read_register(channel_reg)?;
        let setup_index = ((channel_value >> 12) & 0x07) as u8; // Bits[14:12]

        // Get setup configuration
        let config_reg = AD7124Core::get_config_register(setup_index).map_err(AD7124Error::Core)?;
        let config_value = self.read_register(config_reg)?;

        let bipolar = (config_value & (1 << 11)) != 0;
        let gain_bits = config_value & 0x07;
        let gain = crate::registers::PgaGain::from_bits(gain_bits as u8)
            .map_err(|_| AD7124Error::Core(AD7124CoreError::InvalidParameter))?;

        // Assume internal 2.5V reference
        let vref = 2.5;

        let voltage =
            self.core
                .raw_to_voltage_with_setup(raw_data, setup_index, bipolar, gain, vref);

        Ok(voltage)
    }

    /// Read multiple channels sequentially
    pub fn read_multi_channel(
        &mut self,
        channels: &[u8],
    ) -> Result<heapless::Vec<(u8, u32), 8>, AD7124Error<T::Error, ()>> {
        let mut results = heapless::Vec::new();

        for &channel in channels {
            let data = self.read_channel_data(channel)?;
            results
                .push((channel, data))
                .map_err(|_| AD7124Error::Core(AD7124CoreError::InvalidParameter))?;
        }

        Ok(results)
    }

    /// Read voltage from multiple channels
    pub fn read_multi_voltage(
        &mut self,
        channels: &[u8],
    ) -> Result<heapless::Vec<(u8, f32), 8>, AD7124Error<T::Error, ()>> {
        let mut results = heapless::Vec::new();

        for &channel in channels {
            let voltage = self.read_channel_voltage(channel)?;
            results
                .push((channel, voltage))
                .map_err(|_| AD7124Error::Core(AD7124CoreError::InvalidParameter))?;
        }

        Ok(results)
    }

    /// Get all enabled channels (requires I/O to read each channel register)
    pub fn get_enabled_channels(
        &mut self,
    ) -> Result<heapless::Vec<u8, 8>, AD7124Error<T::Error, ()>> {
        let mut enabled_channels = heapless::Vec::new();

        for channel in 0..self.core.capabilities().max_channels {
            if self.is_channel_enabled(channel)? {
                let _ = enabled_channels.push(channel);
            }
        }

        Ok(enabled_channels)
    }

    /// Scan all enabled channels and read their data
    pub fn scan_enabled_channels(
        &mut self,
    ) -> Result<heapless::Vec<(u8, u32), 8>, AD7124Error<T::Error, ()>> {
        let channels = self.get_enabled_channels()?;
        let channel_slice: &[u8] = &channels;
        self.read_multi_channel(channel_slice)
    }

    /// Read data with channel information from status
    pub fn read_data_with_status(&mut self) -> Result<(u8, u32), AD7124Error<T::Error, ()>> {
        self.wait_conv_ready(None)?;
        let channel = self.get_active_channel()?;
        let data = self.read_data()?;
        Ok((channel, data))
    }

    /// Fast data read (no status check)
    pub fn read_data_fast(&mut self) -> Result<u32, AD7124Error<T::Error, ()>> {
        // Direct read of data register without status checks
        self.read_register(Register::Data)
    }

    /// Perform software reset
    pub fn reset(&mut self) -> Result<(), AD7124Error<T::Error, ()>> {
        self.write_register(Register::AdcCtrl, 0x8000)?;
        self.transport
            .delay_ms(10)
            .map_err(AD7124Error::Transport)?;
        Ok(())
    }

    /// Get device capabilities
    pub fn capabilities(&self) -> &crate::device_type::DeviceCapabilities {
        self.core.capabilities()
    }

    /// Get current configuration
    pub fn config(&self) -> &AD7124Config {
        self.core.config()
    }

    /// Get device type
    pub fn device_type(&self) -> DeviceType {
        self.core.device_type()
    }

    /// Check if driver is initialized
    pub fn is_initialized(&self) -> bool {
        self.core.is_initialized()
    }

    /// Get reference to transport layer (for advanced usage)
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Get mutable reference to transport layer (for advanced usage)
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    // Private helper methods
    fn read_device_id_raw(transport: &mut T) -> Result<u8, AD7124Error<T::Error, ()>> {
        let mut read_buf = [0u8; 2];
        let write_buf = [(Register::Id.addr() | 0x40), 0x00]; // Read command

        transport
            .transfer(&mut read_buf, &write_buf)
            .map_err(AD7124Error::Transport)?;
        Ok(read_buf[1])
    }

    /// Read value from specified register
    pub fn read_register(&mut self, register: Register) -> Result<u32, AD7124Error<T::Error, ()>> {
        let size = register.size() as usize;
        let mut read_buf = [0u8; 8]; // Max size for any register
        let mut write_buf = [0u8; 8];
        write_buf[0] = register.addr() | 0x40; // Read command

        self.transport
            .transfer(&mut read_buf[..size + 1], &write_buf[..size + 1])
            .map_err(AD7124Error::Transport)?;

        // Convert bytes to u32 (big-endian)
        let mut value = 0u32;
        for i in 1..=size {
            value = (value << 8) | (read_buf[i] as u32);
        }

        Ok(value)
    }

    /// Write value to specified register
    pub fn write_register(
        &mut self,
        register: Register,
        value: u32,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        if register.is_readonly() {
            return Err(AD7124Error::Core(AD7124CoreError::InvalidParameter));
        }

        let size = register.size() as usize;
        let mut write_buf = [0u8; 8]; // Max size for any register
        write_buf[0] = register.addr(); // Write command

        // According to AD7124 datasheet, all registers use big-endian format (MSB first)
        // Use big-endian format for all registers consistently
        for i in 0..size {
            // Big-endian: most significant byte first
            write_buf[1 + i] = ((value >> ((size - 1 - i) * 8)) & 0xFF) as u8;
        }

        self.transport
            .write(&write_buf[..size + 1])
            .map_err(AD7124Error::Transport)?;
        Ok(())
    }

    fn execute_sequence(
        &mut self,
        sequence: &crate::ad7124::CommandSequence,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        for (register, value) in sequence.commands() {
            self.write_register(*register, *value)?;
        }
        Ok(())
    }
}

/// High-level convenience functions for common operations
impl<T> AD7124Sync<T>
where
    T: SyncSpiTransport,
{
    /// Quick setup for single-ended measurement
    pub fn setup_single_ended(
        &mut self,
        channel: u8,
        positive_input: crate::registers::ChannelInput,
        setup_index: u8,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        // Configure channel for single-ended measurement
        let channel_config = ChannelConfig {
            enabled: true,
            positive_input,
            negative_input: crate::registers::ChannelInput::Dgnd,
            setup_index,
        };
        self.configure_channel(channel, channel_config)?;

        // Configure setup with default values
        let setup_config = SetupConfig::default();
        self.configure_setup(setup_index, setup_config)?;

        // Configure filter with default values
        let filter_config = FilterConfig::default();
        self.configure_filter(setup_index, filter_config)?;

        Ok(())
    }

    /// Quick setup for differential measurement
    pub fn setup_differential(
        &mut self,
        channel: u8,
        positive_input: crate::registers::ChannelInput,
        negative_input: crate::registers::ChannelInput,
        setup_index: u8,
    ) -> Result<(), AD7124Error<T::Error, ()>> {
        // Configure channel for differential measurement
        let channel_config = ChannelConfig {
            enabled: true,
            positive_input,
            negative_input,
            setup_index,
        };
        self.configure_channel(channel, channel_config)?;

        // Configure setup with default values
        let setup_config = SetupConfig::default();
        self.configure_setup(setup_index, setup_config)?;

        // Configure filter with default values
        let filter_config = FilterConfig::default();
        self.configure_filter(setup_index, filter_config)?;

        Ok(())
    }
}
