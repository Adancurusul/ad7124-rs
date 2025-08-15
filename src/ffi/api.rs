//! C FFI API implementation for AD7124 driver
//!
//! This module provides C-callable functions that expose the AD7124 driver
//! functionality through a zero-allocation interface suitable for embedded systems.

use super::transport::{CFfiError, CFfiTransport};
use super::types::*;
use crate::ad7124::sync::AD7124Sync;
use crate::device_type::DeviceType;
use crate::{AD7124Config, SetupConfig};
use crate::{ChannelInput, OperatingMode, PgaGain, PowerMode, ReferenceSource};
use core::mem::{align_of, size_of};
use core::ptr;

/// Convert Rust device type to C device type
fn device_type_to_c(device_type: DeviceType) -> CAd7124DeviceType {
    match device_type {
        DeviceType::AD7124_4 => CAd7124DeviceType::AD7124_4,
        DeviceType::AD7124_8 => CAd7124DeviceType::AD7124_8,
        DeviceType::Unknown => CAd7124DeviceType::Unknown,
    }
}

/// Convert C device type to Rust device type
fn device_type_from_c(device_type: CAd7124DeviceType) -> DeviceType {
    match device_type {
        CAd7124DeviceType::AD7124_4 => DeviceType::AD7124_4,
        CAd7124DeviceType::AD7124_8 => DeviceType::AD7124_8,
        CAd7124DeviceType::Unknown => DeviceType::Unknown,
    }
}

/// Convert C gain to Rust gain
fn gain_from_c(gain: CAd7124Gain) -> PgaGain {
    match gain {
        CAd7124Gain::Gain1 => PgaGain::Gain1,
        CAd7124Gain::Gain2 => PgaGain::Gain2,
        CAd7124Gain::Gain4 => PgaGain::Gain4,
        CAd7124Gain::Gain8 => PgaGain::Gain8,
        CAd7124Gain::Gain16 => PgaGain::Gain16,
        CAd7124Gain::Gain32 => PgaGain::Gain32,
        CAd7124Gain::Gain64 => PgaGain::Gain64,
        CAd7124Gain::Gain128 => PgaGain::Gain128,
    }
}

/// Convert C channel input to Rust channel input
fn channel_input_from_c(input: CAd7124ChannelInput) -> ChannelInput {
    match input {
        CAd7124ChannelInput::Ain0 => ChannelInput::Ain0,
        CAd7124ChannelInput::Ain1 => ChannelInput::Ain1,
        CAd7124ChannelInput::Ain2 => ChannelInput::Ain2,
        CAd7124ChannelInput::Ain3 => ChannelInput::Ain3,
        CAd7124ChannelInput::Ain4 => ChannelInput::Ain4,
        CAd7124ChannelInput::Ain5 => ChannelInput::Ain5,
        CAd7124ChannelInput::Ain6 => ChannelInput::Ain6,
        CAd7124ChannelInput::Ain7 => ChannelInput::Ain7,
        CAd7124ChannelInput::TempSensor => ChannelInput::TempSensor,
        CAd7124ChannelInput::IntRef => ChannelInput::IntRef,
        CAd7124ChannelInput::Dgnd => ChannelInput::Dgnd,
        CAd7124ChannelInput::AvddAvssDiv5 => ChannelInput::AvddAvssDiv5,
    }
}

/// Convert C operating mode to Rust operating mode
fn operating_mode_from_c(mode: CAd7124OperatingMode) -> OperatingMode {
    match mode {
        CAd7124OperatingMode::Continuous => OperatingMode::Continuous,
        CAd7124OperatingMode::SingleConv => OperatingMode::SingleConv,
        CAd7124OperatingMode::Standby => OperatingMode::Standby,
        CAd7124OperatingMode::PowerDown => OperatingMode::PowerDown,
        CAd7124OperatingMode::Idle => OperatingMode::Idle,
        CAd7124OperatingMode::InternalZeroScale => OperatingMode::InternalZeroScale,
        CAd7124OperatingMode::InternalFullScale => OperatingMode::InternalFullScale,
        CAd7124OperatingMode::SystemZeroScale => OperatingMode::SystemZeroScale,
        CAd7124OperatingMode::SystemFullScale => OperatingMode::SystemFullScale,
    }
}

/// Convert C power mode to Rust power mode
fn power_mode_from_c(mode: CAd7124PowerMode) -> PowerMode {
    match mode {
        CAd7124PowerMode::LowPower => PowerMode::LowPower,
        CAd7124PowerMode::MidPower => PowerMode::MidPower,
        CAd7124PowerMode::FullPower => PowerMode::FullPower,
    }
}

/// Convert C reference source to Rust reference source
fn reference_source_from_c(source: CAd7124ReferenceSource) -> ReferenceSource {
    match source {
        CAd7124ReferenceSource::External => ReferenceSource::External1,
        CAd7124ReferenceSource::Internal => ReferenceSource::Internal,
        CAd7124ReferenceSource::AvddAvss => ReferenceSource::Avdd,
    }
}

/// Convert C burnout current to Rust burnout current
fn burnout_current_from_c(current: CAd7124BurnoutCurrent) -> crate::registers::BurnoutCurrent {
    match current {
        CAd7124BurnoutCurrent::Off => crate::registers::BurnoutCurrent::Off,
        CAd7124BurnoutCurrent::Current0_5uA => crate::registers::BurnoutCurrent::Current0_5uA,
        CAd7124BurnoutCurrent::Current2uA => crate::registers::BurnoutCurrent::Current2uA,
        CAd7124BurnoutCurrent::Current4uA => crate::registers::BurnoutCurrent::Current4uA,
    }
}

/// Convert C filter type to Rust filter type
fn filter_type_from_c(filter_type: CAd7124FilterType) -> crate::registers::FilterType {
    match filter_type {
        CAd7124FilterType::Sinc4 => crate::registers::FilterType::Sinc4,
        CAd7124FilterType::Sinc3 => crate::registers::FilterType::Sinc3,
        CAd7124FilterType::FastSettle => crate::registers::FilterType::FastSettle,
    }
}

/// Convert driver error to C error code
fn convert_driver_error(err: crate::errors::AD7124Error<CFfiError, ()>) -> i32 {
    match err {
        crate::errors::AD7124Error::Core(core_err) => match core_err {
            crate::errors::AD7124CoreError::InvalidParameter => {
                CAd7124Error::InvalidParameter as i32
            }
            crate::errors::AD7124CoreError::InvalidConfiguration => {
                CAd7124Error::InvalidConfiguration as i32
            }
            crate::errors::AD7124CoreError::DeviceNotResponding => {
                CAd7124Error::DeviceNotResponding as i32
            }
            crate::errors::AD7124CoreError::InvalidDeviceId => CAd7124Error::InvalidDeviceId as i32,
            crate::errors::AD7124CoreError::NotInitialized => CAd7124Error::NotInitialized as i32,
            crate::errors::AD7124CoreError::Timeout => CAd7124Error::Timeout as i32,
            crate::errors::AD7124CoreError::CalibrationFailed => {
                CAd7124Error::CalibrationFailed as i32
            }
            _ => CAd7124Error::InvalidParameter as i32,
        },
        crate::errors::AD7124Error::Transport(transport_err) => transport_err.to_c_error(),
        crate::errors::AD7124Error::Pin(_) => CAd7124Error::InvalidParameter as i32,
        crate::errors::AD7124Error::Timeout => CAd7124Error::Timeout as i32,
    }
}

// ===== Memory Management API =====

/// Get the size required for the driver structure
#[no_mangle]
pub extern "C" fn ad7124_get_driver_size() -> usize {
    size_of::<AD7124Sync<CFfiTransport>>()
}

/// Get the alignment requirement for the driver structure
#[no_mangle]
pub extern "C" fn ad7124_get_driver_align() -> usize {
    align_of::<AD7124Sync<CFfiTransport>>()
}

/// Initialize driver in provided memory location (placement new)
#[no_mangle]
pub extern "C" fn ad7124_init_in_place(
    instance: *mut u8,
    instance_size: usize,
    spi_interface: *const CAd7124SpiInterface,
    device_type: CAd7124DeviceType,
) -> i32 {
    // Parameter validation
    if instance.is_null() || spi_interface.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    if instance_size < size_of::<AD7124Sync<CFfiTransport>>() {
        return CAd7124Error::InvalidParameter as i32;
    }

    // Check memory alignment
    if (instance as usize) % align_of::<AD7124Sync<CFfiTransport>>() != 0 {
        return CAd7124Error::InvalidParameter as i32;
    }

    // Validate and create transport
    let spi_interface = unsafe { *spi_interface };
    let transport = match CFfiTransport::new(spi_interface) {
        Ok(transport) => transport,
        Err(err) => return err.to_c_error(),
    };

    // Create driver
    let device_type = device_type_from_c(device_type);
    let driver = match AD7124Sync::new(transport, device_type) {
        Ok(driver) => driver,
        Err(err) => return convert_driver_error(err),
    };

    // Place driver in instance
    let driver_ptr = instance as *mut AD7124Sync<CFfiTransport>;
    unsafe {
        ptr::write(driver_ptr, driver);
    }

    CAd7124Error::Ok as i32
}

/// Destroy driver in provided memory location
#[no_mangle]
pub extern "C" fn ad7124_destroy_in_place(instance: *mut u8) -> i32 {
    if instance.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver_ptr = instance as *mut AD7124Sync<CFfiTransport>;
    unsafe {
        ptr::drop_in_place(driver_ptr);
    }

    CAd7124Error::Ok as i32
}

/// Create AD7124 driver (heap allocation - for systems with heap)
#[no_mangle]
pub extern "C" fn ad7124_create(
    spi_interface: *const CAd7124SpiInterface,
    _device_type: CAd7124DeviceType,
) -> *mut Ad7124Driver {
    if spi_interface.is_null() {
        return ptr::null_mut();
    }

    // For no_std, we can't use heap allocation
    // This would require an allocator
    ptr::null_mut()
}

/// Destroy AD7124 driver (heap deallocation)
#[no_mangle]
pub extern "C" fn ad7124_destroy(driver: *mut Ad7124Driver) -> i32 {
    if driver.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    // For no_std, we can't use heap deallocation
    CAd7124Error::InvalidParameter as i32
}

// ===== Driver API =====

/// Initialize the AD7124 device
#[no_mangle]
pub extern "C" fn ad7124_init(instance: *mut u8) -> i32 {
    if instance.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.init() {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Read device ID
#[no_mangle]
pub extern "C" fn ad7124_read_device_id(instance: *mut u8, device_id: *mut u8) -> i32 {
    if instance.is_null() || device_id.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.read_device_id() {
        Ok(id) => {
            unsafe {
                *device_id = id;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Configure ADC settings
#[no_mangle]
pub extern "C" fn ad7124_configure_adc(instance: *mut u8, config: *const CAd7124Config) -> i32 {
    if instance.is_null() || config.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    let c_config = unsafe { *config };

    let rust_config = AD7124Config {
        operating_mode: operating_mode_from_c(c_config.operating_mode),
        power_mode: power_mode_from_c(c_config.power_mode),
        clock_source: crate::registers::ClockSource::Internal, // Default
        reference_source: reference_source_from_c(c_config.reference_source),
        internal_ref_enabled: c_config.internal_ref_enabled,
        data_ready_output_enabled: c_config.data_ready_output_enabled,
    };

    match driver.configure_adc(rust_config) {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Setup single-ended measurement
#[no_mangle]
pub extern "C" fn ad7124_setup_single_ended(
    instance: *mut u8,
    channel: u8,
    positive_input: CAd7124ChannelInput,
    setup_index: u8,
) -> i32 {
    if instance.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    let rust_input = channel_input_from_c(positive_input);

    match driver.setup_single_ended(channel, rust_input, setup_index) {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Setup differential measurement
#[no_mangle]
pub extern "C" fn ad7124_setup_differential(
    instance: *mut u8,
    channel: u8,
    positive_input: CAd7124ChannelInput,
    negative_input: CAd7124ChannelInput,
    setup_index: u8,
) -> i32 {
    if instance.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    let rust_pos = channel_input_from_c(positive_input);
    let rust_neg = channel_input_from_c(negative_input);

    match driver.setup_differential(channel, rust_pos, rust_neg, setup_index) {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Configure setup (PGA, reference, etc.)
#[no_mangle]
pub extern "C" fn ad7124_configure_setup(
    instance: *mut u8,
    setup_index: u8,
    config: *const CAd7124SetupConfig,
) -> i32 {
    if instance.is_null() || config.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    let c_config = unsafe { *config };

    let rust_config = SetupConfig {
        pga_gain: gain_from_c(c_config.pga_gain),
        reference_source: reference_source_from_c(c_config.reference_source),
        bipolar: c_config.bipolar,
        burnout_current: burnout_current_from_c(c_config.burnout_current),
        reference_buffers_enabled: c_config.reference_buffers_enabled,
        input_buffers_enabled: c_config.input_buffers_enabled,
    };

    match driver.configure_setup(setup_index, rust_config) {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Read raw ADC data
#[no_mangle]
pub extern "C" fn ad7124_read_data(instance: *mut u8, data: *mut u32) -> i32 {
    if instance.is_null() || data.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.read_data() {
        Ok(raw_data) => {
            unsafe {
                *data = raw_data;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Read voltage
#[no_mangle]
pub extern "C" fn ad7124_read_voltage(
    instance: *mut u8,
    setup_index: u8,
    voltage: *mut f32,
) -> i32 {
    if instance.is_null() || voltage.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.read_voltage(setup_index) {
        Ok(v) => {
            unsafe {
                *voltage = v;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Wait for data ready
#[no_mangle]
pub extern "C" fn ad7124_wait_for_data_ready(instance: *mut u8, timeout_ms: u32) -> i32 {
    if instance.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.wait_for_data_ready(timeout_ms) {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Software reset
#[no_mangle]
pub extern "C" fn ad7124_reset(instance: *mut u8) -> i32 {
    if instance.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.reset() {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Get device type
#[no_mangle]
pub extern "C" fn ad7124_get_device_type(instance: *mut u8) -> CAd7124DeviceType {
    if instance.is_null() {
        return CAd7124DeviceType::Unknown;
    }

    let driver = unsafe { &*(instance as *const AD7124Sync<CFfiTransport>) };
    device_type_to_c(driver.device_type())
}

/// Check if driver is initialized
#[no_mangle]
pub extern "C" fn ad7124_is_initialized(instance: *mut u8) -> bool {
    if instance.is_null() {
        return false;
    }

    let driver = unsafe { &*(instance as *const AD7124Sync<CFfiTransport>) };
    driver.is_initialized()
}

// ===== Enhanced Channel Management API =====

/// Check if data is ready (non-blocking)
#[no_mangle]
pub extern "C" fn ad7124_is_data_ready(instance: *mut u8) -> bool {
    if instance.is_null() {
        return false;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    match driver.is_data_ready() {
        Ok(ready) => ready,
        Err(_) => false,
    }
}

/// Wait for conversion ready with optional timeout
#[no_mangle]
pub extern "C" fn ad7124_wait_conv_ready(instance: *mut u8, timeout_ms: u32) -> i32 {
    if instance.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    let timeout = if timeout_ms == 0 {
        None
    } else {
        Some(timeout_ms)
    };

    match driver.wait_conv_ready(timeout) {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Enable or disable a specific channel
#[no_mangle]
pub extern "C" fn ad7124_enable_channel(instance: *mut u8, channel: u8, enable: bool) -> i32 {
    if instance.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.enable_channel(channel, enable) {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}

/// Check if a channel is enabled
#[no_mangle]
pub extern "C" fn ad7124_is_channel_enabled(instance: *mut u8, channel: u8) -> bool {
    if instance.is_null() {
        return false;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    match driver.is_channel_enabled(channel) {
        Ok(enabled) => enabled,
        Err(_) => false,
    }
}

/// Get the currently active channel
#[no_mangle]
pub extern "C" fn ad7124_get_active_channel(instance: *mut u8, channel: *mut u8) -> i32 {
    if instance.is_null() || channel.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.get_active_channel() {
        Ok(ch) => {
            unsafe {
                *channel = ch;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Get current channel from status register directly
#[no_mangle]
pub extern "C" fn ad7124_current_channel(instance: *mut u8, channel: *mut u8) -> i32 {
    if instance.is_null() || channel.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.current_channel() {
        Ok(ch) => {
            unsafe {
                *channel = ch;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

// ===== Enhanced Data Reading API =====

/// Read data from a specific channel
#[no_mangle]
pub extern "C" fn ad7124_read_channel_data(instance: *mut u8, channel: u8, data: *mut u32) -> i32 {
    if instance.is_null() || data.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.read_channel_data(channel) {
        Ok(raw_data) => {
            unsafe {
                *data = raw_data;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Read voltage from a specific channel
#[no_mangle]
pub extern "C" fn ad7124_read_channel_voltage(
    instance: *mut u8,
    channel: u8,
    voltage: *mut f32,
) -> i32 {
    if instance.is_null() || voltage.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.read_channel_voltage(channel) {
        Ok(v) => {
            unsafe {
                *voltage = v;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Read multiple channels sequentially
#[no_mangle]
pub extern "C" fn ad7124_read_multi_channel(
    instance: *mut u8,
    channels: *const u8,
    channel_count: usize,
    data_out: *mut u32,
) -> i32 {
    if instance.is_null() || channels.is_null() || data_out.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    if channel_count == 0 || channel_count > 16 {
        return CAd7124Error::InvalidParameter as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    let channel_slice = unsafe { core::slice::from_raw_parts(channels, channel_count) };

    match driver.read_multi_channel(channel_slice) {
        Ok(results) => {
            for (i, (_, data)) in results.iter().enumerate() {
                if i < channel_count {
                    unsafe {
                        *data_out.add(i) = *data;
                    }
                }
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Read voltage from multiple channels
#[no_mangle]
pub extern "C" fn ad7124_read_multi_voltage(
    instance: *mut u8,
    channels: *const u8,
    channel_count: usize,
    voltage_out: *mut f32,
) -> i32 {
    if instance.is_null() || channels.is_null() || voltage_out.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    if channel_count == 0 || channel_count > 16 {
        return CAd7124Error::InvalidParameter as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    let channel_slice = unsafe { core::slice::from_raw_parts(channels, channel_count) };

    match driver.read_multi_voltage(channel_slice) {
        Ok(results) => {
            for (i, (_, voltage)) in results.iter().enumerate() {
                if i < channel_count {
                    unsafe {
                        *voltage_out.add(i) = *voltage;
                    }
                }
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Scan all enabled channels and read their data
#[no_mangle]
pub extern "C" fn ad7124_scan_enabled_channels(
    instance: *mut u8,
    data_out: *mut u32,
    channels_out: *mut u8,
    max_channels: usize,
    channels_read: *mut usize,
) -> i32 {
    if instance.is_null() || data_out.is_null() || channels_out.is_null() || channels_read.is_null()
    {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.scan_enabled_channels() {
        Ok(results) => {
            let count = results.len().min(max_channels);
            for (i, (channel, data)) in results.iter().enumerate().take(count) {
                unsafe {
                    *data_out.add(i) = *data;
                    *channels_out.add(i) = *channel;
                }
            }
            unsafe {
                *channels_read = count;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Read data with channel information from status
#[no_mangle]
pub extern "C" fn ad7124_read_data_with_status(
    instance: *mut u8,
    channel: *mut u8,
    data: *mut u32,
) -> i32 {
    if instance.is_null() || channel.is_null() || data.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.read_data_with_status() {
        Ok((ch, raw_data)) => {
            unsafe {
                *channel = ch;
                *data = raw_data;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Fast data read (no status check)
#[no_mangle]
pub extern "C" fn ad7124_read_data_fast(instance: *mut u8, data: *mut u32) -> i32 {
    if instance.is_null() || data.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };

    match driver.read_data_fast() {
        Ok(raw_data) => {
            unsafe {
                *data = raw_data;
            }
            CAd7124Error::Ok as i32
        }
        Err(err) => convert_driver_error(err),
    }
}

/// Configure digital filter for a setup
#[no_mangle]
pub extern "C" fn ad7124_configure_filter(
    instance: *mut u8,
    setup_index: u8,
    config: *const CAd7124FilterConfig,
) -> i32 {
    if instance.is_null() || config.is_null() {
        return CAd7124Error::NullPointer as i32;
    }

    if setup_index > 7 {
        return CAd7124Error::InvalidParameter as i32;
    }

    let driver = unsafe { &mut *(instance as *mut AD7124Sync<CFfiTransport>) };
    let c_config = unsafe { &*config };

    let rust_config = crate::ad7124::FilterConfig {
        filter_type: filter_type_from_c(c_config.filter_type),
        output_data_rate: c_config.output_data_rate,
        single_cycle: c_config.single_cycle,
        reject_60hz: c_config.reject_60hz,
    };

    match driver.configure_filter(setup_index, rust_config) {
        Ok(_) => CAd7124Error::Ok as i32,
        Err(err) => convert_driver_error(err),
    }
}
