/**
 * @file main.c
 * @brief AD7124 C FFI Example - Comprehensive Feature Demonstration
 * 
 * This example demonstrates how to use the AD7124 driver from C code
 * using the FFI interface. It shows all available features including:
 * - Enhanced channel management (enable/disable, status checking)
 * - Channel-specific data reading
 * - Non-blocking data ready checks
 * - Multi-channel operations
 * - BurnoutCurrent configuration (OFF, 0.5µA, 2µA, 4µA)
 * - FilterConfig configuration (SINC4, SINC3, FastSettle)
 * - Advanced filter settings (data rate, 60Hz rejection, single cycle)
 * - Comprehensive measurement setups combining all features
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// No need for aligned allocation - using static memory with macro

#include "ad7124_ffi.h"

// Global hardware state (example)
static struct {
    int spi_fd;        // SPI file descriptor (for Linux SPI)
    int cs_gpio;       // Chip select GPIO pin
    int reset_gpio;    // Reset GPIO pin
} hw_state = {
    .spi_fd = -1,
    .cs_gpio = 24,     // Example GPIO pin
    .reset_gpio = 25   // Example GPIO pin
};

/**
 * @brief SPI write implementation
 * @param data Data to write
 * @param len Number of bytes to write
 * @return 0 on success, negative on error
 */
int spi_write(const uint8_t* data, size_t len) {
    printf("SPI Write: ");
    for (size_t i = 0; i < len; i++) {
        printf("0x%02X ", data[i]);
    }
    printf("(len=%zu)\n", len);
    
    // In a real implementation, you would write to SPI hardware here
    // using hw_state.spi_fd, hw_state.cs_gpio, etc.
    return 0; // Success
}

/**
 * @brief SPI read implementation
 * @param data Buffer for read data
 * @param len Number of bytes to read
 * @return 0 on success, negative on error
 */
int spi_read(uint8_t* data, size_t len) {
    // In a real implementation, you would read from SPI hardware here
    // For this example, we simulate a device ID response
    if (len >= 2) {
        data[0] = 0x00;  // First byte (don't care)
        data[1] = 0x06;  // AD7124-8 device ID
    }
    
    printf("SPI Read: ");
    for (size_t i = 0; i < len; i++) {
        printf("0x%02X ", data[i]);
    }
    printf("(len=%zu)\n", len);
    
    // In a real implementation, you would read from SPI hardware here
    // using hw_state.spi_fd, hw_state.cs_gpio, etc.
    return 0; // Success
}

/**
 * @brief SPI transfer implementation (write then read)
 * @param read_data Buffer for read data
 * @param write_data Data to write
 * @param len Number of bytes to transfer
 * @return 0 on success, negative on error
 */
int spi_transfer(uint8_t* read_data, const uint8_t* write_data, size_t len) {
    printf("SPI Transfer - Write: ");
    for (size_t i = 0; i < len; i++) {
        printf("0x%02X ", write_data[i]);
    }
    printf("\n");
    
    // Simulate responses based on commands
    for (size_t i = 0; i < len; i++) {
        if (i == 0) {
            read_data[i] = 0x00; // First byte usually ignored
        } else if ((write_data[0] & 0x3F) == 0x05) { // Device ID register read (0x05)
            // AD7124-8 device ID should be 0x12
            read_data[i] = 0x14; // Correct AD7124-8 device ID
        } else if ((write_data[0] & 0x3F) == 0x00) { // Status register read  
            read_data[i] = 0x00; // Data ready (bit 7 = 0 means ready), channel 0 active (bits 3:0)
        } else if ((write_data[0] & 0x3F) == 0x02) { // Data register read
            // Simulate ADC data (24-bit) - incrementing values
            static uint32_t data_counter = 0x800000;
            data_counter += 1000;
            uint32_t data = data_counter & 0xFFFFFF;
            if (i == 1) read_data[i] = (data >> 16) & 0xFF; // MSB
            else if (i == 2) read_data[i] = (data >> 8) & 0xFF; // Middle
            else if (i == 3) read_data[i] = data & 0xFF; // LSB
            else read_data[i] = 0x00;
        } else if ((write_data[0] & 0x3F) >= 0x09 && (write_data[0] & 0x3F) <= 0x10) { // Channel registers
            // Mock enabled channels (bit 15 set for enabled)
            if (i == 1) read_data[i] = 0x80; // Enable bit set
            else if (i == 2) read_data[i] = 0x00;
            else read_data[i] = 0x00;
        } else if ((write_data[0] & 0x3F) >= 0x19 && (write_data[0] & 0x3F) <= 0x20) { // Config registers
            // Mock config with bipolar, gain=1, internal ref
            if (i == 1) read_data[i] = 0x08; // Bipolar bit
            else if (i == 2) read_data[i] = 0x00;
            else read_data[i] = 0x00;
        } else {
            read_data[i] = 0x00; // Default response
        }
    }
    
    printf("SPI Transfer - Read:  ");
    for (size_t i = 0; i < len; i++) {
        printf("0x%02X ", read_data[i]);
    }
    printf("(len=%zu)\n", len);
    
    // In a real implementation, you would do SPI transfer here
    // using hw_state.spi_fd, hw_state.cs_gpio, etc.
    return 0; // Success
}

/**
 * @brief Delay implementation
 * @param ms Delay time in milliseconds
 * @return 0 on success, negative on error
 */
int delay_ms(uint32_t ms) {
    printf("Delay: %u ms\n", ms);
    
    // In a real implementation, you would implement actual delay here
    // For example: usleep(ms * 1000); on Linux
    
    return 0; // Success
}

/**
 * @brief Print error message for AD7124 error codes
 */
void print_error(const char* operation, int error_code) {
    const char* error_msg;
    
    switch (error_code) {
        case AD7124_OK: error_msg = "Success"; break;
        case AD7124_NULL_POINTER: error_msg = "Null pointer"; break;
        case AD7124_SPI_WRITE: error_msg = "SPI write error"; break;
        case AD7124_SPI_READ: error_msg = "SPI read error"; break;
        case AD7124_SPI_TRANSFER: error_msg = "SPI transfer error"; break;
        case AD7124_INVALID_CHANNEL: error_msg = "Invalid channel"; break;
        case AD7124_INVALID_PARAMETER: error_msg = "Invalid parameter"; break;
        case AD7124_NOT_INITIALIZED: error_msg = "Not initialized"; break;
        case AD7124_DEVICE_NOT_RESPONDING: error_msg = "Device not responding"; break;
        case AD7124_INVALID_DEVICE_ID: error_msg = "Invalid device ID"; break;
        case AD7124_TIMEOUT: error_msg = "Timeout"; break;
        default: error_msg = "Unknown error"; break;
    }
    
    printf("ERROR in %s: %s (%d)\n", operation, error_msg, error_code);
}

/**
 * @brief Main function demonstrating AD7124 usage
 */
int main(void) {
    printf("=== AD7124 C FFI Example ===\n\n");
    
    // Declare driver instance using the convenient macro
    AD7124_DECLARE_DRIVER_INSTANCE(driver_instance);
    
    // Setup SPI interface
    ad7124_spi_interface_t spi_interface = {
        .write = spi_write,
        .read = spi_read,
        .transfer = spi_transfer,
        .delay_ms = delay_ms
    };
    
    printf("Driver instance requirements:\n");
    printf("  Size: %d bytes (compile-time constant)\n", AD7124_DRIVER_SIZE);
    printf("  Alignment: %d bytes (compile-time constant)\n\n", AD7124_DRIVER_ALIGN);
    
    printf("Using static instance buffer at %p\n\n", (void*)driver_instance);
    
    // Initialize driver using the convenient macro
    int result = AD7124_INIT_DRIVER(driver_instance, &spi_interface, AD7124_DEVICE_AD7124_8);
    if (result != AD7124_OK) {
        print_error("driver initialization", result);
        return 1;
    }
    
    printf("Driver initialized successfully\n\n");
    
    // Initialize the AD7124 device
    result = ad7124_init(driver_instance);
    if (result != AD7124_OK) {
        print_error("device initialization", result);
        ad7124_destroy_in_place(driver_instance);
        return 1;
    }
    
    printf("Device initialized successfully\n\n");
    
    // Read and verify device ID
    uint8_t device_id;
    result = ad7124_read_device_id(driver_instance, &device_id);
    if (result != AD7124_OK) {
        print_error("device ID read", result);
    } else {
        printf("Device ID: 0x%02X\n", device_id);
        if (device_id == 0x06) {
            printf("  -> AD7124-8 detected\n");
        } else if (device_id == 0x02) {
            printf("  -> AD7124-4 detected\n");
        } else {
            printf("  -> Unknown device\n");
        }
    }
    printf("\n");
    
    // Check if driver is initialized
    bool is_initialized = ad7124_is_initialized(driver_instance);
    printf("Driver initialized: %s\n\n", is_initialized ? "Yes" : "No");
    
    // Get device type
    ad7124_device_type_t device_type = ad7124_get_device_type(driver_instance);
    printf("Device type: ");
    switch (device_type) {
        case AD7124_DEVICE_AD7124_4: printf("AD7124-4\n"); break;
        case AD7124_DEVICE_AD7124_8: printf("AD7124-8\n"); break;
        default: printf("Unknown\n"); break;
    }
    printf("\n");
    
    // Configure ADC settings
    ad7124_config_t adc_config = {
        .operating_mode = AD7124_MODE_CONTINUOUS,
        .power_mode = AD7124_POWER_FULL,
        .reference_source = AD7124_REF_INTERNAL,
        .internal_ref_enabled = true,
        .data_ready_output_enabled = true
    };
    
    result = ad7124_configure_adc(driver_instance, &adc_config);
    if (result != AD7124_OK) {
        print_error("ADC configuration", result);
    } else {
        printf("ADC configured successfully\n");
    }
    printf("\n");
    
    // Setup single-ended measurement on AIN0
    result = ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);
    if (result != AD7124_OK) {
        print_error("single-ended setup", result);
    } else {
        printf("Single-ended measurement configured (Channel 0, AIN0)\n");
    }
    printf("\n");
    
    // Configure setup with gain
    ad7124_setup_config_t setup_config = {
        .pga_gain = AD7124_GAIN_1,
        .reference_source = AD7124_REF_INTERNAL,
        .bipolar = true,
        .reference_buffers_enabled = true,
        .input_buffers_enabled = true
    };
    
    result = ad7124_configure_setup(driver_instance, 0, &setup_config);
    if (result != AD7124_OK) {
        print_error("setup configuration", result);
    } else {
        printf("Setup configured (Gain: 1x, Bipolar, Internal reference)\n");
    }
    printf("\n");
    
    // Perform measurements
    printf("=== Performing Measurements ===\n\n");
    
    for (int i = 0; i < 5; i++) {
        printf("Measurement %d:\n", i + 1);
        
        // Wait for data ready
        result = ad7124_wait_for_data_ready(driver_instance, 1000);
        if (result != AD7124_OK) {
            print_error("wait for data ready", result);
            continue;
        }
        
        // Read raw ADC data
        uint32_t raw_data;
        result = ad7124_read_data(driver_instance, &raw_data);
        if (result != AD7124_OK) {
            print_error("raw data read", result);
            continue;
        }
        
        printf("  Raw data: 0x%06X (%u)\n", raw_data & 0xFFFFFF, raw_data & 0xFFFFFF);
        
        // Read voltage
        float voltage;
        result = ad7124_read_voltage(driver_instance, 0, &voltage);
        if (result != AD7124_OK) {
            print_error("voltage read", result);
            continue;
        }
        
        printf("  Voltage: %.6f V\n", voltage);
        
        // Convert raw data to voltage manually for verification
        float manual_voltage = ad7124_raw_to_voltage(raw_data, 2.5f, AD7124_GAIN_1, true);
        printf("  Manual calculation: %.6f V\n", manual_voltage);
        
        printf("\n");
    }
    
    // Test enhanced channel management features
    printf("=== Testing Enhanced Channel Management ===\n\n");
    
    // Enable multiple channels
    uint8_t channels_to_test[] = {0, 2, 4};
    size_t num_channels = sizeof(channels_to_test) / sizeof(channels_to_test[0]);
    
    for (size_t i = 0; i < num_channels; i++) {
        uint8_t channel = channels_to_test[i];
        
        // Configure channel
        result = ad7124_setup_single_ended(driver_instance, channel, AD7124_AIN0 + channel, 0);
        if (result != AD7124_OK) {
            print_error("channel setup", result);
            continue;
        }
        
        // Enable channel
        result = ad7124_enable_channel(driver_instance, channel, true);
        if (result != AD7124_OK) {
            print_error("channel enable", result);
        } else {
            printf("Channel %d enabled successfully\n", channel);
        }
        
        // Check if channel is enabled
        bool is_enabled = ad7124_is_channel_enabled(driver_instance, channel);
        printf("Channel %d enabled status: %s\n", channel, is_enabled ? "YES" : "NO");
    }
    
    // Get active channel
    uint8_t active_channel;
    result = ad7124_get_active_channel(driver_instance, &active_channel);
    if (result != AD7124_OK) {
        print_error("get active channel", result);
    } else {
        printf("Current active channel: %d\n", active_channel);
    }
    printf("\n");
    
    // Test channel-specific data reading
    printf("=== Testing Channel-Specific Data Reading ===\n\n");
    
    for (size_t i = 0; i < num_channels; i++) {
        uint8_t channel = channels_to_test[i];
        
        printf("Reading from Channel %d:\n", channel);
        
        // Read channel-specific raw data
        uint32_t channel_data;
        result = ad7124_read_channel_data(driver_instance, channel, &channel_data);
        if (result != AD7124_OK) {
            print_error("channel data read", result);
        } else {
            printf("  Raw data: 0x%06X (%u)\n", channel_data & 0xFFFFFF, channel_data & 0xFFFFFF);
        }
        
        // Read channel-specific voltage
        float channel_voltage;
        result = ad7124_read_channel_voltage(driver_instance, channel, &channel_voltage);
        if (result != AD7124_OK) {
            print_error("channel voltage read", result);
        } else {
            printf("  Voltage: %.6f V\n", channel_voltage);
        }
        
        printf("\n");
    }
    
    // Test non-blocking data ready check
    printf("=== Testing Non-blocking Data Ready Check ===\n\n");
    
    int ready_attempts = 0;
    const int max_attempts = 5;
    
    while (ready_attempts < max_attempts) {
        bool data_ready = ad7124_is_data_ready(driver_instance);
        if (data_ready) {
            printf("Data is ready! (attempt %d/%d)\n", ready_attempts + 1, max_attempts);
            
            // Read the data
            uint32_t ready_data;
            uint8_t ready_channel;
            result = ad7124_read_data_with_status(driver_instance, &ready_channel, &ready_data);
            if (result != AD7124_OK) {
                print_error("data with status read", result);
            } else {
                printf("  Channel: %d, Data: 0x%06X\n", ready_channel, ready_data & 0xFFFFFF);
            }
            break;
        } else {
            printf("Data not ready yet (attempt %d/%d)\n", ready_attempts + 1, max_attempts);
            delay_ms(100); // Small delay
        }
        ready_attempts++;
    }
    
    if (ready_attempts >= max_attempts) {
        printf("Timeout waiting for data ready\n");
    }
    printf("\n");
    
    // Test multi-channel operations
    printf("=== Testing Multi-Channel Operations ===\n\n");
    
    // Read multiple channels at once
    uint32_t multi_data[3];
    result = ad7124_read_multi_channel(driver_instance, channels_to_test, num_channels, multi_data);
    if (result != AD7124_OK) {
        print_error("multi-channel read", result);
    } else {
        printf("Multi-channel raw data read:\n");
        for (size_t i = 0; i < num_channels; i++) {
            printf("  Channel %d: 0x%06X (%u)\n", channels_to_test[i], 
                   multi_data[i] & 0xFFFFFF, multi_data[i] & 0xFFFFFF);
        }
    }
    printf("\n");
    
    // Read multiple voltages at once
    float multi_voltages[3];
    result = ad7124_read_multi_voltage(driver_instance, channels_to_test, num_channels, multi_voltages);
    if (result != AD7124_OK) {
        print_error("multi-voltage read", result);
    } else {
        printf("Multi-channel voltage read:\n");
        for (size_t i = 0; i < num_channels; i++) {
            printf("  Channel %d: %.6f V\n", channels_to_test[i], multi_voltages[i]);
        }
    }
    printf("\n");
    
    // Scan all enabled channels
    uint32_t scan_data[16];
    uint8_t scan_channels[16];
    size_t channels_read;
    
    result = ad7124_scan_enabled_channels(driver_instance, scan_data, scan_channels, 16, &channels_read);
    if (result != AD7124_OK) {
        print_error("scan enabled channels", result);
    } else {
        printf("Enabled channels scan (found %zu channels):\n", channels_read);
        for (size_t i = 0; i < channels_read; i++) {
            printf("  Channel %d: 0x%06X (%u)\n", scan_channels[i], 
                   scan_data[i] & 0xFFFFFF, scan_data[i] & 0xFFFFFF);
        }
    }
    printf("\n");
    
    // Test differential measurement
    printf("=== Testing Differential Measurement ===\n\n");
    
    result = ad7124_setup_differential(driver_instance, 1, AD7124_AIN0, AD7124_AIN1, 0);
    if (result != AD7124_OK) {
        print_error("differential setup", result);
    } else {
        printf("Differential measurement configured (Channel 1, AIN0-AIN1)\n");
        
        // Enable differential channel
        result = ad7124_enable_channel(driver_instance, 1, true);
        if (result != AD7124_OK) {
            print_error("differential channel enable", result);
        }
        
        // Read differential voltage
        float diff_voltage;
        result = ad7124_read_channel_voltage(driver_instance, 1, &diff_voltage);
        if (result != AD7124_OK) {
            print_error("differential voltage read", result);
        } else {
            printf("Differential voltage: %.6f V\n", diff_voltage);
        }
    }
    printf("\n");
    
    // Test temperature measurement
    printf("=== Testing Temperature Measurement ===\n\n");
    
    result = ad7124_setup_single_ended(driver_instance, 7, AD7124_TEMP_SENSOR, 0);
    if (result != AD7124_OK) {
        print_error("temperature setup", result);
    } else {
        printf("Temperature measurement configured (Channel 7)\n");
        
        // Enable temperature channel
        result = ad7124_enable_channel(driver_instance, 7, true);
        if (result != AD7124_OK) {
            print_error("temperature channel enable", result);
        }
        
        // Read temperature voltage using channel-specific function
        float temp_voltage;
        result = ad7124_read_channel_voltage(driver_instance, 7, &temp_voltage);
        if (result != AD7124_OK) {
            print_error("temperature voltage read", result);
        } else {
            printf("Temperature sensor voltage: %.6f V\n", temp_voltage);
            
            // Convert to temperature (approximate)
            // AD7124 temperature sensor: ~1.17V at 25°C, ~1.8mV/°C
            float temperature = 25.0f + (temp_voltage - 1.17f) / 0.0018f;
            printf("Estimated temperature: %.1f °C\n", temperature);
        }
    }
    printf("\n");
    
    // Test fast data reading
    printf("=== Testing Fast Data Reading ===\n\n");
    
    printf("Performing fast data reads (no status check):\n");
    for (int i = 0; i < 3; i++) {
        uint32_t fast_data;
        result = ad7124_read_data_fast(driver_instance, &fast_data);
        if (result != AD7124_OK) {
            print_error("fast data read", result);
        } else {
            printf("  Fast read %d: 0x%06X (%u)\n", i + 1, 
                   fast_data & 0xFFFFFF, fast_data & 0xFFFFFF);
        }
    }
    printf("\n");
    
    // Test Enhanced Setup Configuration with BurnoutCurrent
    printf("=== Testing Enhanced Setup Configuration with BurnoutCurrent ===\n\n");
    
    const char* burnout_names[] = {"OFF", "0.5µA", "2µA", "4µA"};
    ad7124_burnout_current_t burnout_values[] = {
        AD7124_BURNOUT_OFF,
        AD7124_BURNOUT_0_5UA,
        AD7124_BURNOUT_2UA,
        AD7124_BURNOUT_4UA
    };
    
    for (int i = 0; i < 4; i++) {
        ad7124_setup_config_t enhanced_setup = {
            .pga_gain = AD7124_GAIN_64,
            .reference_source = AD7124_REF_INTERNAL,
            .bipolar = true,
            .reference_buffers_enabled = true,
            .input_buffers_enabled = false,
            .burnout_current = burnout_values[i]  // Sensor diagnostic configuration
        };
        
        uint8_t setup_id = i % 8;  // Use different setup IDs
        result = ad7124_configure_setup(driver_instance, setup_id, &enhanced_setup);
        if (result == AD7124_OK) {
            printf("Setup %d configured with burnout current: %s\n", setup_id, burnout_names[i]);
        } else {
            print_error("enhanced setup configuration", result);
        }
    }
    printf("\n");
    
    // Test Enhanced Filter Configuration
    printf("=== Testing Enhanced Filter Configuration ===\n\n");
    
    const char* filter_names[] = {"SINC4", "SINC3", "FastSettle"};
    ad7124_filter_type_t filter_values[] = {
        AD7124_FILTER_SINC4,
        AD7124_FILTER_SINC3,
        AD7124_FILTER_FAST_SETTLE
    };
    
    for (int i = 0; i < 3; i++) {
        ad7124_filter_config_t filter_config = {
            .filter_type = filter_values[i],
            .output_data_rate = 50 + (i * 25),  // Different rates: 50, 75, 100 Hz
            .single_cycle = (i == 2),           // Enable single cycle for FastSettle
            .reject_60hz = (i != 1)             // Enable 60Hz rejection except for SINC3
        };
        
        uint8_t setup_id = i % 8;
        result = ad7124_configure_filter(driver_instance, setup_id, &filter_config);
        if (result == AD7124_OK) {
            printf("Setup %d configured with filter: %s at %d Hz\n", 
                   setup_id, filter_names[i], filter_config.output_data_rate);
            printf("  Single cycle: %s, 60Hz rejection: %s\n",
                   filter_config.single_cycle ? "Enabled" : "Disabled",
                   filter_config.reject_60hz ? "Enabled" : "Disabled");
        } else {
            print_error("filter configuration", result);
        }
    }
    printf("\n");
    
    // Test Comprehensive Configuration
    printf("=== Testing Comprehensive Configuration (Burnout + Filter) ===\n\n");
    
    // Configure a complete measurement setup with both enhanced features
    ad7124_setup_config_t comprehensive_setup = {
        .pga_gain = AD7124_GAIN_32,
        .reference_source = AD7124_REF_INTERNAL,
        .bipolar = true,
        .reference_buffers_enabled = true,
        .input_buffers_enabled = true,
        .burnout_current = AD7124_BURNOUT_2UA  // Sensor fault detection enabled
    };
    
    ad7124_filter_config_t comprehensive_filter = {
        .filter_type = AD7124_FILTER_SINC4,
        .output_data_rate = 50,              // 50 Hz for good noise rejection
        .single_cycle = false,
        .reject_60hz = true                  // Enable 60Hz line frequency rejection
    };
    
    result = ad7124_configure_setup(driver_instance, 3, &comprehensive_setup);
    if (result == AD7124_OK) {
        printf("Comprehensive setup configured successfully\n");
        printf("  Gain: 32x, Bipolar, 2µA burnout current\n");
        
        result = ad7124_configure_filter(driver_instance, 3, &comprehensive_filter);
        if (result == AD7124_OK) {
            printf("  Filter: SINC4, 50Hz data rate, 60Hz rejection enabled\n");
            
            // Set up a channel to use this comprehensive configuration
            result = ad7124_setup_single_ended(driver_instance, 3, AD7124_AIN3, 3);
            if (result == AD7124_OK) {
                printf("  Channel 3 configured to use comprehensive setup\n");
                
                // Enable the channel
                result = ad7124_enable_channel(driver_instance, 3, true);
                if (result == AD7124_OK) {
                    printf("  Channel 3 enabled for measurement\n");
                    
                    // Try to read from this enhanced configuration
                    float enhanced_voltage;
                    result = ad7124_read_channel_voltage(driver_instance, 3, &enhanced_voltage);
                    if (result == AD7124_OK) {
                        printf("  Enhanced measurement result: %.6f V\n", enhanced_voltage);
                    } else {
                        print_error("enhanced channel voltage read", result);
                    }
                } else {
                    print_error("comprehensive channel enable", result);
                }
            } else {
                print_error("comprehensive channel setup", result);
            }
        } else {
            print_error("comprehensive filter configuration", result);
        }
    } else {
        print_error("comprehensive setup configuration", result);
    }
    printf("\n");
    
    // Test software reset
    printf("=== Testing Software Reset ===\n\n");
    
    result = ad7124_reset(driver_instance);
    if (result != AD7124_OK) {
        print_error("software reset", result);
    } else {
        printf("Software reset completed\n");
    }
    printf("\n");
    
    // Cleanup
    printf("=== Cleanup ===\n\n");
    
    result = ad7124_destroy_in_place(driver_instance);
    if (result != AD7124_OK) {
        print_error("driver destruction", result);
    } else {
        printf("Driver destroyed successfully\n");
    }
    
    printf("Static instance cleaned up automatically\n\n");
    
    printf("=== Comprehensive AD7124 FFI Example Completed Successfully ===\n");
    printf("\nDemonstrated features:\n");
    printf("✓ Enhanced channel management (enable/disable, status checking)\n");
    printf("✓ Channel-specific data reading\n");
    printf("✓ Non-blocking data ready checks\n");
    printf("✓ Multi-channel operations\n");
    printf("✓ Fast data reading\n");
    printf("✓ Real-time hardware status reading (no stale cache)\n");
    printf("✓ BurnoutCurrent configuration (OFF, 0.5µA, 2µA, 4µA)\n");
    printf("✓ FilterConfig configuration (SINC4, SINC3, FastSettle)\n");
    printf("✓ Advanced filter settings (data rate, 60Hz rejection, single cycle)\n");
    printf("✓ Comprehensive measurement setups combining all features\n");
    
    return 0;
}