/**
 * @file main.c
 * @brief AD7124 Embedded Systems Integration Example
 * 
 * This example demonstrates AD7124 driver integration in embedded environments
 * with static memory allocation, hardware abstraction patterns, and comprehensive
 * sensor measurement capabilities including advanced diagnostics and filtering.
 * 
 * Features demonstrated:
 * - Embedded HAL simulation
 * - Static memory management
 * - Hardware register-level operations
 * - Sensor diagnostics with burnout current detection
 * - Digital filtering for signal processing
 * - Multi-channel measurement systems
 * - System health monitoring
 * - Performance optimization patterns
 * 
 * Target environment: ARM Cortex-M microcontrollers
 * Memory footprint: Minimal (static allocation only)
 * Real-time capabilities: Non-blocking operations supported
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ad7124_ffi.h"

// ===== Embedded Hardware Abstraction Layer =====

/**
 * @brief Simulated embedded hardware state
 */
typedef struct {
    uint32_t spi_base;          // SPI peripheral base address
    uint8_t cs_pin;             // Chip select GPIO pin
    uint8_t reset_pin;          // Reset GPIO pin
    uint32_t spi_frequency;     // SPI clock frequency
    bool cs_state;              // Current CS pin state
    bool reset_state;           // Current reset pin state
    // Simulated register states
    uint8_t device_id;          // Device ID register
    uint8_t status_reg;         // Status register
    uint32_t data_reg;          // Data register
} embedded_hal_t;

static embedded_hal_t hal_state = {
    .spi_base = 0x40003800,     // Example STM32 SPI2 base
    .cs_pin = 12,               // GPIO pin 12
    .reset_pin = 11,            // GPIO pin 11
    .spi_frequency = 1000000,   // 1 MHz
    .cs_state = true,           // Idle high
    .reset_state = true,        // Not in reset
    .device_id = 0x12,          // AD7124-8 ID
    .status_reg = 0x00,         // Ready
    .data_reg = 0x800000        // Sample data
};

/**
 * @brief SPI write operation with hardware simulation
 */
int embedded_spi_write(const uint8_t* data, size_t len) {
    printf("[SPI_WRITE] Base:0x%08X CS:%d Data:", hal_state.spi_base, hal_state.cs_pin);
    for (size_t i = 0; i < len; i++) {
        printf(" %02X", data[i]);
    }
    printf("\n");
    return 0;  // Success
}

/**
 * @brief SPI read operation with hardware simulation
 */
int embedded_spi_read(uint8_t* data, size_t len) {
    printf("[SPI_READ] Base:0x%08X CS:%d Bytes:%zu\n", 
           hal_state.spi_base, hal_state.cs_pin, len);
    
    // Simulate reading device ID
    if (len >= 1) data[0] = hal_state.device_id;
    if (len >= 2) data[1] = hal_state.status_reg;
    
    return 0;  // Success
}

/**
 * @brief SPI transfer operation with hardware simulation
 */
int embedded_spi_transfer(uint8_t* read_data, const uint8_t* write_data, size_t len) {
    printf("[SPI_XFER] Base:0x%08X CS:%d TX:", hal_state.spi_base, hal_state.cs_pin);
    for (size_t i = 0; i < len; i++) {
        printf(" %02X", write_data[i]);
    }
    printf(" -> RX:");
    
    // Simulate hardware responses based on commands
    for (size_t i = 0; i < len; i++) {
        if (i == 0) {
            read_data[i] = 0x00;  // Don't care byte
        } else if (write_data[0] == 0x45 && i == 1) {  // Device ID read
            read_data[i] = hal_state.device_id;
        } else if (write_data[0] == 0x40 && i == 1) {  // Status read
            read_data[i] = hal_state.status_reg;
        } else if (write_data[0] == 0x42) {  // Data read
            if (i == 1) read_data[i] = (hal_state.data_reg >> 16) & 0xFF;
            else if (i == 2) read_data[i] = (hal_state.data_reg >> 8) & 0xFF;
            else if (i == 3) read_data[i] = hal_state.data_reg & 0xFF;
            else read_data[i] = 0x00;
        } else {
            read_data[i] = 0x00;
        }
        printf(" %02X", read_data[i]);
    }
    printf("\n");
    
    return 0;  // Success
}

/**
 * @brief Hardware delay implementation
 */
int embedded_delay_ms(uint32_t ms) {
    printf("[DELAY] %u ms (SPI freq: %u Hz)\n", ms, hal_state.spi_frequency);
    // In real embedded system: implement with timer or systick
    return 0;  // Success
}

/**
 * @brief Initialize embedded hardware peripherals
 */
void embedded_hal_init(void) {
    printf("=== Embedded HAL Initialization ===\n");
    printf("Initializing SPI peripheral at 0x%08X\n", hal_state.spi_base);
    printf("Configuring CS pin: %d\n", hal_state.cs_pin);
    printf("Configuring Reset pin: %d\n", hal_state.reset_pin);
    printf("Setting SPI frequency: %u Hz\n", hal_state.spi_frequency);
    printf("Hardware initialization complete\n\n");
}

/**
 * @brief Display system diagnostics
 */
void embedded_system_diagnostics(uint8_t* driver_instance) {
    printf("=== System Diagnostics ===\n");
    printf("Driver memory address: %p\n", (void*)driver_instance);
    printf("Driver memory size: %d bytes (compile-time constant)\n", AD7124_DRIVER_SIZE);
    printf("Driver initialized: %s\n", ad7124_is_initialized(driver_instance) ? "Yes" : "No");
    printf("SPI base address: 0x%08X\n", hal_state.spi_base);
    printf("CS pin state: %s\n", hal_state.cs_state ? "High (idle)" : "Low (active)");
    printf("Reset pin state: %s\n", hal_state.reset_state ? "High (not reset)" : "Low (reset)");
    printf("Simulated register states:\n");
    printf("  Device ID: 0x%02X\n", hal_state.device_id);
    printf("  Status: 0x%02X\n", hal_state.status_reg);
    printf("  Data: 0x%06X\n", hal_state.data_reg);
    printf("\n");
}

// ===== Main Application =====

int main(void) {
    printf("=== AD7124 Embedded C Example ===\n\n");
    
    // Initialize embedded hardware
    embedded_hal_init();
    
    // Setup SPI interface structure
    ad7124_spi_interface_t spi_interface = {
        .write = embedded_spi_write,
        .read = embedded_spi_read,
        .transfer = embedded_spi_transfer,
        .delay_ms = embedded_delay_ms
    };
    
    // Declare static driver instance (typical for embedded systems)
    AD7124_DECLARE_DRIVER_INSTANCE(driver_instance);
    
    printf("Driver requirements: size=%d bytes, align=%d bytes\n", 
           AD7124_DRIVER_SIZE, AD7124_DRIVER_ALIGN);
    printf("Using static buffer: size=%zu bytes, align=%d bytes\n\n", 
           sizeof(driver_instance), AD7124_DRIVER_ALIGN);
    
    // Initialize driver in static memory
    int result = AD7124_INIT_DRIVER(driver_instance, &spi_interface, AD7124_DEVICE_AD7124_8);
    if (result != AD7124_OK) {
        printf("ERROR: Driver initialization failed: %d\n", result);
        return 1;
    }
    printf("Driver initialized in static memory\n\n");
    
    // Initialize device
    result = ad7124_init(driver_instance);
    if (result != AD7124_OK) {
        printf("ERROR: Device initialization failed: %d\n", result);
        return 1;
    }
    printf("Device initialized successfully\n\n");
    
    // Show system state
    embedded_system_diagnostics(driver_instance);
    
    // Verify device identity
    uint8_t device_id;
    result = ad7124_read_device_id(driver_instance, &device_id);
    if (result == AD7124_OK) {
        printf("Device ID verified: 0x%02X ", device_id);
        switch (device_id) {
            case 0x14: printf("(AD7124-8)\n"); break;
            case 0x04: printf("(AD7124-4)\n"); break;
            default: printf("(Unknown)\n"); break;
        }
    }
    printf("\n");
    
    // ===== ADC Configuration =====
    printf("=== ADC Configuration ===\n");
    
    // Configure ADC for continuous operation
    ad7124_config_t adc_config = {
        .operating_mode = AD7124_MODE_CONTINUOUS,
        .power_mode = AD7124_POWER_FULL,
        .reference_source = AD7124_REF_INTERNAL,
        .internal_ref_enabled = true,
        .data_ready_output_enabled = true
    };
    
    result = ad7124_configure_adc(driver_instance, &adc_config);
    if (result == AD7124_OK) {
        printf("ADC configured for continuous mode\n");
    }
    
    // Setup channel 0 for single-ended measurement
    result = ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);
    if (result == AD7124_OK) {
        printf("Channel 0 configured for AIN0 single-ended\n");
    }
    
    // Configure setup 0 with sensor diagnostics
    ad7124_setup_config_t setup_config = {
        .pga_gain = AD7124_GAIN_1,
        .reference_source = AD7124_REF_INTERNAL,
        .bipolar = true,
        .reference_buffers_enabled = true,
        .input_buffers_enabled = true,
        .burnout_current = AD7124_BURNOUT_OFF  // Start with diagnostics off
    };
    
    result = ad7124_configure_setup(driver_instance, 0, &setup_config);
    if (result == AD7124_OK) {
        printf("Setup 0 configured (Gain=1x, Bipolar, Buffers enabled)\n");
    }
    printf("\n");
    
    // ===== Embedded Measurements =====
    printf("=== Embedded Measurements ===\n");
    
    // Perform sample measurements
    for (int i = 0; i < 3; i++) {
        printf("Measurement %d:\n", i + 1);
        
        // Check data ready status
        if (ad7124_is_data_ready(driver_instance)) {
            // Read raw data
            uint32_t raw_data;
            result = ad7124_read_data(driver_instance, &raw_data);
            if (result == AD7124_OK) {
                printf("  Raw ADC: 0x%06X (%u)\n", raw_data & 0xFFFFFF, raw_data & 0xFFFFFF);
                
                // Convert to voltage (bipolar, 2.5V reference, gain=1)
                float voltage = ad7124_raw_to_voltage(raw_data, 2.5f, AD7124_GAIN_1, true);
                printf("  Voltage: %f V\n", voltage);
                printf("  Percentage: %.1f%% of reference\n", (voltage / 2.5f) * 100.0f);
            }
        }
        printf("\n");
    }
    
    // ===== Gain Setting Test =====
    printf("=== Gain Setting Test ===\n");
    
    ad7124_gain_t gains[] = {AD7124_GAIN_1, AD7124_GAIN_2, AD7124_GAIN_4, AD7124_GAIN_8};
    const char* gain_names[] = {"1x", "2x", "4x", "8x"};
    
    for (int i = 0; i < 4; i++) {
        printf("Testing gain %s:\n", gain_names[i]);
        
        // Update setup with new gain
        setup_config.pga_gain = gains[i];
        ad7124_configure_setup(driver_instance, 0, &setup_config);
        
        // Read measurement
        if (ad7124_is_data_ready(driver_instance)) {
            uint32_t raw_data;
            if (ad7124_read_data(driver_instance, &raw_data) == AD7124_OK) {
                float voltage = ad7124_raw_to_voltage(raw_data, 2.5f, gains[i], true);
                printf("  Voltage: %f V (raw: 0x%06X)\n", voltage, raw_data & 0xFFFFFF);
            }
        }
    }
    printf("\n");
    
    // ===== Sensor Diagnostics Test =====
    printf("=== Sensor Diagnostics Test ===\n");
    
    ad7124_burnout_current_t burnout_settings[] = {
        AD7124_BURNOUT_OFF, AD7124_BURNOUT_0_5UA, 
        AD7124_BURNOUT_2UA, AD7124_BURNOUT_4UA
    };
    const char* burnout_names[] = {"Off", "0.5µA", "2µA", "4µA"};
    
    for (int i = 0; i < 4; i++) {
        printf("Testing burnout current: %s\n", burnout_names[i]);
        
        setup_config.burnout_current = burnout_settings[i];
        result = ad7124_configure_setup(driver_instance, 0, &setup_config);
        if (result == AD7124_OK) {
            printf("  Sensor diagnostics configured\n");
            // In real application: analyze readings for sensor faults
        }
    }
    printf("\n");
    
    // ===== Digital Filtering Test =====
    printf("=== Digital Filtering Test ===\n");
    
    ad7124_filter_config_t filter_configs[] = {
        {AD7124_FILTER_SINC4, 50, false, true},      // 50Hz with 60Hz rejection
        {AD7124_FILTER_SINC3, 100, false, false},    // 100Hz fast response
        {AD7124_FILTER_FAST_SETTLE, 200, true, false} // Fast settling
    };
    const char* filter_names[] = {"SINC4 (50Hz)", "SINC3 (100Hz)", "Fast Settle (200Hz)"};
    
    for (int i = 0; i < 3; i++) {
        printf("Testing filter: %s\n", filter_names[i]);
        
        result = ad7124_configure_filter(driver_instance, 0, &filter_configs[i]);
        if (result == AD7124_OK) {
            printf("  Filter configured successfully\n");
            printf("  Data rate: %d Hz\n", filter_configs[i].output_data_rate);
            printf("  60Hz rejection: %s\n", filter_configs[i].reject_60hz ? "Enabled" : "Disabled");
        }
    }
    printf("\n");
    
    // ===== Final System Status =====
    embedded_system_diagnostics(driver_instance);
    
    // ===== Cleanup =====
    printf("=== Cleanup ===\n");
    result = ad7124_destroy_in_place(driver_instance);
    if (result == AD7124_OK) {
        printf("Driver cleaned up successfully\n");
    }
    printf("Static memory cleared\n\n");
    
    printf("=== Embedded Example Complete ===\n");
    return 0;
}