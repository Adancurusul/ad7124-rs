# AD7124 FFI Interface User Manual

## Table of Contents
- [Overview](#overview)
- [Memory Management Principles](#memory-management-principles)
- [Convenience Macros](#convenience-macros)
- [Quick Start](#quick-start)
- [API Reference](#api-reference)
- [Usage Examples](#usage-examples)
- [Multi-Instance Management](#multi-instance-management)
- [FAQ](#faq)

## Overview

The AD7124 FFI (Foreign Function Interface) provides C/C++ applications with the ability to use the AD7124 driver written in Rust. The interface is designed to be simple, requires no dynamic memory allocation, and is particularly suitable for embedded systems.

### Key Features
- ✅ Zero heap allocation design - No malloc/free in Rust
- ✅ Simplified interface without context parameters
- ✅ Support for all AD7124 features
- ✅ Multi-instance support - Independent memory per instance
- ✅ Memory location control - Can be placed in specific memory sections
- ✅ Compile-time memory requirements - No runtime allocation failures
- ✅ Cross-platform support (Windows/Linux/Embedded)

## Memory Management Principles

### Why Do Users Need to Provide Memory?

Traditional driver libraries typically use dynamic memory allocation (malloc/free), but this poses problems in embedded systems:
- ❌ Many embedded systems disable heap allocation
- ❌ Dynamic allocation can lead to memory fragmentation
- ❌ Allocation failures are unpredictable and hard to handle
- ❌ Non-deterministic execution times

**Our Solution:**
```c
// C side allocates memory (stack or global)
uint8_t driver_instance[64] __attribute__((aligned(8)));

// Rust constructs objects in this memory
ad7124_init_in_place(driver_instance, sizeof(driver_instance), ...);

// All subsequent operations use this memory
ad7124_read_voltage(driver_instance, ...);
```

### Memory Usage Flow

1. **C Allocation** - 64 bytes of raw memory
2. **Rust Reception** - Convert `*mut u8` to `*mut AD7124Sync<CFfiTransport>`
3. **Object Construction** - Use `ptr::write()` to construct Rust object in specified memory
4. **Normal Usage** - All function calls operate on the object in this memory
5. **Cleanup** - Call destructor but don't free memory (memory belongs to C)

### Benefits Summary

| Feature | Traditional Approach | Our Approach |
|---------|---------------------|-------------|
| Dynamic Allocation | malloc/free | None |
| Memory Location | Uncontrolled (heap) | Controlled (stack/BSS/specific sections) |
| Multi-instance | Supported | Supported and simpler |
| Embedded Friendly | ❌ | ✅ |
| Execution Time | Non-deterministic | Deterministic |
| Memory Leak Risk | Exists | None |

## Convenience Macros

To simplify memory management, we provide a series of macros to automatically handle size and alignment:

### Compile-time Constants
```c
#define AD7124_DRIVER_SIZE 64      // Required bytes for driver
#define AD7124_DRIVER_ALIGN 8      // Memory alignment requirement
```

### Automatic Memory Declaration Macros

#### `AD7124_DECLARE_DRIVER_INSTANCE(name)`
Declare static instance buffer:
```c
// Using macro (recommended)
AD7124_DECLARE_DRIVER_INSTANCE(my_driver);

// Equivalent to manual approach
static uint8_t my_driver[64] __attribute__((aligned(8)));
```

#### `AD7124_DECLARE_GLOBAL_DRIVER_INSTANCE(name)`
Declare global instance buffer:
```c
// In .h file declaration
extern uint8_t global_driver[64];

// In .c file definition (using macro)
AD7124_DECLARE_GLOBAL_DRIVER_INSTANCE(global_driver);
```

### Automatic Initialization Macro

#### `AD7124_INIT_DRIVER(instance, spi_interface, device_type)`
Automatically calculate instance size and initialize:
```c
// Using macro (recommended)
AD7124_INIT_DRIVER(driver_instance, &spi_interface, AD7124_DEVICE_AD7124_8);

// Equivalent to manual approach
ad7124_init_in_place(driver_instance, sizeof(driver_instance), 
                     &spi_interface, AD7124_DEVICE_AD7124_8);
```

### Macro Advantages
- ✅ **Automatic size calculation** - No need to remember 64 bytes
- ✅ **Automatic alignment handling** - Compiler ensures correct alignment
- ✅ **Type safety** - Compile-time checking
- ✅ **Code simplicity** - Reduces boilerplate code
- ✅ **Forward compatibility** - If size changes in future, just recompile

## Quick Start

### 1. Include Header File

```c
#include "ad7124_ffi.h"
```

### 2. Implement SPI Interface Functions

You need to implement the following four functions to provide hardware access:

```c
// SPI write function
int spi_write(const uint8_t* data, size_t len) {
    // Implement your SPI write logic
    // Return 0 for success, negative for error
    return 0;
}

// SPI read function
int spi_read(uint8_t* data, size_t len) {
    // Implement your SPI read logic
    // Return 0 for success, negative for error
    return 0;
}

// SPI transfer function (simultaneous read/write)
int spi_transfer(uint8_t* read_data, const uint8_t* write_data, size_t len) {
    // Implement your SPI transfer logic
    // Return 0 for success, negative for error
    return 0;
}

// Delay function
int delay_ms(uint32_t ms) {
    // Implement millisecond delay
    // Return 0 for success, negative for error
    return 0;
}
```

### 3. Initialize Driver

#### Method 1: Using Convenience Macros (Recommended)
```c
// 1. Declare driver instance (automatically handles size and alignment)
AD7124_DECLARE_DRIVER_INSTANCE(driver_instance);

// 2. Configure SPI interface
ad7124_spi_interface_t spi_interface = {
    .write = spi_write,
    .read = spi_read,
    .transfer = spi_transfer,
    .delay_ms = delay_ms
};

// 3. Initialize driver (automatically passes correct size)
int result = AD7124_INIT_DRIVER(driver_instance, &spi_interface, AD7124_DEVICE_AD7124_8);
if (result != AD7124_OK) {
    // Handle error
}

// 4. Initialize AD7124 device
result = ad7124_init(driver_instance);
```

#### Method 2: Manual Method (Full Control)
```c
// 1. Manual instance allocation
uint8_t driver_instance[AD7124_DRIVER_SIZE] __attribute__((aligned(AD7124_DRIVER_ALIGN)));

// 2. Configure SPI interface
ad7124_spi_interface_t spi_interface = { /* ... */ };

// 3. Manual initialization
int result = ad7124_init_in_place(
    driver_instance, 
    sizeof(driver_instance),
    &spi_interface, 
    AD7124_DEVICE_AD7124_8
);

// 4. Initialize device
result = ad7124_init(driver_instance);
```
```

### 4. Configure and Use

```c
// Configure ADC
ad7124_config_t adc_config = {
    .operating_mode = AD7124_MODE_CONTINUOUS,
    .power_mode = AD7124_POWER_FULL,
    .reference_source = AD7124_REF_INTERNAL,
    .internal_ref_enabled = true,
    .data_ready_output_enabled = true
};
ad7124_configure_adc(driver_instance, &adc_config);

// Setup single-ended measurement channel
ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);

// Read voltage
float voltage;
ad7124_read_voltage(driver_instance, 0, &voltage);
printf("Voltage: %.6f V\n", voltage);
```

### 5. Cleanup

```c
// Destroy driver (only calls destructor, doesn't free instance)
ad7124_destroy_in_place(driver_instance);

// Static instance is automatically reclaimed, no manual freeing needed
// Only need free() if you used dynamic allocation
```

## API Reference

### Memory Management Functions

#### `ad7124_get_driver_size`
Get the required memory size for the driver.
```c
size_t ad7124_get_driver_size(void);
```

#### `ad7124_get_driver_align`
Get the required memory alignment for the driver.
```c
size_t ad7124_get_driver_align(void);
```

#### `ad7124_init_in_place`
Initialize the driver in the provided memory location.
```c
int ad7124_init_in_place(
    uint8_t* memory,                           // Memory pointer
    size_t memory_size,                        // Memory size
    const ad7124_spi_interface_t* spi_interface, // SPI interface
    ad7124_device_type_t device_type          // Device type
);
```

#### `ad7124_destroy_in_place`
Destroy the driver instance.
```c
int ad7124_destroy_in_place(uint8_t* memory);
```

### Device Control Functions

#### `ad7124_init`
Initialize the AD7124 device.
```c
int ad7124_init(uint8_t* driver);
```

#### `ad7124_reset`
Software reset the device.
```c
int ad7124_reset(uint8_t* driver);
```

#### `ad7124_read_device_id`
Read the device ID.
```c
int ad7124_read_device_id(uint8_t* driver, uint8_t* device_id);
```

#### `ad7124_is_initialized`
Check if the driver is initialized.
```c
bool ad7124_is_initialized(const uint8_t* driver);
```

### ADC Configuration Functions

#### `ad7124_configure_adc`
Configure ADC basic parameters.
```c
int ad7124_configure_adc(uint8_t* driver, const ad7124_config_t* config);
```

Configuration structure:
```c
typedef struct {
    ad7124_operating_mode_t operating_mode;     // Operating mode
    ad7124_power_mode_t power_mode;             // Power mode
    ad7124_reference_source_t reference_source; // Reference source
    bool internal_ref_enabled;                  // Internal reference enable
    bool data_ready_output_enabled;             // Data ready output enable
} ad7124_config_t;
```

### Channel Configuration Functions

#### `ad7124_setup_single_ended`
Configure single-ended measurement channel.
```c
int ad7124_setup_single_ended(
    uint8_t* driver,
    uint8_t channel,                    // Channel number (0-15 or 0-7)
    ad7124_channel_input_t input,       // Input pin
    uint8_t setup                       // Setup index (0-7)
);
```

#### `ad7124_setup_differential`
Configure differential measurement channel.
```c
int ad7124_setup_differential(
    uint8_t* driver,
    uint8_t channel,                    // Channel number
    ad7124_channel_input_t positive,    // Positive input
    ad7124_channel_input_t negative,    // Negative input
    uint8_t setup                       // Setup index
);
```

#### `ad7124_configure_channel`
Configure channel detailed parameters.
```c
int ad7124_configure_channel(
    uint8_t* driver,
    uint8_t channel,
    const ad7124_channel_config_t* config
);
```

### Setup Configuration Functions

#### `ad7124_configure_setup`
Configure setup parameters (gain, reference source, etc.).
```c
int ad7124_configure_setup(
    uint8_t* driver,
    uint8_t setup,                      // Setup index (0-7)
    const ad7124_setup_config_t* config
);
```

Setup structure:
```c
typedef struct {
    ad7124_gain_t pga_gain;                     // PGA gain
    ad7124_reference_source_t reference_source; // Reference source
    bool bipolar;                               // Bipolar mode
    bool reference_buffers_enabled;             // Reference buffers enable
    bool input_buffers_enabled;                // Input buffers enable
    ad7124_burnout_current_t burnout_current;   // Sensor diagnostics current
} ad7124_setup_config_t;
```

Burnout current options:
```c
typedef enum {
    AD7124_BURNOUT_OFF = 0,      // No burnout current
    AD7124_BURNOUT_0_5UA = 1,    // 0.5 µA diagnostic current
    AD7124_BURNOUT_2UA = 2,      // 2 µA diagnostic current
    AD7124_BURNOUT_4UA = 3,      // 4 µA diagnostic current
} ad7124_burnout_current_t;
```

#### `ad7124_configure_filter`
Configure digital filter for signal processing.
```c
int ad7124_configure_filter(
    uint8_t* driver,
    uint8_t setup,                      // Setup index (0-7)
    const ad7124_filter_config_t* config
);
```

Filter configuration structure:
```c
typedef struct {
    ad7124_filter_type_t filter_type;          // Digital filter type
    uint16_t output_data_rate;                  // Data rate (0-2047 Hz)
    bool single_cycle;                          // Single cycle mode
    bool reject_60hz;                           // 60Hz rejection
} ad7124_filter_config_t;
```

Filter type options:
```c
typedef enum {
    AD7124_FILTER_SINC4 = 0,        // SINC4 filter (highest precision)
    AD7124_FILTER_SINC3 = 5,        // SINC3 filter (faster response)
    AD7124_FILTER_FAST_SETTLE = 4,  // Fast settling filter
} ad7124_filter_type_t;
```

### Data Reading Functions

#### `ad7124_wait_for_data_ready`
Wait for data ready.
```c
int ad7124_wait_for_data_ready(uint8_t* driver, uint32_t timeout_ms);
```

#### `ad7124_is_data_ready`
Check if data is ready (non-blocking).
```c
bool ad7124_is_data_ready(uint8_t* driver);
```

#### `ad7124_read_data`
Read raw ADC data.
```c
int ad7124_read_data(uint8_t* driver, uint32_t* data);
```

#### `ad7124_read_voltage`
Read voltage value.
```c
int ad7124_read_voltage(uint8_t* driver, uint8_t channel, float* voltage);
```

#### `ad7124_raw_to_voltage`
Convert raw data to voltage.
```c
float ad7124_raw_to_voltage(
    uint32_t raw_data,
    float reference_voltage,
    ad7124_gain_t gain,
    bool bipolar
);
```

### Enhanced Channel Management Functions

#### `ad7124_enable_channel`
Enable or disable specified channel.
```c
int ad7124_enable_channel(uint8_t* driver, uint8_t channel, bool enable);
```

#### `ad7124_is_channel_enabled`
Check if channel is enabled.
```c
bool ad7124_is_channel_enabled(uint8_t* driver, uint8_t channel);
```

#### `ad7124_get_active_channel`
Get currently active channel.
```c
int ad7124_get_active_channel(uint8_t* driver, uint8_t* channel);
```

#### `ad7124_read_channel_data`
Read raw data from specified channel.
```c
int ad7124_read_channel_data(uint8_t* driver, uint8_t channel, uint32_t* data);
```

#### `ad7124_read_channel_voltage`
Read voltage from specified channel.
```c
int ad7124_read_channel_voltage(uint8_t* driver, uint8_t channel, float* voltage);
```

### Calibration Functions

#### `ad7124_calibrate_internal_zero`
Perform internal zero-scale calibration.
```c
int ad7124_calibrate_internal_zero(uint8_t* driver, uint8_t setup);
```

#### `ad7124_calibrate_internal_full`
Perform internal full-scale calibration.
```c
int ad7124_calibrate_internal_full(uint8_t* driver, uint8_t setup);
```

## Usage Examples

### Example 1: Basic Voltage Measurement

```c
#include <stdio.h>
#include <stdint.h>
#include "ad7124_ffi.h"

// Global hardware state
static struct {
    int spi_fd;
    int cs_pin;
} hw = {-1, 10};

// Implement SPI functions
int spi_write(const uint8_t* data, size_t len) {
    // Your SPI write implementation
    return 0;
}

int spi_read(uint8_t* data, size_t len) {
    // Your SPI read implementation
    return 0;
}

int spi_transfer(uint8_t* read_data, const uint8_t* write_data, size_t len) {
    // Your SPI transfer implementation
    return 0;
}

int delay_ms(uint32_t ms) {
    // Your delay implementation
    return 0;
}

int main(void) {
    // Static driver memory allocation
    static uint8_t driver_instance[128] __attribute__((aligned(8)));
    
    // Configure SPI interface
    ad7124_spi_interface_t spi = {
        .write = spi_write,
        .read = spi_read,
        .transfer = spi_transfer,
        .delay_ms = delay_ms
    };
    
    // Initialize driver
    if (ad7124_init_in_place(driver_instance, sizeof(driver_instance), 
                             &spi, AD7124_DEVICE_AD7124_8) != AD7124_OK) {
        printf("Driver initialization failed\n");
        return 1;
    }
    
    // Initialize device
    if (ad7124_init(driver_instance) != AD7124_OK) {
        printf("Device initialization failed\n");
        return 1;
    }
    
    // Configure ADC
    ad7124_config_t config = {
        .operating_mode = AD7124_MODE_CONTINUOUS,
        .power_mode = AD7124_POWER_FULL,
        .reference_source = AD7124_REF_INTERNAL,
        .internal_ref_enabled = true,
        .data_ready_output_enabled = true
    };
    ad7124_configure_adc(driver_instance, &config);
    
    // Configure channel 0 for AIN0 single-ended measurement
    ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);
    
    // Configure setup 0
    ad7124_setup_config_t setup = {
        .pga_gain = AD7124_GAIN_1,
        .reference_source = AD7124_REF_INTERNAL,
        .bipolar = true,
        .reference_buffers_enabled = true,
        .input_buffers_enabled = true
    };
    ad7124_configure_setup(driver_instance, 0, &setup);
    
    // Read voltage 10 times
    for (int i = 0; i < 10; i++) {
        float voltage;
        if (ad7124_wait_for_data_ready(driver_instance, 1000) == AD7124_OK) {
            if (ad7124_read_voltage(driver_instance, 0, &voltage) == AD7124_OK) {
                printf("Measurement %d: %.6f V\n", i + 1, voltage);
            }
        }
    }
    
    // Cleanup
    ad7124_destroy_in_place(driver_instance);
    
    return 0;
}
```

### Example 2: Differential Measurement

```c
// Configure differential measurement (AIN0 - AIN1)
ad7124_setup_differential(driver_instance, 0, AD7124_AIN0, AD7124_AIN1, 0);

// Set gain to 8x for better small signal measurement
ad7124_setup_config_t setup = {
    .pga_gain = AD7124_GAIN_8,
    .reference_source = AD7124_REF_INTERNAL,
    .bipolar = true,
    .reference_buffers_enabled = true,
    .input_buffers_enabled = true
};
ad7124_configure_setup(driver_instance, 0, &setup);
```

### Example 3: Temperature Measurement

```c
// Configure internal temperature sensor
ad7124_setup_single_ended(driver_instance, 0, AD7124_TEMP_SENSOR, 0);

// Read temperature sensor voltage
float temp_voltage;
ad7124_read_voltage(driver_instance, 0, &temp_voltage);

// Convert to temperature (approximate formula)
float temperature = 25.0f + (temp_voltage - 1.17f) / 0.0018f;
printf("Temperature: %.1f °C\n", temperature);
```

### Example 4: Multi-Channel Scanning

```c
// Configure multiple channels
ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);
ad7124_setup_single_ended(driver_instance, 1, AD7124_AIN1, 0);
ad7124_setup_single_ended(driver_instance, 2, AD7124_AIN2, 0);
ad7124_setup_single_ended(driver_instance, 3, AD7124_AIN3, 0);

// Enable all channels
for (int ch = 0; ch < 4; ch++) {
    ad7124_enable_channel(driver_instance, ch, true);
}

// Check which channels are enabled
printf("Enabled channels: ");
for (int ch = 0; ch < 4; ch++) {
    if (ad7124_is_channel_enabled(driver_instance, ch)) {
        printf("%d ", ch);
    }
}
printf("\n");

// Read all channels (using channel-specific reading)
for (int ch = 0; ch < 4; ch++) {
    float voltage;
    if (ad7124_read_channel_voltage(driver_instance, ch, &voltage) == AD7124_OK) {
        printf("Channel %d: %.6f V\n", ch, voltage);
    }
}
```

### Example 5: Enhanced Channel Management

```c
// Dynamic channel control example
uint8_t channels_to_read[] = {0, 2, 4, 6};
size_t channel_count = sizeof(channels_to_read) / sizeof(channels_to_read[0]);

// Enable only needed channels
for (size_t i = 0; i < channel_count; i++) {
    ad7124_enable_channel(driver_instance, channels_to_read[i], true);
}

// Disable unneeded channels (save power)
for (int ch = 0; ch < 8; ch++) {
    bool should_enable = false;
    for (size_t i = 0; i < channel_count; i++) {
        if (channels_to_read[i] == ch) {
            should_enable = true;
            break;
        }
    }
    if (!should_enable) {
        ad7124_enable_channel(driver_instance, ch, false);
    }
}

// Read multiple channel data
for (size_t i = 0; i < channel_count; i++) {
    uint32_t raw_data;
    float voltage;
    
    if (ad7124_read_channel_data(driver_instance, channels_to_read[i], &raw_data) == AD7124_OK) {
        printf("Channel %d raw: 0x%08X\n", channels_to_read[i], raw_data);
        
        // Also read as voltage
        if (ad7124_read_channel_voltage(driver_instance, channels_to_read[i], &voltage) == AD7124_OK) {
            printf("Channel %d voltage: %.6f V\n", channels_to_read[i], voltage);
        }
    }
}
```

### Example 6: Data Ready Checking

```c
// Non-blocking data reading
while (true) {
    if (ad7124_is_data_ready(driver_instance)) {
        uint8_t active_channel;
        uint32_t data;
        
        // Get current active channel
        if (ad7124_get_active_channel(driver_instance, &active_channel) == AD7124_OK) {
            printf("Channel %d data ready\n", active_channel);
            
            // Read data
            if (ad7124_read_data(driver_instance, &data) == AD7124_OK) {
                printf("  Raw data: 0x%08X\n", data);
                
                // Convert to voltage
                float voltage = ad7124_raw_to_voltage(data, 2.5f, AD7124_GAIN_1, true);
                printf("  Voltage: %.6f V\n", voltage);
            }
        }
        
        // Other processing...
        break;
    }
    
    // Do other work, avoid blocking
    usleep(1000); // 1ms delay
}
```

## Multi-Instance Management

### Why Support Multiple Instances?

In practical applications, you might need to control multiple AD7124 devices simultaneously:
- Multi-channel sensor data acquisition
- Primary sensor + calibration device
- Different precision requirements for measurement channels

### Multi-Instance Usage Example

```c
// Declare three independent driver memories
AD7124_DECLARE_DRIVER_MEMORY(sensor1_driver);     // Sensor 1
AD7124_DECLARE_DRIVER_MEMORY(sensor2_driver);     // Sensor 2
AD7124_DECLARE_DRIVER_MEMORY(calibrator_driver);  // Calibrator

// Configure different SPI interfaces (if using different SPI buses)
ad7124_spi_interface_t spi1_interface = { /* SPI1 config */ };
ad7124_spi_interface_t spi2_interface = { /* SPI2 config */ };
ad7124_spi_interface_t spi3_interface = { /* SPI3 config */ };

// Initialize separately
AD7124_INIT_DRIVER(sensor1_driver, &spi1_interface, AD7124_DEVICE_AD7124_8);
AD7124_INIT_DRIVER(sensor2_driver, &spi2_interface, AD7124_DEVICE_AD7124_4);
AD7124_INIT_DRIVER(calibrator_driver, &spi3_interface, AD7124_DEVICE_AD7124_8);

// Configure separately (each device can have different settings)
ad7124_config_t high_speed_config = {
    .operating_mode = AD7124_MODE_CONTINUOUS,
    .power_mode = AD7124_POWER_FULL,
    // ...
};

ad7124_config_t low_power_config = {
    .operating_mode = AD7124_MODE_SINGLE_CONV,
    .power_mode = AD7124_POWER_LOW,
    // ...
};

ad7124_configure_adc(sensor1_driver, &high_speed_config);    // High-speed continuous
ad7124_configure_adc(sensor2_driver, &low_power_config);     // Low-power single-shot
ad7124_configure_adc(calibrator_driver, &high_speed_config); // High-precision calibration

// Use simultaneously
float voltage1, voltage2, cal_voltage;
ad7124_read_voltage(sensor1_driver, 0, &voltage1);      // Read from sensor 1
ad7124_read_voltage(sensor2_driver, 0, &voltage2);      // Read from sensor 2
ad7124_read_voltage(calibrator_driver, 0, &cal_voltage); // Read from calibrator

printf("Sensor 1: %.3f V\n", voltage1);
printf("Sensor 2: %.3f V\n", voltage2);
printf("Calibration: %.6f V\n", cal_voltage);

// Clean up separately
ad7124_destroy_in_place(sensor1_driver);
ad7124_destroy_in_place(sensor2_driver);
ad7124_destroy_in_place(calibrator_driver);
```

### Multiple Devices Sharing SPI Bus

If multiple AD7124 devices share the same SPI bus (using different chip select signals):

```c
// Two devices share SPI interface functions but control chip select internally
static int current_device = 0;  // Currently selected device

int spi_write(const uint8_t* data, size_t len) {
    switch(current_device) {
        case 0: set_cs1_low(); break;   // Device 1 chip select
        case 1: set_cs2_low(); break;   // Device 2 chip select
    }
    // SPI transmission logic
    spi_transmit(data, len);
    set_all_cs_high();  // Release all chip selects
    return 0;
}

// Switch device when using
void select_device(int device) {
    current_device = device;
}

// Usage example
select_device(0);
ad7124_read_voltage(device1_driver, 0, &voltage1);

select_device(1);
ad7124_read_voltage(device2_driver, 0, &voltage2);
```

### Memory Usage Statistics

```c
printf("Memory usage statistics:\n");
printf("  Single driver: %d bytes\n", AD7124_DRIVER_SIZE);
printf("  Three instances total: %d bytes\n", AD7124_DRIVER_SIZE * 3);
printf("  Memory address distribution:\n");
printf("    Sensor 1:    %p\n", (void*)sensor1_driver);
printf("    Sensor 2:    %p\n", (void*)sensor2_driver);
printf("    Calibrator:  %p\n", (void*)calibrator_driver);
```

## FAQ

### Q1: Why is the memory parameter required?

**Reasons:**
1. **Zero dynamic allocation** - Rust side doesn't call malloc, all memory provided by C
2. **Memory location control** - Can be placed on stack, BSS, or specific memory sections (like CCRAM)
3. **Multi-instance support** - Different memory = different instances, simple and intuitive
4. **Embedded friendly** - Compile-time determined memory usage, no runtime allocation failures

**Usage recommendations:**
- Single device: Use `AD7124_DECLARE_DRIVER_MEMORY(driver)`
- Multiple devices: Each device uses independent memory buffer
- Special requirements: Manual allocation and specify memory section

### Q2: How to handle errors?

All functions return error codes. It's recommended to wrap error handling:

```c
void check_error(int result, const char* operation) {
    if (result != AD7124_OK) {
        const char* error_msg;
        switch (result) {
            case AD7124_NULL_POINTER: 
                error_msg = "Null pointer"; 
                break;
            case AD7124_SPI_WRITE: 
                error_msg = "SPI write error"; 
                break;
            case AD7124_TIMEOUT: 
                error_msg = "Timeout"; 
                break;
            // ... other errors
            default: 
                error_msg = "Unknown error";
        }
        printf("Error [%s]: %s (%d)\n", operation, error_msg, result);
    }
}

// Usage
check_error(ad7124_init(driver_instance), "initialization");
```

### Q3: How to optimize performance?

1. **Use continuous mode**: Avoid frequent mode switching
2. **Batch reading**: Read multiple channel data at once
3. **Configure filters properly**: Adjust output data rate based on application needs
4. **Use DMA**: Implement DMA transfers in SPI functions

### Q4: How to debug?

1. **Verify memory size matching**:
```c
size_t actual_size = ad7124_get_driver_size();
if (actual_size != AD7124_DRIVER_SIZE) {
    printf("Warning: Memory size mismatch! Actual: %zu, Expected: %d\n", 
           actual_size, AD7124_DRIVER_SIZE);
}
```

2. **Check device ID**: Confirm hardware connection is correct
```c
uint8_t device_id;
ad7124_read_device_id(driver_instance, &device_id);
printf("Device ID: 0x%02X\n", device_id);  // Should be 0x04 or 0x12
```

3. **Memory address checking**:
```c
printf("Driver memory address: %p\n", (void*)driver_instance);
printf("Memory alignment check: %s\n", 
       ((uintptr_t)driver_instance % AD7124_DRIVER_ALIGN == 0) ? "OK" : "Error");
```

4. **Monitor SPI communication**: Add logging in SPI functions
5. **Multi-instance debugging**: Ensure different instances use different memory addresses

### Q5: What are the memory requirements?

- **Driver memory**: 64 bytes (compile-time constant `AD7124_DRIVER_SIZE`)
- **Alignment requirement**: 8 bytes (compile-time constant `AD7124_DRIVER_ALIGN`)
- **Stack usage**: Minimal, suitable for resource-constrained systems
- **Per instance independence**: Each instance occupies 64 bytes in multi-instance scenarios

**Memory composition:**
```
AD7124Sync<CFfiTransport> (64 bytes) = {
  transport: CFfiTransport (32 bytes) {
    interface: { 4 function pointers, 8 bytes each }
  },
  core: AD7124Core (32 bytes) {
    device_type, capabilities, config, 
    initialized, reference_voltage, crc_enabled
    + alignment padding
  }
}
```

## Error Code Reference

| Error Code | Value | Description |
|------------|-------|-------------|
| AD7124_OK | 0 | Success |
| AD7124_NULL_POINTER | -1 | Null pointer error |
| AD7124_SPI_WRITE | -2 | SPI write error |
| AD7124_SPI_READ | -3 | SPI read error |
| AD7124_SPI_TRANSFER | -4 | SPI transfer error |
| AD7124_INVALID_CHANNEL | -5 | Invalid channel |
| AD7124_INVALID_PARAMETER | -6 | Invalid parameter |
| AD7124_NOT_INITIALIZED | -7 | Not initialized |
| AD7124_DEVICE_NOT_RESPONDING | -8 | Device not responding |
| AD7124_CALIBRATION_FAILED | -9 | Calibration failed |
| AD7124_CONVERSION_TIMEOUT | -10 | Conversion timeout |
| AD7124_INVALID_DATA_LENGTH | -11 | Invalid data length |
| AD7124_INVALID_DEVICE_ID | -12 | Invalid device ID |
| AD7124_TIMEOUT | -13 | Timeout |
| AD7124_INVALID_CONFIGURATION | -14 | Invalid configuration |

## Support and Contribution

## Advanced Topics

### Memory Section Control

In some embedded systems, you might want to place the driver in specific memory sections:

```c
// Place in CCRAM (STM32 tightly coupled memory)
__attribute__((section(".ccram")))
AD7124_DECLARE_DRIVER_MEMORY(ccram_driver);

// Place in DMA-accessible area
__attribute__((section(".dma_buffer")))
AD7124_DECLARE_DRIVER_MEMORY(dma_driver);

// Place in fast access area
__attribute__((section(".itcm")))
AD7124_DECLARE_DRIVER_MEMORY(fast_driver);
```

### Integration with RTOS

```c
// FreeRTOS task example
void sensor_task(void* parameter) {
    // Each task has independent driver instance
    AD7124_DECLARE_DRIVER_MEMORY(task_driver);
    
    AD7124_INIT_DRIVER(task_driver, &spi_interface, AD7124_DEVICE_AD7124_8);
    ad7124_init(task_driver);
    
    while(1) {
        float voltage;
        ad7124_read_voltage(task_driver, 0, &voltage);
        
        // Send to queue or semaphore
        xQueueSend(voltage_queue, &voltage, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Compile-time Checks

```c
// Ensure memory size is sufficient at compile time
_Static_assert(sizeof(driver_instance) >= AD7124_DRIVER_SIZE, 
               "Driver memory too small");
_Static_assert(AD7124_DRIVER_SIZE == 64, 
               "Expected driver size changed");
```

For questions or suggestions, please visit the project repository to submit Issues or Pull Requests.

Author: Adancurusul  
License: MIT