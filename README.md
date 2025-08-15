# AD7124 Driver

[![Crates.io](https://img.shields.io/crates/v/ad7124-rs.svg)](https://crates.io/crates/ad7124-rs)
[![Documentation](https://docs.rs/ad7124-rs/badge.svg)](https://docs.rs/ad7124-rs)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE)

A platform-independent Rust driver for the **Analog Devices AD7124 family** (AD7124-4/AD7124-8), precision 24-bit sigma-delta ADCs with integrated PGA and reference.

## Features

- **Zero Dependencies**: Core driver works without external dependencies
- **Optional embedded-hal Integration**: Seamless integration when needed
- **Sync & Async Support**: Both synchronous and asynchronous operations
- **`no_std` Compatible**: Perfect for embedded systems
- **C FFI Ready**: Prepared for C/C++ integration with full API support
- **Simple API**: Clean and intuitive interface
- **High Precision**: 24-bit resolution with low noise
- **Temperature Sensor**: Built-in temperature measurement
- **Embassy Ready**: Full async/await support with Embassy templates
- **Enhanced Channel Management**: Real-time status, non-blocking ops, multi-channel batch
- **Direct Hardware Access**: No stale cache, always accurate status readings

## Hardware Features

The AD7124 family supports:
- 24-bit resolution
- Up to 19,200 SPS sample rate (AD7124-8)
- Integrated PGA (1x to 128x gain)
- Internal 2.5V reference
- 4 channels (AD7124-4) or 8 channels (AD7124-8)
- 16 differential or 32 single-ended inputs
- Built-in temperature sensor
- Digital filter with 50/60Hz rejection
- SPI interface up to 5MHz
- Low power consumption

## Installation

Add to your `Cargo.toml`:

```toml
[dependencies]
ad7124-rs = "0.4.0"
```

Or with specific features:

```toml
[dependencies]
ad7124-rs = { version = "0.4.0", features = ["full-async", "defmt"] }
```

## Quick Start

### Basic Synchronous Usage

```rust
use ad7124_rs::{AD7124Sync, DeviceType, SpiTransport, ChannelInput};

// Create SPI transport (example with embedded-hal)
let transport = SpiTransport::new(spi, cs_pin, delay);

// Create and initialize driver
let mut adc = AD7124Sync::new(transport, DeviceType::AD7124_8)?;
adc.init()?;

// Setup single-ended measurement on channel 0
adc.setup_single_ended(0, ChannelInput::Ain0, 0)?;

// Read voltage
let voltage = adc.read_voltage(0)?;
println!("Voltage: {:.6}V", voltage);
```

### Asynchronous Usage with Embassy

```rust
use ad7124_rs::{AD7124Async, DeviceType, SpiTransport};
use embassy_time::{Duration, Timer};

// Create SPI transport with DMA
let spi = Spi::new(p.SPI2, sck, mosi, miso, 
                  p.DMA1_CH4, p.DMA1_CH3, spi_config);
let transport = SpiTransport::new(spi, cs_pin, delay);

// Create and initialize async driver
let mut adc = AD7124Async::new(transport, DeviceType::AD7124_8)?;
adc.init().await?;

// Setup differential measurement
adc.setup_differential(0, ChannelInput::Ain0, 
                      ChannelInput::Ain1, 0).await?;

// Continuous measurement loop
loop {
    let voltage = adc.read_voltage(0).await?;
    defmt::info!("Differential voltage: {:.6}V", voltage);
    Timer::after(Duration::from_millis(100)).await;
}
```

### Device Type Detection

```rust
use ad7124_rs::{AD7124Sync, DeviceType, SpiTransport};

let transport = SpiTransport::new(spi, cs, delay);

// Auto-detect device type
let mut adc = AD7124Sync::new_with_detection(transport)?;
adc.init()?;

// Or specify device type explicitly
let mut adc = AD7124Sync::new(transport, DeviceType::AD7124_4)?;
// Manual initialization when needed
adc.init()?;
```

### Advanced Configuration

```rust
use ad7124_rs::{
    AD7124Config, ChannelConfig, SetupConfig, FilterConfig,
    OperatingMode, PowerMode, ClockSource, ReferenceSource,
    PgaGain, FilterType, BurnoutCurrent
};

// Configure ADC control settings
let adc_config = AD7124Config {
    operating_mode: OperatingMode::Continuous,
    power_mode: PowerMode::FullPower,
    clock_source: ClockSource::Internal,
    reference_source: ReferenceSource::Internal,
    internal_ref_enabled: true,
    data_ready_output_enabled: true,
};
adc.configure_adc(adc_config).await?;

// Configure setup (PGA, reference, etc.)
let setup_config = SetupConfig {
    pga_gain: PgaGain::Gain64,
    reference_source: ReferenceSource::Internal,
    bipolar: true,
    burnout_current: BurnoutCurrent::Off,
    reference_buffers_enabled: true,
    input_buffers_enabled: true,
};
adc.configure_setup(0, setup_config).await?;

// Configure digital filter
let filter_config = FilterConfig {
    filter_type: FilterType::Sinc4,
    output_data_rate: 50, // 50 Hz
    single_cycle: false,
    reject_60hz: true,
};
adc.configure_filter(0, filter_config).await?;
```

### Enhanced Channel Management

The driver provides advanced channel management and data reading capabilities:

```rust
// Enhanced Channel Control
adc.enable_channel(0, true)?;                    // Enable/disable specific channel
let enabled = adc.is_channel_enabled(0)?;        // Check if channel is enabled (real-time)
let active = adc.get_active_channel()?;          // Get currently active channel
let current = adc.current_channel()?;            // Read current channel from status register

// Non-blocking Operations
if adc.is_data_ready()? {                        // Non-blocking data ready check
    let data = adc.read_data_fast()?;            // Fast read without status check
}
let (channel, data) = adc.read_data_with_status()?; // Read with channel info
adc.wait_conv_ready(Some(1000))?;                // Wait with optional timeout (ms)

// Channel-Specific Reading
let raw = adc.read_channel_data(2)?;             // Read specific channel raw data
let voltage = adc.read_channel_voltage(2)?;      // Read specific channel voltage

// Multi-Channel Operations
let channels = [0, 2, 4, 6];
let data = adc.read_multi_channel(&channels)?;   // Read multiple channels
let voltages = adc.read_multi_voltage(&channels)?; // Read multiple voltages
let enabled = adc.get_enabled_channels()?;       // Get all enabled channels
let scan = adc.scan_enabled_channels()?;         // Scan & read all enabled channels
```

#### Key Features

- **Real-time Hardware Access**: All status queries read directly from hardware registers (no stale cache)
- **Non-blocking Operations**: Check data ready status without blocking
- **Channel-Specific Operations**: Read from specific channels without manual switching
- **Batch Operations**: Efficiently read multiple channels in one call
- **Fast Data Access**: Bypass status checks for high-speed acquisition
- **✅ Dynamic Channel Management**: Enable/disable channels at runtime

## Examples

The repository includes comprehensive examples:

### Rust Examples
- `embassy_usage` - Complete Embassy async example with enhanced channel management features

### C FFI Examples
- `c_ffi_example` - Basic C FFI usage with static memory allocation
- `c_embedded_example` - Embedded-specific patterns with HAL simulation

Run examples with:

```bash
# Rust Embassy Example
cd examples/embassy_usage
cargo build --release

# C FFI Examples
cargo build --release --features capi

# Basic C FFI example
cd examples/c_ffi_example
gcc -I../../include -c -o main.o main.c
gcc -I../../include main.o ../../target/release/ad7124_driver.lib -lws2_32 -ladvapi32 -luserenv -o example.exe
./example.exe

# Embedded C example  
cd examples/c_embedded_example
gcc -I../../include -c -o main.o main.c
gcc -I../../include main.o ../../target/release/ad7124_driver.lib -lws2_32 -ladvapi32 -luserenv -o embedded_example.exe
./embedded_example.exe
```

## Features

### Available Features

- `embedded-hal` - embedded-hal integration (default)
- `embedded-hal-async` - async embedded-hal support
- `async` - async/await support (requires embedded-hal-async)
- `full-async` - complete async functionality
- `defmt` - defmt logging support
- `capi` - C FFI interface
- `panic-handler` - Basic panic handler for testing (applications should provide their own)

### Feature Combinations

- **Default**: `embedded-hal`
- **Full Async**: `full-async`
- **Embassy Ready**: `full-async`, `defmt`
- **Minimal**: `default-features = false`
- **C FFI**: `capi`

### Device Variants

- `ad7124-4` - Enable AD7124-4 specific features
- `ad7124-8` - Enable AD7124-8 specific features

## Architecture

The driver follows a modern layered architecture with core-transport separation:

```
┌─────────────────────────────────────┐
│          Application Layer           │
├─────────────────────────────────────┤
│      High-Level API Layer          │
├─────────────────────────────────────┤
│     AD7124Sync/Async Drivers        │
├─────────────────────────────────────┤
│         AD7124Core Logic            │
│       (Transport Independent)       │
├─────────────────────────────────────┤
│       Transport Abstraction         │
│    (SyncSpiTransport/AsyncSpi)      │
├─────────────────────────────────────┤
│    Hardware Implementation          │
│     (embedded-hal SPI)              │
└─────────────────────────────────────┘
```

### Key Architecture Benefits

- **Core-Transport Separation**: Business logic completely independent of transport layer
- **Unified Naming**: Family-consistent type naming (AD7124xxx throughout)
- **Simple Construction**: Direct driver construction for both sync and async
- **Three-Layer Error Model**: Core, Transport, and Pin errors cleanly separated
- **Embassy Ready**: Full async/await support with DMA templates


## C FFI Interface

The driver provides a complete C-compatible interface with **zero-allocation embedded-friendly design**:

### Key Features

- **Zero Heap Allocation**: All memory managed by C application
- **Static Memory Support**: Compile-time memory size calculation
- **Multi-Instance Support**: Independent driver instances with separate memory buffers
- **Embedded-Friendly**: Suitable for resource-constrained environments
- **Instance-Based Design**: Clear and intuitive API using "instance" terminology

### Quick Start

```c
#include "ad7124_ffi.h"

// Declare static driver instance (176 bytes, 8-byte aligned)
AD7124_DECLARE_DRIVER_INSTANCE(driver_instance);

// Setup SPI interface (no context parameter needed)
ad7124_spi_interface_t spi = {
    .write = my_spi_write,
    .read = my_spi_read, 
    .transfer = my_spi_transfer,
    .delay_ms = my_delay_ms
};

// Initialize driver in static memory
int result = AD7124_INIT_DRIVER(driver_instance, &spi, AD7124_DEVICE_AD7124_8);
if (result != AD7124_OK) {
    // Handle initialization error
}

// Configure and read
ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);
float voltage;
ad7124_read_voltage(driver_instance, 0, &voltage);

// Enhanced Channel Management
ad7124_enable_channel(driver_instance, 0, true);              // Enable channel
bool enabled = ad7124_is_channel_enabled(driver_instance, 0);  // Check status
uint8_t active_ch;
ad7124_get_active_channel(driver_instance, &active_ch);        // Get active channel

// Non-blocking Operations  
if (ad7124_is_data_ready(driver_instance)) {                   // Non-blocking check
    uint32_t data;
    ad7124_read_data_fast(driver_instance, &data);             // Fast read
}

// Channel-Specific Reading
uint32_t ch_data;
ad7124_read_channel_data(driver_instance, 2, &ch_data);        // Read channel 2
float ch_voltage;
ad7124_read_channel_voltage(driver_instance, 2, &ch_voltage);  // Channel 2 voltage

// Multi-Channel Operations
uint8_t channels[] = {0, 2, 4, 6};
uint32_t multi_data[4];
ad7124_read_multi_channel(driver_instance, channels, 4, multi_data);
float multi_voltages[4];
ad7124_read_multi_voltage(driver_instance, channels, 4, multi_voltages);

// Cleanup (memory managed by C)
ad7124_destroy_in_place(driver_instance);
```

### Memory Management

The FFI uses a **placement new** pattern for efficient memory management:

```c
// Convenience macros handle size and alignment automatically
#define AD7124_DRIVER_SIZE    176  // Compile-time constant
#define AD7124_DRIVER_ALIGN   8    // Required alignment

// Declare driver instance
AD7124_DECLARE_DRIVER_INSTANCE(my_driver);

// Initialize in existing memory buffer
AD7124_INIT_DRIVER(my_driver, &spi_interface, device_type);
```

### Multi-Instance Support

```c
// Multiple independent instances
AD7124_DECLARE_DRIVER_INSTANCE(adc1);
AD7124_DECLARE_DRIVER_INSTANCE(adc2);

// Different SPI interfaces
AD7124_INIT_DRIVER(adc1, &spi1_interface, AD7124_DEVICE_AD7124_4);
AD7124_INIT_DRIVER(adc2, &spi2_interface, AD7124_DEVICE_AD7124_8);

// Use independently
ad7124_read_voltage(adc1, 0, &voltage1);
ad7124_read_voltage(adc2, 0, &voltage2);
```

### Building C FFI

The `crate-type` is already configured for FFI builds:

```bash
# Build Rust static library
cargo build --release --features capi

# Compile C example (Windows)
cd examples/c_ffi_example
gcc -I../../include -c -o main.o main.c
gcc -I../../include main.o ../../target/release/ad7124_driver.lib -lws2_32 -ladvapi32 -luserenv -o example.exe

# Run example
./example.exe
```

### Available Examples

Two comprehensive C examples are provided:

1. **`c_ffi_example/`** - Basic C FFI usage with dynamic memory patterns
2. **`c_embedded_example/`** - Embedded-specific patterns with static allocation and HAL simulation

Both examples demonstrate:
- Static memory allocation and management
- Multi-instance usage patterns  
- Error handling
- Real embedded system integration patterns
- Hardware abstraction layer (HAL) integration

### Architecture Benefits

- **Zero Heap**: All memory allocated statically by C application
- **Fast**: Placement new pattern for zero-copy construction  
- **Safe**: Compile-time size and alignment checks
- **Compact**: Only 176 bytes per driver instance
- **Flexible**: Support for multiple independent instances
- **Embedded**: Designed for resource-constrained systems

### Platform Support

The C FFI has been tested on:
- **Embedded Systems** (ARM Cortex-M , RISC-V ...)
- **Windows** (MSVC/MinGW)
- **Linux** (GCC)


For detailed implementation guidance, see:
- [`FFI_USAGE_EN.md`](FFI_USAGE_EN.md) - User manual
- [`examples/c_ffi_example/README.md`](examples/c_ffi_example/README.md) - Basic usage
- [`examples/c_embedded_example/README.md`](examples/c_embedded_example/README.md) - Embedded patterns



## 📄 License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.

## 🔗 References

- [AD7124-4/AD7124-8 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7124-4_ad7124-8.pdf)
- [embedded-hal](https://github.com/rust-embedded/embedded-hal)
- [Embassy](https://embassy.dev/)

### Documentation
- [FFI Usage Manual (English)](./FFI_USAGE_EN.md) - Comprehensive C FFI user guide
- [FFI Usage Manual (Chinese)](./FFI_USAGE_CN.md) - 中文 C FFI 用户指南