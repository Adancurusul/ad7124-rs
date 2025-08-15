# AD7124 Embassy Example

This example demonstrates using the AD7124 driver with Embassy async framework on STM32G431CB.

## Features Demonstrated

### Core Features
- Async/await SPI communication with DMA
- Multiple measurement configurations cycling
- Single-ended and differential measurements
- Temperature sensor reading
- Internal reference measurement
- High-gain measurements
- Real-time status monitoring

### Enhanced Channel Management
- Dynamic channel enable/disable control
- Non-blocking data ready checking
- Channel-specific data reading
- Multi-channel batch operations
- Active channel status monitoring
- Fast data access methods
- Enabled channel scanning

## Hardware Setup

### STM32G431CB Pin Configuration

- **SPI2 Pins:**
  - SCK: PB13
  - MISO: PB15
  - MOSI: PB14
  - CS: PA12 (GPIO Output)
  
- **Control Pins:**
  - SYNC: PA11 (GPIO Output, synchronization)
  - LED: PC13 (Status indicator)

### AD7124 Connections

```
STM32G431CB         AD7124
-----------         ------
PB13 (SCK)    ----> SCLK
PB15 (MISO)   <---- DOUT/RDY
PB14 (MOSI)   ----> DIN
PA12 (CS)     ----> CS
PA11 (SYNC)   ----> SYNC
PC13 (LED)    ----> [Status LED]
GND           ----> GND
3.3V          ----> DVDD/AVDD
```

## Building and Running

### Prerequisites

- Rust nightly toolchain with thumbv7em-none-eabi target
- probe-rs for flashing and debugging
- STM32G431CB development board
- AD7124 evaluation board or custom PCB

### Build

```bash
cargo build --release
```

### Flash and Run

With probe-rs installed:

```bash
cargo run --release
```

Or manually:

```bash
probe-rs run --chip STM32G431CBTx target/thumbv7em-none-eabi/release/embassy_usage
```

### Debug Output

The example uses defmt for logging. Output will appear in the probe-rs console:

```
AD7124 Embassy Example Starting...
SPI configured successfully
Initializing AD7124...
AD7124 initialized successfully, Device ID: 0x12
ADC configuration complete

=== Enhanced Channel Management Demo ===
Configured channel 0
Configured channel 1
Configured channel 2
Configured channel 3
Setup configured for enhanced demo

--- Test 1: Channel Enable/Disable ---
Channel 0 enabled: true
Channel 1 enabled: true
Channel 2 enabled: true
Channel 3 enabled: true

--- Test 2: Active Channel Status ---
Current active channel: 0
Current channel (via status): 0

--- Test 3: Non-blocking Data Ready Check ---
Data is ready!
Read from channel 0: 0x800000

--- Test 4: Channel-Specific Reading ---
Channel 0 raw data: 0x800123
Channel 0 voltage: 2500mV
Channel 1 raw data: 0x800456
Channel 1 voltage: 2500mV
Channel 2 raw data: 0x800789
Channel 2 voltage: 2500mV

--- Test 5: Multi-Channel Operations ---
Multi-channel raw data:
  Channel 0: 0x800ABC
  Channel 1: 0x800DEF
  Channel 2: 0x801234
Multi-channel voltages:
  Channel 0: 2500mV
  Channel 1: 2500mV
  Channel 2: 2500mV

--- Test 6: Scan Enabled Channels ---
Enabled channels: [0, 1, 2, 3]
Scan results:
  Enabled channel 0: 0x801567
  Enabled channel 1: 0x801890
  Enabled channel 2: 0x801ABC
  Enabled channel 3: 0x801DEF

--- Test 7: Fast Data Read ---
Fast read data: 0x802000

=== Enhanced Channel Management Demo Complete ===

=== Single-ended AIN0 Demo ===
Setup configured for gain: 1
Filter configured
Channel configured: 0 - 18
First reading: 2500mV
Single-ended AIN0: 2500mV (avg of 10 readings)
Voltage: 2500mV (100% of reference)
Status: ready=true, error=false

=== Single-ended AIN1 Demo ===
...
```

## Configuration Options

### Modify Measurement Configurations

Edit the `configurations` array in `main.rs`:

```rust
let configurations = [
    ("Single-ended AIN0", ChannelInput::Ain0, ChannelInput::Dgnd, PgaGain::Gain1),
    // Add your own configurations here
];
```

### Adjust Sample Rate

Modify the filter configuration:

```rust
let filter_config = FilterConfig {
    filter_type: FilterType::Sinc4,
    output_data_rate: 50, // Change this value (5-19200 Hz)
    single_cycle: false,
    reject_60hz: true,
};
```

### Change SPI Speed

Edit the SPI frequency in main.rs:

```rust
spi_config.frequency = Hertz(1_000_000);  // Up to 5MHz supported
```

### Enhanced Channel Management APIs

The example demonstrates all new enhanced features:

```rust
// Channel enable/disable
adc.enable_channel(channel, true).await?;
let enabled = adc.is_channel_enabled(channel).await?;

// Active channel monitoring
let active = adc.get_active_channel().await?;
let current = adc.current_channel().await?;

// Non-blocking data ready check
if adc.is_data_ready().await? {
    let (channel, data) = adc.read_data_with_status().await?;
}

// Channel-specific reading
let raw_data = adc.read_channel_data(channel).await?;
let voltage = adc.read_channel_voltage(channel).await?;

// Multi-channel operations
let channels = [0, 1, 2];
let raw_results = adc.read_multi_channel(&channels).await?;
let voltage_results = adc.read_multi_voltage(&channels).await?;

// Channel scanning
let enabled_channels = adc.get_enabled_channels().await?;
let scan_results = adc.scan_enabled_channels().await?;

// Fast data access
let fast_data = adc.read_data_fast().await?;
```

## Power Consumption

The example runs in full power mode for maximum performance. For low power applications:

1. Use `PowerMode::LowPower` in ADC configuration
2. Reduce sample rate
3. Use single conversion mode instead of continuous
4. Implement proper sleep modes between conversions

## Customization

### Different MCU

1. Update `.cargo/config.toml` with your chip
2. Modify pin assignments in `main.rs`
3. Update `Cargo.toml` embassy-stm32 features

### Different Measurement Setup

1. Modify channel configurations
2. Adjust PGA gain for your signal range
3. Configure appropriate filter settings
4. Update reference source if using external reference

## License

This example is provided under the same license as the AD7124 driver.