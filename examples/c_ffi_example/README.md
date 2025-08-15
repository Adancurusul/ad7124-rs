# AD7124 C FFI Example - Enhanced Features

This example demonstrates how to use the AD7124 driver from C code using the Foreign Function Interface (FFI). It showcases the enhanced channel management features.

## Example Features

### Core Features
- Driver instance management (allocation, initialization, cleanup)
- Device initialization and ID verification
- ADC configuration (operating modes, power modes, reference sources)
- Single-ended and differential measurements
- Temperature sensor reading
- Raw ADC data and voltage conversion
- Error handling and software reset

### Enhanced Features
- ✓ **Enhanced Channel Management**: Dynamic channel enabling/disabling
- ✓ **Real-time Status Checking**: Direct hardware register reads (no stale cache)
- ✓ **Channel-Specific Reading**: Read data from specific channels
- ✓ **Non-blocking Operations**: Check data ready status without blocking
- ✓ **Multi-Channel Operations**: Read multiple channels efficiently
- ✓ **Fast Data Reading**: High-speed data acquisition
- ✓ **Active Channel Detection**: Real-time active channel monitoring

## Key Code Patterns

```c
// Declare driver instance
AD7124_DECLARE_DRIVER_INSTANCE(driver_instance);

// Initialize driver
AD7124_INIT_DRIVER(driver_instance, &spi_interface, AD7124_DEVICE_AD7124_8);

// Enhanced features
ad7124_enable_channel(driver_instance, 0, true);           // Enable channel
bool enabled = ad7124_is_channel_enabled(driver_instance, 0); // Check status
ad7124_read_channel_voltage(driver_instance, 0, &voltage);  // Read specific channel
bool ready = ad7124_is_data_ready(driver_instance);          // Non-blocking check

// Multi-channel operations
uint8_t channels[] = {0, 2, 4};
float voltages[3];
ad7124_read_multi_voltage(driver_instance, channels, 3, voltages);

// Cleanup
ad7124_destroy_in_place(driver_instance);
```

## Building

### Prerequisites

- GCC compiler
- Make
- Rust toolchain (for building the library)

### Build Steps

1. **Build everything (Rust library + C example):**
   ```bash
   make
   ```

2. **Run the example:**
   ```bash
   make run
   ```

3. **Clean build artifacts:**
   ```bash
   make clean      # Clean everything
   make clean-c    # Clean only C artifacts
   ```

### Manual Build (Recommended)

For manual building:

1. **Build Rust Library:**
   ```bash
   cd ../..
   cargo build --release --features capi
   ```

2. **Compile C Example:**
   ```bash
   # Syntax check (recommended first step)
   gcc -I../../include -c -o main.o main.c
   
   # Full compilation (Windows)
   gcc -I../../include main.o ../../target/release/ad7124_rs.lib -lws2_32 -ladvapi32 -luserenv -o c_instance_example.exe
   
   # Alternative using -L parameter (may be needed on some systems)
   # gcc -I../../include -L../../target/release -lad7124_rs -lws2_32 -ladvapi32 -luserenv -o c_instance_example.exe main.c
   ```

3. **Run:**
   ```bash
   ./c_instance_example.exe
   ```

## Code Structure

The example implements mock SPI functions that simulate communication with an AD7124 device:

- `spi_write()` - Simulates SPI write operations
- `spi_read()` - Simulates SPI read operations  
- `spi_transfer()` - Simulates SPI transfer operations
- `delay_ms()` - Simulates delay functionality

### Hardware Context

The example uses a `hardware_context_t` structure to represent hardware resources:

```c
typedef struct {
    int spi_fd;        // SPI file descriptor
    int cs_gpio;       // Chip select GPIO pin  
    int reset_gpio;    // Reset GPIO pin
} hardware_context_t;
```

In a real application, you would:
- Open the SPI device (`/dev/spidev0.0` on Linux)
- Configure GPIO pins for chip select and reset
- Implement actual SPI communication

## Example Output

The example produces output showing both basic and enhanced features:

```
=== AD7124 C FFI Example - Enhanced Features ===

Driver instance requirements:
  Size: 64 bytes (compile-time constant)
  Alignment: 8 bytes (compile-time constant)

Using static instance buffer at 0x...

Driver initialized successfully

Device ID: 0x12
  -> AD7124-8 detected

Driver initialized: Yes
Device type: AD7124-8

=== Testing Enhanced Channel Management ===

Channel 0 enabled successfully
Channel 0 enabled status: YES
Channel 2 enabled successfully
Channel 2 enabled status: YES
Channel 4 enabled successfully
Channel 4 enabled status: YES
Current active channel: 0

=== Testing Channel-Specific Data Reading ===

Reading from Channel 0:
  Raw data: 0x801000 (8327168)
  Voltage: 0.001221 V

Reading from Channel 2:
  Raw data: 0x802000 (8331264)
  Voltage: 0.002441 V

=== Testing Non-blocking Data Ready Check ===

Data is ready! (attempt 1/5)
  Channel: 0, Data: 0x803000

=== Testing Multi-Channel Operations ===

Multi-channel raw data read:
  Channel 0: 0x804000 (8404992)
  Channel 2: 0x805000 (8409088)
  Channel 4: 0x806000 (8413184)

Multi-channel voltage read:
  Channel 0: 0.004883 V
  Channel 2: 0.006104 V
  Channel 4: 0.007324 V

Enabled channels scan (found 3 channels):
  Channel 0: 0x807000 (8417280)
  Channel 2: 0x808000 (8421376)
  Channel 4: 0x809000 (8425472)

=== Enhanced Features Example Completed Successfully ===

Demonstrated features:
✓ Enhanced channel management (enable/disable, status checking)
✓ Channel-specific data reading
✓ Non-blocking data ready checks
✓ Multi-channel operations
✓ Fast data reading
✓ Real-time hardware status reading (no stale cache)
```

## Integration with Real Hardware

To integrate with real hardware:

1. **Replace mock SPI functions** with actual hardware communication:
   ```c
   int spi_write(void* context, const uint8_t* data, size_t len) {
       hardware_context_t* hw = (hardware_context_t*)context;
       return write(hw->spi_fd, data, len) >= 0 ? 0 : -1;
   }
   ```

2. **Implement proper delay:**
   ```c
   int delay_ms(void* context, uint32_t ms) {
       usleep(ms * 1000);
       return 0;
   }
   ```

3. **Initialize hardware resources:**
   ```c
   hw_ctx.spi_fd = open("/dev/spidev0.0", O_RDWR);
   // Configure SPI mode, speed, etc.
   ```

## Memory Management

The example demonstrates two memory management approaches:

### 1. Placement New (Recommended for Embedded)

```c
// Allocate aligned memory
uint8_t* memory = aligned_alloc(align, size);

// Initialize driver in memory
ad7124_init_in_place(memory, size, &spi_interface, AD7124_DEVICE_AD7124_8);

// Use driver...

// Cleanup
ad7124_destroy_in_place(memory);
free(memory);
```

### 2. Heap Allocation (Not available in no_std builds)

```c
// Create driver (returns NULL in no_std builds)
ad7124_driver_t* driver = ad7124_create(&spi_interface, AD7124_DEVICE_AD7124_8);

// Use driver...

// Cleanup
ad7124_destroy(driver);
```

## Error Handling

All functions return integer error codes. The example includes a helper function to print human-readable error messages:

```c
void print_error(const char* operation, int error_code);
```

Common error codes:
- `AD7124_OK` (0) - Success
- `AD7124_NULL_POINTER` (-1) - Null pointer passed
- `AD7124_SPI_WRITE` (-2) - SPI write failed
- `AD7124_INVALID_PARAMETER` (-6) - Invalid parameter
- `AD7124_NOT_INITIALIZED` (-7) - Driver not initialized

## Thread Safety

The driver is not inherently thread-safe. If using multiple threads:

1. **Single driver instance per thread**, or
2. **Synchronize access** with mutexes/locks

## Performance Considerations

- Memory alignment is important for performance
- The driver size is 64 bytes (compile-time constant)
- SPI communication is the main bottleneck
- Consider SPI clock speed vs. noise requirements
- Enhanced features add minimal overhead for enhanced functionality
- Direct hardware reading eliminates stale cache issues
- Multi-channel operations reduce SPI transaction overhead

## Troubleshooting

### Compilation Issues

1. **Missing header:** Ensure `-I../../include` points to the correct header location
2. **Library not found:** Check that the Rust library built successfully
3. **Linking errors:** Make sure all required libraries are linked (`-lpthread -ldl -lm`)

### Runtime Issues

1. **Memory allocation fails:** Check available system memory
2. **SPI errors:** Verify SPI device permissions and configuration
3. **Device not responding:** Check hardware connections and power supply

