# Makefile for AD7124 Embedded C Example
# This Makefile simulates embedded development environment

# Cross-compiler settings (example for ARM Cortex-M)
# Uncomment these lines for actual embedded compilation:
# CC = arm-none-eabi-gcc
# OBJCOPY = arm-none-eabi-objcopy
# OBJDUMP = arm-none-eabi-objdump
# SIZE = arm-none-eabi-size

# For simulation, use regular GCC
CC = gcc
OBJCOPY = objcopy
OBJDUMP = objdump
SIZE = size

# Embedded-style compiler flags
CFLAGS = -Wall -Wextra -std=c99 -g -O2
CFLAGS += -ffunction-sections -fdata-sections  # Enable dead code elimination
# CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16  # ARM Cortex-M4
CFLAGS += -DSTM32G431xx  # Example target definition
CFLAGS += -DUSE_HAL_DRIVER  # Example HAL usage

# Linker flags
LDFLAGS = -Wl,--gc-sections  # Dead code elimination
# LDFLAGS += -Wl,-Map=output.map  # Generate memory map
LDFLAGS += -lpthread -lm  # For simulation

# Embedded systems typically avoid these libraries:
# LDFLAGS += -nostdlib -nostartfiles  # No standard library
# LDFLAGS += -T stm32g431xx_flash.ld  # Linker script

# Paths
RUST_LIB_DIR = ../../target/release
INCLUDE_DIR = ../../include
RUST_LIB = libad7124_driver.a

# Targets
TARGET = ad7124_embedded_instance_example
SOURCE = main.c
BINARY = $(TARGET).bin
HEX = $(TARGET).hex
MAP = $(TARGET).map

# Default target
all: info $(TARGET) post-build

# Build information
info:
	@echo "=== AD7124 Embedded Build ==="
	@echo "Target MCU: STM32G431 (simulated)"
	@echo "Compiler: $(CC)"
	@echo "Optimization: -O2"
	@echo "Dead code elimination: Enabled"
	@echo ""

# Ensure Rust library is built
$(RUST_LIB_DIR)/$(RUST_LIB):
	@echo "Building Rust library with C FFI..."
	@echo "NOTE: Make sure crate-type is uncommented in Cargo.toml before building"
	cd ../.. && cargo build --release --features capi

# Build the embedded example
$(TARGET): $(SOURCE) $(RUST_LIB_DIR)/$(RUST_LIB)
	@echo "Building embedded example..."
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -o $(TARGET) $(SOURCE) \
		-L$(RUST_LIB_DIR) -lad7124_driver $(LDFLAGS)

# Post-build analysis (typical for embedded)
post-build: $(TARGET)
	@echo ""
	@echo "=== Build Analysis ==="
	@echo "Memory usage:"
	@$(SIZE) $(TARGET) 2>/dev/null || echo "Size analysis not available"
	@echo ""
	@echo "Binary sections:"
	@$(OBJDUMP) -h $(TARGET) 2>/dev/null || echo "Section analysis not available"
	@echo ""
	@echo "Build complete: $(TARGET)"

# Generate binary file (for embedded programming)
binary: $(TARGET)
	@echo "Generating binary file..."
	$(OBJCOPY) -O binary $(TARGET) $(BINARY)
	@echo "Binary file: $(BINARY)"

# Generate hex file (for embedded programming)  
hex: $(TARGET)
	@echo "Generating hex file..."
	$(OBJCOPY) -O ihex $(TARGET) $(HEX)
	@echo "Hex file: $(HEX)"

# Generate memory map
map: $(TARGET)
	@echo "Memory map available in build output"

# Run the embedded simulation
run: $(TARGET)
	@echo "Running embedded simulation..."
	@echo "NOTE: This simulates embedded behavior on host system"
	./$(TARGET)

# Flash simulation (would program real hardware)
flash: $(TARGET) binary
	@echo "=== Flash Programming Simulation ==="
	@echo "Would program $(BINARY) to target MCU flash memory"
	@echo "Flash base address: 0x08000000"
	@echo "Binary size: $$(wc -c < $(BINARY) 2>/dev/null || echo 'unknown') bytes"
	@echo "Programming... [SIMULATED]"
	@echo "Verification... [SIMULATED]"
	@echo "Flash programming complete [SIMULATED]"

# Debug preparation
debug: $(TARGET)
	@echo "=== Debug Setup ==="
	@echo "Debug symbols: Available in $(TARGET)"
	@echo "To debug with GDB:"
	@echo "  gdb $(TARGET)"
	@echo "For embedded debugging:"
	@echo "  openocd -f interface/stlink.cfg -f target/stm32g4x.cfg"
	@echo "  arm-none-eabi-gdb $(TARGET)"

# Clean build artifacts
clean:
	rm -f $(TARGET) $(BINARY) $(HEX) $(MAP)
	@echo "Cleaned embedded build artifacts"

# Clean everything including Rust library
clean-all: clean
	cd ../.. && cargo clean
	@echo "Cleaned all build artifacts"

# Show embedded development help
help:
	@echo "Available targets:"
	@echo "  all      - Build embedded example (default)"
	@echo "  binary   - Generate binary file for programming"
	@echo "  hex      - Generate hex file for programming"
	@echo "  run      - Run embedded simulation"
	@echo "  flash    - Simulate flash programming"
	@echo "  debug    - Prepare for debugging"
	@echo "  clean    - Clean build artifacts"
	@echo "  clean-all- Clean all including Rust library"
	@echo "  help     - Show this help"
	@echo ""
	@echo "Embedded Development Notes:"
	@echo "- Static memory allocation used (no malloc/free)"
	@echo "- Simulates typical embedded HAL patterns"
	@echo "- Dead code elimination enabled"
	@echo "- Optimized for size and performance"
	@echo "- Ready for cross-compilation to ARM Cortex-M"

# Declare phony targets
.PHONY: all info post-build binary hex map run flash debug clean clean-all help