# Makefile for AD7124 C FFI Example

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g -O2
LDFLAGS = -lpthread -ldl -lm

# Paths
RUST_LIB_DIR = ../../target/release
INCLUDE_DIR = ../../include
RUST_LIB = libad7124_driver.a

# Target
TARGET = ad7124_instance_example
SOURCE = main.c

# Default target
all: $(TARGET)

# Build the Rust library first
$(RUST_LIB_DIR)/$(RUST_LIB):
	@echo "Building Rust library with C FFI..."
	@echo "NOTE: Make sure crate-type is uncommented in Cargo.toml before building"
	cd ../.. && cargo build --release --features capi

# Build the C example
$(TARGET): $(SOURCE) $(RUST_LIB_DIR)/$(RUST_LIB)
	@echo "Building C example..."
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -o $(TARGET) $(SOURCE) -L$(RUST_LIB_DIR) -lad7124_driver $(LDFLAGS)

# Run the example
run: $(TARGET)
	@echo "Running C example..."
	./$(TARGET)

# Clean build artifacts
clean:
	rm -f $(TARGET)
	cd ../.. && cargo clean

# Clean only C artifacts
clean-c:
	rm -f $(TARGET)

# Help
help:
	@echo "Available targets:"
	@echo "  all     - Build the C example (default)"
	@echo "  run     - Build and run the C example"
	@echo "  clean   - Clean all build artifacts"
	@echo "  clean-c - Clean only C build artifacts"
	@echo "  help    - Show this help message"

# Declare phony targets
.PHONY: all run clean clean-c help