/**
 * @file ad7124_ffi.h
 * @brief C FFI interface for AD7124 driver
 * 
 * This header provides a C-compatible interface for the AD7124 driver,
 * enabling integration with C/C++ applications while maintaining
 * zero heap allocation and embedded system compatibility.
 * 
 * @author Adancurusul
 */

#ifndef AD7124_FFI_H
#define AD7124_FFI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== Error Codes =====

/**
 * @brief C-compatible error codes
 */
typedef enum {
    AD7124_OK = 0,
    AD7124_NULL_POINTER = -1,
    AD7124_SPI_WRITE = -2,
    AD7124_SPI_READ = -3,
    AD7124_SPI_TRANSFER = -4,
    AD7124_INVALID_CHANNEL = -5,
    AD7124_INVALID_PARAMETER = -6,
    AD7124_NOT_INITIALIZED = -7,
    AD7124_DEVICE_NOT_RESPONDING = -8,
    AD7124_CALIBRATION_FAILED = -9,
    AD7124_CONVERSION_TIMEOUT = -10,
    AD7124_INVALID_DATA_LENGTH = -11,
    AD7124_INVALID_DEVICE_ID = -12,
    AD7124_TIMEOUT = -13,
    AD7124_INVALID_CONFIGURATION = -14,
} ad7124_error_t;

// ===== Device Types =====

/**
 * @brief C-compatible device types
 */
typedef enum {
    AD7124_DEVICE_AD7124_4 = 0,
    AD7124_DEVICE_AD7124_8 = 1,
    AD7124_DEVICE_UNKNOWN = 255,
} ad7124_device_type_t;

// ===== Configuration Enums =====

/**
 * @brief C-compatible gain values
 */
typedef enum {
    AD7124_GAIN_1 = 0,
    AD7124_GAIN_2 = 1,
    AD7124_GAIN_4 = 2,
    AD7124_GAIN_8 = 3,
    AD7124_GAIN_16 = 4,
    AD7124_GAIN_32 = 5,
    AD7124_GAIN_64 = 6,
    AD7124_GAIN_128 = 7,
} ad7124_gain_t;

/**
 * @brief C-compatible channel input values
 */
typedef enum {
    AD7124_AIN0 = 0,
    AD7124_AIN1 = 1,
    AD7124_AIN2 = 2,
    AD7124_AIN3 = 3,
    AD7124_AIN4 = 4,
    AD7124_AIN5 = 5,
    AD7124_AIN6 = 6,
    AD7124_AIN7 = 7,
    AD7124_TEMP_SENSOR = 16,
    AD7124_INT_REF = 17,
    AD7124_DGND = 18,
    AD7124_AVDD_AVSS_DIV5 = 19,
} ad7124_channel_input_t;

/**
 * @brief C-compatible operating modes
 */
typedef enum {
    AD7124_MODE_CONTINUOUS = 0,
    AD7124_MODE_SINGLE_CONV = 1,
    AD7124_MODE_STANDBY = 2,
    AD7124_MODE_POWER_DOWN = 3,
    AD7124_MODE_IDLE = 4,
    AD7124_MODE_INTERNAL_ZERO_SCALE = 5,
    AD7124_MODE_INTERNAL_FULL_SCALE = 6,
    AD7124_MODE_SYSTEM_ZERO_SCALE = 7,
    AD7124_MODE_SYSTEM_FULL_SCALE = 8,
} ad7124_operating_mode_t;

/**
 * @brief C-compatible power modes
 */
typedef enum {
    AD7124_POWER_LOW = 0,
    AD7124_POWER_MID = 1,
    AD7124_POWER_FULL = 2,
} ad7124_power_mode_t;

/**
 * @brief C-compatible reference sources
 */
typedef enum {
    AD7124_REF_EXTERNAL = 0,
    AD7124_REF_INTERNAL = 1,
    AD7124_REF_AVDD_AVSS = 2,
} ad7124_reference_source_t;

/**
 * @brief C-compatible burnout current sources
 */
typedef enum {
    AD7124_BURNOUT_OFF = 0,
    AD7124_BURNOUT_0_5UA = 1,
    AD7124_BURNOUT_2UA = 2,
    AD7124_BURNOUT_4UA = 3,
} ad7124_burnout_current_t;

/**
 * @brief C-compatible filter types
 */
typedef enum {
    AD7124_FILTER_SINC4 = 0,
    AD7124_FILTER_SINC3 = 5,
    AD7124_FILTER_FAST_SETTLE = 4,
} ad7124_filter_type_t;

// ===== Function Pointer Types =====

/**
 * @brief SPI write function pointer
 * @param data Pointer to data to write
 * @param len Number of bytes to write
 * @return 0 on success, negative error code on failure
 */
typedef int (*ad7124_spi_write_fn)(const uint8_t* data, size_t len);

/**
 * @brief SPI read function pointer
 * @param data Pointer to buffer for read data
 * @param len Number of bytes to read
 * @return 0 on success, negative error code on failure
 */
typedef int (*ad7124_spi_read_fn)(uint8_t* data, size_t len);

/**
 * @brief SPI transfer function pointer
 * @param read_data Pointer to buffer for read data
 * @param write_data Pointer to data to write
 * @param len Number of bytes to transfer
 * @return 0 on success, negative error code on failure
 */
typedef int (*ad7124_spi_transfer_fn)(uint8_t* read_data, const uint8_t* write_data, size_t len);

/**
 * @brief Delay function pointer
 * @param ms Delay time in milliseconds
 * @return 0 on success, negative error code on failure
 */
typedef int (*ad7124_delay_ms_fn)(uint32_t ms);

// ===== Configuration Structures =====

/**
 * @brief C-compatible SPI interface structure
 */
typedef struct {
    ad7124_spi_write_fn write;        /**< SPI write function */
    ad7124_spi_read_fn read;          /**< SPI read function */
    ad7124_spi_transfer_fn transfer;  /**< SPI transfer function */
    ad7124_delay_ms_fn delay_ms;      /**< Delay function */
} ad7124_spi_interface_t;

/**
 * @brief C-compatible AD7124 configuration structure
 */
typedef struct {
    ad7124_operating_mode_t operating_mode;     /**< Operating mode */
    ad7124_power_mode_t power_mode;             /**< Power mode */
    ad7124_reference_source_t reference_source; /**< Reference source */
    bool internal_ref_enabled;                  /**< Internal reference enabled */
    bool data_ready_output_enabled;             /**< Data ready output enabled */
} ad7124_config_t;

// Type alias for FFI compatibility
typedef ad7124_config_t CAd7124Config;

/**
 * @brief C-compatible channel configuration structure
 */
typedef struct {
    bool enabled;                               /**< Channel enabled */
    ad7124_channel_input_t positive_input;     /**< Positive input */
    ad7124_channel_input_t negative_input;     /**< Negative input */
    uint8_t setup_index;                       /**< Setup index (0-7) */
} ad7124_channel_config_t;

/**
 * @brief C-compatible setup configuration structure
 */
typedef struct {
    ad7124_gain_t pga_gain;                     /**< PGA gain */
    ad7124_reference_source_t reference_source; /**< Reference source */
    bool bipolar;                               /**< Bipolar mode */
    bool reference_buffers_enabled;             /**< Reference buffers enabled */
    bool input_buffers_enabled;                /**< Input buffers enabled */
    ad7124_burnout_current_t burnout_current;   /**< Burnout current source for sensor diagnostics */
} ad7124_setup_config_t;

/**
 * @brief C-compatible filter configuration structure
 */
typedef struct {
    ad7124_filter_type_t filter_type;          /**< Digital filter type */
    uint16_t output_data_rate;                  /**< Output data rate (0-2047) */
    bool single_cycle;                          /**< Single cycle conversion mode */
    bool reject_60hz;                           /**< Enable 60Hz rejection */
} ad7124_filter_config_t;

/**
 * @brief Opaque driver handle
 */
typedef struct ad7124_driver ad7124_driver_t;

// ===== Static Memory Allocation Helpers =====

/**
 * @brief Fixed size required for AD7124 driver (compile-time constant)
 */
#define AD7124_DRIVER_SIZE 176

/**
 * @brief Alignment requirement for AD7124 driver (compile-time constant)
 */
#define AD7124_DRIVER_ALIGN 8

/**
 * @brief Declare a static driver instance buffer with proper size and alignment
 * @param name Variable name for the instance buffer
 * 
 * Example:
 * AD7124_DECLARE_DRIVER_INSTANCE(my_driver);
 * // This creates: static uint8_t my_driver[64] __attribute__((aligned(8)));
 */
#define AD7124_DECLARE_DRIVER_INSTANCE(name) \
    static uint8_t name[AD7124_DRIVER_SIZE] __attribute__((aligned(AD7124_DRIVER_ALIGN)))

/**
 * @brief Declare a global driver instance buffer with proper size and alignment
 * @param name Variable name for the instance buffer
 */
#define AD7124_DECLARE_GLOBAL_DRIVER_INSTANCE(name) \
    uint8_t name[AD7124_DRIVER_SIZE] __attribute__((aligned(AD7124_DRIVER_ALIGN)))

/**
 * @brief Initialize driver with automatic size calculation for static instance
 * @param instance Instance buffer created with AD7124_DECLARE_DRIVER_INSTANCE
 * @param spi_interface SPI interface configuration
 * @param device_type Device type (AD7124_DEVICE_AD7124_4 or AD7124_DEVICE_AD7124_8)
 * @return AD7124_OK on success, negative error code on failure
 */
#define AD7124_INIT_DRIVER(instance, spi_interface, device_type) \
    ad7124_init_in_place(instance, sizeof(instance), spi_interface, device_type)

// ===== Backward Compatibility Macros =====

/**
 * @brief Deprecated: Use AD7124_DECLARE_DRIVER_INSTANCE instead
 * @deprecated This macro is deprecated in favor of AD7124_DECLARE_DRIVER_INSTANCE
 */
#define AD7124_DECLARE_DRIVER_MEMORY(name) AD7124_DECLARE_DRIVER_INSTANCE(name)

/**
 * @brief Deprecated: Use AD7124_DECLARE_GLOBAL_DRIVER_INSTANCE instead
 * @deprecated This macro is deprecated in favor of AD7124_DECLARE_GLOBAL_DRIVER_INSTANCE
 */
#define AD7124_DECLARE_GLOBAL_DRIVER_MEMORY(name) AD7124_DECLARE_GLOBAL_DRIVER_INSTANCE(name)

// ===== Memory Management API =====

/**
 * @brief Get the size required for the driver structure
 * @return Size in bytes required for driver allocation
 */
size_t ad7124_get_driver_size(void);

/**
 * @brief Get the alignment requirement for the driver structure
 * @return Alignment requirement in bytes
 */
size_t ad7124_get_driver_align(void);

/**
 * @brief Initialize driver in provided instance location (placement new)
 * @param instance Pointer to allocated instance
 * @param instance_size Size of allocated instance
 * @param spi_interface Pointer to SPI interface configuration
 * @param device_type Device type (AD7124-4 or AD7124-8)
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_init_in_place(uint8_t* instance, size_t instance_size, 
                         const ad7124_spi_interface_t* spi_interface,
                         ad7124_device_type_t device_type);

/**
 * @brief Destroy driver in provided instance location
 * @param instance Pointer to driver instance
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_destroy_in_place(uint8_t* instance);

/**
 * @brief Create AD7124 driver (heap allocation - requires allocator)
 * @param spi_interface Pointer to SPI interface configuration
 * @param device_type Device type (AD7124-4 or AD7124-8)
 * @return Pointer to driver handle on success, NULL on failure
 * @note This function is not available in no_std builds
 */
ad7124_driver_t* ad7124_create(const ad7124_spi_interface_t* spi_interface,
                               ad7124_device_type_t device_type);

/**
 * @brief Destroy AD7124 driver (heap deallocation)
 * @param driver Pointer to driver handle
 * @return AD7124_OK on success, negative error code on failure
 * @note This function is not available in no_std builds
 */
int ad7124_destroy(ad7124_driver_t* driver);

// ===== Driver API =====

/**
 * @brief Initialize the AD7124 device
 * @param instance Pointer to driver instance
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_init(uint8_t* instance);

/**
 * @brief Read device ID
 * @param instance Pointer to driver instance
 * @param device_id Pointer to store device ID
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_read_device_id(uint8_t* instance, uint8_t* device_id);

/**
 * @brief Configure ADC settings
 * @param instance Pointer to driver instance
 * @param config Pointer to ADC configuration
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_configure_adc(uint8_t* instance, const ad7124_config_t* config);

/**
 * @brief Setup single-ended measurement
 * @param instance Pointer to driver instance
 * @param channel Channel number (0-7, depending on device)
 * @param positive_input Positive input selection
 * @param setup_index Setup configuration index (0-7)
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_setup_single_ended(uint8_t* instance, uint8_t channel,
                              ad7124_channel_input_t positive_input,
                              uint8_t setup_index);

/**
 * @brief Setup differential measurement
 * @param instance Pointer to driver instance
 * @param channel Channel number (0-7, depending on device)
 * @param positive_input Positive input selection
 * @param negative_input Negative input selection
 * @param setup_index Setup configuration index (0-7)
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_setup_differential(uint8_t* instance, uint8_t channel,
                              ad7124_channel_input_t positive_input,
                              ad7124_channel_input_t negative_input,
                              uint8_t setup_index);

/**
 * @brief Configure setup (PGA, reference, etc.)
 * @param instance Pointer to driver instance
 * @param setup_index Setup configuration index (0-7)
 * @param config Pointer to setup configuration
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_configure_setup(uint8_t* instance, uint8_t setup_index,
                           const ad7124_setup_config_t* config);

/**
 * @brief Configure digital filter for a setup
 * @param instance Pointer to driver instance
 * @param setup_index Setup configuration index (0-7)
 * @param config Pointer to filter configuration
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_configure_filter(uint8_t* instance, uint8_t setup_index,
                           const ad7124_filter_config_t* config);

/**
 * @brief Read raw ADC data
 * @param instance Pointer to driver instance
 * @param data Pointer to store raw ADC data (24-bit)
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_read_data(uint8_t* instance, uint32_t* data);

/**
 * @brief Read voltage
 * @param instance Pointer to driver instance
 * @param setup_index Setup configuration index used for conversion
 * @param voltage Pointer to store voltage reading
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_read_voltage(uint8_t* instance, uint8_t setup_index, float* voltage);

/**
 * @brief Wait for data ready
 * @param instance Pointer to driver instance
 * @param timeout_ms Timeout in milliseconds
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_wait_for_data_ready(uint8_t* instance, uint32_t timeout_ms);

// ===== Enhanced Channel Management API =====

/**
 * @brief Check if data is ready (non-blocking)
 * @param instance Pointer to driver instance
 * @return true if data is ready, false otherwise
 */
bool ad7124_is_data_ready(uint8_t* instance);

/**
 * @brief Wait for conversion ready with optional timeout
 * @param instance Pointer to driver instance
 * @param timeout_ms Timeout in milliseconds (0 = use default timeout)
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_wait_conv_ready(uint8_t* instance, uint32_t timeout_ms);

/**
 * @brief Enable or disable a specific channel
 * @param instance Pointer to driver instance
 * @param channel Channel number (0-15, depending on device)
 * @param enable true to enable, false to disable
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_enable_channel(uint8_t* instance, uint8_t channel, bool enable);

/**
 * @brief Check if a channel is enabled
 * @param instance Pointer to driver instance
 * @param channel Channel number (0-15, depending on device)
 * @return true if channel is enabled, false otherwise
 */
bool ad7124_is_channel_enabled(uint8_t* instance, uint8_t channel);

/**
 * @brief Get the currently active channel
 * @param instance Pointer to driver instance
 * @param channel Pointer to store active channel number
 * @return AD7124_OK on success, negative error code on failure
 * @note Returns AD7124_INVALID_PARAMETER if no channel is currently active
 */
int ad7124_get_active_channel(uint8_t* instance, uint8_t* channel);

/**
 * @brief Get current channel from status register directly
 * @param instance Pointer to driver instance
 * @param channel Pointer to store current channel number
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_current_channel(uint8_t* instance, uint8_t* channel);

// ===== Enhanced Data Reading API =====

/**
 * @brief Read data from a specific channel
 * @param instance Pointer to driver instance
 * @param channel Channel number to read from
 * @param data Pointer to store raw ADC data (24-bit)
 * @return AD7124_OK on success, negative error code on failure
 * @note This function automatically switches to the specified channel if needed
 */
int ad7124_read_channel_data(uint8_t* instance, uint8_t channel, uint32_t* data);

/**
 * @brief Read voltage from a specific channel
 * @param instance Pointer to driver instance
 * @param channel Channel number to read from
 * @param voltage Pointer to store voltage reading
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_read_channel_voltage(uint8_t* instance, uint8_t channel, float* voltage);

/**
 * @brief Read multiple channels sequentially
 * @param instance Pointer to driver instance
 * @param channels Array of channel numbers to read
 * @param channel_count Number of channels to read
 * @param data_out Array to store raw ADC data (must be at least channel_count elements)
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_read_multi_channel(uint8_t* instance, 
                              const uint8_t* channels, 
                              size_t channel_count,
                              uint32_t* data_out);

/**
 * @brief Read voltage from multiple channels
 * @param instance Pointer to driver instance
 * @param channels Array of channel numbers to read
 * @param channel_count Number of channels to read
 * @param voltage_out Array to store voltage readings (must be at least channel_count elements)
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_read_multi_voltage(uint8_t* instance, 
                              const uint8_t* channels, 
                              size_t channel_count,
                              float* voltage_out);

/**
 * @brief Scan all enabled channels and read their data
 * @param instance Pointer to driver instance
 * @param data_out Array to store raw ADC data (must be at least 16 elements)
 * @param channels_out Array to store channel numbers (must be at least 16 elements)
 * @param max_channels Maximum number of channels to read
 * @param channels_read Pointer to store actual number of channels read
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_scan_enabled_channels(uint8_t* instance, 
                                 uint32_t* data_out,
                                 uint8_t* channels_out,
                                 size_t max_channels,
                                 size_t* channels_read);

/**
 * @brief Read data with channel information from status
 * @param instance Pointer to driver instance
 * @param channel Pointer to store channel number
 * @param data Pointer to store raw ADC data
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_read_data_with_status(uint8_t* instance, uint8_t* channel, uint32_t* data);

/**
 * @brief Fast data read (no status check)
 * @param instance Pointer to driver instance
 * @param data Pointer to store raw ADC data (24-bit)
 * @return AD7124_OK on success, negative error code on failure
 * @warning This function does not check if data is ready - use with caution
 */
int ad7124_read_data_fast(uint8_t* instance, uint32_t* data);

/**
 * @brief Software reset
 * @param instance Pointer to driver instance
 * @return AD7124_OK on success, negative error code on failure
 */
int ad7124_reset(uint8_t* instance);

/**
 * @brief Get device type
 * @param instance Pointer to driver instance
 * @return Device type
 */
ad7124_device_type_t ad7124_get_device_type(uint8_t* instance);

/**
 * @brief Check if driver is initialized
 * @param instance Pointer to driver instance
 * @return true if initialized, false otherwise
 */
bool ad7124_is_initialized(uint8_t* instance);

// ===== Utility Functions =====

/**
 * @brief Convert raw ADC data to voltage
 * @param raw_data Raw 24-bit ADC data
 * @param reference_voltage Reference voltage in volts
 * @param gain PGA gain setting
 * @param bipolar true for bipolar mode, false for unipolar
 * @return Voltage in volts
 */
static inline float ad7124_raw_to_voltage(uint32_t raw_data, 
                                          float reference_voltage,
                                          ad7124_gain_t gain,
                                          bool bipolar) {
    // Extract 24-bit data
    raw_data &= 0xFFFFFF;
    
    // Calculate gain multiplier
    float gain_multiplier = 1.0f;
    switch (gain) {
        case AD7124_GAIN_1:   gain_multiplier = 1.0f; break;
        case AD7124_GAIN_2:   gain_multiplier = 2.0f; break;
        case AD7124_GAIN_4:   gain_multiplier = 4.0f; break;
        case AD7124_GAIN_8:   gain_multiplier = 8.0f; break;
        case AD7124_GAIN_16:  gain_multiplier = 16.0f; break;
        case AD7124_GAIN_32:  gain_multiplier = 32.0f; break;
        case AD7124_GAIN_64:  gain_multiplier = 64.0f; break;
        case AD7124_GAIN_128: gain_multiplier = 128.0f; break;
    }
    
    if (bipolar) {
        // Bipolar: -Vref to +Vref
        // Two's complement 24-bit
        int32_t signed_data = (int32_t)raw_data;
        if (signed_data > 0x7FFFFF) {
            signed_data -= 0x1000000; // Convert to signed
        }
        return (reference_voltage * (float)signed_data) / (8388608.0f * gain_multiplier);
    } else {
        // Unipolar: 0 to +Vref
        return (reference_voltage * (float)raw_data) / (16777216.0f * gain_multiplier);
    }
}

/**
 * @brief Example usage showing memory management
 * 
 * @code
 * // SPI interface implementation
 * int my_spi_write(void* ctx, const uint8_t* data, size_t len) { ... }
 * int my_spi_read(void* ctx, uint8_t* data, size_t len) { ... }
 * int my_spi_transfer(void* ctx, uint8_t* read, const uint8_t* write, size_t len) { ... }
 * int my_delay_ms(void* ctx, uint32_t ms) { ... }
 * 
 * // Setup interface
 * ad7124_spi_interface_t spi = {
 *     .write = my_spi_write,
 *     .read = my_spi_read,
 *     .transfer = my_spi_transfer,
 *     .delay_ms = my_delay_ms,
 *     .context = &my_hardware_context
 * };
 * 
 * // Allocate instance
 * size_t size = ad7124_get_driver_size();
 * size_t align = ad7124_get_driver_align();
 * uint8_t* instance = aligned_alloc(align, size);
 * 
 * // Initialize driver
 * if (ad7124_init_in_place(instance, size, &spi, AD7124_DEVICE_AD7124_8) == AD7124_OK) {
 *     // Initialize device
 *     ad7124_init(instance);
 *     
 *     // Setup measurement
 *     ad7124_setup_single_ended(instance, 0, AD7124_AIN0, 0);
 *     
 *     // Read voltage
 *     float voltage;
 *     ad7124_read_voltage(instance, 0, &voltage);
 *     
 *     // Cleanup
 *     ad7124_destroy_in_place(instance);
 * }
 * free(instance);
 * @endcode
 */

#ifdef __cplusplus
}
#endif

#endif /* AD7124_FFI_H */