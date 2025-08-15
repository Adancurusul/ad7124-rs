# AD7124 FFI 接口使用手册

## 目录
- [概述](#概述)
- [内存管理原理](#内存管理原理)
- [便利宏函数](#便利宏函数)
- [快速开始](#快速开始)
- [详细API说明](#详细api说明)
- [使用示例](#使用示例)
- [多实例管理](#多实例管理)
- [常见问题](#常见问题)

## 概述

AD7124 FFI (Foreign Function Interface) 接口为 C/C++ 应用程序提供了使用 Rust 编写的 AD7124 驱动的能力。该接口设计简洁，无需动态内存分配，特别适合嵌入式系统使用。

### 主要特性
- ✅ 零堆分配设计 - Rust 端无 malloc/free
- ✅ 简化的接口，无需 context 参数
- ✅ 支持所有 AD7124 功能
- ✅ 多实例支持 - 每个实例独立内存
- ✅ 内存位置可控 - 可放置在特定内存段
- ✅ 编译时确定内存需求 - 无运行时分配失败
- ✅ 跨平台支持（Windows/Linux/嵌入式）

## 内存管理原理

### 为什么需要用户提供内存？

传统的驱动库通常使用动态内存分配（malloc/free），但这在嵌入式系统中存在问题：
- ❌ 很多嵌入式系统禁用堆分配
- ❌ 动态分配可能导致内存碎片
- ❌ 分配失败难以预测和处理
- ❌ 不确定的执行时间

**我们的解决方案：**
```c
// C 端分配内存（栈或全局）
uint8_t driver_instance[64] __attribute__((aligned(8)));

// Rust 端在此内存中构造对象
ad7124_init_in_place(driver_instance, sizeof(driver_instance), ...);

// 后续操作都使用这块内存
ad7124_read_voltage(driver_instance, ...);
```

### 内存使用流程

1. **C 端分配** - 64 字节的原始内存
2. **Rust 接收** - 将 `*mut u8` 转换为 `*mut AD7124Sync<CFfiTransport>`
3. **对象构造** - 使用 `ptr::write()` 在指定内存中构造 Rust 对象
4. **正常使用** - 所有函数调用都操作这个内存中的对象
5. **清理销毁** - 调用析构函数但不释放内存（内存属于 C）

### 优势总结

| 特性 | 传统方案 | 我们的方案 |
|------|----------|------------|
| 动态分配 | malloc/free | 无 |
| 内存位置 | 不可控（堆） | 可控（栈/BSS/特定段） |
| 多实例 | 支持 | 支持且更简单 |
| 嵌入式友好 | ❌ | ✅ |
| 执行时间 | 不确定 | 确定 |
| 内存泄漏风险 | 存在 | 无 |

## 便利宏函数

为了简化内存管理，我们提供了一系列宏来自动处理大小和对齐：

### 编译时常量
```c
#define AD7124_DRIVER_SIZE 64      // 驱动所需字节数
#define AD7124_DRIVER_ALIGN 8      // 内存对齐要求
```

### 自动内存声明宏

#### `AD7124_DECLARE_DRIVER_INSTANCE(name)`
声明静态实例缓冲区：
```c
// 使用宏（推荐）
AD7124_DECLARE_DRIVER_INSTANCE(my_driver);

// 等同于手动写法
static uint8_t my_driver[64] __attribute__((aligned(8)));
```

#### `AD7124_DECLARE_GLOBAL_DRIVER_INSTANCE(name)`
声明全局实例缓冲区：
```c
// 在 .h 文件中声明
extern uint8_t global_driver[64];

// 在 .c 文件中定义（使用宏）
AD7124_DECLARE_GLOBAL_DRIVER_INSTANCE(global_driver);
```

### 自动初始化宏

#### `AD7124_INIT_DRIVER(instance, spi_interface, device_type)`
自动计算实例大小并初始化：
```c
// 使用宏（推荐）
AD7124_INIT_DRIVER(driver_instance, &spi_interface, AD7124_DEVICE_AD7124_8);

// 等同于手动写法
ad7124_init_in_place(driver_instance, sizeof(driver_instance), 
                     &spi_interface, AD7124_DEVICE_AD7124_8);
```

### 宏的优势
- ✅ **自动大小计算** - 无需记忆 64 字节
- ✅ **自动对齐处理** - 编译器保证正确对齐
- ✅ **类型安全** - 编译时检查
- ✅ **代码简洁** - 减少样板代码
- ✅ **向前兼容** - 如果将来大小变化，只需重新编译

## 快速开始

### 1. 包含头文件

```c
#include "ad7124_ffi.h"
```

### 2. 实现 SPI 接口函数

您需要实现以下四个函数来提供硬件访问：

```c
// SPI 写函数
int spi_write(const uint8_t* data, size_t len) {
    // 实现您的 SPI 写逻辑
    // 返回 0 表示成功，负数表示错误
    return 0;
}

// SPI 读函数
int spi_read(uint8_t* data, size_t len) {
    // 实现您的 SPI 读逻辑
    // 返回 0 表示成功，负数表示错误
    return 0;
}

// SPI 传输函数（同时读写）
int spi_transfer(uint8_t* read_data, const uint8_t* write_data, size_t len) {
    // 实现您的 SPI 传输逻辑
    // 返回 0 表示成功，负数表示错误
    return 0;
}

// 延时函数
int delay_ms(uint32_t ms) {
    // 实现毫秒级延时
    // 返回 0 表示成功，负数表示错误
    return 0;
}
```

### 3. 初始化驱动

#### 方法 1：使用便利宏（推荐）
```c
// 1. 声明驱动实例（自动处理大小和对齐）
AD7124_DECLARE_DRIVER_INSTANCE(driver_instance);

// 2. 配置 SPI 接口
ad7124_spi_interface_t spi_interface = {
    .write = spi_write,
    .read = spi_read,
    .transfer = spi_transfer,
    .delay_ms = delay_ms
};

// 3. 初始化驱动（自动传递正确的大小）
int result = AD7124_INIT_DRIVER(driver_instance, &spi_interface, AD7124_DEVICE_AD7124_8);
if (result != AD7124_OK) {
    // 处理错误
}

// 4. 初始化 AD7124 设备
result = ad7124_init(driver_instance);
```

#### 方法 2：手动方式（完全控制）
```c
// 1. 手动分配实例
uint8_t driver_instance[AD7124_DRIVER_SIZE] __attribute__((aligned(AD7124_DRIVER_ALIGN)));

// 2. 配置 SPI 接口
ad7124_spi_interface_t spi_interface = { /* ... */ };

// 3. 手动初始化
int result = ad7124_init_in_place(
    driver_instance, 
    sizeof(driver_instance),
    &spi_interface, 
    AD7124_DEVICE_AD7124_8
);

// 4. 初始化设备
result = ad7124_init(driver_instance);
```
```

### 4. 配置和使用

```c
// 配置 ADC
ad7124_config_t adc_config = {
    .operating_mode = AD7124_MODE_CONTINUOUS,
    .power_mode = AD7124_POWER_FULL,
    .reference_source = AD7124_REF_INTERNAL,
    .internal_ref_enabled = true,
    .data_ready_output_enabled = true
};
ad7124_configure_adc(driver_instance, &adc_config);

// 设置单端测量通道
ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);

// 读取电压
float voltage;
ad7124_read_voltage(driver_instance, 0, &voltage);
printf("电压: %.6f V\n", voltage);
```

### 5. 清理

```c
// 销毁驱动（只调用析构函数，不释放内存）
ad7124_destroy_in_place(driver_instance);

// 静态内存自动回收，无需手动释放
// 如果使用了动态分配才需要 free(driver_instance);
```

## 详细API说明

### 内存管理函数

#### `ad7124_get_driver_size`
获取驱动所需的内存大小。
```c
size_t ad7124_get_driver_size(void);
```

#### `ad7124_get_driver_align`
获取驱动所需的内存对齐要求。
```c
size_t ad7124_get_driver_align(void);
```

#### `ad7124_init_in_place`
在提供的内存位置初始化驱动。
```c
int ad7124_init_in_place(
    uint8_t* memory,                           // 内存指针
    size_t memory_size,                        // 内存大小
    const ad7124_spi_interface_t* spi_interface, // SPI 接口
    ad7124_device_type_t device_type          // 设备类型
);
```

#### `ad7124_destroy_in_place`
销毁驱动实例。
```c
int ad7124_destroy_in_place(uint8_t* memory);
```

### 设备控制函数

#### `ad7124_init`
初始化 AD7124 设备。
```c
int ad7124_init(uint8_t* driver);
```

#### `ad7124_reset`
软件复位设备。
```c
int ad7124_reset(uint8_t* driver);
```

#### `ad7124_read_device_id`
读取设备 ID。
```c
int ad7124_read_device_id(uint8_t* driver, uint8_t* device_id);
```

#### `ad7124_is_initialized`
检查驱动是否已初始化。
```c
bool ad7124_is_initialized(const uint8_t* driver);
```

### ADC 配置函数

#### `ad7124_configure_adc`
配置 ADC 基本参数。
```c
int ad7124_configure_adc(uint8_t* driver, const ad7124_config_t* config);
```

配置结构体：
```c
typedef struct {
    ad7124_operating_mode_t operating_mode;     // 工作模式
    ad7124_power_mode_t power_mode;             // 功耗模式
    ad7124_reference_source_t reference_source; // 参考源
    bool internal_ref_enabled;                  // 内部参考使能
    bool data_ready_output_enabled;             // 数据就绪输出使能
} ad7124_config_t;
```

### 通道配置函数

#### `ad7124_setup_single_ended`
配置单端测量通道。
```c
int ad7124_setup_single_ended(
    uint8_t* driver,
    uint8_t channel,                    // 通道号 (0-15 或 0-7)
    ad7124_channel_input_t input,       // 输入引脚
    uint8_t setup                       // 设置索引 (0-7)
);
```

#### `ad7124_setup_differential`
配置差分测量通道。
```c
int ad7124_setup_differential(
    uint8_t* driver,
    uint8_t channel,                    // 通道号
    ad7124_channel_input_t positive,    // 正输入
    ad7124_channel_input_t negative,    // 负输入
    uint8_t setup                       // 设置索引
);
```

#### `ad7124_configure_channel`
配置通道详细参数。
```c
int ad7124_configure_channel(
    uint8_t* driver,
    uint8_t channel,
    const ad7124_channel_config_t* config
);
```

### 设置配置函数

#### `ad7124_configure_setup`
配置设置参数（增益、参考源等）。
```c
int ad7124_configure_setup(
    uint8_t* driver,
    uint8_t setup,                      // 设置索引 (0-7)
    const ad7124_setup_config_t* config
);
```

设置结构体：
```c
typedef struct {
    ad7124_gain_t pga_gain;                     // PGA 增益
    ad7124_reference_source_t reference_source; // 参考源
    bool bipolar;                               // 双极性模式
    bool reference_buffers_enabled;             // 参考缓冲器使能
    bool input_buffers_enabled;                // 输入缓冲器使能
    ad7124_burnout_current_t burnout_current;   // 传感器诊断电流
} ad7124_setup_config_t;
```

烧断电流选项：
```c
typedef enum {
    AD7124_BURNOUT_OFF = 0,      // 无烧断电流
    AD7124_BURNOUT_0_5UA = 1,    // 0.5 µA 诊断电流
    AD7124_BURNOUT_2UA = 2,      // 2 µA 诊断电流
    AD7124_BURNOUT_4UA = 3,      // 4 µA 诊断电流
} ad7124_burnout_current_t;
```

#### `ad7124_configure_filter`
配置数字滤波器进行信号处理。
```c
int ad7124_configure_filter(
    uint8_t* driver,
    uint8_t setup,                      // 设置索引 (0-7)
    const ad7124_filter_config_t* config
);
```

滤波器配置结构体：
```c
typedef struct {
    ad7124_filter_type_t filter_type;          // 数字滤波器类型
    uint16_t output_data_rate;                  // 数据率 (0-2047 Hz)
    bool single_cycle;                          // 单周期模式
    bool reject_60hz;                           // 60Hz 抑制
} ad7124_filter_config_t;
```

滤波器类型选项：
```c
typedef enum {
    AD7124_FILTER_SINC4 = 0,        // SINC4 滤波器（最高精度）
    AD7124_FILTER_SINC3 = 5,        // SINC3 滤波器（更快响应）
    AD7124_FILTER_FAST_SETTLE = 4,  // 快速稳定滤波器
} ad7124_filter_type_t;
```

### 数据读取函数

#### `ad7124_wait_for_data_ready`
等待数据就绪。
```c
int ad7124_wait_for_data_ready(uint8_t* driver, uint32_t timeout_ms);
```

#### `ad7124_is_data_ready`
检查数据是否就绪（非阻塞）。
```c
bool ad7124_is_data_ready(uint8_t* driver);
```

#### `ad7124_read_data`
读取原始 ADC 数据。
```c
int ad7124_read_data(uint8_t* driver, uint32_t* data);
```

#### `ad7124_read_voltage`
读取电压值。
```c
int ad7124_read_voltage(uint8_t* driver, uint8_t channel, float* voltage);
```

#### `ad7124_raw_to_voltage`
将原始数据转换为电压。
```c
float ad7124_raw_to_voltage(
    uint32_t raw_data,
    float reference_voltage,
    ad7124_gain_t gain,
    bool bipolar
);
```

### 增强通道管理函数

#### `ad7124_enable_channel`
启用或禁用指定通道。
```c
int ad7124_enable_channel(uint8_t* driver, uint8_t channel, bool enable);
```

#### `ad7124_is_channel_enabled`
检查通道是否启用。
```c
bool ad7124_is_channel_enabled(uint8_t* driver, uint8_t channel);
```

#### `ad7124_get_active_channel`
获取当前活动通道。
```c
int ad7124_get_active_channel(uint8_t* driver, uint8_t* channel);
```

#### `ad7124_read_channel_data`
读取指定通道的原始数据。
```c
int ad7124_read_channel_data(uint8_t* driver, uint8_t channel, uint32_t* data);
```

#### `ad7124_read_channel_voltage`
读取指定通道的电压值。
```c
int ad7124_read_channel_voltage(uint8_t* driver, uint8_t channel, float* voltage);
```

#### `ad7124_read_multi_channel`
读取多个通道的数据。
```c
int ad7124_read_multi_channel(
    uint8_t* driver,
    const uint8_t* channels,     // 通道数组
    size_t channel_count,        // 通道数量
    uint32_t* data,             // 输出数据数组
    size_t* data_count          // 输出：实际读取的数据数量
);
```

### 校准函数

#### `ad7124_calibrate_internal_zero`
执行内部零点校准。
```c
int ad7124_calibrate_internal_zero(uint8_t* driver, uint8_t setup);
```

#### `ad7124_calibrate_internal_full`
执行内部满量程校准。
```c
int ad7124_calibrate_internal_full(uint8_t* driver, uint8_t setup);
```

## 使用示例

### 示例1：基本电压测量

```c
#include <stdio.h>
#include <stdint.h>
#include "ad7124_ffi.h"

// 全局硬件状态
static struct {
    int spi_fd;
    int cs_pin;
} hw = {-1, 10};

// 实现 SPI 函数
int spi_write(const uint8_t* data, size_t len) {
    // 您的 SPI 写实现
    return 0;
}

int spi_read(uint8_t* data, size_t len) {
    // 您的 SPI 读实现
    return 0;
}

int spi_transfer(uint8_t* read_data, const uint8_t* write_data, size_t len) {
    // 您的 SPI 传输实现
    return 0;
}

int delay_ms(uint32_t ms) {
    // 您的延时实现
    return 0;
}

int main(void) {
    // 静态分配驱动内存
    static uint8_t driver_instance[128] __attribute__((aligned(8)));
    
    // 配置 SPI 接口
    ad7124_spi_interface_t spi = {
        .write = spi_write,
        .read = spi_read,
        .transfer = spi_transfer,
        .delay_ms = delay_ms
    };
    
    // 初始化驱动
    if (ad7124_init_in_place(driver_instance, sizeof(driver_instance), 
                             &spi, AD7124_DEVICE_AD7124_8) != AD7124_OK) {
        printf("驱动初始化失败\n");
        return 1;
    }
    
    // 初始化设备
    if (ad7124_init(driver_instance) != AD7124_OK) {
        printf("设备初始化失败\n");
        return 1;
    }
    
    // 配置 ADC
    ad7124_config_t config = {
        .operating_mode = AD7124_MODE_CONTINUOUS,
        .power_mode = AD7124_POWER_FULL,
        .reference_source = AD7124_REF_INTERNAL,
        .internal_ref_enabled = true,
        .data_ready_output_enabled = true
    };
    ad7124_configure_adc(driver_instance, &config);
    
    // 配置通道0为AIN0单端测量
    ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);
    
    // 配置设置0
    ad7124_setup_config_t setup = {
        .pga_gain = AD7124_GAIN_1,
        .reference_source = AD7124_REF_INTERNAL,
        .bipolar = true,
        .reference_buffers_enabled = true,
        .input_buffers_enabled = true,
        .burnout_current = AD7124_BURNOUT_OFF  // 传感器诊断关闭
    };
    ad7124_configure_setup(driver_instance, 0, &setup);
    
    // 读取10次电压
    for (int i = 0; i < 10; i++) {
        float voltage;
        if (ad7124_wait_for_data_ready(driver_instance, 1000) == AD7124_OK) {
            if (ad7124_read_voltage(driver_instance, 0, &voltage) == AD7124_OK) {
                printf("测量 %d: %.6f V\n", i + 1, voltage);
            }
        }
    }
    
    // 清理
    ad7124_destroy_in_place(driver_instance);
    
    return 0;
}
```

### 示例2：差分测量

```c
// 配置差分测量（AIN0 - AIN1）
ad7124_setup_differential(driver_instance, 0, AD7124_AIN0, AD7124_AIN1, 0);

// 设置增益为 8x 以提高小信号测量精度
ad7124_setup_config_t setup = {
    .pga_gain = AD7124_GAIN_8,
    .reference_source = AD7124_REF_INTERNAL,
    .bipolar = true,
    .reference_buffers_enabled = true,
    .input_buffers_enabled = true,
    .burnout_current = AD7124_BURNOUT_OFF  // 传感器诊断关闭
};
ad7124_configure_setup(driver_instance, 0, &setup);
```

### 示例3：温度测量

```c
// 配置内部温度传感器
ad7124_setup_single_ended(driver_instance, 0, AD7124_TEMP_SENSOR, 0);

// 读取温度传感器电压
float temp_voltage;
ad7124_read_voltage(driver_instance, 0, &temp_voltage);

// 转换为温度（近似公式）
float temperature = 25.0f + (temp_voltage - 1.17f) / 0.0018f;
printf("温度: %.1f °C\n", temperature);
```

### 示例4：传感器诊断和滤波器配置

```c
// 配置带有传感器诊断的设置
ad7124_setup_config_t diagnostic_setup = {
    .pga_gain = AD7124_GAIN_32,
    .reference_source = AD7124_REF_INTERNAL,
    .bipolar = true,
    .reference_buffers_enabled = true,
    .input_buffers_enabled = true,
    .burnout_current = AD7124_BURNOUT_2UA  // 2µA 传感器故障检测
};
ad7124_configure_setup(driver_instance, 0, &diagnostic_setup);

// 配置数字滤波器
ad7124_filter_config_t filter_config = {
    .filter_type = AD7124_FILTER_SINC4,    // 高精度 SINC4 滤波器
    .output_data_rate = 50,                 // 50 Hz 数据率
    .single_cycle = false,
    .reject_60hz = true                     // 启用 60Hz 工频抑制
};
ad7124_configure_filter(driver_instance, 0, &filter_config);

printf("传感器诊断和滤波器配置完成\n");
printf("  烧断电流: 2µA (传感器故障检测)\n");
printf("  滤波器类型: SINC4 (高精度)\n");
printf("  数据率: 50 Hz\n");
printf("  60Hz 抑制: 启用\n");
```

### 示例5：多通道扫描

```c
// 配置基本设置（无诊断电流）
ad7124_setup_config_t basic_setup = {
    .pga_gain = AD7124_GAIN_1,
    .reference_source = AD7124_REF_INTERNAL,
    .bipolar = true,
    .reference_buffers_enabled = true,
    .input_buffers_enabled = true,
    .burnout_current = AD7124_BURNOUT_OFF
};
ad7124_configure_setup(driver_instance, 0, &basic_setup);

// 配置多个通道
ad7124_setup_single_ended(driver_instance, 0, AD7124_AIN0, 0);
ad7124_setup_single_ended(driver_instance, 1, AD7124_AIN1, 0);
ad7124_setup_single_ended(driver_instance, 2, AD7124_AIN2, 0);
ad7124_setup_single_ended(driver_instance, 3, AD7124_AIN3, 0);

// 启用所有通道
for (int ch = 0; ch < 4; ch++) {
    ad7124_enable_channel(driver_instance, ch, true);
}

// 检查哪些通道已启用
printf("启用的通道: ");
for (int ch = 0; ch < 4; ch++) {
    if (ad7124_is_channel_enabled(driver_instance, ch)) {
        printf("%d ", ch);
    }
}
printf("\n");

// 读取所有通道（使用通道特定读取）
for (int ch = 0; ch < 4; ch++) {
    float voltage;
    if (ad7124_read_channel_voltage(driver_instance, ch, &voltage) == AD7124_OK) {
        printf("通道 %d: %.6f V\n", ch, voltage);
    }
}
```

### 示例6：增强通道管理

```c
// 动态通道控制示例
uint8_t channels_to_read[] = {0, 2, 4, 6};
size_t channel_count = sizeof(channels_to_read) / sizeof(channels_to_read[0]);

// 只启用需要的通道
for (size_t i = 0; i < channel_count; i++) {
    ad7124_enable_channel(driver_instance, channels_to_read[i], true);
}

// 禁用不需要的通道（节省功耗）
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

// 读取多个通道的数据
uint32_t data_buffer[8];
size_t actual_count = 0;
if (ad7124_read_multi_channel(driver_instance, channels_to_read, channel_count, 
                              data_buffer, &actual_count) == AD7124_OK) {
    printf("成功读取 %zu 个通道:\n", actual_count);
    for (size_t i = 0; i < actual_count; i++) {
        printf("  通道 %d: 0x%08X (%u)\n", 
               channels_to_read[i], data_buffer[i], data_buffer[i]);
    }
}
```

### 示例7：数据就绪检查

```c
// 非阻塞数据读取
while (true) {
    if (ad7124_is_data_ready(driver_instance)) {
        uint8_t active_channel;
        uint32_t data;
        
        // 获取当前活动通道
        if (ad7124_get_active_channel(driver_instance, &active_channel) == AD7124_OK) {
            printf("通道 %d 数据就绪\n", active_channel);
            
            // 读取数据
            if (ad7124_read_data(driver_instance, &data) == AD7124_OK) {
                printf("  原始数据: 0x%08X\n", data);
                
                // 转换为电压
                float voltage = ad7124_raw_to_voltage(data, 2.5f, AD7124_GAIN_1, true);
                printf("  电压: %.6f V\n", voltage);
            }
        }
        
        // 其他处理...
        break;
    }
    
    // 做其他工作，避免阻塞
    usleep(1000); // 1ms延时
}
```

## 多实例管理

### 为什么支持多实例？

在实际应用中，您可能需要同时控制多个 AD7124 设备：
- 多路传感器数据采集
- 主传感器 + 校准设备
- 不同精度要求的测量通道

### 多实例使用示例

```c
// 声明三个独立的驱动内存
AD7124_DECLARE_DRIVER_MEMORY(sensor1_driver);     // 传感器1
AD7124_DECLARE_DRIVER_MEMORY(sensor2_driver);     // 传感器2
AD7124_DECLARE_DRIVER_MEMORY(calibrator_driver);  // 校准器

// 配置不同的 SPI 接口（如果使用不同的 SPI 总线）
ad7124_spi_interface_t spi1_interface = { /* SPI1 配置 */ };
ad7124_spi_interface_t spi2_interface = { /* SPI2 配置 */ };
ad7124_spi_interface_t spi3_interface = { /* SPI3 配置 */ };

// 分别初始化
AD7124_INIT_DRIVER(sensor1_driver, &spi1_interface, AD7124_DEVICE_AD7124_8);
AD7124_INIT_DRIVER(sensor2_driver, &spi2_interface, AD7124_DEVICE_AD7124_4);
AD7124_INIT_DRIVER(calibrator_driver, &spi3_interface, AD7124_DEVICE_AD7124_8);

// 分别配置（每个设备可以有不同配置）
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

ad7124_configure_adc(sensor1_driver, &high_speed_config);    // 高速连续
ad7124_configure_adc(sensor2_driver, &low_power_config);     // 低功耗单次
ad7124_configure_adc(calibrator_driver, &high_speed_config); // 高精度校准

// 同时使用
float voltage1, voltage2, cal_voltage;
ad7124_read_voltage(sensor1_driver, 0, &voltage1);      // 传感器1读取
ad7124_read_voltage(sensor2_driver, 0, &voltage2);      // 传感器2读取
ad7124_read_voltage(calibrator_driver, 0, &cal_voltage); // 校准器读取

printf("传感器1: %.3f V\n", voltage1);
printf("传感器2: %.3f V\n", voltage2);
printf("校准值: %.6f V\n", cal_voltage);

// 分别清理
ad7124_destroy_in_place(sensor1_driver);
ad7124_destroy_in_place(sensor2_driver);
ad7124_destroy_in_place(calibrator_driver);
```

### 共享 SPI 总线的多设备

如果多个 AD7124 共享同一个 SPI 总线（使用不同的片选信号）：

```c
// 两个设备共享 SPI 接口函数，但在函数内部通过全局变量控制片选
static int current_device = 0;  // 当前选择的设备

int spi_write(const uint8_t* data, size_t len) {
    switch(current_device) {
        case 0: set_cs1_low(); break;   // 设备1片选
        case 1: set_cs2_low(); break;   // 设备2片选
    }
    // SPI 传输逻辑
    spi_transmit(data, len);
    set_all_cs_high();  // 释放所有片选
    return 0;
}

// 使用时切换设备
void select_device(int device) {
    current_device = device;
}

// 使用示例
select_device(0);
ad7124_read_voltage(device1_driver, 0, &voltage1);

select_device(1);
ad7124_read_voltage(device2_driver, 0, &voltage2);
```

### 内存使用统计

```c
printf("内存使用统计:\n");
printf("  单个驱动: %d 字节\n", AD7124_DRIVER_SIZE);
printf("  三个实例总计: %d 字节\n", AD7124_DRIVER_SIZE * 3);
printf("  内存地址分布:\n");
printf("    传感器1:  %p\n", (void*)sensor1_driver);
printf("    传感器2:  %p\n", (void*)sensor2_driver);
printf("    校准器:   %p\n", (void*)calibrator_driver);
```

## 常见问题

### Q1: 为什么必须提供 memory 参数？

**原因：**
1. **零动态分配** - Rust 端不调用 malloc，所有内存由 C 端提供
2. **内存位置可控** - 可放在栈、BSS、特定内存段（如 CCRAM）
3. **多实例支持** - 不同内存 = 不同实例，简单直观
4. **嵌入式友好** - 编译时确定内存使用，无运行时分配失败

**使用建议：**
- 单设备：使用 `AD7124_DECLARE_DRIVER_MEMORY(driver)`
- 多设备：每个设备用独立的内存缓冲区
- 特殊需求：手动分配并指定内存段

### Q2: 如何处理错误？

所有函数返回错误码，建议封装错误处理：

```c
void check_error(int result, const char* operation) {
    if (result != AD7124_OK) {
        const char* error_msg;
        switch (result) {
            case AD7124_NULL_POINTER: 
                error_msg = "空指针"; 
                break;
            case AD7124_SPI_WRITE: 
                error_msg = "SPI写入错误"; 
                break;
            case AD7124_TIMEOUT: 
                error_msg = "超时"; 
                break;
            // ... 其他错误
            default: 
                error_msg = "未知错误";
        }
        printf("错误 [%s]: %s (%d)\n", operation, error_msg, result);
    }
}

// 使用
check_error(ad7124_init(driver_instance), "初始化");
```

### Q3: 如何优化性能？

1. **使用连续模式**：避免频繁切换模式
2. **批量读取**：一次读取多个通道数据
3. **合理设置滤波器**：根据应用需求调整输出数据率
4. **使用 DMA**：在 SPI 函数中实现 DMA 传输

### Q4: 如何调试？

1. **验证内存大小匹配**：
```c
size_t actual_size = ad7124_get_driver_size();
if (actual_size != AD7124_DRIVER_SIZE) {
    printf("警告：内存大小不匹配！实际: %zu, 预期: %d\n", 
           actual_size, AD7124_DRIVER_SIZE);
}
```

2. **检查设备ID**：确认硬件连接正常
```c
uint8_t device_id;
ad7124_read_device_id(driver_instance, &device_id);
printf("设备ID: 0x%02X\n", device_id);  // 应该是 0x04 或 0x12
```

3. **内存地址检查**：
```c
printf("驱动内存地址: %p\n", (void*)driver_instance);
printf("内存对齐检查: %s\n", 
       ((uintptr_t)driver_instance % AD7124_DRIVER_ALIGN == 0) ? "OK" : "错误");
```

4. **监控SPI通信**：在 SPI 函数中添加日志
5. **多实例调试**：确认不同实例使用不同内存地址

### Q5: 内存要求是多少？

- **驱动内存**：64 字节（编译时常量 `AD7124_DRIVER_SIZE`）
- **对齐要求**：8 字节（编译时常量 `AD7124_DRIVER_ALIGN`）
- **栈使用**：最小，适合资源受限系统
- **每个实例独立**：多实例时每个占用 64 字节

**内存组成：**
```
AD7124Sync<CFfiTransport> (64 字节) = {
  transport: CFfiTransport (32 字节) {
    interface: { 4个函数指针，每个8字节 }
  },
  core: AD7124Core (32 字节) {
    device_type, capabilities, config, 
    initialized, reference_voltage, crc_enabled
    + 对齐填充
  }
}
```

## 错误码参考

| 错误码 | 值 | 说明 |
|-------|-----|------|
| AD7124_OK | 0 | 成功 |
| AD7124_NULL_POINTER | -1 | 空指针错误 |
| AD7124_SPI_WRITE | -2 | SPI 写入错误 |
| AD7124_SPI_READ | -3 | SPI 读取错误 |
| AD7124_SPI_TRANSFER | -4 | SPI 传输错误 |
| AD7124_INVALID_CHANNEL | -5 | 无效通道 |
| AD7124_INVALID_PARAMETER | -6 | 无效参数 |
| AD7124_NOT_INITIALIZED | -7 | 未初始化 |
| AD7124_DEVICE_NOT_RESPONDING | -8 | 设备无响应 |
| AD7124_CALIBRATION_FAILED | -9 | 校准失败 |
| AD7124_CONVERSION_TIMEOUT | -10 | 转换超时 |
| AD7124_INVALID_DATA_LENGTH | -11 | 无效数据长度 |
| AD7124_INVALID_DEVICE_ID | -12 | 无效设备ID |
| AD7124_TIMEOUT | -13 | 超时 |
| AD7124_INVALID_CONFIGURATION | -14 | 无效配置 |

## 支持与贡献

## 高级主题

### 内存段控制

在某些嵌入式系统中，您可能希望将驱动放在特定的内存段：

```c
// 放在 CCRAM（STM32 紧耦合内存）
__attribute__((section(".ccram")))
AD7124_DECLARE_DRIVER_MEMORY(ccram_driver);

// 放在 DMA 可访问区域
__attribute__((section(".dma_buffer")))
AD7124_DECLARE_DRIVER_MEMORY(dma_driver);

// 放在快速访问区域
__attribute__((section(".itcm")))
AD7124_DECLARE_DRIVER_MEMORY(fast_driver);
```

### 与 RTOS 的集成

```c
// FreeRTOS 任务示例
void sensor_task(void* parameter) {
    // 每个任务有独立的驱动实例
    AD7124_DECLARE_DRIVER_MEMORY(task_driver);
    
    AD7124_INIT_DRIVER(task_driver, &spi_interface, AD7124_DEVICE_AD7124_8);
    ad7124_init(task_driver);
    
    while(1) {
        float voltage;
        ad7124_read_voltage(task_driver, 0, &voltage);
        
        // 发送到队列或信号量
        xQueueSend(voltage_queue, &voltage, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### 编译时检查

```c
// 编译时确保内存大小足够
_Static_assert(sizeof(driver_instance) >= AD7124_DRIVER_SIZE, 
               "Driver memory too small");
_Static_assert(AD7124_DRIVER_SIZE == 64, 
               "Expected driver size changed");
```

如有问题或建议，请访问项目仓库提交 Issue 或 Pull Request。

作者：Adancurusul  
许可证：MIT