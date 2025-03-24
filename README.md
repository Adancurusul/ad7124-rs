[![crates.io](https://img.shields.io/crates/v/ad7124-rs.svg)](https://crates.io/crates/ad7124-rs)
# Rust driver for 24-Bit ADC with PGA and Reference AD7124
> This is a platform-independent Rust driver for AD7124, provided in both synchronous and asynchronous versions, based on the [`embedded-hal`](https://github.com/japaric/embedded-hal)

The following chips are supported：
* [```AD7124-8```](https://www.analog.com/en/products/ad7124-8.html) eight-channel 
* [```AD7124-4```](https://www.analog.com/en/products/ad7124-4.html) four-channel 

## The Device
The AD7124-8 is a low power, low noise, completely integrated analog front end for high precision measurement applications. The AD7124-8 W grade is AEC-Q100 qualified for automotive applications. The device contains a low noise, 24-bit Σ-Δ analog-to-digital converter (ADC), and can be configured to have 8 differential inputs or 15 single-ended or pseudo differential inputs. The on-chip low gain stage ensures that signals of small amplitude can be interfaced directly to the ADC.

## Initialization Methods
The library provides two ways to create an AD7124 instance:

### 1. Using `new` method (backward compatible)
Directly accepts SPI, CS, and Delay parameters - ideal for users upgrading from previous versions:

```rust
// Synchronous version
let mut adc = AD7124Sync::new(spi, cs, delay).unwrap();

// Asynchronous version
let mut adc = AD7124Async::new(spi, cs, delay).unwrap();
```

### 2. Using `from_transport` method (new flexible approach)
For more control over the transport layer:

```rust
// Create custom transport
let transport = RealSyncTransport { spi, cs, delay };

// Use transport to create AD7124 instance
let mut adc = AD7124Sync::from_transport(transport);
```

These methods are functionally equivalent, but `from_transport` provides more flexibility by allowing custom transport implementations.

### Simple Usage Example

```rust
// Initialize the AD7124 (synchronous version)
let mut adc = AD7124Sync::new(spi, cs, delay).unwrap();

// Initialize the device
adc.init().unwrap();

// Read device ID
let id = adc.read_id().unwrap();
println!("Device ID: {}", id);

// Configure ADC
adc.set_adc_control(
    AD7124OperatingMode::Continuous,
    AD7124PowerMode::FullPower,
    AD7124ClkSource::ClkInternal,
    true
).unwrap();

// Configure channel
adc.set_channel(0, AD7124Channel::AIN0, AD7124Channel::AIN1, 0, true).unwrap();
adc.enable_channel(0, true).unwrap();

// Read data
let data = adc.read_single_raw_data(0).unwrap();
println!("Channel 0 data: {}", data);
```

For the asynchronous version, simply use `await` with the same methods:

```rust
// Initialize the AD7124 (asynchronous version)
let mut adc = AD7124Async::new(spi, cs, delay).unwrap();

// Initialize and read (with await)
adc.init().await.unwrap();
let data = adc.read_single_raw_data(0).await.unwrap();
```

> **Note:** Starting from v0.3.0, the initialization API has been restructured while maintaining backward compatibility. Existing code using the previous API will continue to work without modifications.

## Usage
The example based on ```embassy```  is given below, using the ```STM32L432KBUx```
### Asynchronous version
add the following to your Cargo.toml
```toml
[dependencies]
ad7124-rs = { version = "0.3.0", features = ["async"] }
```
```rust
#![no_std]
#![no_main]

#![allow(dead_code)]
#![allow(unused_imports)]
use ad7124_rs::*;
use defmt::*;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config as spicon, Spi};
use embassy_stm32::Config;
use embassy_time::Delay;
use embassy_stm32::time::Hertz;
use {defmt_rtt as _, panic_probe as _};
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::MSI,
            prediv: PllPreDiv::DIV1,   //pllm
            mul: PllMul::MUL40,        //plln
            divr: Some(PllRDiv::DIV2), //pllr
            divq: Some(PllQDiv::DIV2), //pllq
            divp: Some(PllPDiv::DIV7), //pllp
        });
        config.rcc.pllsai1 = Some(Pll {
            source: PllSource::MSI,
            prediv: PllPreDiv::DIV1,   //pllm
            mul: PllMul::MUL16,        //plln
            divr: Some(PllRDiv::DIV2), //pllr
            divq: Some(PllQDiv::DIV2), //pllq
            divp: Some(PllPDiv::DIV7), //pllp
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.mux.adcsel = mux::Adcsel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);
    let mut spi_config = spicon::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi = Spi::new_blocking(p.SPI3, p.PB3, p.PB5, p.PB4, spi_config);
    let mut spi = BlockingAsync::new(spi);

    let mut cs = Output::new(p.PA15, Level::Low, Speed::High);
    let mut sync = Output::new(p.PB6, Level::Low, Speed::High);
    sync.set_high();

    let mut sensor_clk = p.PB0;
    let mut sensor_data = p.PB1;
    let mut adc = AD7124Async::new(spi, cs, Delay).unwrap();
    adc.init().await.unwrap();
    let reg_now = adc.read_id().await;
    info!("ID:{}", reg_now);
    let init_ctrl = adc
        .set_adc_control(
            AD7124OperatingMode::Continuous,
            AD7124PowerMode::FullPower,
            AD7124ClkSource::ClkInternal,
            true,
        )
        .await;
    info!("init_ctrl:{}", init_ctrl);
    let config_set = adc
        .set_config(
            0,
            AD7124RefSource::Internal,
            AD7124GainSel::_64,
            AD7124BurnoutCurrent::Off,
            true,
        )
        .await;
    info!("config_set:{}", config_set);
    let filter_set = adc
        .set_filter(0, AD7124Filter::POST, 1, AD7124PostFilter::NoPost, false, true)
        .await;

    info!("filter_set:{}", filter_set);
    let adc_ch_set = adc
        .set_channel(0, AD7124Channel::AIN0, AD7124Channel::AIN1, 0, true)
        .await;
    info!("adc_ch_set:{}", adc_ch_set);
    let adc_ch_set2 = adc
        .set_channel(1, AD7124Channel::AIN2, AD7124Channel::AIN3, 0, true)
        .await;
    info!("adc_ch_set2:{}", adc_ch_set2);
    let pws = adc.set_pwrsw(true).await;
    info!("pws:{}", pws);

    adc.enable_channel(0, true).await.unwrap();
    adc.enable_channel(1, true).await.unwrap();
    let status = adc.read_register(AD7124RegId::RegStatus).await;
    info!("status:{}", status);
    let mut data_list = [0u32; 16];
    let errif = adc.read_multi_raw_data(&mut data_list).await;
    match errif {
        Ok(_) => {
            info!("read_multi_raw_data success");
            for i in 0..16 {
                info!("data_list[{}]:{}", i, data_list[i]);
            }
        }
        Err(e) => {
            info!("read_multi_raw_data error:{}", e);
        }
    }
    let ch1_data = adc.read_single_raw_data(0).await;
    match ch1_data {
        Ok(data) => {
            info!("ch1_data:{}", data);
        }
        Err(e) => {
            info!("ch1_data error:{}", e);
        }
    }
    let ch2_data = adc.read_single_raw_data(1).await;
    match ch2_data {
        Ok(data) => {
            info!("ch2_data:{}", data);
        }
        Err(e) => {
            info!("ch2_data error:{}", e);
        }
    }

    loop {

    }
}

```

### Synchronous version
add the following to your Cargo.toml
```toml
[dependencies]
ad7124-rs = { version = "0.3.0", features = ["sync"] }
```
```rust
#![no_std]
#![no_main]

#![allow(dead_code)]
#![allow(unused_imports)]
use ad7124_rs::*;
use defmt::*;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config as spicon, Spi};
use embassy_stm32::Config;
use embassy_time::Delay;
use embassy_stm32::time::Hertz;
use {defmt_rtt as _, panic_probe as _};
#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::MSI,
            prediv: PllPreDiv::DIV1,   //pllm
            mul: PllMul::MUL40,        //plln
            divr: Some(PllRDiv::DIV2), //pllr
            divq: Some(PllQDiv::DIV2), //pllq
            divp: Some(PllPDiv::DIV7), //pllp
        });
        config.rcc.pllsai1 = Some(Pll {
            source: PllSource::MSI,
            prediv: PllPreDiv::DIV1,   //pllm
            mul: PllMul::MUL16,        //plln
            divr: Some(PllRDiv::DIV2), //pllr
            divq: Some(PllQDiv::DIV2), //pllq
            divp: Some(PllPDiv::DIV7), //pllp
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.mux.adcsel = mux::Adcsel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);
    let mut spi_config = spicon::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi = Spi::new_blocking(p.SPI3, p.PB3, p.PB5, p.PB4, spi_config);

    let mut cs = Output::new(p.PA15, Level::Low, Speed::High);
    let mut sync = Output::new(p.PB6, Level::Low, Speed::High);
    sync.set_high();

    let mut sensor_clk = p.PB0;
    let mut sensor_data = p.PB1;
    let mut adc = AD7124Sync::new(spi, cs, Delay).unwrap();
    adc.init().unwrap();
    let reg_now = adc.read_id();
    info!("ID:{}", reg_now);
    let init_ctrl = adc
        .set_adc_control(
            AD7124OperatingMode::Continuous,
            AD7124PowerMode::FullPower,
            AD7124ClkSource::ClkInternal,
            true,
        )
        ;
    info!("init_ctrl:{}", init_ctrl);
    let config_set = adc
        .set_config(
            0,
            AD7124RefSource::Internal,
            AD7124GainSel::_64,
            AD7124BurnoutCurrent::Off,
            true,
        )
        ;
    info!("config_set:{}", config_set);
    let filter_set = adc
        .set_filter(0, AD7124Filter::POST, 1, AD7124PostFilter::NoPost, false, true)
        ;

    info!("filter_set:{}", filter_set);
    let adc_ch_set = adc
        .set_channel(0, AD7124Channel::AIN0, AD7124Channel::AIN1, 0, true)
        ;
    info!("adc_ch_set:{}", adc_ch_set);
    let adc_ch_set2 = adc
        .set_channel(1, AD7124Channel::AIN2, AD7124Channel::AIN3, 0, true)
        ;
    info!("adc_ch_set2:{}", adc_ch_set2);
    let pws = adc.set_pwrsw(true);
    info!("pws:{}", pws);

    adc.enable_channel(0, true).unwrap();
    adc.enable_channel(1, true).unwrap();
    let status = adc.read_register(AD7124RegId::RegStatus);
    info!("status:{}", status);
    let mut data_list = [0u32; 16];
    let errif = adc.read_multi_raw_data(&mut data_list);
    match errif {
        Ok(_) => {
            info!("read_multi_raw_data success");
            for i in 0..16 {
                info!("data_list[{}]:{}", i, data_list[i]);
            }
        }
        Err(e) => {
            info!("read_multi_raw_data error:{}", e);
        }
    }
    let ch1_data = adc.read_single_raw_data(0);
    match ch1_data {
        Ok(data) => {
            info!("ch1_data:{}", data);
        }
        Err(e) => {
            info!("ch1_data error:{}", e);
        }
    }
    let ch2_data = adc.read_single_raw_data(1);
    match ch2_data {
        Ok(data) => {
            info!("ch2_data:{}", data);
        }
        Err(e) => {
            info!("ch2_data error:{}", e);
        }
    }

    loop {

    }
}

```
