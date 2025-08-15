#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Output, Level, Speed},
    spi::{Spi, Config as SpiConfig, Mode, Polarity, Phase},
    time::Hertz,
};
use embassy_time::{Duration, Timer, Delay};
use ad7124_rs::{
    device_type::DeviceType,
    registers::{ChannelInput, PgaGain, FilterType, PowerMode, OperatingMode, ClockSource, ReferenceSource, BurnoutCurrent},
    ad7124::{AD7124Config, ChannelConfig, SetupConfig, FilterConfig, AD7124Async},
    transport::SpiTransport,
};
use {defmt_rtt as _, panic_halt as _};


const REFERENCE_VOLTAGE: f32 = 2.5; // Internal reference voltage

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        // Use HSI clock (16 MHz)
        config.rcc.hsi = true;
        config.rcc.hse = None;
        // Use HSI as system clock directly for simplicity
        config.rcc.sys = Sysclk::HSI;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        // Simple configuration for testing
    }
    let p = embassy_stm32::init(config);
    
    info!("AD7124 Embassy Example Starting...");
    
    // Configure LED status indicator (using PC13 for STM32G431CB)
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);
    
    // Configure AD7124 SPI pins for STM32G4 SPI2
    let cs_pin = Output::new(p.PB12, Level::High, Speed::High);
    
    // SYNC pin configuration  
    let mut _sync = Output::new(p.PA11, Level::High, Speed::High);
    
    // Configure SPI for AD7124
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(1_000_000);  // 1MHz SPI clock
    spi_config.mode = Mode {
        polarity: Polarity::IdleHigh,  // CPOL = 1
        phase: Phase::CaptureOnSecondTransition,  // CPHA = 1
    };
    
    info!("SPI configuration: Mode 3 (CPOL=1, CPHA=1), 1MHz");
    
    // Initialize async SPI2 with DMA for STM32G4
    let spi = Spi::new(p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH1, p.DMA1_CH2, spi_config);
    
    info!("SPI configured successfully");
    
    // Create SPI transport adapter
    let delay = Delay;
    let transport = SpiTransport::new(spi, cs_pin, delay);
    
    info!("SPI transport created successfully");
    
    // Initialize AD7124 driver
    let mut adc = match AD7124Async::new(transport, DeviceType::AD7124_8) {
        Ok(driver) => driver,
        Err(_) => {
            info!("Failed to create AD7124 driver");
            loop {
                led.set_low();
                Timer::after(Duration::from_millis(100)).await;
                led.set_high();
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };
    
    info!("Initializing AD7124...");
    
    // Initialize AD7124 configuration
    match adc.init().await {
        Ok(_) => {
            let device_id = adc.read_device_id().await.unwrap_or(0);
            info!("AD7124 initialized successfully, Device ID: 0x{:02X}", device_id);
        },
        Err(_) => {
            info!("Failed to initialize AD7124");
            loop {
                led.set_low();
                Timer::after(Duration::from_millis(200)).await;
                led.set_high();
                Timer::after(Duration::from_millis(200)).await;
            }
        }
    }
    
    // Configure ADC for continuous conversion
    let adc_config = AD7124Config {
        operating_mode: OperatingMode::Continuous,
        power_mode: PowerMode::FullPower,
        clock_source: ClockSource::Internal,
        reference_source: ReferenceSource::Internal,
        internal_ref_enabled: true,
        data_ready_output_enabled: false,
    };
    
    match adc.configure_adc(adc_config).await {
        Ok(_) => info!("ADC configuration complete"),
        Err(_) => info!("Failed to configure ADC"),
    }
    
    // Setup configurations for different measurement types
    let configurations = [
        ("Single-ended AIN0", ChannelInput::Ain0, ChannelInput::Dgnd, PgaGain::Gain1),
        ("Single-ended AIN1", ChannelInput::Ain1, ChannelInput::Dgnd, PgaGain::Gain1),
        ("Differential AIN0-AIN1", ChannelInput::Ain0, ChannelInput::Ain1, PgaGain::Gain1),
        ("Temperature Sensor", ChannelInput::TempSensor, ChannelInput::Dgnd, PgaGain::Gain1),
        ("Internal Reference", ChannelInput::IntRef, ChannelInput::Dgnd, PgaGain::Gain1),
        ("High-gain AIN2", ChannelInput::Ain2, ChannelInput::Dgnd, PgaGain::Gain64),
    ];
    
    let mut config_index = 0;
    
    info!("AD7124 configuration complete");
    
    // Demonstrate enhanced channel management features
    enhanced_channel_management_demo(&mut adc).await;
    
    // Main loop - cycle through different configurations and measurements
    loop {
        // LED indicator
        led.toggle();
        
        let (config_name, pos_input, neg_input, gain) = &configurations[config_index];
        
        info!("=== {} Demo ===", config_name);
        
        // Configure setup 0 for this measurement
        let setup_config = SetupConfig {
            pga_gain: *gain,
            reference_source: ReferenceSource::Internal,
            bipolar: true,
            burnout_current: BurnoutCurrent::Off,
            reference_buffers_enabled: true,
            input_buffers_enabled: true,
        };
        
        match adc.configure_setup(0, setup_config).await {
            Ok(_) => info!("Setup configured for gain: {}", gain.value()),
            Err(_) => info!("Failed to configure setup"),
        }
        
        // Configure filter for this measurement
        let filter_config = FilterConfig {
            filter_type: FilterType::Sinc4,
            output_data_rate: 50, // 50 Hz
            single_cycle: false,
            reject_60hz: true,
        };
        
        match adc.configure_filter(0, filter_config).await {
            Ok(_) => info!("Filter configured"),
            Err(_) => info!("Failed to configure filter"),
        }
        
        // Configure channel 0 for this measurement
        let channel_config = ChannelConfig {
            enabled: true,
            positive_input: *pos_input,
            negative_input: *neg_input,
            setup_index: 0,
        };
        
        match adc.configure_channel(0, channel_config).await {
            Ok(_) => info!("Channel configured: {} - {}", *pos_input as u8, *neg_input as u8),
            Err(_) => info!("Failed to configure channel"),
        }
        
        // Wait for ADC to settle
        Timer::after(Duration::from_millis(100)).await;
        
        // Take multiple readings and average
        let mut voltage_sum = 0.0f32;
        let mut successful_readings = 0u32;
        const NUM_READINGS: u32 = 10;
        
        for i in 0..NUM_READINGS {
            // Wait for data ready
            match adc.wait_for_data_ready(100).await {
                Ok(_) => {
                    // Read voltage
                    match adc.read_voltage(0).await {
                        Ok(voltage) => {
                            voltage_sum += voltage;
                            successful_readings += 1;
                            if i == 0 {
                                info!("First reading: {}V", voltage);
                            }
                        },
                        Err(_) => {
                            info!("Failed to read voltage (attempt {})", i + 1);
                        }
                    }
                },
                Err(_) => {
                    info!("Timeout waiting for data ready (attempt {})", i + 1);
                }
            }
            
            Timer::after(Duration::from_millis(20)).await;
        }
        
        if successful_readings > 0 {
            let average_voltage = voltage_sum / (successful_readings as f32);
            info!("{}: {}V (avg of {} readings)", 
                  config_name, average_voltage, successful_readings);
            
            // Special handling for different input types
            match pos_input {
                ChannelInput::TempSensor => {
                    // Convert to temperature (approximate)
                    // AD7124 temperature sensor: ~1.17V at 25°C, ~1.8mV/°C
                    let temp_c = 25.0 + (average_voltage - 1.17) / 0.0018;
                    info!("Estimated temperature: {}°C", temp_c as u32);
                },
                ChannelInput::IntRef => {
                    info!("Internal reference measurement: {}V (should be ~2.5V)", average_voltage);
                },
                _ => {
                    // Regular voltage measurement
                    let percentage = (average_voltage / REFERENCE_VOLTAGE) * 100.0;
                    info!("Voltage: {}V ({}% of reference)", average_voltage, percentage as u32);
                }
            }
        } else {
            info!("No successful readings for {}", config_name);
        }
        
        // Display current status
        match adc.read_status().await {
            Ok(status) => {
                info!("Status: ready={}, error={}", status.rdy(), status.error());
            },
            Err(_) => {
                info!("Failed to read status");
            }
        }
        
        // Hold current configuration for a moment
        Timer::after(Duration::from_millis(2000)).await;
        
        // Move to next configuration
        config_index = (config_index + 1) % configurations.len();
        if config_index == 0 {
            info!("--- Cycling back to first configuration ---");
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}

/// Demonstrate enhanced channel management features
async fn enhanced_channel_management_demo(adc: &mut AD7124Async<SpiTransport<embassy_stm32::spi::Spi<'_, embassy_stm32::mode::Async>, Output<'_>, Delay>>) {
    info!("=== Enhanced Channel Management Demo ===");
    
    // Setup multiple channels for demonstration
    let channels_to_setup = [0, 1, 2, 3];
    
    // Configure channels
    for &channel in &channels_to_setup {
        let channel_config = ChannelConfig {
            enabled: true,
            positive_input: match channel {
                0 => ChannelInput::Ain0,
                1 => ChannelInput::Ain1,
                2 => ChannelInput::Ain2,
                3 => ChannelInput::Ain3,
                _ => ChannelInput::Ain0,
            },
            negative_input: ChannelInput::Dgnd,
            setup_index: 0,
        };
        
        match adc.configure_channel(channel, channel_config).await {
            Ok(_) => info!("Configured channel {}", channel),
            Err(_) => info!("Failed to configure channel {}", channel),
        }
    }
    
    // Configure setup for measurements
    let setup_config = SetupConfig {
        pga_gain: PgaGain::Gain1,
        reference_source: ReferenceSource::Internal,
        bipolar: true,
        burnout_current: BurnoutCurrent::Off,
        reference_buffers_enabled: true,
        input_buffers_enabled: true,
    };
    
    match adc.configure_setup(0, setup_config).await {
        Ok(_) => info!("Setup configured for enhanced demo"),
        Err(_) => info!("Failed to configure setup"),
    }
    
    // Test 1: Enable/disable specific channels
    info!("--- Test 1: Channel Enable/Disable ---");
    for &channel in &channels_to_setup {
        match adc.enable_channel(channel, true).await {
            Ok(_) => {
                match adc.is_channel_enabled(channel).await {
                    Ok(enabled) => info!("Channel {} enabled: {}", channel, enabled),
                    Err(_) => info!("Failed to check if channel {} is enabled", channel),
                }
            },
            Err(_) => info!("Failed to enable channel {}", channel),
        }
    }
    
    // Test 2: Check current active channel
    info!("--- Test 2: Active Channel Status ---");
    match adc.get_active_channel().await {
        Ok(active_channel) => {
            info!("Current active channel: {}", active_channel);
            
            // Also check using current_channel method
            match adc.current_channel().await {
                Ok(current) => info!("Current channel (via status): {}", current),
                Err(_) => info!("Failed to get current channel from status"),
            }
        },
        Err(_) => info!("Failed to get active channel"),
    }
    
    // Test 3: Non-blocking data ready checking
    info!("--- Test 3: Non-blocking Data Ready Check ---");
    let mut attempts = 0;
    let max_attempts = 5;
    
    while attempts < max_attempts {
        match adc.is_data_ready().await {
            Ok(ready) => {
                if ready {
                    info!("Data is ready!");
                    
                    // Read data with status information
                    match adc.read_data_with_status().await {
                        Ok((channel, data)) => {
                            info!("Read from channel {}: 0x{:06X}", channel, data);
                        },
                        Err(_) => info!("Failed to read data with status"),
                    }
                    break;
                } else {
                    info!("Data not ready yet, attempt {}/{}", attempts + 1, max_attempts);
                }
            },
            Err(_) => info!("Failed to check data ready status"),
        }
        
        Timer::after(Duration::from_millis(50)).await;
        attempts += 1;
    }
    
    // Test 4: Channel-specific reading
    info!("--- Test 4: Channel-Specific Reading ---");
    for &channel in &[0, 1, 2] {  // Test first 3 channels
        match adc.read_channel_data(channel).await {
            Ok(raw_data) => {
                info!("Channel {} raw data: 0x{:06X}", channel, raw_data);
                
                // Also read voltage
                match adc.read_channel_voltage(channel).await {
                    Ok(voltage) => {
                        info!("Channel {} voltage: {}mV", channel, (voltage * 1000.0) as i32);
                    },
                    Err(_) => info!("Failed to read voltage from channel {}", channel),
                }
            },
            Err(_) => info!("Failed to read raw data from channel {}", channel),
        }
        
        Timer::after(Duration::from_millis(100)).await;
    }
    
    // Test 5: Multi-channel operations
    info!("--- Test 5: Multi-Channel Operations ---");
    let test_channels = [0, 1, 2];
    
    // Read multiple channels at once
    match adc.read_multi_channel(&test_channels).await {
        Ok(results) => {
            info!("Multi-channel raw data:");
            for (channel, data) in results {
                info!("  Channel {}: 0x{:06X}", channel, data);
            }
        },
        Err(_) => info!("Failed to read multiple channels"),
    }
    
    Timer::after(Duration::from_millis(200)).await;
    
    // Read multiple voltages at once
    match adc.read_multi_voltage(&test_channels).await {
        Ok(results) => {
            info!("Multi-channel voltages:");
            for (channel, voltage) in results {
                info!("  Channel {}: {}mV", channel, (voltage * 1000.0) as i32);
            }
        },
        Err(_) => info!("Failed to read multiple voltages"),
    }
    
    // Test 6: Scan all enabled channels
    info!("--- Test 6: Scan Enabled Channels ---");
    match adc.get_enabled_channels().await {
        Ok(enabled_channels) => {
            info!("Enabled channels: {:?}", enabled_channels.as_slice());
            
            // Scan and read all enabled channels
            match adc.scan_enabled_channels().await {
                Ok(results) => {
                    info!("Scan results:");
                    for (channel, data) in results {
                        info!("  Enabled channel {}: 0x{:06X}", channel, data);
                    }
                },
                Err(_) => info!("Failed to scan enabled channels"),
            }
        },
        Err(_) => info!("Failed to get enabled channels list"),
    }
    
    // Test 7: Fast data read (no status check)
    info!("--- Test 7: Fast Data Read ---");
    match adc.read_data_fast().await {
        Ok(fast_data) => {
            info!("Fast read data: 0x{:06X}", fast_data);
        },
        Err(_) => info!("Failed to perform fast data read"),
    }
    
    info!("=== Enhanced Channel Management Demo Complete ===");
    Timer::after(Duration::from_millis(1000)).await;
}