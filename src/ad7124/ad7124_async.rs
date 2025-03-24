use hal::digital::OutputPin;
use hal_async::spi;

use crate::ad7124::*;
use crate::errors::*;
use crate::regs::*;
use crate::{calc_crc8, AD7124_DEFAULT_TIMEOUT_MS, AD7124_MAX_CHANNELS};

/// pub struct AD7124<SPI, CS, Delay>
#[derive(Debug, Default)]
pub struct AD7124<T> {
    transport: T,
    core: AD7124Core,
}

impl<T> AD7124<T>
where
    T: AsyncTransport<Error = AD7124Error<()>>,
{
    /// Create a new AD7124 instance directly from a transport
    pub fn from_transport(transport: T) -> Self {
        Self {
            transport,
            core: AD7124Core::new(),
        }
    }

    /// Backward compatible creation method, directly accepts SPI, CS and Delay parameters
    pub fn new<SPI, CS, D, E1, E2>(spi: SPI, cs: CS, delay: D) -> Result<AD7124<RealAsyncTransport<SPI, CS, D>>, AD7124Error<E1>>
    where
        SPI: spi::SpiBus<Error = E1>,
        CS: OutputPin<Error = E2>,
        D: hal::delay::DelayNs,
        E1: core::fmt::Debug,
        E2: core::fmt::Debug,
    {
        let transport = RealAsyncTransport { spi, cs, delay };
        Ok(AD7124 {
            transport,
            core: AD7124Core::new(),
        })
    }

    /// reset the AD7124
    pub async fn init(&mut self) -> Result<(), T::Error> {
        self.reset().await?;
        Ok(())
    }

    /// reset the AD7124
    pub async fn reset(&mut self) -> Result<(), T::Error> {
        self.transport.set_cs(false).await?;
        self.transport.spi_write(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]).await?;
        self.transport.set_cs(true).await?;
        //self.wait_for_power_on(AD7124_DEFAULT_TIMEOUT_MS).await?;
        Ok(())
    }

    /// wait for the power on
    pub async fn wait_for_power_on(&mut self, timeout: u32) -> Result<(), T::Error> {
        let mut timeout = timeout;
        while timeout > 0 {
            let data = self.read_register(AD7124RegId::RegStatus).await?;
            if (data) != 0 {
                return Ok(());
            }
            self.transport.delay_ms(1).await?;
            timeout -= 1;
        }
        Err(AD7124Error::TimeOut)
    }

    /// set the chip select pin state
    pub async fn set_cs_state(&mut self, state: bool) -> Result<(), T::Error> {
        self.transport.set_cs(state).await
    }

    /// read the ID of the AD7124 ,Can be used to confirm ad7124 model
    pub async fn read_id(&mut self) -> Result<u32, T::Error> {
        self.read_register(AD7124RegId::RegId).await
    }

    /// use to read the register of the AD7124
    pub async fn spi_write_read(&mut self, read: &mut [u8], write: &mut [u8]) -> Result<(), AD7124Error<()>> {
        self.transport.set_cs(false).await?;
        self.transport.spi_transfer(read, write).await?;
        self.transport.set_cs(true).await?;
        Ok(())
    }

    /// use to write the register of the AD7124
    pub async fn spi_write(&mut self, write: &mut [u8]) -> Result<(), AD7124Error<()>> {
        self.transport.set_cs(false).await?;
        self.transport.spi_write(write).await?;
        self.transport.set_cs(true).await?;
        Ok(())
    }

    /// wait for the SPI bus to be ready
    pub async fn wait_spi_ready(&mut self, timeout: u32) -> Result<(), AD7124Error<()>> {
        let mut timeout = timeout;
        while timeout > 0 {
            let data = self.read_register_no_check(AD7124RegId::RegError).await?;
            if (data & AD7124_ERR_REG_SPI_IGNORE_ERR) == 0 {
                return Ok(());
            }
            self.transport.delay_ms(1).await?;
            timeout -= 1;
        }
        Err(AD7124Error::TimeOut)
    }

    /// read the register of the AD7124 without checking if ready
    /// it won't write the vitual register
    /// can be used for temporary reads outside of registers
    pub async fn read_register_no_check_direct(&mut self, size_now: u8, reg_addr: u8) -> Result<u32, AD7124Error<()>> {
        let reg_size = size_now as usize;
        let buf_len: usize = reg_size + 1 + self.core.is_crc_enabled() as usize;
        let mut buf_write = [0u8; 8];
        let mut buf_read = [0u8; 8];
        let mut msg_buf = [0u8; 8];
        
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
        buf_write.iter_mut().take(buf_len).skip(1).for_each(|b| *b = 0);
        
        self.transport.set_cs(false).await?;
        self.transport.spi_transfer(&mut buf_read[0..buf_len], &buf_write[0..buf_len]).await?;
        self.transport.set_cs(true).await?;

        if self.core.is_crc_enabled() {
            msg_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
            msg_buf[1..buf_len].copy_from_slice(&buf_read[1..buf_len]);
            let crc = calc_crc8(&msg_buf[0..buf_len]);
            if crc != 0 {
                return Err(AD7124Error::CommunicationError);
            }
        }

        let mut reg_value = 0;
        for i in 0..reg_size {
            reg_value = (reg_value << 8) | buf_read[i + 1] as u32;
        }

        Ok(reg_value)
    }

    /// read the register of the AD7124 without checking if ready
    /// Needed in existing registers
    pub async fn read_register_no_check(&mut self, reg: AD7124RegId) -> Result<u32, AD7124Error<()>> {
        let reg_info = self.core.get_register_info(reg);
        let reg_size = reg_info.size as usize;
        let reg_addr = reg as u8;
        
        if (reg_addr > 0x42) || (reg_size == 0) {
            return Err(AD7124Error::InvalidValue);
        }
        let buf_len: usize = reg_size + 1 + self.core.is_crc_enabled() as usize;
        let mut buf_write = [0u8; 8];
        let mut buf_read = [0u8; 8];
        let mut msg_buf = [0u8; 8];
        
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
        buf_write.iter_mut().take(buf_len).skip(1).for_each(|b| *b = 0);
        
        self.transport.set_cs(false).await?;
        self.transport.spi_transfer(&mut buf_read[0..buf_len], &buf_write[0..buf_len]).await?;
        self.transport.set_cs(true).await?;

        if self.core.is_crc_enabled() {
            msg_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
            msg_buf[1..buf_len].copy_from_slice(&buf_read[1..buf_len]);
            let crc = calc_crc8(&msg_buf[0..buf_len]);
            if crc != 0 {
                return Err(AD7124Error::CommunicationError);
            }
        }

        let mut reg_value = 0;
        for i in 0..reg_size {
            reg_value = (reg_value << 8) | buf_read[i + 1] as u32;
        }

        self.core.update_register_value(reg, reg_value);
        Ok(reg_value)
    }

    /// write the register of the AD7124 without checking if ready
    pub async fn write_register_no_check(&mut self, reg: AD7124RegId, value: u32) -> Result<(), AD7124Error<()>> {
        let reg_info = self.core.get_register_info(reg);
        let reg_size = reg_info.size as usize;
        let reg_addr = reg as u8;
        
        if (reg_addr > 0x42) || (reg_size == 0) {
            return Err(AD7124Error::InvalidValue);
        }
        let buf_len: usize = reg_size + self.core.is_crc_enabled() as usize + 1;
        let mut buf_write = [0u8; 8];
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA(reg_addr);
        let mut value_now = value;
        for i in 0..reg_size {
            buf_write[reg_size - i] = value_now as u8;
            value_now >>= 8;
        }
        if self.core.is_crc_enabled() {
            let crc = calc_crc8(&buf_write[0..buf_len]);
            buf_write[buf_len] = crc;
        }
        let buf_write = &buf_write[0..buf_len + self.core.is_crc_enabled() as usize];
        
        self.transport.set_cs(false).await?;
        self.transport.spi_write(buf_write).await?;
        self.transport.set_cs(true).await?;

        self.core.update_register_value(reg, value);
        Ok(())
    }

    /// read the register of the AD7124
    /// wait for the SPI bus to be ready
    pub async fn read_register(&mut self, reg: AD7124RegId) -> Result<u32, AD7124Error<()>> {
        self.wait_spi_ready(AD7124_DEFAULT_TIMEOUT_MS).await?;
        self.read_register_no_check(reg).await
    }

    /// write the register of the AD7124
    /// wait for the SPI bus to be ready
    pub async fn write_register(&mut self, reg: AD7124RegId, value: u32) -> Result<(), AD7124Error<()>> {
        self.wait_spi_ready(AD7124_DEFAULT_TIMEOUT_MS).await?;
        self.write_register_no_check(reg, value).await
    }

    /// check the AD7124 id
    /// for ad7124-8 : 0x12
    /// for ad7124-4 : 0x04
    pub async fn self_check(&mut self) -> Result<(), AD7124Error<()>> {
        let id = self.read_id().await?;
        match self.core.verify_chip_id(id) {
            Ok(_) => Ok(()),
            Err(_) => Err(AD7124Error::InvalidValue),
        }
    }

    /// set the ADC control register
    pub async fn set_adc_control(&mut self, mode: AD7124OperatingMode, power_mode: AD7124PowerMode, clk_sel: AD7124ClkSource, ref_en: bool) -> Result<(), AD7124Error<()>> {
        // Defining individual configuration options
        let mode_option = AD7124_ADC_CTRL_REG_MODE(mode as u16);
        let power_mode_option = AD7124_ADC_CTRL_REG_POWER_MODE(power_mode as u16);
        let clk_sel_option = AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel as u16);
        let ref_en_option = if ref_en { AD7124_ADC_CTRL_REG_REF_EN } else { 0 };
        let data_status_option = AD7124_ADC_CTRL_REG_DATA_STATUS;
        let dout_rdy_del_option = AD7124_ADC_CTRL_REG_DOUT_RDY_DEL;
        // Combined Configuration Options
        let current_val = mode_option | power_mode_option | clk_sel_option | ref_en_option | data_status_option | dout_rdy_del_option;
        self.write_register(AD7124RegId::RegControl, current_val as u32).await?;
        self.core.set_op_mode(mode);
        Ok(())
    }

    /// set the configuration
    /// setup_index: 0-7 .Corresponds to Eight configurations
    pub async fn set_config(&mut self, setup_index: u8, ref_now: AD7124RefSource, gain: AD7124GainSel, burnout_current: AD7124BurnoutCurrent, bipolar: bool) -> Result<(), AD7124Error<()>> {
        // Defining individual configuration options
        let ref_sel_option = AD7124_CFG_REG_REF_SEL(ref_now as u16);
        let pga_option = AD7124_CFG_REG_PGA(gain as u16);
        let bipolar_option = if bipolar { AD7124_CFG_REG_BIPOLAR } else { 0 };
        let burnout_option = AD7124_CFG_REG_BURNOUT(burnout_current as u16);
        let ref_buf_option = AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_REF_BUFM;
        let ain_buf_option = AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM;

        // Combined Configuration Options
        let current_val = ref_sel_option | pga_option | bipolar_option | burnout_option | ref_buf_option | ain_buf_option;
        let reg_now = AD7124RegId::from(AD7124RegId::RegConfig0 as u8 + setup_index);

        self.write_register(reg_now, current_val.into()).await
    }

    /// set the filter and output data rate
    pub async fn set_filter(&mut self, setup_index: u8, filter: AD7124Filter, fs: u16, post_filter: AD7124PostFilter, rej60: bool, single: bool) -> Result<(), AD7124Error<()>> {
        let filter_option = AD7124_FILT_REG_FILTER(filter as u32);
        let post_filter_option = AD7124_FILT_REG_POST_FILTER(post_filter as u32);
        let fs_option = AD7124_FILT_REG_FS(fs as u32);
        let rej60_option = if rej60 { AD7124_FILT_REG_REJ60 } else { 0 };
        let single_cycle_option = if single { AD7124_FILT_REG_SINGLE_CYCLE } else { 0 };

        let current_val = filter_option | post_filter_option | fs_option | rej60_option | single_cycle_option;
        let reg_now = AD7124RegId::from(AD7124RegId::RegFilter0 as u8 + setup_index);
        self.write_register(reg_now, current_val).await
    }

    /// configure channel
    pub async fn set_channel(&mut self, channel_index: u8, ainp: AD7124Channel, ainm: AD7124Channel, setup_index: u8, enable: bool) -> Result<(), AD7124Error<()>> {
        let ainp_option = AD7124_CH_MAP_REG_AINP(ainp as u16);
        let ainm_option = AD7124_CH_MAP_REG_AINM(ainm as u16);
        let setup_option = AD7124_CH_MAP_REG_SETUP(setup_index as u16);
        let enable_option = if enable { AD7124_CH_MAP_REG_CH_ENABLE } else { 0 };
        let current_val = ainp_option | ainm_option | setup_option | enable_option;
        let reg_now = AD7124RegId::from(AD7124RegId::RegChannel0 as u8 + channel_index);
        self.write_register(reg_now, current_val as u32).await
    }

    /// set the offset calibration for a setup
    pub async fn set_offset_calibration(&mut self, setup_index: u8, offset: u32) -> Result<(), AD7124Error<()>> {
        let reg = AD7124RegId::from(AD7124RegId::RegOffset0 as u8 + setup_index);
        self.write_register(reg, offset).await
    }

    /// set the gain calibration for a setup
    pub async fn set_gain_calibration(&mut self, setup_index: u8, gain: u32) -> Result<(), AD7124Error<()>> {
        let reg = AD7124RegId::from(AD7124RegId::RegGain0 as u8 + setup_index);
        self.write_register(reg, gain).await
    }

    /// enable (close) the on-chip low side power switch.
    pub async fn set_pwrsw(&mut self, enable: bool) -> Result<(), AD7124Error<()>> {
        //let mut current_value = self.read_register(AD7124RegId::RegIocon1).await?;//REGS[AD7124RegId::RegIocon1 as usize].value;
        let mut current_value = self.core.get_register_value(AD7124RegId::RegIocon1);
        if enable {
            current_value |= AD7124_IO_CTRL1_REG_PDSW
        } else {
            current_value &= !AD7124_IO_CTRL1_REG_PDSW
        };
        self.write_register(AD7124RegId::RegIocon1, current_value).await
    }

    /// Sets enables the bias voltage generator on the given pin.
    /// A bias voltage is necessary to read truly bipolar output from thermocouples
    pub async fn set_vbias(&mut self, vbias_pin: AD7124VBiasPin, enable: bool) -> Result<(), AD7124Error<()>> {
        //let mut current_value = self.read_register(AD7124RegId::RegIocon2).await?;//REGS[AD7124RegId::RegIocon2 as usize].value;
        let mut current_value = self.core.get_register_value(AD7124RegId::RegIocon2);
        if enable {
            current_value |= 1 << vbias_pin as u32
        } else {
            current_value &= !(1 << vbias_pin as u32)
        };
        self.write_register(AD7124RegId::RegIocon2, current_value).await
    }

    /// get a reading in raw counts from a single channel
    pub async fn read_single_raw_data(&mut self, channel: u8) -> Result<u32, AD7124Error<()>> {
        let active_channel = self.get_active_channel().await?;
        
        if active_channel != channel {
            self.enable_channel(active_channel, false).await?;
            if self.core.get_op_mode() == AD7124OperatingMode::SingleConv {
                self.enable_channel(channel, true).await?;
                self.set_op_mode(AD7124OperatingMode::SingleConv).await?;
            } else {
                self.enable_channel(channel, true).await?;
            }
        } else if self.core.get_op_mode() == AD7124OperatingMode::SingleConv {
            self.set_op_mode(AD7124OperatingMode::SingleConv).await?;
        }
        
        self.wait_conv_ready(AD7124_DEFAULT_TIMEOUT_MS).await?;
        let data_now = self.get_data().await?;
        Ok(data_now)
    }

    /// check if a channel is enabled
    #[inline]
    pub fn check_if_enabled(&mut self, ch: u8) -> bool {
        let channel = ch + AD7124RegId::RegChannel0 as u8;
        let reg_value = self.core.get_register_value(AD7124RegId::from(channel));
        reg_value & AD7124_CH_MAP_REG_CH_ENABLE as u32 != 0
    }

    /// read multiple channels in raw counts
    pub async fn read_multi_raw_data(&mut self, buffer: &mut [u32; 16]) -> Result<(), AD7124Error<()>> {
        //let active_channel = self.get_active_channel().await?;
        if self.core.get_op_mode() == AD7124OperatingMode::SingleConv {
            self.set_op_mode(AD7124OperatingMode::SingleConv).await?;
        }
        let first_channel: u8;
        //let mut first_channel: u8 = 0;
        // for i in 0..AD7124_MAX_CHANNELS {
        //     if self.check_if_enabled(i) {
        //     first_channel = i;
        //     break;
        //     }
        // }
        let channel_select = (0..AD7124_MAX_CHANNELS).find(|&i| self.check_if_enabled(i));
        if let Some(channel) = channel_select {
            first_channel = channel;
        } else {
            return Err(AD7124Error::InvalidValue);
        }

        // In order to make sure we don't start filling the buffer from
        // the middle of a sampling sequence, we will wait until the
        // first array element corresponds to the first enabled channel.
        // (This should only happen at high data rates in continuous mode)
        let mut data_started = false;
        let mut data_ended = false;

        while !data_ended {
            match self.wait_conv_ready(AD7124_DEFAULT_TIMEOUT_MS).await {
                Ok(()) => {
                    let channel = self.get_active_channel().await?;
                    if channel == first_channel && !data_started {
                        data_started = true;
                    } else if channel == first_channel && data_started {
                        data_ended = true;
                    }

                    if data_started && !data_ended {
                        buffer[channel as usize] = self.get_data().await?;
                    }
                }
                Err(AD7124Error::TimeOut) => return Err(AD7124Error::TimeOut),
                _ => continue,
            }
        }

        Ok(())
    }

    /// get current channel
    pub async fn current_channel(&mut self) -> Result<u8, AD7124Error<()>> {
        let data = self.read_register(AD7124RegId::RegStatus).await?;
        Ok(data as u8 & AD7124_STATUS_REG_CH_ACTIVE(0x0F))
    }

    /// get the data of adc data register
    pub async fn get_data(&mut self) -> Result<u32, AD7124Error<()>> {
        self.read_register(AD7124RegId::RegData).await
    }

    /// Get the data of ADC data register fast way
    pub async fn get_data_fast(&mut self) -> Result<u32, AD7124Error<()>> {
        // Temporary reg struct for data with extra byte to hold status bits
        let reg_size = 4;  // Data register size + status byte
        let reg_addr = 0x02;  // Data register address
        
        let buf_len = reg_size as usize + 1 + self.core.is_crc_enabled() as usize;
        let mut buf_write = [0u8; 8];
        let mut buf_read = [0u8; 8];
        
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
        
        self.transport.spi_transfer(&mut buf_read[0..buf_len], &buf_write[0..buf_len]).await?;
        
        if self.core.is_crc_enabled() {
            let crc = calc_crc8(&buf_read[0..buf_len]);
            if crc != 0 {
                return Err(AD7124Error::CommunicationError);
            }
        }
        
        // Update register values
        let status_value = buf_read[1] as u32;
        let data_value = buf_read[2..buf_len].iter().fold(0, |acc, &b| (acc << 8) + b as u32);
        
        self.core.update_register_value(AD7124RegId::RegStatus, status_value);
        self.core.update_register_value(AD7124RegId::RegData, data_value);
        
        Ok(data_value)
    }

    /// wait for the conversion to be ready
    pub async fn wait_conv_ready(&mut self, timeout: u32) -> Result<(), AD7124Error<()>> {
        let mut timeout = timeout;
        while timeout > 0 {
            let data = self.read_register_no_check(AD7124RegId::RegStatus).await?;
            if (data & AD7124_STATUS_REG_RDY as u32) == 0 {
                return Ok(());
            }
            self.transport.delay_ms(1).await?;
            timeout -= 1;
        }
        Err(AD7124Error::TimeOut)
    }

    /// set operating mode of the ADC
    pub async fn set_op_mode(&mut self, mode: AD7124OperatingMode) -> Result<(), AD7124Error<()>> {
        let mut current_val = self.core.get_register_value(AD7124RegId::RegControl);
        current_val &= !AD7124_ADC_CTRL_REG_MODE(0xF) as u32;
        current_val |= AD7124_ADC_CTRL_REG_MODE(mode as u16) as u32;
        self.write_register(AD7124RegId::RegControl, current_val).await?;
        self.core.set_op_mode(mode);
        Ok(())
    }

    /// read the error register
    pub async fn read_error_register(&mut self) -> Result<u32, AD7124Error<()>> {
        self.read_register(AD7124RegId::RegError).await
    }

    /// enable channel
    pub async fn enable_channel(&mut self, channel: u8, enable: bool) -> Result<(), AD7124Error<()>> {
        if channel >= 16 {
            return Err(AD7124Error::InvalidValue);
        }
        let reg_now = AD7124RegId::from(AD7124RegId::RegChannel0 as u8 + channel);
        let mut current_value = self.read_register(reg_now).await?;
        if enable {
            current_value |= AD7124_CH_MAP_REG_CH_ENABLE as u32;
        } else {
            current_value &= !AD7124_CH_MAP_REG_CH_ENABLE as u32;
        }
        self.write_register(reg_now, current_value).await
    }

    //get active channel
    pub async fn get_active_channel(&mut self) -> Result<u8, AD7124Error<()>> {
        let data = self.read_register(AD7124RegId::RegStatus).await?;
        Ok(data as u8 & 0x07)
    }
}
