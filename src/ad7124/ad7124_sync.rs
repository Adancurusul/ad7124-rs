use hal::delay::DelayNs;
use hal::digital::OutputPin;
use hal::spi;

use crate::defs::*;
use crate::errors::*;
use crate::regs::*;
use crate::{calc_crc8, AD7124_DEFAULT_TIMEOUT_MS, AD7124_MAX_CHANNELS};

/// pub struct AD7124<SPI, CS, Delay>
#[derive(Debug, Default)]
pub struct AD7124<SPI, CS, Delay> {
    spi: SPI,
    cs: CS,
    delay: Delay,
    crc_enabled: bool,
    op_mode: AD7124OperatingMode,
}

impl<SPI, CS, Delay, E> AD7124<SPI, CS, Delay>
where
    SPI: spi::SpiBus<Error = E>,
    CS: OutputPin,
    Delay: DelayNs,
{
    pub fn new(spi: SPI, cs: CS, delay: Delay) -> Result<Self, E> {
        let ad7124 = AD7124 {
            spi,
            cs,
            delay,
            crc_enabled: false,
            op_mode: AD7124OperatingMode::Continuous,
        };
        Ok(ad7124)
    }

    /// reset the AD7124
    pub fn init(&mut self) -> Result<(), AD7124Error<E>> {
        self.reset()?;
        Ok(())
    }

    /// reset the AD7124
    pub fn reset(&mut self) -> Result<(), AD7124Error<E>> {
        self.cs.set_low().unwrap();
        self.spi.write(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]).map_err(AD7124Error::SPI)?;
        self.cs.set_high().unwrap();
        //self.wait_for_power_on(AD7124_DEFAULT_TIMEOUT_MS)?;
        Ok(())
    }

    /// wait for the power on
    pub fn wait_for_power_on(&mut self, timeout: u32) -> Result<(), AD7124Error<E>> {
        let mut timeout = timeout;
        while timeout > 0 {
            let data = self.read_register(AD7124RegId::RegStatus)?;
            if (data) != 0 {
                return Ok(());
            }
            self.delay.delay_ms(1);
            timeout -= 1;
        }
        Err(AD7124Error::TimeOut)
    }

    /// set the chip select pin state
    pub fn set_cs_state(&mut self, state: bool) -> Result<(), <CS as hal::digital::ErrorType>::Error> {
        self.cs.set_state(hal::digital::PinState::from(state)).map_err(|e| e)
    }

    /// read the ID of the AD7124 ,Can be used to confirm ad7124 model
    pub fn read_id(&mut self) -> Result<u32, AD7124Error<E>> {
        self.read_register(AD7124RegId::RegId)
    }

    /// use to read the register of the AD7124
    pub fn spi_write_read(&mut self, read: &mut [u8], write: &mut [u8]) -> Result<(), AD7124Error<E>> {
        self.cs.set_low().unwrap();
        self.spi.transfer(read, write).map_err(AD7124Error::SPI)?;
        self.cs.set_high().unwrap();
        Ok(())
    }

    /// use to write the register of the AD7124
    pub fn spi_write(&mut self, write: &mut [u8]) -> Result<(), AD7124Error<E>> {
        self.cs.set_low().unwrap();
        self.spi.write(write).map_err(AD7124Error::SPI)?;
        self.cs.set_high().unwrap();
        Ok(())
    }

    /// wait for the SPI bus to be ready
    pub fn wait_spi_ready(&mut self, timeout: u32) -> Result<(), AD7124Error<E>> {
        let mut timeout = timeout;
        while timeout > 0 {
            let data = self.read_register_no_check(AD7124RegId::RegError)?;
            if (data & AD7124_ERR_REG_SPI_IGNORE_ERR) == 0 {
                return Ok(());
            }
            self.delay.delay_ms(1);
            timeout -= 1;
        }
        Err(AD7124Error::TimeOut)
    }

    /// read the register of the AD7124 without checking if ready
    /// it won't write the vitual register
    /// can be used for temporary reads outside of registers
    pub fn read_register_no_check_direct(&mut self, size_now: u8, reg_addr: u8) -> Result<u32, AD7124Error<E>> {
        let reg_size = size_now as usize;
        let buf_len: usize = reg_size + 1 + self.crc_enabled as usize;
        let mut buf_write = [0u8; 8];
        let mut buf_read = [0u8; 8];
        let mut msg_buf = [0u8; 8];
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
        let buf_write = &mut buf_write[0..buf_len];
        let buf_read = &mut buf_read[0..buf_len];
        self.spi_write_read(buf_read, buf_write)?;

        if self.crc_enabled {
            msg_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
            msg_buf[1..buf_len].copy_from_slice(&buf_read[1..buf_len]);
            let crc = calc_crc8(&buf_read[0..buf_len]);
            if crc != 0 {
                return Err(AD7124Error::CommunicationError);
            }
        }
        let reg_value = buf_read[1..buf_len].iter().fold(0, |acc, &b| (acc << 8) + b as u32);

        Ok(reg_value)
    }

    /// read the register of the AD7124 without checking if ready
    /// Needed in existing registers
    pub fn read_register_no_check(&mut self, reg: AD7124RegId) -> Result<u32, AD7124Error<E>> {
        let reg_now = unsafe { REGS[reg as usize] };
        let reg_size = reg_now.size as usize;
        let reg_addr = reg_now.addr;
        if reg_now.rw == AD7124RW::Write {
            return Err(AD7124Error::InvalidValue);
        }
        let buf_len: usize = reg_size + 1 + self.crc_enabled as usize;
        let mut buf_write = [0u8; 8];
        let mut buf_read = [0u8; 8];
        let mut msg_buf = [0u8; 8];
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
        let buf_write = &mut buf_write[0..buf_len];
        let buf_read = &mut buf_read[0..buf_len];
        self.spi_write_read(buf_read, buf_write)?;

        if self.crc_enabled {
            msg_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
            msg_buf[1..buf_len].copy_from_slice(&buf_read[1..buf_len]);
            let crc = calc_crc8(&buf_read[0..buf_len]);
            if crc != 0 {
                return Err(AD7124Error::CommunicationError);
            }
        }
        let reg_value = buf_read[1..buf_len].iter().fold(0, |acc, &b| (acc << 8) + b as u32);

        unsafe {
            REGS[reg as usize].value = reg_value;
        }
        Ok(reg_value)
    }

    /// write the register of the AD7124 without checking if ready
    pub fn write_register_no_check(&mut self, reg: AD7124RegId, value: u32) -> Result<(), AD7124Error<E>> {
        let reg_now = unsafe { REGS[reg as usize] };
        let reg_size = reg_now.size as usize;
        let reg_addr = reg_now.addr;
        if reg_now.rw == AD7124RW::Read {
            return Err(AD7124Error::InvalidValue);
        }
        let buf_len: usize = reg_size + self.crc_enabled as usize + 1;
        let mut buf_write = [0u8; 8];
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA(reg_addr);
        let mut value_now = value;
        for i in 0..reg_size {
            buf_write[reg_size - i] = value_now as u8 & 0xFF;
            value_now >>= 8;
        }
        // for i in 1..buf_len {
        //     buf_write[i] = (value >> (8 * (buf_len - i - 1))) as u8;
        // }
        if self.crc_enabled {
            let crc = calc_crc8(&buf_write[0..buf_len]);
            buf_write[buf_len] = crc;
        }
        let buf_write = &mut buf_write[0..buf_len];
        self.spi_write(buf_write)?;

        unsafe {
            REGS[reg as usize].value = value;
        } //
        Ok(())
    }

    /// read the register of the AD7124
    /// wait for the SPI bus to be ready
    pub fn read_register(&mut self, reg: AD7124RegId) -> Result<u32, AD7124Error<E>> {
        self.wait_spi_ready(AD7124_DEFAULT_TIMEOUT_MS)?;
        self.read_register_no_check(reg)
    }

    /// write the register of the AD7124
    /// wait for the SPI bus to be ready
    pub fn write_register(&mut self, reg: AD7124RegId, value: u32) -> Result<(), AD7124Error<E>> {
        self.wait_spi_ready(AD7124_DEFAULT_TIMEOUT_MS)?;
        self.write_register_no_check(reg, value)
    }

    /// check the AD7124 id
    /// for ad7124-8 : 0x12
    /// for ad7124-4 : 0x04
    pub fn self_check(&mut self) -> Result<(), AD7124Error<E>> {
        let id = self.read_register(AD7124RegId::RegId)?;
        if id == 0x12 || id == 0x04 {
            return Ok(());
        } else {
            return Err(AD7124Error::InvalidValue);
        }
    }

    /// set the ADC control register
    pub fn set_adc_control(&mut self, mode: AD7124OperatingMode, power_mode: AD7124PowerMode, clk_sel: AD7124ClkSource, ref_en: bool) -> Result<(), AD7124Error<E>> {
        // Defining individual configuration options
        let mode_option = AD7124_ADC_CTRL_REG_MODE(mode as u16);
        let power_mode_option = AD7124_ADC_CTRL_REG_POWER_MODE(power_mode as u16);
        let clk_sel_option = AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel as u16);
        let ref_en_option = if ref_en { AD7124_ADC_CTRL_REG_REF_EN } else { 0 };
        let data_status_option = AD7124_ADC_CTRL_REG_DATA_STATUS;
        let dout_rdy_del_option = AD7124_ADC_CTRL_REG_DOUT_RDY_DEL;
        // Combined Configuration Options
        let current_val = mode_option | power_mode_option | clk_sel_option | ref_en_option | data_status_option | dout_rdy_del_option;
        self.write_register(AD7124RegId::RegControl, current_val as u32)?;
        self.op_mode = mode;
        Ok(())
    }

    /// set the configuration
    /// setup_index: 0-7 .Corresponds to Eight configurations
    pub fn set_config(&mut self, setup_index: u8, ref_now: AD7124RefSource, gain: AD7124GainSel, burnout_current: AD7124BurnoutCurrent, bipolar: bool) -> Result<(), AD7124Error<E>> {
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

        self.write_register(reg_now, current_val as u32)
    }

    /// set the filter and output data rate
    pub fn set_filter(&mut self, setup_index: u8, filter: AD7124Filter, fs: u16, post_filter: AD7124PostFilter, rej60: bool, single: bool) -> Result<(), AD7124Error<E>> {
        let filter_option = AD7124_FILT_REG_FILTER(filter as u32);
        let post_filter_option = AD7124_FILT_REG_POST_FILTER(post_filter as u32);
        let fs_option = AD7124_FILT_REG_FS(fs as u32);
        let rej60_option = if rej60 { AD7124_FILT_REG_REJ60 } else { 0 };
        let single_cycle_option = if single { AD7124_FILT_REG_SINGLE_CYCLE } else { 0 };

        let current_val = filter_option | post_filter_option | fs_option | rej60_option | single_cycle_option;
        let reg_now = AD7124RegId::from(AD7124RegId::RegFilter0 as u8 + setup_index);
        self.write_register(reg_now, current_val as u32)
    }

    /// configure channel
    pub fn set_channel(&mut self, channel_index: u8, ainp: AD7124Channel, ainm: AD7124Channel, setup_index: u8, enable: bool) -> Result<(), AD7124Error<E>> {
        let ainp_option = AD7124_CH_MAP_REG_AINP(ainp as u16);
        let ainm_option = AD7124_CH_MAP_REG_AINM(ainm as u16);
        let setup_option = AD7124_CH_MAP_REG_SETUP(setup_index as u16);
        let enable_option = if enable { AD7124_CH_MAP_REG_CH_ENABLE } else { 0 };
        let current_val = ainp_option | ainm_option | setup_option | enable_option;
        let reg_now = AD7124RegId::from(AD7124RegId::RegChannel0 as u8 + channel_index);
        self.write_register(reg_now, current_val as u32)
    }

    /// set the offset calibration for a setup
    pub fn set_offset_calibration(&mut self, setup_index: u8, offset: u32) -> Result<(), AD7124Error<E>> {
        let reg_now = AD7124RegId::from(AD7124RegId::RegOffset0 as u8 + setup_index);
        self.write_register(reg_now, offset)
    }

    /// set the gain calibration for a setup
    pub fn set_gain_calibration(&mut self, setup_index: u8, gain: u32) -> Result<(), AD7124Error<E>> {
        let reg_now = AD7124RegId::from(AD7124RegId::RegGain0 as u8 + setup_index);
        self.write_register(reg_now, gain)
    }

    /// enable (close) the on-chip low side power switch.
    pub fn set_pwrsw(&mut self, enable: bool) -> Result<(), AD7124Error<E>> {
        //let mut current_value = self.read_register(AD7124RegId::RegIocon1)?;//REGS[AD7124RegId::RegIocon1 as usize].value;
        let mut current_value = unsafe { REGS[AD7124RegId::RegIocon1 as usize].value };
        if enable {
            current_value |= AD7124_IO_CTRL1_REG_PDSW
        } else {
            current_value &= !AD7124_IO_CTRL1_REG_PDSW
        };
        self.write_register(AD7124RegId::RegIocon1, current_value)
    }

    /// Sets enables the bias voltage generator on the given pin.
    /// A bias voltage is necessary to read truly bipolar output from thermocouples
    pub fn set_vbias(&mut self, vbias_pin: AD7124VBiasPin, enable: bool) -> Result<(), AD7124Error<E>> {
        //let mut current_value = self.read_register(AD7124RegId::RegIocon2)?;//REGS[AD7124RegId::RegIocon2 as usize].value;
        let mut current_value = unsafe { REGS[AD7124RegId::RegIocon2 as usize].value };
        if enable {
            current_value |= 1 << vbias_pin as u32
        } else {
            current_value &= !(1 << vbias_pin as u32)
        };
        self.write_register(AD7124RegId::RegIocon2, current_value)
    }

    /// get a reading in raw counts from a single channel
    pub fn read_single_raw_data(&mut self, channel: u8) -> Result<u32, AD7124Error<E>> {
        let active_channel = self.get_active_channel()?;
        let data_now: u32;
        if active_channel != channel {
            self.enable_channel(active_channel, false)?;
            if self.op_mode == AD7124OperatingMode::SingleConv {
                self.enable_channel(channel, true)?;
                self.set_op_mode(AD7124OperatingMode::SingleConv)?;
            } else {
                self.enable_channel(channel, true)?;
            }
        } else {
            if self.op_mode == AD7124OperatingMode::SingleConv {
                self.set_op_mode(AD7124OperatingMode::SingleConv)?;
            }
        }
        self.wait_conv_ready(AD7124_DEFAULT_TIMEOUT_MS)?;
        data_now = self.get_data()?;
        Ok(data_now)
    }

    /// check if a channel is enabled
    #[inline]
    pub fn check_if_enabled(&mut self, ch: u8) -> bool {
        let channel = ch + AD7124RegId::RegChannel0 as u8;
        if unsafe { REGS[channel as usize].value } & AD7124_CH_MAP_REG_CH_ENABLE as u32 == 0 {
            return false;
        } else {
            return true;
        }
    }

    /// read multiple channels in raw counts
    pub fn read_multi_raw_data(&mut self, buffer: &mut [u32; 16]) -> Result<(), AD7124Error<E>> {
        //let active_channel = self.get_active_channel()?;
        if self.op_mode == AD7124OperatingMode::SingleConv {
            self.set_op_mode(AD7124OperatingMode::SingleConv)?;
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
            match self.wait_conv_ready(AD7124_DEFAULT_TIMEOUT_MS) {
                Ok(()) => {
                    let channel = self.get_active_channel()?;
                    if channel == first_channel && !data_started {
                        data_started = true;
                    } else if channel == first_channel && data_started {
                        data_ended = true;
                    }

                    if data_started && !data_ended {
                        buffer[channel as usize] = self.get_data()?;
                    }
                }
                Err(AD7124Error::TimeOut) => return Err(AD7124Error::TimeOut),
                _ => continue,
            }
        }

        Ok(())
    }

    /// get current channel
    pub fn current_channel(&mut self) -> Result<u8, AD7124Error<E>> {
        let data = self.read_register(AD7124RegId::RegStatus)?;
        Ok(data as u8 & AD7124_STATUS_REG_CH_ACTIVE(0x0F))
    }

    /// get the data of adc data register
    pub fn get_data(&mut self) -> Result<u32, AD7124Error<E>> {
        self.read_register(AD7124RegId::RegData)
    }

    /// get the data of adc data register fast way
    pub fn get_data_fast(&mut self) -> Result<u32, AD7124Error<E>> {
        // Temporary reg struct for data with extra byte to hold status bits
        let register_temp = AD7124Register {
            addr: 0x02,
            value: 0x0000,
            size: 4,
            rw: AD7124RW::Read,
        };
        let data_temp = self.read_register_no_check_direct(register_temp.size, register_temp.addr)?;
        unsafe { REGS[AD7124RegId::RegStatus as usize].value = data_temp & 0xFF };
        unsafe { REGS[AD7124RegId::RegData as usize].value = data_temp >> 8 & 0x00FFFFFF };
        let data_now = unsafe { REGS[AD7124RegId::RegData as usize].value };
        Ok(data_now)
    }

    /// wait for the conversion to be ready
    pub fn wait_conv_ready(&mut self, timeout: u32) -> Result<(), AD7124Error<E>> {
        let mut timeout = timeout;
        while timeout > 0 {
            let data = self.read_register_no_check(AD7124RegId::RegStatus)?;
            if (data & AD7124_STATUS_REG_RDY as u32) == 0 {
                return Ok(());
            }
            self.delay.delay_ms(1);
            timeout -= 1;
        }
        Err(AD7124Error::TimeOut)
    }

    /// set operating mode of the ADC
    pub fn set_op_mode(&mut self, mode: AD7124OperatingMode) -> Result<(), AD7124Error<E>> {
        let mut current_val = unsafe { REGS[AD7124RegId::RegControl as usize].value };
        current_val &= !AD7124_ADC_CTRL_REG_MODE(0xF) as u32;
        current_val |= AD7124_ADC_CTRL_REG_MODE(mode as u16) as u32;
        //let current_val = AD7124_ADC_CTRL_REG_MODE(mode as u16);
        self.write_register(AD7124RegId::RegControl, current_val as u32)
    }

    /// read the error register
    pub fn read_error_register(&mut self) -> Result<u32, AD7124Error<E>> {
        self.read_register(AD7124RegId::RegError)
    }

    /// enable channel
    pub fn enable_channel(&mut self, channel: u8, enable: bool) -> Result<(), AD7124Error<E>> {
        if channel >= 16 {
            return Err(AD7124Error::InvalidValue);
        }
        let reg_now = AD7124RegId::from(AD7124RegId::RegChannel0 as u8 + channel);
        let mut current_value = self.read_register(reg_now)?;
        if enable {
            current_value |= AD7124_CH_MAP_REG_CH_ENABLE as u32;
        } else {
            current_value &= !AD7124_CH_MAP_REG_CH_ENABLE as u32;
        }
        self.write_register(reg_now, current_value)
    }

    //get active channel
    pub fn get_active_channel(&mut self) -> Result<u8, AD7124Error<E>> {
        let data = self.read_register(AD7124RegId::RegStatus)?;
        Ok(data as u8 & 0x07)
    }
}
