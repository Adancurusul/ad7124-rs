#[cfg(feature = "sync")]
pub mod ad7124_sync;

#[cfg(feature = "async")]
pub mod ad7124_async;

use crate::defs::*;
use crate::errors::*;
use crate::regs::*;

/// Synchronous Transport Interface
pub trait SyncTransport {
    type Error;
    
    fn spi_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error>;
    fn spi_write(&mut self, write: &[u8]) -> Result<(), Self::Error>;
    fn set_cs(&mut self, state: bool) -> Result<(), Self::Error>;
    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error>;
}

/// Asynchronous Transport Interface
#[allow(async_fn_in_trait)]
pub trait AsyncTransport {
    type Error;
    
    async fn spi_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error>;
    async fn spi_write(&mut self, write: &[u8]) -> Result<(), Self::Error>;
    async fn set_cs(&mut self, state: bool) -> Result<(), Self::Error>;
    async fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error>;
}

/// Real device implementing the synchronous transport interface
pub struct RealSyncTransport<SPI, CS, D> {
    pub spi: SPI,
    pub cs: CS,
    pub delay: D,
}

/// Real device implementing the asynchronous transport interface
pub struct RealAsyncTransport<SPI, CS, D> {
    pub spi: SPI,
    pub cs: CS,
    pub delay: D,
}

/// Core functionality logic, without IO operations
#[derive(Debug, Default)]
pub struct AD7124Core {
    registers: AD7124Registers,
    crc_enabled: bool,
    op_mode: AD7124OperatingMode,
}

impl AD7124Core {
    pub fn new() -> Self {
        Self {
            registers: AD7124Registers::new(),
            crc_enabled: false,
            op_mode: AD7124OperatingMode::Continuous,
        }
    }
    
    // Prepare register read operation
    pub fn prepare_register_read(&self, reg: AD7124RegId) -> Result<(u8, u8, usize), AD7124Error<()>> {
        let reg_info = self.registers.get(reg);
        if reg_info.rw == AD7124RW::Write {
            return Err(AD7124Error::InvalidValue);
        }
        
        let reg_size = reg_info.size;
        let reg_addr = reg_info.addr;
        let buf_len = reg_size as usize + 1 + self.crc_enabled as usize;
        
        Ok((reg_addr, reg_size, buf_len))
    }
    
    // Prepare register write operation
    pub fn prepare_register_write(&self, reg: AD7124RegId, value: u32) -> Result<(u8, u8, usize, u32), AD7124Error<()>> {
        let reg_info = self.registers.get(reg);
        if reg_info.rw == AD7124RW::Read {
            return Err(AD7124Error::InvalidValue);
        }
        
        let reg_size = reg_info.size;
        let reg_addr = reg_info.addr;
        let buf_len = reg_size as usize + 1 + self.crc_enabled as usize;
        
        Ok((reg_addr, reg_size, buf_len, value))
    }
    
    // Get register information
    pub fn get_register_info(&self, reg: AD7124RegId) -> &AD7124Register {
        self.registers.get(reg)
    }
    
    // Get register value
    pub fn get_register_value(&self, reg: AD7124RegId) -> u32 {
        self.registers.get(reg).value
    }
    
    // Update register value
    pub fn update_register_value(&mut self, reg: AD7124RegId, value: u32) {
        self.registers.update_value(reg, value);
    }
    
    // Check if channel is enabled
    pub fn is_channel_enabled(&self, ch: u8) -> bool {
        let reg = AD7124RegId::from(AD7124RegId::RegChannel0 as u8 + ch);
        self.registers.get(reg).value & AD7124_CH_MAP_REG_CH_ENABLE as u32 != 0
    }
    
    // Set operation mode
    pub fn set_op_mode(&mut self, mode: AD7124OperatingMode) {
        self.op_mode = mode;
    }
    
    // Get current operation mode
    pub fn get_op_mode(&self) -> AD7124OperatingMode {
        self.op_mode
    }
    
    // Verify chip ID
    pub fn verify_chip_id(&self, id: u32) -> Result<(), AD7124Error<()>> {
        if id == 0x12 || id == 0x04 {
            Ok(())
        } else {
            Err(AD7124Error::InvalidValue)
        }
    }
    
    // Calculate ADC control register value
    pub fn calculate_adc_control(&self, mode: AD7124OperatingMode, 
                             power_mode: AD7124PowerMode, 
                             clk_sel: AD7124ClkSource, 
                             ref_en: bool) -> u32 {
        let mode_option = AD7124_ADC_CTRL_REG_MODE(mode as u16);
        let power_mode_option = AD7124_ADC_CTRL_REG_POWER_MODE(power_mode as u16);
        let clk_sel_option = AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel as u16);
        let ref_en_option = if ref_en { AD7124_ADC_CTRL_REG_REF_EN } else { 0 };
        let data_status_option = AD7124_ADC_CTRL_REG_DATA_STATUS;
        let dout_rdy_del_option = AD7124_ADC_CTRL_REG_DOUT_RDY_DEL;
        
        (mode_option | power_mode_option | clk_sel_option | ref_en_option | 
         data_status_option | dout_rdy_del_option) as u32
    }
    
    // Calculate configuration register value
    pub fn calculate_config(&self, 
                       ref_now: AD7124RefSource, 
                       gain: AD7124GainSel, 
                       burnout_current: AD7124BurnoutCurrent, 
                       bipolar: bool) -> u32 {
        let ref_sel_option = AD7124_CFG_REG_REF_SEL(ref_now as u16);
        let pga_option = AD7124_CFG_REG_PGA(gain as u16);
        let bipolar_option = if bipolar { AD7124_CFG_REG_BIPOLAR } else { 0 };
        let burnout_option = AD7124_CFG_REG_BURNOUT(burnout_current as u16);
        let ref_buf_option = AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_REF_BUFM;
        let ain_buf_option = AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM;
        
        (ref_sel_option | pga_option | bipolar_option | burnout_option | 
         ref_buf_option | ain_buf_option) as u32
    }
    
    // Calculate filter register value
    pub fn calculate_filter(&self, 
                       filter: AD7124Filter, 
                       fs: u16, 
                       post_filter: AD7124PostFilter, 
                       rej60: bool, 
                       single: bool) -> u32 {
        let filter_option = AD7124_FILT_REG_FILTER(filter as u32);
        let post_filter_option = AD7124_FILT_REG_POST_FILTER(post_filter as u32);
        let fs_option = AD7124_FILT_REG_FS(fs as u32);
        let rej60_option = if rej60 { AD7124_FILT_REG_REJ60 } else { 0 };
        let single_cycle_option = if single { AD7124_FILT_REG_SINGLE_CYCLE } else { 0 };
        
        filter_option | post_filter_option | fs_option | rej60_option | single_cycle_option
    }
    
    // Calculate channel configuration register value
    pub fn calculate_channel(&self, 
                        ainp: AD7124Channel, 
                        ainm: AD7124Channel, 
                        setup_index: u8, 
                        enable: bool) -> u32 {
        let ainp_option = AD7124_CH_MAP_REG_AINP(ainp as u16);
        let ainm_option = AD7124_CH_MAP_REG_AINM(ainm as u16);
        let setup_option = AD7124_CH_MAP_REG_SETUP(setup_index as u16);
        let enable_option = if enable { AD7124_CH_MAP_REG_CH_ENABLE } else { 0 };
        
        (ainp_option | ainm_option | setup_option | enable_option) as u32
    }
    
    // Enable/disable CRC
    pub fn set_crc_enabled(&mut self, enabled: bool) {
        self.crc_enabled = enabled;
    }
    
    // Get CRC status
    pub fn is_crc_enabled(&self) -> bool {
        self.crc_enabled
    }
}

// Implementation of the synchronous transport interface
impl<SPI, CS, D, E1, E2> SyncTransport for RealSyncTransport<SPI, CS, D>
where
    SPI: hal::spi::SpiBus<Error = E1>,
    CS: hal::digital::OutputPin<Error = E2>,
    D: hal::delay::DelayNs,
    E1: core::fmt::Debug,
    E2: core::fmt::Debug,
{
    type Error = AD7124Error<E1>;
    
    fn spi_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|_| AD7124Error::CSError)?;
        let result = self.spi.transfer(read, write).map_err(AD7124Error::SPI);
        self.cs.set_high().map_err(|_| AD7124Error::CSError)?;
        result
    }
    
    fn spi_write(&mut self, write: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|_| AD7124Error::CSError)?;
        let result = self.spi.write(write).map_err(AD7124Error::SPI);
        self.cs.set_high().map_err(|_| AD7124Error::CSError)?;
        result
    }
    
    fn set_cs(&mut self, state: bool) -> Result<(), Self::Error> {
        if state {
            self.cs.set_high().map_err(|_| AD7124Error::CSError)
        } else {
            self.cs.set_low().map_err(|_| AD7124Error::CSError)
        }
    }
    
    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.delay.delay_ms(ms);
        Ok(())
    }
}

// Implementation of the asynchronous transport interface
impl<SPI, CS, D, E1, E2> AsyncTransport for RealAsyncTransport<SPI, CS, D>
where
    SPI: hal_async::spi::SpiBus<Error = E1>,
    CS: hal::digital::OutputPin<Error = E2>,
    D: hal_async::delay::DelayNs,
    E1: core::fmt::Debug,
    E2: core::fmt::Debug,
{
    type Error = AD7124Error<E1>;
    
    async fn spi_transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|_| AD7124Error::CSError)?;
        let result = self.spi.transfer(read, write).await.map_err(AD7124Error::SPI);
        self.cs.set_high().map_err(|_| AD7124Error::CSError)?;
        result
    }
    
    async fn spi_write(&mut self, write: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(|_| AD7124Error::CSError)?;
        let result = self.spi.write(write).await.map_err(AD7124Error::SPI);
        self.cs.set_high().map_err(|_| AD7124Error::CSError)?;
        result
    }
    
    async fn set_cs(&mut self, state: bool) -> Result<(), Self::Error> {
        if state {
            self.cs.set_high().map_err(|_| AD7124Error::CSError)
        } else {
            self.cs.set_low().map_err(|_| AD7124Error::CSError)
        }
    }
    
    async fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.delay.delay_ms(ms).await;
        Ok(())
    }
}