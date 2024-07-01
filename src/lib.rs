


#![no_std]
#![no_main]

extern crate embedded_hal_async as hal_async;
extern crate embedded_hal as hal;
use hal_async::spi;
use hal::digital::OutputPin;

mod crc8;
use crc8::calc_crc8;

mod defs;
use defs::*;

pub const AD7124_DEFAULT_TIMEOUT_MS:u32 = 200; // milliseconds


pub enum AD7124Error<E> {
    SPI(E),
    OutputPin(E),
    CommunicationError,
    InvalidValue,
    TimeOut,
}



#[derive(Copy, Clone,PartialEq)]
pub enum AD7124RW {
    ReadWrite = 1,
    Read,
    Write,
}


#[derive(Copy, Clone,PartialEq,Debug,Default)]
pub enum AD7124OperatingMode
{
    Continuous = 0,            // Continuous conversion mode (default). In continuous conversion mode, the ADC continuously performs conversions and places the result in the data register.
    SingleConv,                // Single conversion mode. When single conversion mode is selected, the ADC powers up and performs a single conversion on the selected channel.
    Standby,                   // Standby mode. In standby mode, all sections of the AD7124 can be powered down except the LDOs.
    PowerDown,                 // Power-down mode. In power-down mode, all the AD7124 circuitry is powered down, including the current sources, power switch, burnout currents, bias voltage generator, and clock circuitry.
    Idle,                      // Idle mode. In idle mode, the ADC filter and modulator are held in a reset state even though the modulator clocks continue to be provided.
    InternalOffsetCalibration, // Internal zero-scale (offset) calibration. An internal short is automatically connected to the input. RDY goes high when the calibration is initiated and returns low when the calibration is complete.
    InternalGainCalibration,   // Internal full-scale (gain) calibration. A full-scale input voltage is automatically connected to the selected analog input for this calibration. */
    ASystemOffsetCalibration,   // System zero-scale (offset) calibration. Connect the system zero-scale input to the channel input pins of the selected channel. RDY goes high when the calibration is initiated and returns low when the calibration is complete.
    SystemGainCalibration      // System full-scale (gain) calibration. Connect the system full-scale input to the channel input pins of the selected channel. RDY goes high when the calibration is initiated and returns low when the calibration is complete.
}

#[derive(Copy, Clone,PartialEq)]
pub enum AD7124PowerMode {
    LowPower = 0,
    MidPower,
    FullPower
}

#[derive(Copy, Clone,PartialEq)]
pub enum AD7124ClkSource {
    ClkInternal = 0,       // internal 614.4 kHz clock. The internal clock is not available at the CLK pin.
    ClkInternalWithOutput, // internal 614.4 kHz clock. This clock is available at the CLK pin.
    ClkExternal,           // external 614.4 kHz clock.
    ClkExternalDiv4        // external clock. The external clock is divided by 4 within the AD7124.

}

#[derive(Copy, Clone,PartialEq)]
pub enum AD7124InputSel {
    AIN0 = 0,
    AIN1,
    AIN2,
    AIN3,
    AIN4,
    AIN5,
    AIN6,
    AIN7,
    AIN8,
    AIN9,
    AIN10,
    AIN11,
    AIN12,
    AIN13,
    AIN14,
    AIN15,
    TEMP = 16, // Temperature sensor (internal)
    AVSS,      // Connect to AVss
    REF,       // Connect to Internal reference
    DGND,      // Connect to DGND.
    AVDD6P,    // (AVdd − AVss)/6+. Use in conjunction with (AVdd − AVss)/6− to monitor supply AVdd − AVss .
    AVDD6M,    // (AVdd − AVss)/6−. Use in conjunction with (AVdd − AVss)/6+ to monitor supply AVdd − AVss .
    IOVDD6P,   // (IOVdd − DGND)/6+. Use in conjunction with (IOVdd − DGND)/6− to monitor IOVdd − DGND.
    IOVDD6M,   // (IOVdd − DGND)/6−. Use in conjunction with (IOVdd − DGND)/6+ to monitor IOVdd − DGND.
    ALDO6P,    // (ALDO − AVss)/6+. Use in conjunction with (ALDO − AVss)/6− to monitor the analog LDO.
    ALDO6M,    // (ALDO − AVss)/6−. Use in conjunction with (ALDO − AVss)/6+ to monitor the analog LDO.
    DLDO6P,    // (DLDO − DGND)/6+. Use in conjunction with (DLDO − DGND)/6− to monitor the digital LDO.
    DLDO6M,    // (DLDO − DGND)/6−. Use in conjunction with (DLDO − DGND)/6+ to monitor the digital LDO.
    V20mVP,    // V_20MV_P. Use in conjunction with V_20MV_M to apply a 20 mV p-p signal to the ADC.
    V20mVM     // V_20MV_M. Use in conjunction with V_20MV_P to apply a 20 mV p-p signal to the ADC.
}
#[derive(Copy, Clone,PartialEq)]
pub enum AD7124GainSel {
    _1 = 0, // Gain 1, Input Range When VREF = 2.5 V: ±2.5 V
    _2,     // Gain 2, Input Range When VREF = 2.5 V: ±1.25 V
    _4,     // Gain 4, Input Range When VREF = 2.5 V: ± 625 mV
    _8,     // Gain 8, Input Range When VREF = 2.5 V: ±312.5 mV
    _16,    // Gain 16, Input Range When VREF = 2.5 V: ±156.25 mV
    _32,    // Gain 32, Input Range When VREF = 2.5 V: ±78.125 mV
    _64,    // Gain 64, Input Range When VREF = 2.5 V: ±39.06 mV
    _128    // Gain 128, Input Range When VREF = 2.5 V: ±19.53 mV
}

//NOTE: These are different for the -8
#[derive(Copy, Clone,PartialEq)]
pub enum AD7124VBiasPin {
    AIN0 = 0x00,
    AIN1 = 0x01,
    AIN2 = 0x04,
    AIN3 = 0x05,
    AIN4 = 0x0A,
    AIN5 = 0x0B,
    AIN6 = 0x0E,
    AIN7 = 0x0F,
}
#[derive(Copy, Clone,PartialEq)]
pub enum AD7124RefSource {
    ExtRef1 = 0x00,
    ExtRef2 = 0x01,
    Internal = 0x02,
    Avdd = 0x03
}
#[derive(Copy, Clone,PartialEq)]
pub enum AD7124Filter {
    SINC4 = 0x00, // SINC4 Filter - Default after reset. This filter gives excellent noise performance over the complete range of output data rates. It also gives the best 50 Hz/60 Hz rejection, but it has a long settling time.
    SINC3 = 0x02, // SINC3 Filter - This filter has good noise performance, moderate settling time, and moderate 50 Hz and 60 Hz (±1 Hz) rejection.
    FAST4 = 0x04, // Fast settling + Sinc4
    FAST3 = 0x05, // Fast settling + Sinc3
    POST = 0x07   // Post filter enable - The post filters provide rejection of 50 Hz and 60 Hz simultaneously and allow the user to trade off settling time and rejection. These filters can operate up to 27.27 SPS or can reject up to 90 dB of 50 Hz ± 1 Hz and 60 Hz ± 1 Hz interference
}
#[derive(Copy, Clone,PartialEq)]
pub enum AD7124PostFilter {
    NoPost = 0, // No Post Filter (Default value)
    DB47 = 2,   // Rejection at 50 Hz and 60 Hz ± 1 Hz: 47 dB, Output Data Rate (SPS): 27.27 Hz
    DB62 = 3,   // Rejection at 50 Hz and 60 Hz ± 1 Hz: 62 dB, Output Data Rate (SPS): 25 Hz
    DB86 = 5,   // Rejection at 50 Hz and 60 Hz ± 1 Hz: 86 dB, Output Data Rate (SPS): 20 Hz
    DB92 = 6    // Rejection at 50 Hz and 60 Hz ± 1 Hz: 92 dB, Output Data Rate (SPS): 16.7 Hz
}
#[derive(Copy, Clone,PartialEq)]
pub enum AD7124BurnoutCurrent {
    Off = 0, // burnout current source off (default).
    _500nA,   // burnout current source on, 0.5 μA.
    _2uA,     // burnout current source on, 2 μA.
    _4uA      // burnout current source on, 4 μA.
}

// Excitation currents - Not used yet.
// TODO: Look up actual value in datasheet (IO_CONTROL_1 Register)
#[derive(Copy, Clone,PartialEq)]
pub enum AD7124ExCurrent {
    Off = 0x00,
    _50uA,
    _100uA,
    _250uA,
    _500uA,
    _750uA,
    _1mA
}




#[derive(Copy, Clone)]
pub struct AD7124Register {
    pub addr: u8,
    pub value: u32,
    pub size: u8,
    pub rw: AD7124RW,
} 

#[derive(Copy, Clone,PartialEq)]
pub enum AD7124RegId
{
    RegStatus = 0x00,
    RegControl,
    RegData,
    RegIocon1,
    RegIocon2,
    RegId,
    RegError,
    RegErrorEn,
    RegMclkCount,
    RegChannel0,
    RegChannel1,
    RegChannel2,
    RegChannel3,
    RegChannel4,
    RegChannel5,
    RegChannel6,
    RegChannel7,
    RegChannel8,
    RegChannel9,
    RegChannel10,
    RegChannel11,
    RegChannel12,
    RegChannel13,
    RegChannel14,
    RegChannel15,
    RegConfig0,
    RegConfig1,
    RegConfig2,
    RegConfig3,
    RegConfig4,
    RegConfig5,
    RegConfig6,
    RegConfig7,
    RegFilter0,
    RegFilter1,
    RegFilter2,
    RegFilter3,
    RegFilter4,
    RegFilter5,
    RegFilter6,
    RegFilter7,
    RegOffset0,
    RegOffset1,
    RegOffset2,
    RegOffset3,
    RegOffset4,
    RegOffset5,
    RegOffset6,
    RegOffset7,
    RegGain0,
    RegGain1,
    RegGain2,
    RegGain3,
    RegGain4,
    RegGain5,
    RegGain6,
    RegGain7,
    RegREGNO
}
impl From<u8> for AD7124RegId {
    fn from(value: u8) -> Self {
        match value {
            0x00 => AD7124RegId::RegStatus,
            0x01 => AD7124RegId::RegControl,
            0x02 => AD7124RegId::RegData,
            0x03 => AD7124RegId::RegIocon1,
            0x04 => AD7124RegId::RegIocon2,
            0x05 => AD7124RegId::RegId,
            0x06 => AD7124RegId::RegError,
            0x07 => AD7124RegId::RegErrorEn,
            0x08 => AD7124RegId::RegMclkCount,
            0x09 => AD7124RegId::RegChannel0,
            0x0A => AD7124RegId::RegChannel1,
            0x0B => AD7124RegId::RegChannel2,
            0x0C => AD7124RegId::RegChannel3,
            0x0D => AD7124RegId::RegChannel4,
            0x0E => AD7124RegId::RegChannel5,
            0x0F => AD7124RegId::RegChannel6,
            0x10 => AD7124RegId::RegChannel7,
            0x11 => AD7124RegId::RegChannel8,
            0x12 => AD7124RegId::RegChannel9,
            0x13 => AD7124RegId::RegChannel10,
            0x14 => AD7124RegId::RegChannel11,
            0x15 => AD7124RegId::RegChannel12,
            0x16 => AD7124RegId::RegChannel13,
            0x17 => AD7124RegId::RegChannel14,
            0x18 => AD7124RegId::RegChannel15,
            0x19 => AD7124RegId::RegConfig0,
            0x1A => AD7124RegId::RegConfig1,
            0x1B => AD7124RegId::RegConfig2,
            0x1C => AD7124RegId::RegConfig3,
            0x1D => AD7124RegId::RegConfig4,
            0x1E => AD7124RegId::RegConfig5,
            0x1F => AD7124RegId::RegConfig6,
            0x20 => AD7124RegId::RegConfig7,
            0x21 => AD7124RegId::RegFilter0,
            0x22 => AD7124RegId::RegFilter1,
            0x23 => AD7124RegId::RegFilter2,
            0x24 => AD7124RegId::RegFilter3,
            0x25 => AD7124RegId::RegFilter4,
            0x26 => AD7124RegId::RegFilter5,
            0x27 => AD7124RegId::RegFilter6,
            0x28 => AD7124RegId::RegFilter7,
            0x29 => AD7124RegId::RegOffset0,
            0x2A => AD7124RegId::RegOffset1,
            0x2B => AD7124RegId::RegOffset2,
            0x2C => AD7124RegId::RegOffset3,
            0x2D => AD7124RegId::RegOffset4,
            0x2E => AD7124RegId::RegOffset5,
            0x2F => AD7124RegId::RegOffset6,
            0x30 => AD7124RegId::RegOffset7,
            0x31 => AD7124RegId::RegGain0,
            0x32 => AD7124RegId::RegGain1,
            0x33 => AD7124RegId::RegGain2,
            0x34 => AD7124RegId::RegGain3,
            0x35 => AD7124RegId::RegGain4,
            0x36 => AD7124RegId::RegGain5,
            0x37 => AD7124RegId::RegGain6,
            0x38 => AD7124RegId::RegGain7,
            _ => AD7124RegId::RegREGNO,
        }
    }
}

pub static mut REGS: [AD7124Register; AD7124RegId::RegREGNO as usize] = [
    AD7124Register { addr: 0x00, value: 0x00, size: 1, rw: AD7124RW::ReadWrite },     // Status
    AD7124Register { addr: 0x01, value: 0x0000, size: 2, rw: AD7124RW::Read },   // ADC_Control
    AD7124Register { addr: 0x02, value: 0x0000, size: 3, rw: AD7124RW::ReadWrite },   // Data
    AD7124Register { addr: 0x03, value: 0x0000, size: 3, rw: AD7124RW::Read },   // IOCon1
    AD7124Register { addr: 0x04, value: 0x0000, size: 2, rw: AD7124RW::Read },   // IOCon2
    AD7124Register { addr: 0x05, value: 0x02, size: 1, rw: AD7124RW::ReadWrite },     // ID
    AD7124Register { addr: 0x06, value: 0x0000, size: 3, rw: AD7124RW::ReadWrite },   // Error
    AD7124Register { addr: 0x07, value: 0x0044, size: 3, rw: AD7124RW::Read },   // Error_En
    AD7124Register { addr: 0x08, value: 0x00, size: 1, rw: AD7124RW::ReadWrite },     // Mclk_Count
    AD7124Register { addr: 0x09, value: 0x8001, size: 2, rw: AD7124RW::Read },   // Channel_0
    AD7124Register { addr: 0x0A, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_1
    AD7124Register { addr: 0x0B, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_2
    AD7124Register { addr: 0x0C, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_3
    AD7124Register { addr: 0x0D, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_4
    AD7124Register { addr: 0x0E, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_5
    AD7124Register { addr: 0x0F, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_6
    AD7124Register { addr: 0x10, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_7
    AD7124Register { addr: 0x11, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_8
    AD7124Register { addr: 0x12, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_9
    AD7124Register { addr: 0x13, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_10
    AD7124Register { addr: 0x14, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_11
    AD7124Register { addr: 0x15, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_12
    AD7124Register { addr: 0x16, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_13
    AD7124Register { addr: 0x17, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_14
    AD7124Register { addr: 0x18, value: 0x0001, size: 2, rw: AD7124RW::Read },   // Channel_15
    AD7124Register { addr: 0x19, value: 0x0860, size: 2, rw: AD7124RW::Read },   // Config_0
    AD7124Register { addr: 0x1A, value: 0x0860, size: 2, rw: AD7124RW::Read },   // Config_1
    AD7124Register { addr: 0x1B, value: 0x0860, size: 2, rw: AD7124RW::Read },   // Config_2
    AD7124Register { addr: 0x1C, value: 0x0860, size: 2, rw: AD7124RW::Read },   // Config_3
    AD7124Register { addr: 0x1D, value: 0x0860, size: 2, rw: AD7124RW::Read },   // Config_4
    AD7124Register { addr: 0x1E, value: 0x0860, size: 2, rw: AD7124RW::Read },   // Config_5
    AD7124Register { addr: 0x1F, value: 0x0860, size: 2, rw: AD7124RW::Read },   // Config_6
    AD7124Register { addr: 0x20, value: 0x0860, size: 2, rw: AD7124RW::Read },   // Config_7
    AD7124Register { addr: 0x21, value: 0x060180, size: 3, rw: AD7124RW::Read }, // Filter_0
    AD7124Register { addr: 0x22, value: 0x060180, size: 3, rw: AD7124RW::Read }, // Filter_1
    AD7124Register { addr: 0x23, value: 0x060180, size: 3, rw: AD7124RW::Read }, // Filter_2
    AD7124Register { addr: 0x24, value: 0x060180, size: 3, rw: AD7124RW::Read }, // Filter_3
    AD7124Register { addr: 0x25, value: 0x060180, size: 3, rw: AD7124RW::Read }, // Filter_4
    AD7124Register { addr: 0x26, value: 0x060180, size: 3, rw: AD7124RW::Read }, // Filter_5
    AD7124Register { addr: 0x27, value: 0x060180, size: 3, rw: AD7124RW::Read }, // Filter_6
    AD7124Register { addr: 0x28, value: 0x060180, size: 3, rw: AD7124RW::Read }, // Filter_7
    AD7124Register { addr: 0x29, value: 0x800000, size: 3, rw: AD7124RW::Read }, // Offset_0
    AD7124Register { addr: 0x2A, value: 0x800000, size: 3, rw: AD7124RW::Read }, // Offset_1
    AD7124Register { addr: 0x2B, value: 0x800000, size: 3, rw: AD7124RW::Read }, // Offset_2
    AD7124Register { addr: 0x2C, value: 0x800000, size: 3, rw: AD7124RW::Read }, // Offset_3
    AD7124Register { addr: 0x2D, value: 0x800000, size: 3, rw: AD7124RW::Read }, // Offset_4
    AD7124Register { addr: 0x2E, value: 0x800000, size: 3, rw: AD7124RW::Read }, // Offset_5
    AD7124Register { addr: 0x2F, value: 0x800000, size: 3, rw: AD7124RW::Read }, // Offset_6
    AD7124Register { addr: 0x30, value: 0x800000, size: 3, rw: AD7124RW::Read }, // Offset_7
    AD7124Register { addr: 0x31, value: 0x500000, size: 3, rw: AD7124RW::Read }, // Gain_0
    AD7124Register { addr: 0x32, value: 0x500000, size: 3, rw: AD7124RW::Read }, // Gain_1
    AD7124Register { addr: 0x33, value: 0x500000, size: 3, rw: AD7124RW::Read }, // Gain_2
    AD7124Register { addr: 0x34, value: 0x500000, size: 3, rw: AD7124RW::Read }, // Gain_3
    AD7124Register { addr: 0x35, value: 0x500000, size: 3, rw: AD7124RW::Read }, // Gain_4
    AD7124Register { addr: 0x36, value: 0x500000, size: 3, rw: AD7124RW::Read }, // Gain_5
    AD7124Register { addr: 0x37, value: 0x500000, size: 3, rw: AD7124RW::Read }, // Gain_6
    AD7124Register { addr: 0x38, value: 0x500000, size: 3, rw: AD7124RW::Read }, // Gain_7
];

#[derive(Debug, Default)]
pub struct AD7124<SPI,CS,Delay> {
    spi: SPI,
    cs: CS,
    delay: Delay,
    crc_enabled: bool,
    op_mode : AD7124OperatingMode,
}

impl<SPI,CS,Delay,E> AD7124<SPI,CS,Delay>
where 
    SPI: spi::SpiBus<Error = E>,
    CS: OutputPin<Error = E>,
    Delay: hal_async::delay::DelayNs,
{

    pub fn new(spi: SPI, cs: CS,delay:Delay) -> Result<Self,E> {
        let ad7124 = AD7124 {
            spi,
            cs,
            delay,
            crc_enabled: false,
            op_mode: AD7124OperatingMode::Continuous,
        };
        Ok(ad7124)
    }

    pub async fn init(&mut self) -> Result<(), AD7124Error<E>> {
        //self.cs.set_high().map_err(AD7124Error::OutputPin)?;
        self.reset().await?;
        Ok(())
    }

    pub async fn spi_write_read(&mut self , read:&mut [u8], write:&mut [u8]) -> Result<(), AD7124Error<E>> {
        self.cs.set_low().map_err(AD7124Error::OutputPin)?;
        self.spi.transfer(read, write).await.map_err(AD7124Error::SPI)?;
        self.cs.set_high().map_err(AD7124Error::OutputPin)?;
        Ok(())
    }

    pub async fn spi_write (&mut self, write:&mut [u8]) -> Result<(), AD7124Error<E>> {
        self.cs.set_low().map_err(AD7124Error::OutputPin)?;
        self.spi.write(write).await.map_err(AD7124Error::SPI)?;
        self.cs.set_high().map_err(AD7124Error::OutputPin)?;
        Ok(())
    }
    pub async fn wait_spi_ready(&mut self, timeout:u32) -> Result<(), AD7124Error<E>> {
        let mut timeout = timeout;
        while timeout > 0 {
            let data = self.read_register_no_check(AD7124RegId::RegError).await?;
            if (data & AD7124_ERR_REG_SPI_IGNORE_ERR) == 0 {
                return Ok(());
            }
            self.delay.delay_ms(1).await;
            timeout -= 1;
        }
        Err(AD7124Error::TimeOut)
    }

    pub async fn read_register_no_check(&mut self, reg: AD7124RegId) -> Result<u32, AD7124Error<E>> {
        let reg_now = unsafe{REGS[reg as usize]};
        let reg_size = reg_now.size as usize;
        let reg_addr = reg_now.addr;
        if reg_now.rw == AD7124RW::Write  {
            return Err(AD7124Error::InvalidValue);
        }
        let buf_len: usize = reg_size + 1 + self.crc_enabled as usize;
        let mut buf_write = [0u8; 8];
        let mut buf_read = [0u8; 8];
        let mut msg_buf = [0u8; 8];
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA (reg_addr);
        let buf_write = &mut buf_write[0..buf_len];
        let buf_read = &mut buf_read[0..buf_len];
        self.spi_write_read(buf_read, buf_write).await?;

        if self.crc_enabled {
            msg_buf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(reg_addr);
            msg_buf[1..buf_len].copy_from_slice(&buf_read[1..buf_len]);
            let crc = calc_crc8(&buf_read[0..buf_len]);
            if crc != 0 {
                return Err(AD7124Error::CommunicationError);
            }
        }
        // for i in 1..buf_len {
        //     reg_value <<= 8;
        //     reg_value += buf_read[i] as u32;
        // }
        let reg_value = buf_read[1..buf_len]
            .iter()
            .fold(0, |acc, &b| (acc << 8) + b as u32);

        unsafe{REGS[reg as usize].value = reg_value;}// 
        Ok(reg_value)

    }

    pub async fn write_register_no_check(&mut self, reg: AD7124RegId, value: u32) -> Result<(), AD7124Error<E>> {
        let reg_now = unsafe{REGS[reg as usize]};
        let reg_size = reg_now.size as usize;
        let reg_addr = reg_now.addr;
        if reg_now.rw == AD7124RW::Read  {
            return Err(AD7124Error::InvalidValue);
        }
        let buf_len: usize = reg_size +  self.crc_enabled as usize;
        let mut buf_write = [0u8; 8];
        buf_write[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA (reg_addr);
        let mut value_now = value;
        for i in 0..reg_size-1 {
            buf_write[reg_size - i] = value_now as u8  & 0xFF;
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
        self.spi_write(buf_write).await?;

        unsafe{REGS[reg as usize].value = value;}// 
        Ok(())

    }


    pub async fn read_register(&mut self, reg: AD7124RegId) -> Result<u32, AD7124Error<E>> {
        self.wait_spi_ready(AD7124_DEFAULT_TIMEOUT_MS).await?;
        self.read_register_no_check(reg).await
    }

    pub async fn write_register(&mut self, reg: AD7124RegId, value: u32) -> Result<(), AD7124Error<E>> {
        self.wait_spi_ready(AD7124_DEFAULT_TIMEOUT_MS).await?;
        self.write_register_no_check(reg, value).await
    }


    pub async fn reset(&mut self) -> Result<(), AD7124Error<E>> {
        self.cs.set_low().map_err(AD7124Error::OutputPin)?;
        self.spi.write(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]).await.map_err(AD7124Error::SPI)?;
        self.cs.set_high().map_err(AD7124Error::OutputPin)?;
        //self.wait_for_power_on(AD7124_DEFAULT_TIMEOUT_MS).await?;
        Ok(())
    }

    
    pub async fn wait_for_power_on(&mut self,timeout:u32) -> Result<(), AD7124Error<E>> {
        let mut timeout = timeout;
        while timeout > 0 {
            let data = self.read_register(AD7124RegId::RegStatus).await?;
            if (data) != 0 {
                return Ok(());
            }
            self.delay.delay_ms(1).await;
            timeout -= 1;
        }
        Err(AD7124Error::TimeOut)
    }

    pub async fn read_id (&mut self) -> Result<u32, AD7124Error<E>> {
        self.read_register(AD7124RegId::RegId).await
    }

    pub async fn self_check(&mut self) -> Result<(), AD7124Error<E>> {
        let id = self.read_register(AD7124RegId::RegId).await?;
        //Todo: check the id
        Ok(())
    }

    pub async fn set_adc_control (&mut self, mode :AD7124OperatingMode, power_mode: AD7124PowerMode, clk_sel: AD7124ClkSource,ref_en:bool) -> Result<(), AD7124Error<E>> {
        // Defining individual configuration options
        let mode_option = AD7124_ADC_CTRL_REG_MODE(mode as u16);
        let power_mode_option = AD7124_ADC_CTRL_REG_POWER_MODE(power_mode as u16);
        let clk_sel_option = AD7124_ADC_CTRL_REG_CLK_SEL(clk_sel as u16);
        let ref_en_option = if ref_en { AD7124_ADC_CTRL_REG_REF_EN } else { 0 };
        let data_status_option = AD7124_ADC_CTRL_REG_DATA_STATUS; // 启用数据+状态模式
        let dout_rdy_del_option = AD7124_ADC_CTRL_REG_DOUT_RDY_DEL;
        // Combined Configuration Options
        let current_val = mode_option
                        | power_mode_option
                        | clk_sel_option
                        | ref_en_option
                        | data_status_option
                        | dout_rdy_del_option;
        self.write_register(AD7124RegId::RegControl, current_val as u32 ).await?;
        self.op_mode = mode;
        Ok(())
    }

    pub async fn set_config(&mut self,
        setup_index: u8, ref_now : AD7124RefSource,
        gain: AD7124GainSel,
        burnout_current: AD7124BurnoutCurrent,
        bipolar: bool) 
        -> Result<(), AD7124Error<E>> {
        // Defining individual configuration options
        let ref_sel_option = AD7124_CFG_REG_REF_SEL(ref_now as u16);
        let pga_option = AD7124_CFG_REG_PGA(gain as u16);
        let bipolar_option = if bipolar { AD7124_CFG_REG_BIPOLAR } else { 0 };
        let burnout_option = AD7124_CFG_REG_BURNOUT(burnout_current as u16);
        let ref_buf_option = AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_REF_BUFM;
        let ain_buf_option = AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM;
        
        // Combined Configuration Options
        let current_val = ref_sel_option
                        | pga_option
                        | bipolar_option
                        | burnout_option
                        | ref_buf_option
                        | ain_buf_option;
        let reg_now = AD7124RegId::from(AD7124RegId::RegConfig0 as u8 + setup_index);
        self.write_register(reg_now, current_val as u32).await
    }

    pub async fn set_filter(&mut self, 
            setup_index: u8, 
            filter: AD7124Filter, 
            fs:u16,
            post_filter: AD7124PostFilter,
            rej60:bool,
            single:bool) 
            -> Result<(), AD7124Error<E>> {
        let filter_option = AD7124_FILT_REG_FILTER(filter as u32);
        let post_filter_option = AD7124_FILT_REG_POST_FILTER(post_filter as u32);
        let fs_option = AD7124_FILT_REG_FS(fs as u32 );
        let rej60_option = if rej60 { AD7124_FILT_REG_REJ60 } else { 0 };
        let single_cycle_option = if single { AD7124_FILT_REG_SINGLE_CYCLE } else { 0 };
        
        let current_val = filter_option
                        | post_filter_option
                        | fs_option
                        | rej60_option
                        | single_cycle_option;
        let reg_now = AD7124RegId::from(AD7124RegId::RegFilter0 as u8 + setup_index);
        self.write_register(reg_now, current_val as u32).await
    }

    pub async fn set_channel(&mut self, 
            channel_index: u8, 
            ainp: AD7124VBiasPin, 
            ainm: AD7124VBiasPin, 
            setup_index: u8,
            enable:bool) 
            -> Result<(), AD7124Error<E>> {
        let ainp_option = AD7124_CH_MAP_REG_AINP(ainp as u16);
        let ainm_option = AD7124_CH_MAP_REG_AINM(ainm as u16);
        let setup_option = AD7124_CH_MAP_REG_SETUP(setup_index as u16);
        let enable_option = if enable { AD7124_CH_MAP_REG_CH_ENABLE } else { 0 };
        let current_val = ainp_option
                        | ainm_option
                        | setup_option
                        | enable_option;
        let reg_now = AD7124RegId::from(AD7124RegId::RegChannel0 as u8 + channel_index);
        self.write_register(reg_now, current_val as u32).await
    }

    pub async fn set_offset_calibration(&mut self, setup_index: u8, offset: u32) -> Result<(), AD7124Error<E>> {
        let reg_now = AD7124RegId::from(AD7124RegId::RegOffset0 as u8 + setup_index);
        self.write_register(reg_now, offset).await
    }

    pub async fn set_gain_calibration(&mut self, setup_index: u8, gain: u32) -> Result<(), AD7124Error<E>> {
        let reg_now = AD7124RegId::from(AD7124RegId::RegGain0 as u8 + setup_index);
        self.write_register(reg_now, gain).await
    }

    pub async fn set_pwrsw(&mut self, enable:bool) -> Result<(), AD7124Error<E>> {
        //let mut current_value = self.read_register(AD7124RegId::RegIocon1).await?;//REGS[AD7124RegId::RegIocon1 as usize].value;
        let mut current_value = unsafe{REGS[AD7124RegId::RegIocon1 as usize].value};
        if enable { current_value |= AD7124_IO_CTRL1_REG_PDSW } else { current_value &= !AD7124_IO_CTRL1_REG_PDSW };
        self.write_register(AD7124RegId::RegIocon1, current_value).await
    }
    
    pub async fn set_vbias(&mut self,vbias_pin : AD7124VBiasPin, enable:bool) -> Result<(), AD7124Error<E>> {
        //let mut current_value = self.read_register(AD7124RegId::RegIocon2).await?;//REGS[AD7124RegId::RegIocon2 as usize].value;
        let mut current_value = unsafe{REGS[AD7124RegId::RegIocon2 as usize].value};
        if enable { current_value |= 1 << vbias_pin as u32 } else { current_value &= !(1 << vbias_pin as u32) };
        self.write_register(AD7124RegId::RegIocon2, current_value).await
    }

    pub async fn read_raw_data(&mut self,channel:u8) -> Result<u32, AD7124Error<E>> {
        let active_channel = self.get_active_channel().await?;
        if active_channel != channel {
            self.enable_channel(active_channel, false).await?;
            
            if self.op_mode == AD7124OperatingMode::SingleConv {
                self.enable_channel(channel, true).await?;
                self.set_op_mode(AD7124OperatingMode::SingleConv).await?;
                //self.set_adc_control(AD7124OperatingMode::SingleConv, AD7124PowerMode::Low, AD7124ClkSource::Internal, false).await?;
            }
        }
        Ok(1u32)
        
    }

    pub async fn set_op_mode(&mut self,mode:AD7124OperatingMode) -> Result<(), AD7124Error<E>> {
        let mut current_val = unsafe {REGS[AD7124RegId::RegControl as usize].value};
        current_val &= !AD7124_ADC_CTRL_REG_MODE(0xF) as u32;
        current_val |= AD7124_ADC_CTRL_REG_MODE(mode as u16) as u32;
        //let current_val = AD7124_ADC_CTRL_REG_MODE(mode as u16);
        self.write_register(AD7124RegId::RegControl, current_val as u32).await
    }

    pub async fn enable_channel(&mut self, channel:u8,enable:bool) -> Result<(), AD7124Error<E>> {
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
    pub async fn get_active_channel(&mut self) -> Result<u8, AD7124Error<E>> {
        let data = self.read_register(AD7124RegId::RegStatus).await?;
        Ok(data as u8 & 0x07)
    }
}