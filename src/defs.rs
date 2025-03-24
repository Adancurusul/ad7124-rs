#![allow(dead_code)]
/* AD7124 Register Map */
pub const AD7124_COMM_REG: u8 = 0x00;
pub const AD7124_STATUS_REG: u8 = 0x00;
pub const AD7124_ADC_CTRL_REG: u8 = 0x01;
pub const AD7124_DATA_REG: u8 = 0x02;
pub const AD7124_IO_CTRL1_REG: u8 = 0x03;
pub const AD7124_IO_CTRL2_REG: u8 = 0x04;
pub const AD7124_ID_REG: u8 = 0x05;
pub const AD7124_ERR_REG: u8 = 0x06;
pub const AD7124_ERREN_REG: u8 = 0x07;
pub const AD7124_CH0_MAP_REG: u8 = 0x09;
pub const AD7124_CH1_MAP_REG: u8 = 0x0A;
pub const AD7124_CH2_MAP_REG: u8 = 0x0B;
pub const AD7124_CH3_MAP_REG: u8 = 0x0C;
pub const AD7124_CH4_MAP_REG: u8 = 0x0D;
pub const AD7124_CH5_MAP_REG: u8 = 0x0E;
pub const AD7124_CH6_MAP_REG: u8 = 0x0F;
pub const AD7124_CH7_MAP_REG: u8 = 0x10;
pub const AD7124_CH8_MAP_REG: u8 = 0x11;
pub const AD7124_CH9_MAP_REG: u8 = 0x12;
pub const AD7124_CH10_MAP_REG: u8 = 0x13;
pub const AD7124_CH11_MAP_REG: u8 = 0x14;
pub const AD7124_CH12_MAP_REG: u8 = 0x15;
pub const AD7124_CH13_MAP_REG: u8 = 0x16;
pub const AD7124_CH14_MAP_REG: u8 = 0x17;
pub const AD7124_CH15_MAP_REG: u8 = 0x18;
pub const AD7124_CFG0_REG: u8 = 0x19;
pub const AD7124_CFG1_REG: u8 = 0x1A;
pub const AD7124_CFG2_REG: u8 = 0x1B;
pub const AD7124_CFG3_REG: u8 = 0x1C;
pub const AD7124_CFG4_REG: u8 = 0x1D;
pub const AD7124_CFG5_REG: u8 = 0x1E;
pub const AD7124_CFG6_REG: u8 = 0x1F;
pub const AD7124_CFG7_REG: u8 = 0x20;
pub const AD7124_FILT0_REG: u8 = 0x21;
pub const AD7124_FILT1_REG: u8 = 0x22;
pub const AD7124_FILT2_REG: u8 = 0x23;
pub const AD7124_FILT3_REG: u8 = 0x24;
pub const AD7124_FILT4_REG: u8 = 0x25;
pub const AD7124_FILT5_REG: u8 = 0x26;
pub const AD7124_FILT6_REG: u8 = 0x27;
pub const AD7124_FILT7_REG: u8 = 0x28;
pub const AD7124_OFFS0_REG: u8 = 0x29;
pub const AD7124_OFFS1_REG: u8 = 0x2A;
pub const AD7124_OFFS2_REG: u8 = 0x2B;
pub const AD7124_OFFS3_REG: u8 = 0x2C;
pub const AD7124_OFFS4_REG: u8 = 0x2D;
pub const AD7124_OFFS5_REG: u8 = 0x2E;
pub const AD7124_OFFS6_REG: u8 = 0x2F;
pub const AD7124_OFFS7_REG: u8 = 0x30;
pub const AD7124_GAIN0_REG: u8 = 0x31;
pub const AD7124_GAIN1_REG: u8 = 0x32;
pub const AD7124_GAIN2_REG: u8 = 0x33;
pub const AD7124_GAIN3_REG: u8 = 0x34;
pub const AD7124_GAIN4_REG: u8 = 0x35;
pub const AD7124_GAIN5_REG: u8 = 0x36;
pub const AD7124_GAIN6_REG: u8 = 0x37;
pub const AD7124_GAIN7_REG: u8 = 0x38;

/* Communication Register bits */
pub const AD7124_COMM_REG_WEN: u8 = 0 << 7;
pub const AD7124_COMM_REG_WR: u8 = 0 << 6;
pub const AD7124_COMM_REG_RD: u8 = 1 << 6;
pub const AD7124_COMM_REG_RA: fn(u8) -> u8 = |x| (x & 0x3F);

/* Status Register bits */
pub const AD7124_STATUS_REG_RDY: u8 = 1 << 7;
pub const AD7124_STATUS_REG_ERROR_FLAG: u8 = 1 << 6;
pub const AD7124_STATUS_REG_POR_FLAG: u8 = 1 << 4;
pub const AD7124_STATUS_REG_CH_ACTIVE: fn(u8) -> u8 = |x| (x & 0xF);

/* ADC_Control Register bits */
pub const AD7124_ADC_CTRL_REG_DOUT_RDY_DEL: u16 = 1 << 12;
pub const AD7124_ADC_CTRL_REG_CONT_READ: u16 = 1 << 11;
pub const AD7124_ADC_CTRL_REG_DATA_STATUS: u16 = 1 << 10;
pub const AD7124_ADC_CTRL_REG_CS_EN: u16 = 1 << 9;
pub const AD7124_ADC_CTRL_REG_REF_EN: u16 = 1 << 8;
pub const AD7124_ADC_CTRL_REG_POWER_MODE: fn(u16) -> u16 = |x| ((x & 0x3) << 6);
pub const AD7124_ADC_CTRL_REG_MODE: fn(u16) -> u16 = |x| ((x & 0xF) << 2);
pub const AD7124_ADC_CTRL_REG_CLK_SEL: fn(u16) -> u16 = |x| (x & 0x3);

/* IO_Control_1 Register bits */
pub const AD7124_IO_CTRL1_REG_GPIO_DAT2: u32 = 1 << 23;
pub const AD7124_IO_CTRL1_REG_GPIO_DAT1: u32 = 1 << 22;
pub const AD7124_IO_CTRL1_REG_GPIO_CTRL2: u32 = 1 << 19;
pub const AD7124_IO_CTRL1_REG_GPIO_CTRL1: u32 = 1 << 18;
pub const AD7124_IO_CTRL1_REG_PDSW: u32 = 1 << 15;
pub const AD7124_IO_CTRL1_REG_IOUT1: fn(u32) -> u32 = |x| ((x & 0x7) << 11);
pub const AD7124_IO_CTRL1_REG_IOUT0: fn(u32) -> u32 = |x| ((x & 0x7) << 8);
pub const AD7124_IO_CTRL1_REG_IOUT_CH1: fn(u32) -> u32 = |x| ((x & 0xF) << 4);
pub const AD7124_IO_CTRL1_REG_IOUT_CH0: fn(u32) -> u32 = |x| (x & 0xF);

// IO_Control_1 AD7124-8 specific bits
pub const AD7124_8_IO_CTRL1_REG_GPIO_DAT4: u32 = 1 << 23;
pub const AD7124_8_IO_CTRL1_REG_GPIO_DAT3: u32 = 1 << 22;
pub const AD7124_8_IO_CTRL1_REG_GPIO_DAT2: u32 = 1 << 21;
pub const AD7124_8_IO_CTRL1_REG_GPIO_DAT1: u32 = 1 << 20;
pub const AD7124_8_IO_CTRL1_REG_GPIO_CTRL4: u32 = 1 << 19;
pub const AD7124_8_IO_CTRL1_REG_GPIO_CTRL3: u32 = 1 << 18;
pub const AD7124_8_IO_CTRL1_REG_GPIO_CTRL2: u32 = 1 << 17;
pub const AD7124_8_IO_CTRL1_REG_GPIO_CTRL1: u32 = 1 << 16;

/* IO_Control_2 Register bits */
pub const AD7124_IO_CTRL2_REG_GPIO_VBIAS7: u16 = 1 << 15;
pub const AD7124_IO_CTRL2_REG_GPIO_VBIAS6: u16 = 1 << 14;
pub const AD7124_IO_CTRL2_REG_GPIO_VBIAS5: u16 = 1 << 11;
pub const AD7124_IO_CTRL2_REG_GPIO_VBIAS4: u16 = 1 << 10;
pub const AD7124_IO_CTRL2_REG_GPIO_VBIAS3: u16 = 1 << 5;
pub const AD7124_IO_CTRL2_REG_GPIO_VBIAS2: u16 = 1 << 4;
pub const AD7124_IO_CTRL2_REG_GPIO_VBIAS1: u16 = 1 << 1;
pub const AD7124_IO_CTRL2_REG_GPIO_VBIAS0: u16 = 1 << 0;

/* IO_Control_2 AD7124-8 specific bits */
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS15: u16 = 1 << 15;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS14: u16 = 1 << 14;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS13: u16 = 1 << 13;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS12: u16 = 1 << 12;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS11: u16 = 1 << 11;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS10: u16 = 1 << 10;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS9: u16 = 1 << 9;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS8: u16 = 1 << 8;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS7: u16 = 1 << 7;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS6: u16 = 1 << 6;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS5: u16 = 1 << 5;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS4: u16 = 1 << 4;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS3: u16 = 1 << 3;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS2: u16 = 1 << 2;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS1: u16 = 1 << 1;
pub const AD7124_8_IO_CTRL2_REG_GPIO_VBIAS0: u16 = 1 << 0;

/* ID Register bits */
//pub const AD7124_ID_REG_DEVICE_ID: u8 = |x| ((x & 0xF) << 4);
pub const AD7124_ID_REG_DEVICE_ID: fn(u8) -> u8 = |x| ((x & 0xF) << 4);
pub const AD7124_ID_REG_SILICON_REV: fn(u8) -> u8 = |x| (x & 0xF);

/* Error Register bits */
pub const AD7124_ERR_REG_LDO_CAP_ERR: u32 = 1 << 19;
pub const AD7124_ERR_REG_ADC_CAL_ERR: u32 = 1 << 18;
pub const AD7124_ERR_REG_ADC_CONV_ERR: u32 = 1 << 17;
pub const AD7124_ERR_REG_ADC_SAT_ERR: u32 = 1 << 16;
pub const AD7124_ERR_REG_AINP_OV_ERR: u32 = 1 << 15;
pub const AD7124_ERR_REG_AINP_UV_ERR: u32 = 1 << 14;
pub const AD7124_ERR_REG_AINM_OV_ERR: u32 = 1 << 13;
pub const AD7124_ERR_REG_AINM_UV_ERR: u32 = 1 << 12;
pub const AD7124_ERR_REG_REF_DET_ERR: u32 = 1 << 11;
pub const AD7124_ERR_REG_DLDO_PSM_ERR: u32 = 1 << 9;
pub const AD7124_ERR_REG_ALDO_PSM_ERR: u32 = 1 << 7;
pub const AD7124_ERR_REG_SPI_IGNORE_ERR: u32 = 1 << 6;
pub const AD7124_ERR_REG_SPI_SLCK_CNT_ERR: u32 = 1 << 5;
pub const AD7124_ERR_REG_SPI_READ_ERR: u32 = 1 << 4;
pub const AD7124_ERR_REG_SPI_WRITE_ERR: u32 = 1 << 3;
pub const AD7124_ERR_REG_SPI_CRC_ERR: u32 = 1 << 2;
pub const AD7124_ERR_REG_MM_CRC_ERR: u32 = 1 << 1;

/* Error_En Register bits */
pub const AD7124_ERREN_REG_MCLK_CNT_EN: u32 = 1 << 22;
pub const AD7124_ERREN_REG_LDO_CAP_CHK_TEST_EN: u32 = 1 << 21;
pub const AD7124_ERREN_REG_LDO_CAP_CHK: fn(u32) -> u32 = |x| (x & 0x3) << 19;
pub const AD7124_ERREN_REG_ADC_CAL_ERR_EN: u32 = 1 << 18;
pub const AD7124_ERREN_REG_ADC_CONV_ERR_EN: u32 = 1 << 17;
pub const AD7124_ERREN_REG_ADC_SAT_ERR_EN: u32 = 1 << 16;
pub const AD7124_ERREN_REG_AINP_OV_ERR_EN: u32 = 1 << 15;
pub const AD7124_ERREN_REG_AINP_UV_ERR_EN: u32 = 1 << 14;
pub const AD7124_ERREN_REG_AINM_OV_ERR_EN: u32 = 1 << 13;
pub const AD7124_ERREN_REG_AINM_UV_ERR_EN: u32 = 1 << 12;
pub const AD7124_ERREN_REG_REF_DET_ERR_EN: u32 = 1 << 11;
pub const AD7124_ERREN_REG_DLDO_PSM_TRIP_TEST_EN: u32 = 1 << 10;
pub const AD7124_ERREN_REG_DLDO_PSM_ERR_EN: u32 = 1 << 9;
pub const AD7124_ERREN_REG_ALDO_PSM_TRIP_TEST_EN: u32 = 1 << 8;
pub const AD7124_ERREN_REG_ALDO_PSM_ERR_EN: u32 = 1 << 7;
pub const AD7124_ERREN_REG_SPI_IGNORE_ERR_EN: u32 = 1 << 6;
pub const AD7124_ERREN_REG_SPI_SCLK_CNT_ERR_EN: u32 = 1 << 5;
pub const AD7124_ERREN_REG_SPI_READ_ERR_EN: u32 = 1 << 4;
pub const AD7124_ERREN_REG_SPI_WRITE_ERR_EN: u32 = 1 << 3;
pub const AD7124_ERREN_REG_SPI_CRC_ERR_EN: u32 = 1 << 2;
pub const AD7124_ERREN_REG_MM_CRC_ERR_EN: u32 = 1 << 1;

/* Channel Registers 0-15 bits */
pub const AD7124_CH_MAP_REG_CH_ENABLE: u16 = 1 << 15;
pub const AD7124_CH_MAP_REG_SETUP: fn(u16) -> u16 = |x| (x & 0x7) << 12;
pub const AD7124_CH_MAP_REG_AINP: fn(u16) -> u16 = |x| (x & 0x1F) << 5;
pub const AD7124_CH_MAP_REG_AINM: fn(u16) -> u16 = |x| (x & 0x1F);

/* Configuration Registers 0-7 bits */
pub const AD7124_CFG_REG_BIPOLAR: u16 = 1 << 11;
pub const AD7124_CFG_REG_BURNOUT: fn(u16) -> u16 = |x| (x & 0x3) << 9;
pub const AD7124_CFG_REG_REF_BUFP: u16 = 1 << 8;
pub const AD7124_CFG_REG_REF_BUFM: u16 = 1 << 7;
pub const AD7124_CFG_REG_AIN_BUFP: u16 = 1 << 6;
pub const AD7124_CFG_REG_AINN_BUFM: u16 = 1 << 5;
pub const AD7124_CFG_REG_REF_SEL: fn(u16) -> u16 = |x| (x & 0x3) << 3;
pub const AD7124_CFG_REG_PGA: fn(u16) -> u16 = |x| (x & 0x7);

/* Filter Register 0-7 bits */
pub const AD7124_FILT_REG_FILTER: fn(u32) -> u32 = |x| (x & 0x7) << 21;
pub const AD7124_FILT_REG_REJ60: u32 = 1 << 20;
pub const AD7124_FILT_REG_POST_FILTER: fn(u32) -> u32 = |x| (x & 0x7) << 17;
pub const AD7124_FILT_REG_SINGLE_CYCLE: u32 = 1 << 16;
pub const AD7124_FILT_REG_FS: fn(u32) -> u32 = |x| (x & 0x7FF);

/* AD7124 Constants */
pub const AD7124_CRC8_POLYNOMIAL_REPRESENTATION: u8 = 0x07; /* x8 + x2 + x + 1 */
pub const AD7124_DISABLE_CRC: u8 = 0;
pub const AD7124_USE_CRC: bool = true;
