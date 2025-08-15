//! Error handling for AD7124 driver
//!
//! Provides a unified error model with family-consistent naming following v3.0 standards

/// Core AD7124 errors (transport-layer independent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AD7124CoreError {
    /// Invalid parameter provided
    InvalidParameter,
    /// Invalid configuration
    InvalidConfiguration,
    /// Device not responding
    DeviceNotResponding,
    /// Invalid device ID
    InvalidDeviceId,
    /// Unsupported operation for this device variant
    UnsupportedOperation,
    /// Data corruption detected
    DataCorruption,
    /// Buffer overflow in command sequence
    BufferOverflow,
    /// Value out of valid range
    ValueOutOfRange,
    /// Device not initialized
    NotInitialized,
    /// Communication timeout
    Timeout,
    /// CRC validation failed
    CrcError,
    /// ADC calibration failed
    CalibrationFailed,
    /// ADC conversion error
    ConversionError,
    /// Channel not enabled
    ChannelNotEnabled,
    /// Reference voltage error
    ReferenceVoltageError,
    /// Power supply fault
    PowerSupplyFault,
}

/// AD7124 driver error type (supports transport and pin errors)
#[derive(Debug)]
pub enum AD7124Error<TransportE = (), PinE = ()> {
    /// Core driver error
    Core(AD7124CoreError),
    /// Transport error (SPI)
    Transport(TransportE),
    /// Pin control error
    Pin(PinE),
    /// Operation timeout
    Timeout,
}

impl<TransportE, PinE> From<AD7124CoreError> for AD7124Error<TransportE, PinE> {
    fn from(err: AD7124CoreError) -> Self {
        AD7124Error::Core(err)
    }
}

// Note: From<TransportE> implementation would conflict with From<AD7124CoreError>
// Users must wrap transport errors manually: AD7124Error::Transport(transport_err)

impl<TransportE, PinE> core::fmt::Display for AD7124Error<TransportE, PinE>
where
    TransportE: core::fmt::Display,
    PinE: core::fmt::Display,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AD7124Error::Core(err) => match err {
                AD7124CoreError::InvalidParameter => write!(f, "Invalid parameter"),
                AD7124CoreError::InvalidConfiguration => write!(f, "Invalid configuration"),
                AD7124CoreError::DeviceNotResponding => write!(f, "Device not responding"),
                AD7124CoreError::InvalidDeviceId => write!(f, "Invalid device ID"),
                AD7124CoreError::UnsupportedOperation => write!(f, "Unsupported operation"),
                AD7124CoreError::DataCorruption => write!(f, "Data corruption detected"),
                AD7124CoreError::BufferOverflow => write!(f, "Buffer overflow"),
                AD7124CoreError::ValueOutOfRange => write!(f, "Value out of range"),
                AD7124CoreError::NotInitialized => write!(f, "Device not initialized"),
                AD7124CoreError::Timeout => write!(f, "Communication timeout"),
                AD7124CoreError::CrcError => write!(f, "CRC validation failed"),
                AD7124CoreError::CalibrationFailed => write!(f, "ADC calibration failed"),
                AD7124CoreError::ConversionError => write!(f, "ADC conversion error"),
                AD7124CoreError::ChannelNotEnabled => write!(f, "Channel not enabled"),
                AD7124CoreError::ReferenceVoltageError => write!(f, "Reference voltage error"),
                AD7124CoreError::PowerSupplyFault => write!(f, "Power supply fault"),
            },
            AD7124Error::Transport(err) => write!(f, "Transport error: {}", err),
            AD7124Error::Pin(err) => write!(f, "Pin error: {}", err),
            AD7124Error::Timeout => write!(f, "Operation timeout"),
        }
    }
}

#[cfg(feature = "defmt")]
impl<TransportE, PinE> defmt::Format for AD7124Error<TransportE, PinE>
where
    TransportE: defmt::Format,
    PinE: defmt::Format,
{
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            AD7124Error::Core(err) => match err {
                AD7124CoreError::InvalidParameter => defmt::write!(f, "Invalid parameter"),
                AD7124CoreError::InvalidConfiguration => defmt::write!(f, "Invalid configuration"),
                AD7124CoreError::DeviceNotResponding => defmt::write!(f, "Device not responding"),
                AD7124CoreError::InvalidDeviceId => defmt::write!(f, "Invalid device ID"),
                AD7124CoreError::UnsupportedOperation => defmt::write!(f, "Unsupported operation"),
                AD7124CoreError::DataCorruption => defmt::write!(f, "Data corruption detected"),
                AD7124CoreError::BufferOverflow => defmt::write!(f, "Buffer overflow"),
                AD7124CoreError::ValueOutOfRange => defmt::write!(f, "Value out of range"),
                AD7124CoreError::NotInitialized => defmt::write!(f, "Device not initialized"),
                AD7124CoreError::Timeout => defmt::write!(f, "Communication timeout"),
                AD7124CoreError::CrcError => defmt::write!(f, "CRC validation failed"),
                AD7124CoreError::CalibrationFailed => defmt::write!(f, "ADC calibration failed"),
                AD7124CoreError::ConversionError => defmt::write!(f, "ADC conversion error"),
                AD7124CoreError::ChannelNotEnabled => defmt::write!(f, "Channel not enabled"),
                AD7124CoreError::ReferenceVoltageError => {
                    defmt::write!(f, "Reference voltage error")
                }
                AD7124CoreError::PowerSupplyFault => defmt::write!(f, "Power supply fault"),
            },
            AD7124Error::Transport(err) => defmt::write!(f, "Transport error: {:?}", err),
            AD7124Error::Pin(err) => defmt::write!(f, "Pin error: {:?}", err),
            AD7124Error::Timeout => defmt::write!(f, "Operation timeout"),
        }
    }
}
