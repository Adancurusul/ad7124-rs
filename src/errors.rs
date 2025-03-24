#[derive(Debug, PartialEq, Eq, Copy, Clone, defmt::Format)]
pub enum AD7124Error<E> {
    SPI(E),
    PinError,
    CSError,
    CommunicationError,
    InvalidValue,
    TimeOut,
}

impl<E> AD7124Error<E> {
    pub fn map_err_type<T>(self) -> AD7124Error<T> {
        match self {
            AD7124Error::SPI(_) => AD7124Error::CommunicationError,
            AD7124Error::PinError => AD7124Error::PinError,
            AD7124Error::CSError => AD7124Error::CSError,
            AD7124Error::CommunicationError => AD7124Error::CommunicationError,
            AD7124Error::InvalidValue => AD7124Error::InvalidValue,
            AD7124Error::TimeOut => AD7124Error::TimeOut,
        }
    }
}
