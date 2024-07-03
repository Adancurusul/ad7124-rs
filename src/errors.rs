use defmt;

#[derive(Debug, PartialEq, Eq, Copy, Clone,defmt::Format)]
pub enum AD7124Error<E> {
    SPI(E),
    OutputPin(E),
    CommunicationError,
    InvalidValue,
    TimeOut,
}