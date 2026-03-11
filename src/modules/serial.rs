use embassy_stm32::mode::Blocking;
use embassy_stm32::usart::{Config, ConfigError, Instance, RxPin, TxPin, Uart};
use embassy_stm32::Peri;
use embedded_hal_nb::serial::Read as _;

/// Minimal bidirectional serial interface (blocking write + nonblocking read).
pub struct SerialInterface<'d> {
    uart: Uart<'d, Blocking>,
}

impl<'d> SerialInterface<'d> {
    pub fn new<T: Instance>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let uart = Uart::new_blocking(peri, rx, tx, config)?;
        Ok(Self { uart })
    }

    pub fn write(&mut self, bytes: &[u8]) {
        // Best-effort: if TX errors happen, we keep running the control loop.
        let _ = self.uart.blocking_write(bytes);
    }

    /// Read one byte if available.
    pub fn try_read(&mut self) -> Option<u8> {
        match self.uart.read() {
            Ok(b) => Some(b),
            Err(nb::Error::WouldBlock) => None,
            Err(nb::Error::Other(_e)) => None,
        }
    }
}
