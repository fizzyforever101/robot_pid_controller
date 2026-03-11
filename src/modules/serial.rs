// Serial interface module using Embassy
use embassy_stm32::usart::{Usart, Config as UsartConfig};
use embassy_stm32::peripherals::{USART, Pin};

pub struct SerialInterface<'a> {
    pub usart: Usart<'a>,
}

impl<'a> SerialInterface<'a> {
    pub fn new(usart_periph: USART, tx_pin: Pin, rx_pin: Pin) -> Self {
        let config = UsartConfig::default();
        let usart = Usart::new(usart_periph, tx_pin, rx_pin, config);
        SerialInterface { usart }
    }

    pub async fn send(&mut self, data: &[u8]) {
        let _ = self.usart.write(data).await;
    }
}
