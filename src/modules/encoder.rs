// Encoder module for STM32 QEI
use embassy_stm32::timer::qei::{Qei, QeiPin, Config as QeiConfig};
use embassy_stm32::gpio::{Input, Pull};

pub struct Encoder {
    pub qei: Qei,
}

impl Encoder {
    pub fn new(timer: embassy_stm32::peripherals::TIM, pin_a: embassy_stm32::peripherals::Pin, pin_b: embassy_stm32::peripherals::Pin) -> Self {
        // Configure pins as input with pull-up
        let pin_a = Input::new(pin_a, Pull::Up);
        let pin_b = Input::new(pin_b, Pull::Up);
        // QEI config: mode 3
        let qei_config = QeiConfig { mode: 3, ..Default::default() };
        let qei = Qei::new(timer, QeiPin::new(pin_a), QeiPin::new(pin_b)).with_config(qei_config);
        Encoder { qei }
    }

    pub fn count(&self) -> u16 {
        self.qei.count()
    }

    pub fn reset(&mut self) {
        self.qei.reset_count();
    }
}
