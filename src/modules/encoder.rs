use embassy_stm32::timer::qei::{Qei, QeiPin};
use embassy_stm32::timer::{Ch1, Ch2, GeneralInstance4Channel, TimerPin};
use embassy_stm32::gpio::Pull;
use embassy_stm32::Peri;

/// QEI encoder reader with delta sampling.
///
/// `delta_counts()` uses wrapping subtraction and returns an `i16` delta. As long as your
/// sample period is short enough that the encoder doesn't move more than ~32k counts between
/// samples, the sign will be correct for both directions.
pub struct EncoderQei<'d, T: GeneralInstance4Channel> {
    qei: Qei<'d, T>,
    last: u16,
}

impl<'d, T: GeneralInstance4Channel> EncoderQei<'d, T> {
    pub fn new(
        tim: Peri<'d, T>,
        ch1: Peri<'d, impl TimerPin<T, Ch1>>,
        ch2: Peri<'d, impl TimerPin<T, Ch2>>,
    ) -> Self {
        // Many optical encoders (including NPN open-collector outputs) require pull-ups.
        let qei = Qei::new(
            tim,
            QeiPin::new_with_pull(ch1, Pull::Up),
            QeiPin::new_with_pull(ch2, Pull::Up),
        );
        let last = qei.count();
        Self { qei, last }
    }

    #[allow(dead_code)]
    pub fn count(&self) -> u16 {
        self.qei.count()
    }

    pub fn delta_counts(&mut self) -> i16 {
        let now = self.qei.count();
        let delta = now.wrapping_sub(self.last) as i16;
        self.last = now;
        delta
    }
}
