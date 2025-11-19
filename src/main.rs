#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use panic_probe as _;

use defmt_rtt as _;

use embassy_stm32::{
    gpio::{Input, OutputType, Pull},
    time::khz,
    timer::simple_pwm::{PwmPin, SimplePwm},
    Config,
};
use pid::Pid;

// Set both CH1 and CH2 fully off (coast)
macro_rules! pwm_off {
    ($pwm:expr) => {{
        {
            let mut ch = $pwm.ch1();
            ch.set_duty_cycle_fully_off();
        }
        {
            let mut ch = $pwm.ch2();
            ch.set_duty_cycle_fully_off();
        }
    }};
}

// Drive forward: CH1 = duty (RPWM), CH2 = 0 (LPWM)
macro_rules! pwm_fwd {
    ($pwm:expr, $duty:expr) => {{
        {
            let mut ch = $pwm.ch1();
            ch.set_duty_cycle($duty);
        }
        {
            let mut ch = $pwm.ch2();
            ch.set_duty_cycle_fully_off();
        }
    }};
}

// Drive reverse: CH1 = 0 (RPWM), CH2 = duty (LPWM)
macro_rules! pwm_rev {
    ($pwm:expr, $duty:expr) => {{
        {
            let mut ch = $pwm.ch1();
            ch.set_duty_cycle_fully_off();
        }
        {
            let mut ch = $pwm.ch2();
            ch.set_duty_cycle($duty);
        }
    }};
}

const SAMPLE_HZ: u32 = 1000; // encoder polling rate (Hz)
const CPR_X4: f32 = 600.0 * 4.0; // 2400 counts/rev
const FWD_CPS: f32 = 4000.0; // forward target cps per wheel
const REV_CPS: f32 = -4000.0; // reverse target cps per wheel
const FWD_TIME_MS: u32 = 3000;
const REV_TIME_MS: u32 = 3000;
const COAST_TIME_MS: u32 = 800;

const PID_DT_MS: u32 = 10; // PID update rate (100 Hz) inside SAMPLE_HZ loop

// Map PID effort (counts/s error) -> (direction, duty), with cap
fn duty_from_effort(effort: f32, max: u16, cap_percent: u16) -> (bool, u16) {
    let dir_fwd = effort >= 0.0;
    let mut duty = effort.abs() as u32 * 6400 as u32;
    // let mut duty = (effort.abs() / (CPR_X4 * 50.0) * (max as f32)) as u32;
    // duty = 60000;
    // duty = ((duty * (cap_percent as u32)) / 100).min(max as u32);
    info!("duty is: {}", duty);
    (dir_fwd, duty as u16)
}

// State = (A<<1)|B ∈ {00,01,11,10}. LUT maps (prev<<2)|curr → -1/0/1.
// Forward: 00→10→11→01→00. Reverse: 00→01→11→10→00.
const QUAD_LUT: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];

struct PollEncoder {
    a: Input<'static>,
    b: Input<'static>,
    prev: u8,
}
impl PollEncoder {
    fn new(a: Input<'static>, b: Input<'static>) -> Self {
        let mut me = Self { a, b, prev: 0 };
        me.prev = me.state();
        me
    }
    #[inline]
    fn state(&self) -> u8 {
        let a = self.a.is_high() as u8;
        let b = self.b.is_high() as u8;
        (a << 1) | b
    }
    // returns -1/0/1 counts-per-sample
    fn sample_delta(&mut self) -> i32 {
        let curr = self.state();
        let idx = ((self.prev) << 2) | curr;
        self.prev = curr;
        QUAD_LUT[idx as usize] as i32
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum Phase {
    Forward,
    Coast1,
    Reverse,
    Coast2,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());
    // // info!("I am here!");
    //
    // left motor (TIM4: PD12=CH1 RPWM, PD13=CH2 LPWM)
    let left_ch1_pin = PwmPin::new(p.PD12, OutputType::PushPull);
    let left_ch2_pin = PwmPin::new(p.PD13, OutputType::PushPull);
    let mut left_pwm = SimplePwm::new(
        p.TIM4,
        Some(left_ch1_pin),
        Some(left_ch2_pin),
        None,
        None,
        khz(1),
        Default::default(),
    );
    {
        let mut ch = left_pwm.ch1();

        ch.enable();
    }
    {
        let mut ch = left_pwm.ch2();

        ch.enable();
    }
    let l_max: u16 = {
        let ch = left_pwm.ch1();
        ch.max_duty_cycle()
    };
    info!("l_max is: {}", l_max);
    pwm_off!(left_pwm); // start coasting
                        //
    let right_ch1_pin = PwmPin::new(p.PF0, OutputType::PushPull);
    let right_ch2_pin = PwmPin::new(p.PF1, OutputType::PushPull);
    let mut right_pwm = SimplePwm::new(
        p.TIM23,
        Some(right_ch1_pin),
        Some(right_ch2_pin),
        None,
        None,
        khz(1),
        Default::default(),
    );
    {
        let mut ch = right_pwm.ch1();
        ch.enable();
    }
    {
        let mut ch = right_pwm.ch2();

        ch.enable();
    }
    let r_max: u16 = {
        let ch = right_pwm.ch1();
        ch.max_duty_cycle()
    };
    info!("r_max is: {}", r_max);
    pwm_off!(right_pwm);
    // loop {
    //     info!("driving forward!");
    //     pwm_fwd!(right_pwm, 20000);
    //     pwm_fwd!(left_pwm, 20000);
    // }
    //
    let left_a = Input::new(p.PA0, Pull::Up);
    let left_b = Input::new(p.PA1, Pull::Up);
    let mut enc_left = PollEncoder::new(left_a, left_b);

    let right_a = Input::new(p.PA6, Pull::Up);
    let right_b = Input::new(p.PA7, Pull::Up);
    let mut enc_right = PollEncoder::new(right_a, right_b);

    let mut pid_l = Pid::new(100.0, 10.0);
    pid_l.p(1.0, 10.0);
    let mut pid_r = Pid::new(100.0, 10.0);
    pid_r.p(1.0, 10.0);

    let mut phase = Phase::Forward;
    info!("moving forward now!");
    let mut phase_remaining_ms: u32 = FWD_TIME_MS;
    let mut target_l: f32 = FWD_CPS;
    let mut target_r: f32 = FWD_CPS;

    let mut cps_l: f32 = 0.0;
    let mut cps_r: f32 = 0.0;
    let mut pid_accum_ms: u32 = 0;

    // 1 kHz loop
    let period_ms = 1000 / SAMPLE_HZ;
    let mut tick = Ticker::every(Duration::from_millis(period_ms as u64));

    loop {
        let dl = enc_left.sample_delta();
        info!("dl: {}", dl);
        let dr = enc_right.sample_delta();
        info!("dr: {}", dr);
        cps_l = (dl as f32) * (SAMPLE_HZ as f32);
        info!("cps_l: {}", cps_l);
        cps_r = (dr as f32) * (SAMPLE_HZ as f32);
        info!("cps_r: {}", cps_r);

        pid_accum_ms += period_ms;
        if pid_accum_ms >= PID_DT_MS {
            pid_accum_ms = 0;

            let u_l = pid_l.next_control_output(target_l - cps_l).output;
            info!("u_l is: {}", u_l);
            let u_r = pid_r.next_control_output(target_r - cps_r).output;
            info!("u_r is: {}", u_r);

            let (fwd_l, duty_l) = duty_from_effort(u_l, l_max, 100);
            info!("fwd_l, duty_l is: {}", (fwd_l, duty_l));
            let (fwd_r, duty_r) = duty_from_effort(u_r, r_max, 100);
            info!("fwd_r, duty_r is: {}", (fwd_r, duty_r));

            if u_l.abs() < 1.0 {
                pwm_off!(left_pwm);
            } else if fwd_l {
                // pwm_fwd!(left_pwm, duty_l)

                pwm_fwd!(left_pwm, 50)
            } else {
                // pwm_rev!(left_pwm, duty_l)

                pwm_fwd!(left_pwm, 50)
            }

            if u_r.abs() < 1.0 {
                pwm_off!(right_pwm);
            } else if fwd_r {
                // pwm_fwd!(right_pwm, duty_r)

                pwm_fwd!(right_pwm, 50)
            } else {
                // pwm_rev!(right_pwm, duty_r)

                pwm_fwd!(right_pwm, 50)
            }
        }

        if phase_remaining_ms > 0 {
            phase_remaining_ms -= period_ms;
        } else {
            match phase {
                Phase::Forward => {
                    target_l = 0.0;
                    target_r = 0.0;
                    phase = Phase::Coast1;
                    phase_remaining_ms = COAST_TIME_MS;
                }
                Phase::Coast1 => {
                    target_l = REV_CPS;
                    target_r = REV_CPS;
                    phase = Phase::Reverse;
                    phase_remaining_ms = REV_TIME_MS;
                }
                Phase::Reverse => {
                    target_l = 0.0;
                    target_r = 0.0;
                    phase = Phase::Coast2;
                    phase_remaining_ms = COAST_TIME_MS;
                }
                Phase::Coast2 => {
                    target_l = FWD_CPS;
                    target_r = FWD_CPS;
                    phase = Phase::Forward;
                    phase_remaining_ms = FWD_TIME_MS;
                }
            }
        }

        tick.next().await;
    }
}
