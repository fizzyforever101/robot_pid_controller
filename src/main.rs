#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::timer::qei::{Ch1, Ch2, Direction, Qei, QeiPin};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal::digital::{Error, InputPin};
use panic_probe as _;
use stm32_metapac as pac;

use defmt_rtt as _;

use embassy_stm32::{
    gpio::{Flex, Input, Level, Output, OutputType, Pull, Speed},
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

const PULSES_PER_REV: u32 = 600;
const WHEEL_DIAMETER_INCHES: f32 = 5.5;
const INCH_TO_METER: f32 = 0.0254;
const PI: f32 = 3.14159265359;

const MAX_PWM: u32 = 50000;
const MIN_PWM: u32 = 30000;
const SAMPLE_HZ: u32 = 1000; // encoder polling rate (Hz) make this slower
const CPR_X4: f32 = 600.0 * 4.0; // 2400 counts/rev
const FWD_CPS: f32 = 4000.0; // forward target cps per wheel
const REV_CPS: f32 = -4000.0; // reverse target cps per wheel
const FWD_TIME_MS: u32 = 3000;
const REV_TIME_MS: u32 = 3000;
const COAST_TIME_MS: u32 = 800;

const PID_DT_MS: u32 = 10; // PID update rate (100 Hz) inside SAMPLE_HZ loop

// Map PID effort (counts/s error) -> (direction, duty), with cap
fn duty_from_effort(effort: f32) -> u16 {
    let mut duty = (effort.abs() as u32 * MAX_PWM / 10)
        .min(MAX_PWM)
        .max(MIN_PWM);
    // duty = ((duty * MAX_PWM) / (CPR_X4 as u32 * 10)).min(MAX_PWM);
    duty as u16
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
    pwm_off!(left_pwm);

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
    pwm_off!(right_pwm);

    let pa6 = QeiPin::new(p.PA6);
    let pc7 = QeiPin::new(p.PC7);
    let enc_left = Qei::new(p.TIM3, pa6, pc7);
    let pe9 = QeiPin::new(p.PE9);
    let pe11 = QeiPin::new(p.PE11);
    let enc_right = Qei::new(p.TIM1, pe9, pe11);

    let mut enc_left_prev: u16 = enc_left.count();
    info!("first enc_left_prev: {}", enc_left_prev);
    let mut enc_right_prev: u16 = enc_right.count();

    info!("first enc_right_prev: {}", enc_right_prev);

    Timer::after(Duration::from_secs(5)).await;

    let mut enc_left_pos: i32 = 0;
    let mut enc_right_pos: i32 = 0;

    let period_ms: u32 = 5;
    let dt_s: f32 = period_ms as f32 / 1000.0;
    let mut tick = Ticker::every(Duration::from_millis(period_ms as u64));

    let wheel_diameter_meters = WHEEL_DIAMETER_INCHES * INCH_TO_METER;
    let wheel_circumference_meters = PI * wheel_diameter_meters;
    let ticks_per_meter = PULSES_PER_REV as f32 / wheel_circumference_meters;

    let target_ticks =
        (wheel_diameter_meters / wheel_circumference_meters) * PULSES_PER_REV as f32 * 4.0;
    info!("target_ticks: {}", target_ticks);

    let mut pid_left = Pid::new(target_ticks, MAX_PWM as f32);
    let mut pid_right = Pid::new(target_ticks, MAX_PWM as f32);
    let mut enc_right_diff: i32 = 0;
    let mut enc_left_diff: i32 = 0;

    loop {
        tick.next().await;

        const ENCODER_MAX: u16 = 0xFFFF;
        const THRESHOLD: i32 = 10000;

        let enc_right_raw_now = enc_right.count();
        let enc_left_raw_now = enc_left.count();

        let enc_right_diff: i32;
        if enc_right_raw_now as i32 - enc_right_prev as i32 > THRESHOLD {
            enc_right_diff =
                (enc_right_raw_now as i32 - ENCODER_MAX as i32 - 1) - enc_right_prev as i32;
        } else if enc_right_prev as i32 - enc_right_raw_now as i32 > THRESHOLD {
            enc_right_diff =
                ((ENCODER_MAX as i32 - enc_right_prev as i32) + enc_right_raw_now as i32 + 1);
        } else {
            enc_right_diff = enc_right_raw_now as i32 - enc_right_prev as i32;
        }
        enc_right_prev = enc_right_raw_now;
        enc_right_pos += enc_right_diff.abs();
        let enc_right_vel = enc_right_diff.abs() as f32 / dt_s;

        let enc_left_diff: i32;
        if enc_left_raw_now as i32 - enc_left_prev as i32 > THRESHOLD {
            enc_left_diff =
                (enc_left_raw_now as i32 - ENCODER_MAX as i32 - 1) - enc_left_prev as i32;
        } else if enc_left_prev as i32 - enc_left_raw_now as i32 > THRESHOLD {
            enc_left_diff =
                ((ENCODER_MAX as i32 - enc_left_prev as i32) + enc_left_raw_now as i32 + 1);
        } else {
            enc_left_diff = enc_left_raw_now as i32 - enc_left_prev as i32;
        }
        enc_left_prev = enc_left_raw_now;
        enc_left_pos += enc_left_diff.abs();
        let enc_left_vel = enc_left_diff.abs() as f32 / dt_s;

        enc_right.reset_count();
        enc_left.reset_count();

        let enc_right_distance_m = enc_right_pos as f32 / ticks_per_meter;
        let enc_right_velocity_mps = enc_right_vel / ticks_per_meter;
        let enc_left_distance_m = enc_left_pos as f32 / ticks_per_meter;
        let enc_left_velocity_mps = enc_left_vel / ticks_per_meter;

        info!(
            "enc_right -> now: {}, diff: {}, pos: {}, vel {} ticks/s, distance traveled: {} m, speed: {} m/s",
            enc_right_raw_now,
            enc_right_diff,
            enc_right_pos,
            enc_right_vel,
            enc_right_distance_m,
            enc_right_velocity_mps
        );

        info!(
            "enc_left -> now: {}, diff: {}, pos: {}, vel {} ticks/s, distance traveled: {} m, speed: {} m/s",
            enc_left_raw_now,
            enc_left_diff,
            enc_left_pos,
            enc_left_vel,
            enc_left_distance_m,
            enc_left_velocity_mps
        );

        let effort_left = pid_left
            .next_control_output(target_ticks - enc_left_pos as f32)
            .output;

        let effort_right = pid_right
            .next_control_output(target_ticks - enc_right_pos as f32)
            .output;

        let duty_left = duty_from_effort(effort_left);
        let duty_right = duty_from_effort(effort_right);

        pwm_fwd!(left_pwm, duty_left);
        pwm_fwd!(right_pwm, duty_right);

        if (enc_left_pos as f32 >= target_ticks) && (enc_right_pos as f32 >= target_ticks) {
            pwm_off!(left_pwm);
            pwm_off!(right_pwm);
            info!("done!");
            break;
        }

        // loop {
        //     tick.next().await;
        //     const ENCODER_MAX: u32 = 0xFFFF;
        //
        //     let enc_right_now = enc_right.count();
        //     let enc_right_diff: i32;
        //     if (enc_right_now as i32 - enc_right_prev as i32).abs() >= 10000 {
        //         info!("too big!");
        //         continue;
        //     } else {
        //         enc_right_diff = enc_right_now as i32 - enc_right_prev as i32;
        //     }
        //
        //     // if enc_right_now >= enc_right_prev {
        //     //     enc_right_diff = enc_right_now as i32 - enc_right_prev as i32;
        //     // } else {
        //     //     enc_right_diff =
        //     //         (enc_right_now as i32) + (ENCODER_MAX as i32 + 1 - enc_right_prev as i32);
        //     // }
        //     enc_right_prev = enc_right_now;
        //     enc_right_pos += enc_right_diff;
        //     let enc_right_vel = enc_right_diff as f32 / dt_s;
        //     let enc_right_dir = 1;
        //
        //     let enc_right_distance_m = enc_right_pos as f32 / ticks_per_meter;
        //     let enc_right_velocity_mps = enc_right_vel / ticks_per_meter;
        //
        //     // info!(
        //     //     "enc_right distance: {} m, velocity: {} m/s",
        //     //     enc_right_distance_m, enc_right_velocity_mps
        //     // );
        //
        //     let enc_left_now = enc_left.count();
        //     let enc_left_diff: i32;
        //
        //     if (enc_left_now as i32 - enc_left_prev as i32).abs() >= 10000 {
        //         info!("too big!");
        //         continue;
        //     } else {
        //         enc_left_diff = enc_left_now as i32 - enc_left_prev as i32;
        //     }
        //
        //     // if enc_left_now >= enc_left_prev {
        //     //     enc_left_diff = enc_left_now as i32 - enc_left_prev as i32;
        //     // } else {
        //     //     enc_left_diff = (enc_left_now as i32) + (ENCODER_MAX as i32 + 1 - enc_left_prev as i32);
        //     // }
        //     enc_left_prev = enc_left_now;
        //     enc_left_pos += enc_left_diff;
        //     let enc_left_vel = enc_left_diff as f32 / dt_s;
        //
        //     let enc_left_distance_m = enc_left_pos as f32 / ticks_per_meter;
        //     let enc_left_velocity_mps = enc_left_vel / ticks_per_meter;
        //     // info!(
        //     //     "enc_left distance: {} m, velocity: {} m/s",
        //     //     enc_left_distance_m, enc_left_velocity_mps
        //     // );
        //
        //     let enc_left_dir = 1;
        //
        //     info!(
        //         "enc_right -> prev: {}, now: {}, diff: {}, pos: {}, vel {} ticks/s, dir: {}",
        //         enc_right_prev,
        //         enc_right_now,
        //         enc_right_diff,
        //         enc_right_pos,
        //         enc_right_vel,
        //         enc_right_dir
        //     );
        //
        //     info!(
        //         "enc_left -> prev: {}, now: {}, diff: {}, pos: {}, vel {} ticks/s, dir: {}",
        //         enc_left_prev, enc_left_now, enc_left_diff, enc_left_pos, enc_left_vel, enc_left_dir
        //     );
        //
        //     let effort_left = pid_left
        //         .next_control_output(target_ticks - enc_left_pos as f32)
        //         .output;
        //
        //     let effort_right = pid_right
        //         .next_control_output(target_ticks - enc_right_pos as f32)
        //         .output;
        //
        //     let duty_left = duty_from_effort(effort_left);
        //
        //     let duty_right = duty_from_effort(effort_right);
        //
        //     if enc_left_dir == 1 {
        //         pwm_fwd!(left_pwm, duty_left);
        //     } else {
        //         pwm_rev!(left_pwm, duty_left);
        //     }
        //
        //     if enc_right_dir == 1 {
        //         pwm_fwd!(right_pwm, duty_right);
        //     } else {
        //         pwm_rev!(right_pwm, duty_right);
        //     }
        //
        //     if (enc_left_pos as f32) >= target_ticks && (enc_right_pos as f32) >= target_ticks {
        //         pwm_off!(left_pwm);
        //         pwm_off!(right_pwm);
        //         info!("done!");
        //         break;
        //     }

        //
        // if phase_remaining_ms > 0 {
        //     phase_remaining_ms -= period_ms;
        // } else {
        //     match phase {
        //         Phase::Forward => {
        //             target_l = 0.0;
        //             target_r = 0.0;
        //             phase = Phase::Coast1;
        //             phase_remaining_ms = COAST_TIME_MS;
        //         }
        //         Phase::Coast1 => {
        //             target_l = REV_CPS;
        //             target_r = REV_CPS;
        //             phase = Phase::Reverse;
        //             phase_remaining_ms = REV_TIME_MS;
        //         }
        //         Phase::Reverse => {
        //             target_l = 0.0;
        //             target_r = 0.0;
        //             phase = Phase::Coast2;
        //             phase_remaining_ms = COAST_TIME_MS;
        //         }
        //         Phase::Coast2 => {
        //             target_l = FWD_CPS;
        //             target_r = FWD_CPS;
        //             phase = Phase::Forward;
        //             phase_remaining_ms = FWD_TIME_MS;
        //         }
        //     }
        // }
    }

    loop {
        pwm_off!(right_pwm);
        pwm_off!(left_pwm);
    }
}
