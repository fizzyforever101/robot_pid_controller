#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{time::khz, Config};
use embassy_time::{Duration, Instant, Ticker};
use panic_probe as _;

use defmt_rtt as _;

mod modules;

const CONTROL_PERIOD_MS: u64 = 10;
const TELEMETRY_PERIOD_MS: u64 = 100;

const TARGET_CPS: f32 = 4000.0;
const MIN_DUTY: u16 = 10_000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());

    // QEI encoders (TIM3: PA6=CH1, PC7=CH2 for left; TIM1: PE9=CH1, PE11=CH2 for right)
    let mut enc_left = modules::encoder::EncoderQei::new(p.TIM3, p.PA6, p.PC7);
    let mut enc_right = modules::encoder::EncoderQei::new(p.TIM1, p.PE9, p.PE11);

    // Serial TX for simple telemetry. May need adjust USART/pin/baud
    let mut uart_cfg = embassy_stm32::usart::Config::default();
    uart_cfg.baudrate = 115_200;
    let mut serial = modules::serial::SerialInterface::new(p.USART1, p.PB7, p.PB6, uart_cfg).unwrap();

    // left motor PWM (TIM4: PD12=CH1 forward, PD13=CH2 reverse)
    let left_ch1_pin = PwmPin::new(p.PD12, embassy_stm32::gpio::OutputType::PushPull);
    let left_ch2_pin = PwmPin::new(p.PD13, embassy_stm32::gpio::OutputType::PushPull);
    let left_pwm = SimplePwm::new(
        p.TIM4,
        Some(left_ch1_pin),
        Some(left_ch2_pin),
        None,
        None,
        khz(1),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let mut left_hbridge = modules::motor_control::HBridgePwm::new(left_pwm);

    // right motor PWM (TIM23: PF0=CH1 forward, PF1=CH2 reverse)
    let right_ch1_pin = PwmPin::new(p.PF0, embassy_stm32::gpio::OutputType::PushPull);
    let right_ch2_pin = PwmPin::new(p.PF1, embassy_stm32::gpio::OutputType::PushPull);
    let right_pwm = SimplePwm::new(
        p.TIM23,
        Some(right_ch1_pin),
        Some(right_ch2_pin),
        None,
        None,
        khz(1),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let mut right_hbridge = modules::motor_control::HBridgePwm::new(right_pwm);

    let mut left_pid =
        modules::motor_control::MotorPid::new(TARGET_CPS, MIN_DUTY, left_hbridge.max_duty());
    let mut right_pid =
        modules::motor_control::MotorPid::new(TARGET_CPS, MIN_DUTY, right_hbridge.max_duty());

    let dt_s = (CONTROL_PERIOD_MS as f32) / 1000.0;
    let mut tick = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));
    let mut next_telemetry = Instant::now();
    let mut line = LineBuffer::new();

    loop {
        tick.next().await;

        // apply any pending setpoint updates from UART
        while let Some(b) = serial.try_read() {
            if let Some(cmd) = line.push(b) {
                match cmd {
                    Command::SetTargetCpsAll(t) => {
                        left_pid.set_setpoint(t as f32);
                        right_pid.set_setpoint(t as f32);
                    }
                    Command::SetTargetCpsLeft(t) => left_pid.set_setpoint(t as f32),
                    Command::SetTargetCpsRight(t) => right_pid.set_setpoint(t as f32),
                }
            }
        }

        // convert delta counts per period into counts per second
        let left_cps = (enc_left.delta_counts() as f32) / dt_s;
        let right_cps = (enc_right.delta_counts() as f32) / dt_s;

        // Compute drive commands
        let left_cmd = left_pid.update(left_cps);
        let right_cmd = right_pid.update(right_cps);

        left_hbridge.apply(left_cmd);
        right_hbridge.apply(right_cmd);

        if Instant::now() >= next_telemetry {
            next_telemetry += Duration::from_millis(TELEMETRY_PERIOD_MS);
            info!("cps L={} R={}", left_cps, right_cps);

            let mut buf = [0u8; 64];
            let n = build_telemetry_line(&mut buf, left_cps as i32, right_cps as i32);
            serial.write(&buf[..n]);
        }
    }
}

fn build_telemetry_line(buf: &mut [u8; 64], left_cps: i32, right_cps: i32) -> usize {
    let mut i = 0usize;

    i = push_bytes(buf, i, b"L:");
    i = push_i32(buf, i, left_cps);
    i = push_bytes(buf, i, b" R:");
    i = push_i32(buf, i, right_cps);
    i = push_bytes(buf, i, b"\r\n");

    i
}

fn push_bytes<const N: usize>(buf: &mut [u8; N], i: usize, s: &[u8]) -> usize {
    let copy_len = s.len().min(N.saturating_sub(i));
    buf[i..i + copy_len].copy_from_slice(&s[..copy_len]);
    i + copy_len
}

fn push_i32<const N: usize>(buf: &mut [u8; N], mut i: usize, mut v: i32) -> usize {
    if i >= N {
        return i;
    }
    if v == 0 {
        buf[i] = b'0';
        return i + 1;
    }

    if v < 0 {
        buf[i] = b'-';
        i += 1;
        v = -v;
        if i >= N {
            return i;
        }
    }

    // write digits to a small stack buffer in reverse
    let mut tmp = [0u8; 11];
    let mut n = 0usize;
    let mut x = v as u32;
    while x != 0 && n < tmp.len() {
        tmp[n] = b'0' + (x % 10) as u8;
        x /= 10;
        n += 1;
    }

    // copy reversed into output
    while n != 0 && i < N {
        n -= 1;
        buf[i] = tmp[n];
        i += 1;
    }

    i
}

enum Command {
    SetTargetCpsAll(i32),
    SetTargetCpsLeft(i32),
    SetTargetCpsRight(i32),
}

struct LineBuffer {
    buf: [u8; 48],
    len: usize,
}

impl LineBuffer {
    fn new() -> Self {
        Self { buf: [0; 48], len: 0 }
    }

    fn push(&mut self, b: u8) -> Option<Command> {
        match b {
            b'\r' => None,
            b'\n' => {
                let cmd = parse_command(&self.buf[..self.len]);
                self.len = 0;
                cmd
            }
            _ => {
                if self.len < self.buf.len() {
                    self.buf[self.len] = b;
                    self.len += 1;
                }
                None
            }
        }
    }
}

// Supported commands (ASCII, newline-terminated):
// - "T <int>"  : set target CPS for both motors
// - "TL <int>" : set target CPS for left motor
// - "TR <int>" : set target CPS for right motor
fn parse_command(line: &[u8]) -> Option<Command> {
    let line = trim_ascii(line);
    if line.is_empty() {
        return None;
    }

    let (head, rest) = split_once_space(line);
    let value = parse_i32(trim_ascii(rest))?;

    match head {
        b"T" => Some(Command::SetTargetCpsAll(value)),
        b"TL" => Some(Command::SetTargetCpsLeft(value)),
        b"TR" => Some(Command::SetTargetCpsRight(value)),
        _ => None,
    }
}

fn trim_ascii(mut s: &[u8]) -> &[u8] {
    while let Some((&b, tail)) = s.split_first() {
        if b == b' ' || b == b'\t' {
            s = tail;
        } else {
            break;
        }
    }
    while let Some((&b, head)) = s.split_last() {
        if b == b' ' || b == b'\t' {
            s = head;
        } else {
            break;
        }
    }
    s
}

fn split_once_space(s: &[u8]) -> (&[u8], &[u8]) {
    for (i, &b) in s.iter().enumerate() {
        if b == b' ' || b == b'\t' {
            return (&s[..i], &s[i + 1..]);
        }
    }
    (s, &[])
}

fn parse_i32(s: &[u8]) -> Option<i32> {
    let s = trim_ascii(s);
    if s.is_empty() {
        return None;
    }

    let mut i = 0usize;
    let mut neg = false;
    if s[0] == b'-' {
        neg = true;
        i = 1;
    } else if s[0] == b'+' {
        i = 1;
    }

    if i >= s.len() {
        return None;
    }

    let mut acc: i32 = 0;
    while i < s.len() {
        let b = s[i];
        if !(b'0'..=b'9').contains(&b) {
            return None;
        }
        acc = acc.saturating_mul(10).saturating_add((b - b'0') as i32);
        i += 1;
    }

    Some(if neg { -acc } else { acc })
}
