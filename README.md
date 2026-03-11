# robot_pid_controller

Embassy-based STM32 motor speed controller using:

- Quadrature encoders (timer QEI)
- H-bridge PWM (two PWM channels per motor: forward/reverse)
- PID speed loop (counts-per-second setpoint)
- UART telemetry + simple command interface (newline-terminated ASCII)

## Hardware Assumptions (Current Pin Map)

These are the pins/timers currently wired in [`src/main.rs`](src/main.rs):

### Encoders (QEI)

- Left encoder: `TIM3` with `PA6` (CH1) and `PC7` (CH2)
- Right encoder: `TIM1` with `PE9` (CH1) and `PE11` (CH2)

### Motors (PWM H-bridge)

Each motor uses 2 PWM channels:

- CH1 = forward, CH2 = reverse

Configured as:

- Left motor: `TIM4` with `PD12` (CH1 forward) and `PD13` (CH2 reverse)
- Right motor: `TIM23` with `PF0` (CH1 forward) and `PF1` (CH2 reverse)

### Serial (UART)

- USART1: `PB7` = RX, `PB6` = TX, `115200` baud

## Code Structure

- [`src/main.rs`](src/main.rs): hardware init + control loop + UART command parsing + telemetry formatting.
- [`src/modules/encoder.rs`](src/modules/encoder.rs): `EncoderQei` wrapper around Embassy `Qei`, plus `delta_counts()` sampling.
- [`src/modules/motor_control.rs`](src/modules/motor_control.rs):
  - `HBridgePwm`: applies `MotorCommand` to `SimplePwm` CH1/CH2 (forward/reverse).
  - `MotorPid`: PID speed controller that outputs `MotorCommand` from measured counts/s.
- [`src/modules/serial.rs`](src/modules/serial.rs): `SerialInterface` using blocking UART writes + nonblocking byte reads.

## How It Works (Control Loop)

The main loop runs at a fixed period (`CONTROL_PERIOD_MS`, currently 10ms).

Per tick:

1. Read any available UART bytes and parse newline-terminated commands.
2. Sample encoder deltas via `EncoderQei::delta_counts()` and convert to counts per second.
3. Run PID for each motor to generate a signed output in [-1, 1].
4. Map the PID output to:
   - drive direction (`Forward`/`Reverse`/`Coast`)
   - duty cycle between `MIN_DUTY` and the timer’s `max_duty`.
5. Apply PWM to each H-bridge.

Telemetry is emitted every `TELEMETRY_PERIOD_MS` (currently 100ms) over:

- `defmt` logging (RTT)
- UART as an ASCII line: `L:<left_cps> R:<right_cps>\r\n`

## UART Command Interface

Commands are ASCII, newline-terminated (`\n`). `\r` is ignored.

- `T <int>`: set target CPS for both motors
- `TL <int>`: set target CPS for left motor
- `TR <int>`: set target CPS for right motor

Examples:

```text
T 4000
TL 2500
TR -3000
```

Values are interpreted as counts-per-second setpoints. Negative values command reverse.

## Building and Flashing

This project targets `thumbv7em-none-eabihf` and uses a runner from [`.cargo/config.toml`](.cargo/config.toml):

```toml
[target.thumbv7em-none-eabihf]
runner = 'probe-rs run --chip STM32H723ZGTx'
```

Common commands:

```bash
cargo build
cargo run
```

`DEFMT_LOG` is set in `.cargo/config.toml` (currently `trace`).

## Testing Components (On Hardware)

These are practical smoke tests you can run without changing code.

### 1) Serial TX/RX

Goal: confirm UART wiring, baud, and command parsing.

1. Connect a USB-UART adapter to USART1:
   - Adapter TX -> `PB7` (MCU RX)
   - Adapter RX -> `PB6` (MCU TX)
   - Common GND
2. Open a serial terminal at `115200 8N1`.
3. Reset the board and observe periodic telemetry lines:
   - `L:<...> R:<...>`
4. Send a command, e.g.:
   - `T 0\n` to coast both motors
   - `T 2000\n` to command forward

If telemetry prints but commands do nothing:

- Verify you are sending a newline (`\n`).
- Verify RX/TX are not swapped.

### 2) Encoders (QEI)

Goal: confirm encoder counts move and direction is correct.

1. Run firmware with motors disabled or setpoint `0` (`T 0\n`).
2. Spin the left wheel/encoder by hand.
3. Observe `L:` change; repeat for the right wheel/encoder and `R:`.

Notes:

- `EncoderQei::delta_counts()` uses wrapping subtraction and assumes you do not exceed ~32k counts per sample.
- If you see wildly incorrect readings, reduce speed or reduce `CONTROL_PERIOD_MS`.
- If your encoder outputs are open-collector (common on NPN optical encoders), A/B require pull-ups. This firmware enables pull-ups on the QEI pins via a local patch to `embassy-stm32` and uses `Pull::Up` when configuring QEI pins.

Expected scaling (Taiss 600 P/R encoder):

- With x4 quadrature decoding, counts/rev = `600 * 4 = 2400`.
- Telemetry reports counts/s (CPS). With 2400 counts/rev: `RPM = CPS / 40`.

### 3) PWM Outputs

Goal: confirm the MCU is generating PWM on the expected pins.

1. With the motor driver enabled but wheels off the ground, send:
   - `TL 1000\n` and `TR 1000\n`
2. Use a scope/logic analyzer on:
   - Left: `PD12`/`PD13`
   - Right: `PF0`/`PF1`
3. Expect:
   - Forward: CH1 toggling with duty, CH2 low (0% duty)
   - Reverse: CH2 toggling with duty, CH1 low

### 4) PID Speed Loop

Goal: confirm closed-loop behavior and basic stability.

1. Start with a small setpoint:
   - `T 500\n`
2. Increase gradually:
   - `T 1000\n`, `T 2000\n`, ...
3. Watch telemetry counts/s and check it tracks the setpoint.

If the loop oscillates or saturates:

- Tune PID gains in [`src/modules/motor_control.rs`](src/modules/motor_control.rs) (`MotorPid::new`).
- Adjust `MIN_DUTY` (static friction compensation) in [`src/main.rs`](src/main.rs).
- Consider increasing PWM frequency if your motor driver prefers it (currently `khz(1)` to match prior code).

## Safety Notes

- Test with wheels off the ground first.
- Be ready to stop the motors quickly (`T 0\n`).
- If direction is inverted, swap encoder channels or swap the motor outputs (or invert the sign of the setpoint).
