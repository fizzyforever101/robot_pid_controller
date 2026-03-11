use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::GeneralInstance4Channel;
use pid::Pid;

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum Drive {
    Coast,
    Forward,
    Reverse,
}

#[derive(Copy, Clone)]
pub struct MotorCommand {
    pub drive: Drive,
    pub duty: u16,
}

impl MotorCommand {
    pub fn coast() -> Self {
        Self {
            drive: Drive::Coast,
            duty: 0,
        }
    }
}

/// H-bridge helper for a `SimplePwm` using CH1 as "forward" and CH2 as "reverse".
pub struct HBridgePwm<'d, T: GeneralInstance4Channel> {
    pwm: SimplePwm<'d, T>,
    max: u16,
}

impl<'d, T: GeneralInstance4Channel> HBridgePwm<'d, T> {
    pub fn new(mut pwm: SimplePwm<'d, T>) -> Self {
        // Enable both channels once; we’ll still set duty to 0 for coasting.
        {
            let mut ch = pwm.ch1();
            ch.enable();
        }
        {
            let mut ch = pwm.ch2();
            ch.enable();
        }
        let max = pwm.ch1().max_duty_cycle();
        Self { pwm, max }
    }

    pub fn max_duty(&self) -> u16 {
        self.max
    }

    pub fn apply(&mut self, cmd: MotorCommand) {
        match cmd.drive {
            Drive::Coast => {
                let mut ch1 = self.pwm.ch1();
                ch1.set_duty_cycle_fully_off();
                let mut ch2 = self.pwm.ch2();
                ch2.set_duty_cycle_fully_off();
            }
            Drive::Forward => {
                let mut ch1 = self.pwm.ch1();
                ch1.set_duty_cycle(cmd.duty);
                let mut ch2 = self.pwm.ch2();
                ch2.set_duty_cycle_fully_off();
            }
            Drive::Reverse => {
                let mut ch1 = self.pwm.ch1();
                ch1.set_duty_cycle_fully_off();
                let mut ch2 = self.pwm.ch2();
                ch2.set_duty_cycle(cmd.duty);
            }
        }
    }
}

/// PID-based speed controller producing an H-bridge command.
///
/// - `setpoint_cps` and `measured_cps` are in counts per second.
/// - Output is normalized to [-1, 1] and then mapped to a duty range.
pub struct MotorPid {
    pid: Pid<f32>,
    min_duty: u16,
    max_duty: u16,
    deadband: f32,
}

impl MotorPid {
    pub fn new(setpoint_cps: f32, min_duty: u16, max_duty: u16) -> Self {
        let mut pid = Pid::new(setpoint_cps, 1.0);
        // Conservative defaults. Tune these for your mechanical system.
        pid.p(0.0004, 1.0).i(0.00002, 1.0).d(0.0, 1.0);

        Self {
            pid,
            min_duty,
            max_duty,
            deadband: 0.02,
        }
    }

    pub fn set_setpoint(&mut self, setpoint_cps: f32) {
        self.pid.setpoint(setpoint_cps);
    }

    pub fn update(&mut self, measured_cps: f32) -> MotorCommand {
        let out = self.pid.next_control_output(measured_cps).output;

        if out.abs() < self.deadband {
            return MotorCommand::coast();
        }

        let duty_span = self.max_duty.saturating_sub(self.min_duty) as f32;
        let duty = (self.min_duty as f32 + out.abs().min(1.0) * duty_span) as u16;

        let drive = if out >= 0.0 { Drive::Forward } else { Drive::Reverse };
        MotorCommand { drive, duty }
    }
}

