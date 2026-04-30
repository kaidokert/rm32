//! Integer PID controller.
//!
//! Matches the C `fastPID` struct behavior exactly.

#[derive(Clone, Default)]
pub(crate) struct Pid {
    error: i32,
    kp: u32,
    ki: u32,
    kd: u32,
    integral: i32,
    derivative: i32,
    last_error: i32,
    pid_output: i32,
    integral_limit: i32,
    output_limit: i32,
}

impl Pid {
    pub(crate) fn new(kp: u32, ki: u32, kd: u32, integral_limit: i32, output_limit: i32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            output_limit,
            ..Default::default()
        }
    }

    /// Compute one PID iteration. Returns clamped output.
    /// Mirrors `doPidCalculations` from main.c exactly.
    pub(crate) fn calculate(&mut self, actual: i32, target: i32) -> i32 {
        self.error = actual - target;
        self.integral += self.error * self.ki as i32;

        if self.integral > self.integral_limit {
            self.integral = self.integral_limit;
        }
        if self.integral < -self.integral_limit {
            self.integral = -self.integral_limit;
        }

        self.derivative = self.kd as i32 * (self.error - self.last_error);
        self.last_error = self.error;

        self.pid_output = self.error * self.kp as i32 + self.integral + self.derivative;

        if self.pid_output > self.output_limit {
            self.pid_output = self.output_limit;
        }
        if self.pid_output < -self.output_limit {
            self.pid_output = -self.output_limit;
        }

        self.pid_output
    }

    /// Update PID gains and reset accumulated state.
    pub(crate) fn set_gains(&mut self, kp: u32, ki: u32, kd: u32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
        self.reset();
    }

    pub(crate) fn reset(&mut self) {
        self.error = 0;
        self.integral = 0;
        self.derivative = 0;
        self.last_error = 0;
        self.pid_output = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proportional_response() {
        let mut pid = Pid::new(100, 0, 0, 10000, 50000);
        let out = pid.calculate(500, 400);
        assert_eq!(out, 10000);
        assert_eq!(pid.error, 100);
    }

    #[test]
    fn output_clamping() {
        let mut pid = Pid::new(1000, 0, 0, 10000, 5000);
        assert_eq!(pid.calculate(500, 400), 5000);
        assert_eq!(pid.calculate(400, 500), -5000);
    }

    #[test]
    fn integral_accumulation() {
        let mut pid = Pid::new(0, 10, 0, 10000, 50000);
        pid.calculate(150, 100);
        pid.calculate(150, 100);
        let out = pid.calculate(150, 100);
        assert_eq!(pid.integral, 1500);
        assert_eq!(out, 1500);
    }

    #[test]
    fn integral_anti_windup() {
        let mut pid = Pid::new(0, 100, 0, 1000, 50000);
        pid.calculate(200, 100); // integral = 10000 -> clamped to 1000
        assert_eq!(pid.integral, 1000);

        let mut pid = Pid::new(0, 100, 0, 1000, 50000);
        pid.calculate(100, 200); // integral = -10000 -> clamped to -1000
        assert_eq!(pid.integral, -1000);
    }

    #[test]
    fn reset_clears_state() {
        let mut pid = Pid::new(100, 10, 10, 10000, 50000);
        pid.calculate(500, 400);
        pid.calculate(500, 400);
        pid.reset();
        assert_eq!(pid.integral, 0);
        assert_eq!(pid.last_error, 0);
        assert_eq!(pid.pid_output, 0);
    }

    #[test]
    fn zero_gains_zero_output() {
        let mut pid = Pid::new(0, 0, 0, 10000, 50000);
        assert_eq!(pid.calculate(500, 400), 0);
    }

    #[test]
    fn negative_error() {
        let mut pid = Pid::new(100, 0, 0, 10000, 50000);
        let out = pid.calculate(200, 500);
        assert_eq!(out, -30000); // (200-500)*100
    }

    #[test]
    fn derivative_response() {
        let mut pid = Pid::new(0, 0, 10, 10000, 50000);
        pid.calculate(150, 100); // derivative = 10*(50-0) = 500
        assert_eq!(pid.derivative, 500);
        let out = pid.calculate(170, 100); // derivative = 10*(70-50) = 200
        assert_eq!(pid.derivative, 200);
        assert_eq!(out, 200);
    }
}
