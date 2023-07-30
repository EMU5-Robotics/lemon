use crate::InitState;
use std::time::Instant;

struct Pid {
    proportional_gain: f64,
    integral_gain: f64,
    derivitive_gain: f64,

    target: f64,

    integral_accum: f64,

    prev_err: f64,
    time: Instant,
}

impl Pid {
    const INTEGRAL_MAX: f64 = f64::INFINITY;
    const INTEGAAL_MIN: f64 = f64::NEG_INFINITY;
    
    #[inline]
    pub fn kp(&self) -> f64 {
        self.proportional_gain
    }
    #[inline]
    pub fn ki(&self) -> f64 {
        self.integral_gain
    }
    #[inline]
    pub fn kd(&self) -> f64 {
        self.derivitive_gain
    }

    ///creates a new PID from a proportional gain `kp`, a integral gain `ki`, a derivitive gain `kd` and a setpoint and sample time.
    pub fn new(kp: f64, ki: f64, kd: f64, target: f64) -> Self {
        Self {
            proportional_gain: kp,
            integral_gain: ki,
            derivitive_gain: kd,
            target,
            integral_accum: 0.0,
            prev_error: 0.0,
            time = Instant::now(),
        }
    }

    pub fn compute(&mut self, pv: f64) -> f64 {
        let current = Instant::now();
        let dt = current.duration_since(self.time).as_secs_f64();
        let err = self.target - pv;

        let prop_term = self.proportional_gain * err;

        let int_term = (self.integral_accum + self.integral_gain * err * dt).clamp(Self::INTEGRAL_MIN, Self::INTEGRAL_MAX);

        self.integral_accum = int_term;

        let deriv_term = self.derivitive_gain * (err - self.prev_err) / dt;

        let out = prop_term + int_term + deriv_term;
        self.prev_err = err;
        self.time = current;
        out
    }


    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) -> &mut Self {
        self.proportional_gain = kp;
        self.integral_gain = ki;
        self.derivitive_gain = kd;
        self
    }
    
    pub fn set_target(&mut self, target: f64) -> &mut Self {
        self.target = target;
        self.integral_accum = 0.0;
        self.prev_err = 0.0;
        self
    }

}
