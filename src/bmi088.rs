use std::time::Instant;

use rppal::i2c::I2c;

pub const ROBOT_A_IMU_BIAS: f64 = 0.0004146448; //0.0002138361;

const ANGULAR_CODE: u8 = 0x01;
const ANGULAR_SCALE: f64 = match ANGULAR_CODE {
    0x00 => 2000.0,
    0x01 => 1000.0,
    0x02 => 500.0,
    0x03 => 250.0,
    0x04 => 125.0,
    _ => panic!("Invalid ANGULAR_CODE"),
} * (std::f64::consts::PI / 180.0)
    / 2u16.pow(15) as f64;

pub struct Bmi088 {
    pub i2c: I2c,
    last_read: Instant,
    last_angular_vel_z: f64,
    heading: f64,
    bias: f64,
}

impl Bmi088 {
    pub fn new(bias: f64) -> Self {
        let mut i2c = I2c::new().unwrap();
        log::info!("IMU clock speed: {:?}", i2c.clock_speed());

        // gyroscope address
        i2c.set_slave_address(0x68u16).unwrap();
        i2c.write(&[0x0F, ANGULAR_CODE]).unwrap();
        // set filtering (test if this performs the best)
        i2c.write(&[0x10, 0x02]).unwrap();
        // read vel_z
        let mut buf = [0u8; 2];
        i2c.write_read(&[0x6u8], &mut buf).unwrap();
        let last_angular_vel_z = i16::from_le_bytes(buf) as f64 * ANGULAR_SCALE;

        Self {
            i2c,
            last_read: Instant::now(),
            last_angular_vel_z,
            heading: 0.0,
            bias,
        }
    }
    fn read_vel_z(&mut self) -> f64 {
        let mut buf = [0u8; 2];
        match self.i2c.write_read(&[0x6u8], &mut buf) {
            Ok(()) => i16::from_le_bytes(buf) as f64 * ANGULAR_SCALE + self.bias,
            Err(e) => {
                log::warn!("imu read failed: {e}");
                self.last_angular_vel_z
            }
        }
    }
    pub fn heading(&self) -> f64 {
        self.heading
    }
    pub fn angular_velocity(&self) -> f64 {
        self.last_angular_vel_z
    }
    pub fn calc_heading(&mut self) -> f64 {
        let new_angular_vel_z = self.read_vel_z();
        let now = Instant::now();
        let dt = now.duration_since(self.last_read).as_secs_f64();
        self.heading += new_angular_vel_z * dt;
        self.last_angular_vel_z = new_angular_vel_z;
        self.last_read = now;
        self.heading
    }
    pub fn reset(&mut self) {
        self.last_read = Instant::now();
        self.heading = 0.0;
    }
}
