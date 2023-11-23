pub use uom::si::f64::*;

macro_rules! unit {
	($unit:ident, $class_lower:ident, $class_upper:ident) => {
		pub mod $unit {
			#[macro_export]
			macro_rules! $unit {
				($v:expr) => {
					$class_upper::new::<uom::si::$class_lower::$unit>($v)
				};
			}
			pub use $unit;
		}
	};
}

unit!(degree, angle, Angle);
unit!(kilogram_meter_per_second, momentum, Momentum);
unit!(meter, length, Length);
unit!(meter_per_second, velocity, Velocity);
unit!(millisecond, time, Time);
unit!(newton, force, Force);
unit!(radian, angle, Angle);
unit!(radian_per_second, angular_velocity, AngularVelocity);
unit!(revolution_per_minute, angular_velocity, AngularVelocity);
unit!(second, time, Time);
unit!(watt, power, Power);
unit!(watt_per_meter, linear_power_density, LinearPowerDensity);

pub use degree::degree;
pub use meter::meter;
pub use meter_per_second::meter_per_second;
pub use millisecond::millisecond;
pub use radian::radian;
pub use revolution_per_minute::revolution_per_minute;
pub use second::second;
pub use watt::watt;
