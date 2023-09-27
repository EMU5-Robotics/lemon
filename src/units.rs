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

unit!(meter, length, Length);
unit!(second, time, Time);
unit!(millisecond, time, Time);
unit!(radian, angle, Angle);
unit!(degree, angle, Angle);

pub use degree::degree;
pub use meter::meter;
pub use millisecond::millisecond;
pub use radian::radian;
pub use second::second;
