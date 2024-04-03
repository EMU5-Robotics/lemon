#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2([f64; 2]);

impl Vec2 {
    pub fn x(self) -> f64 {
        self.0[0]
    }
    pub fn y(self) -> f64 {
        self.0[1]
    }
    pub fn dot(self, rhs: Self) -> f64 {
        self.x() * rhs.x() + self.y() * rhs.y()
    }
    pub fn mag_sq(self) -> f64 {
        self.dot(self)
    }
    pub fn mag(self) -> f64 {
        self.mag_sq().sqrt()
    }
    pub fn normalised(self) -> Self {
        self / self.mag()
    }
}

use std::ops::*;

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Vec2([self.x() + rhs.x(), self.y() + rhs.y()])
    }
}

impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Vec2([self.x() + rhs.x(), self.y() + rhs.y()])
    }
}

impl Neg for Vec2 {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Vec2([-self.x(), -self.y()])
    }
}
impl<T: Into<f64>> Mul<T> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: T) -> Self::Output {
        let rhs = rhs.into();
        Vec2([self.x() * rhs, self.y() * rhs])
    }
}
impl<T: Into<f64>> Div<T> for Vec2 {
    type Output = Self;
    fn div(self, rhs: T) -> Self::Output {
        let rhs = rhs.into();
        Vec2([self.x() / rhs, self.y() / rhs])
    }
}
impl<T: Into<[f64; 2]>> From<T> for Vec2 {
    fn from(v: T) -> Self {
        Self(v.into())
    }
}
