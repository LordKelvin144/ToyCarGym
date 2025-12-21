use std::ops::{Add, Sub, Mul, Div, Neg};
use num_traits::{Signed, Float};

use macroquad::prelude as mq;


#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Vec2<T>(pub T, pub T);


impl<T> Add for Vec2<T>
where
    T: Add<Output = T>,
{
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Vec2(self.0 + rhs.0, self.1 + rhs.1)
    }
}

impl<T> Sub for Vec2<T>
where
    T: Sub<Output = T>,
{
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Vec2(self.0 - rhs.0, self.1 - rhs.1)
    }
}


impl<T> Mul<T> for Vec2<T>
where
    T: Mul<Output = T> + Copy,
{
    type Output = Self;

    fn mul(self, rhs: T) -> Self {
        Vec2(self.0 * rhs, self.1 * rhs)
    }
}

impl<T> Neg for Vec2<T>
where
    T: Neg<Output = T> + Copy
{
    type Output = Self;

    fn neg(self) -> Self {
        Vec2(-self.0, -self.1)
    }
}

impl<T> Div<T> for Vec2<T>
where
    T: Div<Output = T> + Copy,
{
    type Output = Self;

    fn div(self, rhs: T) -> Self {
        Vec2(self.0 / rhs, self.1 / rhs)
    }
}

impl<T> Vec2<T>
where
    T: Signed,
{
    pub fn rotate90(self) -> Self {
        Vec2(-self.1, self.0)
    }
}

impl<T> Vec2<T>
where
    T: Float
{
    pub fn norm(self) -> T {
        (self.0 * self.0 + self.1 * self.1).sqrt()
    }

    pub fn normalized(self) -> Self {
        self / self.norm()
    }
}

impl<T> Vec2<T>
where T: Float + Signed,
{
    pub fn rotate(self, angle: T) -> Self {
        let sin = angle.sin();
        let cos = angle.cos();
        Vec2(self.0 * cos - self.1 * sin, self.0 * sin + self.1 * cos)
    }
}


impl<T> std::convert::From<Vec2<T>> for mq::Vec2 
where T: Into<f32>,
{
    fn from(myvec: Vec2::<T>) -> mq::Vec2 {
        mq::Vec2 { x: myvec.0.into(), y: myvec.1.into() }
    }

}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add() {
        let v1 = Vec2(1.0, 2.0);
        let v2 = Vec2(3.0, 5.0);
        let sum = v1 + v2;
        assert_eq!(sum, Vec2(4.0, 7.0));

        let v1 = Vec2(1, 2);
        let v2 = Vec2(3, 5);
        let sum = v1 + v2;
        assert_eq!(sum, Vec2(4, 7));
    }
    #[test]
    fn test_mul() {
        let v1 = Vec2(1.0, 2.0);
        let rprod = v1*2.0;
        assert_eq!(rprod, Vec2(2.0, 4.0));

        let v1 = Vec2(1, 2);
        let rprod = v1*2;
        assert_eq!(rprod, Vec2(2, 4));
    }

    #[test]
    fn test_rotate90() {
        let v1 = Vec2(1.0, 0.0);
        assert_eq!(v1.rotate90(), Vec2(0.0, 1.0));

        let v1 = Vec2(0.0, 1.0);
        assert_eq!(v1.rotate90(), Vec2(-1.0, 0.0))
    }

    #[test]
    fn test_rotate_general() {
        let v1 = Vec2(1.0, 0.0);
        assert_eq!(v1.rotate(0.1).rotate(-0.1), v1);

        // Test a 30 degree rotation
        let thirty = 30.0.to_radians();
        assert!((v1.rotate(thirty).1 - 0.5).abs() < 0.001);  // y should be 0.5
        assert!((v1.rotate(2.0*thirty).0 - 0.5).abs() < 0.001);  // x should be 0.5
        assert!((v1.rotate(3.0*thirty).0).abs() < 0.001);  // x should be 0
        assert!((v1.rotate(4.0*thirty).0 + 0.5).abs() < 0.001);  // x should be -0.5
    }

    #[test]
    fn test_norm() {
        let v1 = Vec2(3.0, 4.0);
        assert_eq!(v1.norm(), 5.0);
    }
}


