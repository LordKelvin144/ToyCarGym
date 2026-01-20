use std::ops::{Add, Sub, Mul, Div, Neg};


#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2(pub f32, pub f32);


impl Add for Vec2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Vec2(self.0 + rhs.0, self.1 + rhs.1)
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Vec2(self.0 - rhs.0, self.1 - rhs.1)
    }
}


impl Mul<f32> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self {
        Vec2(self.0 * rhs, self.1 * rhs)
    }
}

impl Neg for Vec2 {
    type Output = Self;

    fn neg(self) -> Self {
        Vec2(-self.0, -self.1)
    }
}

impl Div<f32> for Vec2 {
    type Output = Self;

    fn div(self, rhs: f32) -> Self {
        Vec2(self.0 / rhs, self.1 / rhs)
    }
}

// Custom methods
impl Vec2 {
    pub fn dot(self, rhs: Self) -> f32 {
        self.0 * rhs.0 + self.1 * rhs.1
    }

    pub fn rotate90(self) -> Self {
        Vec2(-self.1, self.0)
    }

    pub fn norm(self) -> f32 {
        (self.0 * self.0 + self.1 * self.1).sqrt()
    }

    pub fn normalized(self) -> Self {
        self / self.norm()
    }

    pub fn rotate(self, angle: f32) -> Self {
        let sin = angle.sin();
        let cos = angle.cos();
        Vec2(self.0 * cos - self.1 * sin, self.0 * sin + self.1 * cos)
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
    }

    #[test]
    fn test_sub() {
        let v1 = Vec2(4.0, 6.0);
        let v2 = Vec2(1.0, 3.0);
        assert_eq!(v1-v2, Vec2(3.0, 3.0));
    }

    #[test]
    fn test_mul() {
        let v1 = Vec2(1.0, 2.0);
        let rprod = v1*2.0;
        assert_eq!(rprod, Vec2(2.0, 4.0));
    }

    #[test]
    fn test_div() {
        let v1 = Vec2(2.0, 4.0);
        assert_eq!(v1 / 2.0, Vec2(1.0, 2.0))
    }

    #[test]
    fn test_dot() {
        let v1 = Vec2(2.0, 4.0);
        let v2 = Vec2(-1.0, 1.0);
        assert_eq!(v1.dot(v2), 2.0);
        assert_eq!(v2.dot(v1), 2.0);
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
        let thirty = 30.0_f32.to_radians();
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


