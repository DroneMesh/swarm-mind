use core::ops::{Add, Mul, Sub};

/// World-space vector. `y` is up.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Squared length. The brain compares distances squared where it can,
    /// because `sqrt` isn't available in `core`.
    pub fn length_sq(self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Squared distance in the ground plane, ignoring height.
    pub fn dist_xz_sq(self, o: Self) -> f32 {
        let (dx, dz) = (self.x - o.x, self.z - o.z);
        dx * dx + dz * dz
    }

    /// Distance in the ground plane, ignoring height.
    pub fn dist_xz(self, o: Self) -> f32 {
        sqrt(self.dist_xz_sq(o))
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

/// Square root without `std` or an FPU instruction: a bit-level first guess
/// refined by three Newton steps (relative error below 1e-6 for normal
/// inputs). Non-positive and non-finite inputs give 0.
pub fn sqrt(x: f32) -> f32 {
    if x.is_nan() || x <= 0.0 || x.is_infinite() {
        return 0.0;
    }
    // Halving the exponent bits gives a guess within a few percent.
    let mut y = f32::from_bits((x.to_bits() >> 1) + 0x1fbd_1df5);
    for _ in 0..3 {
        y = 0.5 * (y + x / y);
    }
    y
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sqrt_is_accurate() {
        for x in [1e-6f32, 0.25, 1.0, 2.0, 9.0, 1234.5, 6.4e5, 1e12] {
            let got = sqrt(x);
            assert!((got * got - x).abs() <= x * 2e-6, "sqrt({x}) = {got}");
        }
        assert_eq!(sqrt(0.0), 0.0);
        assert_eq!(sqrt(-4.0), 0.0);
        assert_eq!(sqrt(f32::NAN), 0.0);
        assert_eq!(sqrt(f32::INFINITY), 0.0);
    }
}
