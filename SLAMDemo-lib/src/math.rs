use std::ops;
use num_traits;
use gdnative::prelude::*;
use rand;
use rand::distributions::{Normal, Distribution};


pub fn wrap<F>(val: F, mut from: F, mut to: F) -> F 
where F: num_traits::Float
{
    if from > to { std::mem::swap(&mut from, &mut to); }
    let cycle = to - from;
    if cycle == F::zero() { return to; }
    val - cycle * ((val - from) / cycle).floor()
}


// pub struct WrappedAngle<F: num_traits::Float, const WRAP: f32>(F);


#[derive(Debug, Clone, Copy)]
pub struct Matrix2 {
	// basis vector representation
	pub a: Vector2, 
	pub b: Vector2,
}

impl Into<Transform2D> for Matrix2 {
	fn into(self) -> Transform2D {
		Transform2D::from_basis_origin(self.a, self.b, Vector2::ZERO)
	}
}
impl From<Transform2D> for Matrix2 {
	fn from(xform: Transform2D) -> Self {
		Self::from_basis(xform.a, xform.b)
	}
}

impl Matrix2 {
	pub const IDENTITY: Self = Self::from_basis(Vector2::RIGHT, Vector2::DOWN);

	pub const fn from_basis(a: Vector2, b: Vector2) -> Self {
		Self { a, b }
	}

	pub fn from_rotation_scale(rotation: f32, scale: Vector2) -> Self {
		Self::IDENTITY
			.rotated(rotation)
			.scaled(scale)
	}

	#[inline]
	pub fn transposed(&self) -> Self {
		Self::from_basis(
			Vector2::new(self.a.x, self.b.y),
			Vector2::new(self.b.x, self.a.y),
		)
	}

	#[inline]
	pub fn rotated(&self, rotation: f32) -> Self {
		let scale = self.scale();
		let cos_r = f32::cos(rotation);
		let sin_r = f32::sin(rotation);
		let a = Vector2::new(cos_r, sin_r) * scale.x;
		let b = Vector2::new(-sin_r, cos_r) * scale.y;
		Self::from_basis(a, b)
	}

	#[inline]
	pub fn scaled(&self, scale: Vector2) -> Self {
		let mut a = self.a;
		let mut b = self.b;
		a.x *= scale.x;
		a.y *= scale.y;
		b.x *= scale.x;
		b.y *= scale.y;
		Self::from_basis(a, b)
	}

	#[inline]
	pub fn rotation(&self) -> f32 {
		f32::atan2(self.a.y, self.a.x)
	}

	#[inline]
	pub fn scale(&self) -> Vector2 {
		let det_sign = self.determinant().signum();
		Vector2::new(self.a.length(), det_sign*self.b.length())
	}

	pub fn determinant(&self) -> f32 {
		self.a.x*self.b.y - self.a.y*self.b.x
	}
}

impl ops::Mul for &Matrix2 {
	type Output = Matrix2;
	fn mul(self, rhs: &Matrix2) -> Matrix2 {
		let lhs = self.transposed();
		Matrix2::from_basis(
			Vector2::new(lhs.a.dot(rhs.a), lhs.a.dot(rhs.b)),
			Vector2::new(lhs.b.dot(rhs.a), lhs.b.dot(rhs.b)),
		)
	}
}


#[derive(Clone)]
pub struct Gaussian {
	mean: f32,
	std_dev: f32,
}

impl Gaussian {
	pub fn new(mean: f32, std_dev: f32) -> Self {
		Self { mean, std_dev }
	}

	#[inline]
	pub fn mean(&self) -> f32 { self.mean }
	#[inline]
	pub fn variance(&self) -> f32 { self.std_dev.powi(2) }
	#[inline]
	pub fn std_dev(&self) -> f32 { self.std_dev }

	#[inline]
	pub fn sample(&self) -> f32 {
		let normal = Normal::new(self.mean as f64, self.std_dev as f64);
		normal.sample(&mut rand::thread_rng()) as f32
	}
}

impl ops::Mul for &Gaussian {
	type Output = Gaussian;
	fn mul(self, rhs: &Gaussian) -> Gaussian {
		let m1 = self.mean;
		let v1 = self.variance();
		let m2 = rhs.mean;
		let v2 = rhs.variance();

		let mean = (m1*v2 + m2*v1)/(v1 + v2);
		let var = v1*v2/(v1 + v2);
		Gaussian::new(mean, f32::sqrt(var))
	}
}


struct Gaussian2D {
	mean: Vector2,
	covar: Matrix2,
}

