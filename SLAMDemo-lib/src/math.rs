use std::ops;
use std::f32::consts::PI;
use num_traits::Float;
use gdnative::prelude::*;
use rand;
use rand_distr::{Normal, StandardNormal, Distribution};


pub fn wrap<F>(val: F, mut from: F, mut to: F) -> F 
where F: Float
{
    if from > to { std::mem::swap(&mut from, &mut to); }
    let cycle = to - from;
    if cycle == F::zero() { return to; }
    val - cycle * ((val - from) / cycle).floor()
}


// pub struct WrappedAngle<F: num_traits::Float, const WRAP: f32>(F);


#[derive(Debug, Clone)]
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
		Self::from_xform(xform)
	}
}

impl Matrix2 {
	pub const ZERO: Self = Self::from_basis(Vector2::ZERO, Vector2::ZERO);

	pub const IDENTITY: Self = Self::from_basis(
		Vector2::new(1., 0.),
		Vector2::new(0., 1.)
	);

	pub const fn from_basis(a: Vector2, b: Vector2) -> Self {
		Self { a, b }
	}

	pub const fn from_xform(xform: Transform2D) -> Self {
		Self::from_basis(xform.a, xform.b)
	}


	pub fn from_rotation(rotation: f32) -> Self {
		let cos_r = f32::cos(rotation);
		let sin_r = f32::sin(rotation);
		Self::from_basis(
			Vector2::new(cos_r, sin_r),
			Vector2::new(-sin_r, cos_r),
		)
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
		&Self::from_rotation(rotation) * self
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

	#[inline]
	pub fn xform(&self, u: Vector2) -> Vector2 {
		Vector2::new(
			self.a.x*u.x + self.b.x*u.y,
			self.a.y*u.x + self.b.y*u.y,
		)
	}

	#[inline]
	pub fn xform_inv(&self, u: Vector2) -> Vector2 {
		Vector2::new(
			self.a.dot(u),
			self.b.dot(u),
		)
	}

	#[inline]
	pub fn dot(&self, rhs: &Self) -> Self {
		let lhs = self.transposed();
		Matrix2::from_basis(
			Vector2::new(lhs.a.dot(rhs.a), lhs.a.dot(rhs.b)),
			Vector2::new(lhs.b.dot(rhs.a), lhs.b.dot(rhs.b)),
		)
	}

	pub fn determinant(&self) -> f32 {
		self.a.x*self.b.y - self.a.y*self.b.x
	}

	#[inline]
	pub fn is_symmetric(&self) -> bool {
		(self.a.y - self.b.x).abs() <= f32::EPSILON
	}

	pub fn cholesky(&self) -> Matrix2 {
		debug_assert!(self.is_symmetric());
		let a = self.a.x.sqrt();
		let b = self.a.y/a;
		let c = (self.b.y - b*b).sqrt();
		Self::from_basis(
			Vector2::new(a, b),
			Vector2::new(0., c),
		)
	}
}

impl ops::Mul<&Matrix2> for &Matrix2 {
	type Output = Matrix2;
	fn mul(self, rhs: &Matrix2) -> Matrix2 {
		self.dot(rhs)
	}
}

impl ops::Mul<f32> for &Matrix2 {
	type Output = Matrix2;
	fn mul(self, rhs: f32) -> Matrix2 {
		Matrix2::from_basis(
			self.a * rhs,
			self.b * rhs,
		)
	}
}

impl ops::Add<&Matrix2> for &Matrix2 {
	type Output = Matrix2;
	fn add(self, rhs: &Matrix2) -> Matrix2 {
		Matrix2::from_basis(
			self.a + rhs.a,
			self.b + rhs.b,
		)
	}
}

impl ops::Sub<&Matrix2> for &Matrix2 {
	type Output = Matrix2;
	fn sub(self, rhs: &Matrix2) -> Matrix2 {
		Matrix2::from_basis(
			self.a - rhs.a,
			self.b - rhs.b,
		)
	}
}


#[derive(Clone)]
pub struct Gaussian {
	normal: Normal<f32>
}

impl Gaussian {
	pub fn new(mean: f32, std_dev: f32) -> Self {
		Self {
			normal: Normal::new(mean, std_dev).unwrap()
		}
	}

	#[inline]
	pub fn mean(&self) -> f32 { self.normal.mean() }
	#[inline]
	pub fn std_dev(&self) -> f32 { self.normal.std_dev() }
	#[inline]
	pub fn variance(&self) -> f32 { self.std_dev().powi(2) }

	#[inline]
	pub fn sample(&self) -> f32 {
		self.normal.sample(&mut rand::thread_rng())
	}

	pub fn probability_density(&self, x: f32) -> f32 {
		let sigma = self.std_dev();
		let n = sigma*(2.0*PI).sqrt();
		let arg = -0.5*((x - self.mean())/sigma).powi(2);
		f32::exp(arg)/n
	}
}

impl ops::Mul<&Gaussian> for &Gaussian {
	type Output = Gaussian;
	fn mul(self, rhs: &Gaussian) -> Gaussian {
		let m1 = self.mean();
		let v1 = self.variance();
		let m2 = rhs.mean();
		let v2 = rhs.variance();

		let mean = (m1*v2 + m2*v1)/(v1 + v2);
		let var = v1*v2/(v1 + v2);
		Gaussian::new(mean, f32::sqrt(var))
	}
}


#[derive(Clone)]
pub struct Gaussian2D {
	mean: Vector2,
	covar: Matrix2,
}

impl Gaussian2D {
	pub fn new(mean: Vector2, covar: Matrix2) -> Self {
		debug_assert!(covar.is_symmetric());
		Self { mean, covar }
	}

	pub fn from_std_dev_rotation(mean: Vector2, std_dev: Vector2, rotation: f32) -> Self {
		let covar = Matrix2::from_basis(
			Vector2::new(std_dev.x.powi(2), 0.),
			Vector2::new(0., std_dev.y.powi(2)),
		);
		let rot = Matrix2::from_rotation(rotation);
		let covar = rot.dot(&covar).dot(&rot.transposed());
		// godot_print!("covar={:?}", covar);
		Self::new(mean, covar)
	}

	#[inline]
	pub fn mean(&self) -> &Vector2 { &self.mean }
	#[inline]
	pub fn covariance(&self) -> &Matrix2 { &self.covar }

	pub fn probability_density(&self, x: Vector2) -> f32 {
		let z = x - self.mean;
		let n = 2.0*PI*self.covar.determinant().sqrt();
		let arg = -0.5*z.dot(self.covar.xform_inv(z));
		f32::exp(arg)/n
	}

	pub fn sample(&self) -> Vector2 {
		// Add a small multiple of I to the covariance matrix to ensure 
		// numerical stability of Cholesky decomposion
		let sigma = &self.covar + &((&Matrix2::IDENTITY)*f32::EPSILON);
		let ll = sigma.cholesky();
		// godot_print!("sigma={:?}", sigma);
		// godot_print!("ll={:?}", ll);
		let u = Vector2::new(
			StandardNormal.sample(&mut rand::thread_rng()),
			StandardNormal.sample(&mut rand::thread_rng()),
		);
		self.mean + ll.xform(u)
	}
}
