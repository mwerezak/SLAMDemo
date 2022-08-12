use rand;
use gdnative::prelude::*;
use crate::math::{Matrix2, Gaussian};


trait MotionModel {
	type State;
	type Control;
	type MotionEstimate;
	fn motion_update(input: Self::Control, prev_state: Self::State) -> Self::MotionEstimate;
	// fn next_meas_state(prev_meas_state: Self::State, next_state: Self::State, prev_state: Self::State) -> Self::State;
}


#[derive(Clone, Copy, Debug)]
pub struct Pose2D {
	pub loc: Vector2,
	pub rot: f32,
}

impl From<Transform2D> for Pose2D {
	fn from(xform: Transform2D) -> Self {
		Self {
			loc: xform.origin,
			rot: xform.rotation(),
		}
	}
}

impl Into<Transform2D> for Pose2D {
	fn into(self) -> Transform2D {
		let mut result = Transform2D::IDENTITY.rotated(self.rot);
		result.origin = self.loc;
		result
	}
}

impl Pose2D {
	pub fn new(x: f32, y: f32, rot: f32) -> Self {
		Self { loc: Vector2::new(x,y), rot }
	}
}


// See figure 5.33
struct OdoControl2D {
	next: Pose2D,
	prev: Pose2D,
}


struct OdoEstimate2D {

}

// Adapted from chapter 5.4
pub struct OdometryNoise {
	pub alpha: (f32, f32, f32, f32),
}

impl OdometryNoise {
	pub fn new(alpha1: f32, alpha2: f32, alpha3: f32, alpha4: f32) -> Self {
		Self { alpha: (alpha1, alpha2, alpha3, alpha4) }
	}
}


struct OdometryModel2D {

}



