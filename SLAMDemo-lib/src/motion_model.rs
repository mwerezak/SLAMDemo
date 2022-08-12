use gdnative::prelude::*;


pub mod odometry;


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

impl std::ops::Add for &Pose2D {
	type Output = Pose2D;
	fn add(self, rhs: &Pose2D) -> Pose2D {
		Pose2D {
			loc: self.loc + rhs.loc,
			rot: self.rot + rhs.rot,
		}
	}
}

impl std::ops::Sub for &Pose2D {
	type Output = Pose2D;
	fn sub(self, rhs: &Pose2D) -> Pose2D {
		Pose2D {
			loc: self.loc - rhs.loc,
			rot: self.rot - rhs.rot,
		}
	}
}
