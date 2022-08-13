use gdnative::prelude::*;
use crate::math::{self, Gaussian};
use crate::motion_model::Pose2D;

#[derive(Clone)]
pub struct OdoUpdate2D {
	pub prev: Pose2D,
	pub next: Pose2D,
	pub delta: f32, // timedelta
}

impl OdoUpdate2D {
	pub fn new(prev: Pose2D, next: Pose2D, delta: f32) -> Self {
		Self { prev, next, delta }
	}
}


pub struct OdoMotionBuilder2D {
	pub speed_threshold: f32,
	pub allow_reverse: bool,
}

impl Default for OdoMotionBuilder2D {
	fn default() -> OdoMotionBuilder2D {
		Self {
			speed_threshold: 1.0,
			allow_reverse: true,
		}
	}
}

impl OdoMotionBuilder2D {
	pub fn with_threshold(speed_threshold: f32) -> Self {
		Self { speed_threshold, allow_reverse: true, }
	}

	pub fn from_update(&self, update: &OdoUpdate2D) -> OdoMotion2D {
		const PI: f32 = std::f32::consts::PI;

		// model odometry as: rot1 -> translation -> rot2
		let cur_pose = update.next;
		let last_pose = update.prev;
		
		let mut trans = (cur_pose.loc - last_pose.loc).length();

		// if we aren't actually moving, atan2() will introduce a lot of error, so apply a threshold
		let mut rot1 = 0.0;
		if trans > self.speed_threshold * update.delta {
			rot1 = f32::atan2(cur_pose.loc.y - last_pose.loc.y, cur_pose.loc.x - last_pose.loc.x) - last_pose.rot;
			rot1 = math::wrap(rot1, -PI, PI);
		}
		
		if self.allow_reverse && rot1.abs() > PI/2.0 {
			trans *= -1.0;
			rot1 = math::wrap(rot1-PI, -PI, PI);
		}

		let rot2 = cur_pose.rot - last_pose.rot - rot1;
		let rot2 = math::wrap(rot2, -PI, PI);

		OdoMotion2D { rot1, trans, rot2, delta: update.delta }
	}
}

#[derive(Clone)]
pub struct OdoMotion2D {
	pub rot1: f32,
	pub trans: f32,
	pub rot2: f32,
	pub delta: f32,
}

impl OdoMotion2D {
	pub fn apply_update(&self, pose: &Pose2D) -> Pose2D {
		Pose2D {
			loc: pose.loc + Vector2::RIGHT.rotated(pose.rot + self.rot1) * self.trans,
			rot: pose.rot + self.rot1 + self.rot2,
		}
	}
}

#[derive(Clone)]
pub struct OdoMotionModel2D {
	pub rot1: Gaussian,
	pub trans: Gaussian,
	pub rot2: Gaussian,
	pub delta: f32,
}

impl OdoMotionModel2D {
	pub fn sample_pose(&self, base: &Pose2D) -> Pose2D {
		self.sample_motion()
			.apply_update(base)
	}

	pub fn sample_motion(&self) -> OdoMotion2D {
		OdoMotion2D {
			rot1: self.rot1.sample(),
			trans: self.trans.sample(),
			rot2: self.rot2.sample(),
			delta: self.delta,
		}
	}

	pub fn mean_motion(&self) -> OdoMotion2D {
		OdoMotion2D {
			rot1: self.rot1.mean(),
			trans: self.trans.mean(),
			rot2: self.rot2.mean(),
			delta: self.delta,
		}
	}
}

// Adapted from chapter 5.4
#[derive(Debug)]
pub struct OdometryNoise {
	pub rot_rot: f32,     // effect of rotation speed on rotation noise
	pub trans_rot: f32,   // effect of translation speed on rotation noise
	pub trans_trans: f32, // effect of translation speed on translation noise
	pub rot_trans: f32,   // effect of rotation speed on translation noise
}

impl Default for OdometryNoise {
	fn default() -> Self {
		Self { rot_rot: 0., trans_rot: 0., trans_trans: 0., rot_trans: 0. }
	}
}

impl OdometryNoise {
	pub fn new(rot_rot: f32, trans_rot: f32, trans_trans: f32, rot_trans: f32) -> Self {
		Self {
			rot_rot, rot_trans,
			trans_rot, trans_trans,
		}
	}
}

pub struct OdometryModel2D {
	noise: OdometryNoise,
	builder: OdoMotionBuilder2D,
}

impl OdometryModel2D {
	pub fn new(noise_params: OdometryNoise, motion_params: OdoMotionBuilder2D) -> Self {
		Self {
			noise: noise_params,
			builder: motion_params,
		}
	}

	pub fn noise_params(&self) -> &OdometryNoise { &self.noise }
	pub fn noise_params_mut(&mut self) -> &mut OdometryNoise { &mut self.noise }

	pub fn get_motion_model(&self, update: &OdoUpdate2D) -> OdoMotionModel2D {
		let motion = self.builder.from_update(update);

		let rot1_sqr = motion.rot1.powi(2);
		let rot2_sqr = motion.rot2.powi(2);
		let trans_sqr = motion.trans.powi(2);

		let rot1_stdev  = rot1_sqr*self.noise.rot_rot + trans_sqr*self.noise.trans_rot;
		let trans_stdev = trans_sqr*self.noise.trans_trans + (rot1_sqr + rot2_sqr)*self.noise.rot_trans;
		let rot2_stdev  = rot2_sqr*self.noise.rot_rot + trans_sqr*self.noise.trans_rot;

		OdoMotionModel2D {
			rot1:  Gaussian::new(motion.rot1,  rot1_stdev),
			trans: Gaussian::new(motion.trans, trans_stdev),
			rot2:  Gaussian::new(motion.rot2,  rot2_stdev),
			delta: motion.delta,
		}
	}

	// get measured motion with noise applied
	pub fn get_motion_measurement(&self, true_update: &OdoUpdate2D) -> OdoMotion2D {		
		self.get_motion_model(true_update)
			.sample_motion()
	}
}
