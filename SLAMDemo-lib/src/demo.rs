use gdnative::prelude::*;
use crate::math::{self, Gaussian};
use crate::motion_model::{Pose2D, OdometryNoise};



trait HasPose2D {
	fn get_pose(&self) -> Pose2D;
	fn get_global_pose(&self) -> Pose2D;
}

impl HasPose2D for Node2D {
	fn get_pose(&self) -> Pose2D {
		Pose2D::from(self.get_transform())
	}
	fn get_global_pose(&self) -> Pose2D {
		Pose2D::from(self.get_global_transform())
	}	
}



#[derive(NativeClass)]
#[inherit(Node)]
pub struct DemoController {

}

impl DemoController {
	fn new(_owner: &Node) -> Self { Self {} }
}

#[methods]
impl DemoController {
	#[export]
	fn _ready(&self, _owner: &Node) {
		godot_print!("Hello, world!");
	}

	#[export]
	fn _process(&self, _owner: &Node, _delta: f64) {
		// godot_print!("delta: {}", delta);
	}
}



// odometry tracker with simulated noise
#[derive(NativeClass)]
#[inherit(Node2D)]
pub struct Odometry {
	est_pose: Option<Pose2D>,
	last_pose: Option<Pose2D>,  // last true pose
	noise: OdometryNoise,
	speed_threshold: f32, // min speed to consider that we are moving
}

impl Odometry {
	fn new(_owner: &Node2D) -> Self { 
		Odometry {
			est_pose: None,
			last_pose: None,
			noise: OdometryNoise::new(0.01, 0., 0.00001, 0.01),
			speed_threshold: 1.0,
		} 
	}
}


#[methods]
impl Odometry {


	#[export]
	fn _ready(&mut self, owner: &Node2D) {
		self.est_pose = Some(owner.get_global_pose());
		self.last_pose = Some(owner.get_global_pose());
	}

	#[export]
	fn _physics_process(&mut self, owner: &Node2D, delta: f32) {
		if let (Some(mut est_pose), Some(last_pose)) = (self.est_pose.as_mut(), self.last_pose.as_mut()) {
			const PI: f32 = std::f32::consts::PI;

			// model odometry as rot1, translation, rot2
			let cur_pose = owner.get_global_pose();
			
			let true_trans = (cur_pose.loc - last_pose.loc).length();

			// if we aren't actually moving, atan2() will introduce a lot of error, so apply a threshold
			let true_rot1 = if true_trans > self.speed_threshold*delta {
				f32::atan2(cur_pose.loc.y - last_pose.loc.y, cur_pose.loc.x - last_pose.loc.x) - last_pose.rot
			} else { 0.0 };
			
			// hack to handle reversing
			let mut rot_rev = 0.0;
			let mut true_rot1 = math::wrap(true_rot1, -PI, PI);
			if true_rot1.abs() > PI/2.0 {
				rot_rev = PI;
				true_rot1 = math::wrap(true_rot1 - rot_rev, -PI, PI);
			}


			let true_rot2 = cur_pose.rot - last_pose.rot - true_rot1;
			let true_rot2 = math::wrap(true_rot2, -PI, PI);

			let (alpha1, alpha2, alpha3, alpha4) = self.noise.alpha;
			let rot1_noise = alpha1*true_rot1.powi(2) + alpha2*true_trans.powi(2);
			let lin_noise = alpha3*true_trans.powi(2) + alpha4*(true_rot1.powi(2) + true_rot2.powi(2));
			let rot2_noise = alpha1*true_rot2.powi(2) + alpha2*true_trans.powi(2);

			let meas_rot1 = Gaussian::new(true_rot1, rot1_noise).sample();
			let meas_trans = Gaussian::new(true_trans, lin_noise).sample();
			let meas_rot2 = Gaussian::new(true_rot2, rot2_noise).sample();

			// update pose
			*last_pose = cur_pose;
			est_pose.loc += Vector2::RIGHT.rotated(est_pose.rot + meas_rot1 + rot_rev) * meas_trans;
			est_pose.rot += meas_rot1 + meas_rot2;
		}
	}

	#[export]
	fn get_estimated_global_transform(&self, owner: &Node2D) -> Transform2D {
		self.est_pose
			.map(|pose| pose.into())
			.unwrap_or_else(|| owner.get_global_transform())
	}
}