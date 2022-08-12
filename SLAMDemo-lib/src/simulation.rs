use gdnative::prelude::*;
use crate::motion_model::{Pose2D};
use crate::motion_model::odometry::{OdometryNoise, OdometryModel2D, OdoUpdate2D, OdoMotionBuilder2D};


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
// A Node2D that tracks it's own position with simulated odometry noise
#[derive(NativeClass)]
#[inherit(Node2D)]
pub struct Odometry {
	est_pose: Option<Pose2D>,   // accumulate estimated pose
	last_pose: Option<Pose2D>,  // last true pose
	model: OdometryModel2D,
}

impl Odometry {
	const MOTION_THRESHOLD_SPEED: f32 = 0.5; // min speed to consider that we are moving

	fn new(_owner: &Node2D) -> Self { 
		// actual values are loaded later from editor by set_noise_params()
		let noise_params = OdometryNoise::default();
		let motion_params = OdoMotionBuilder2D {
			speed_threshold: Self::MOTION_THRESHOLD_SPEED,
			allow_reverse: true,
		};

		Self {
			est_pose: None,
			last_pose: None,
			model: OdometryModel2D::new(noise_params, motion_params),
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
	fn set_noise_params(&mut self, _owner: &Node2D, rot_rot: f32, trans_rot: f32, trans_trans: f32, rot_trans: f32) {
		let noise_params = self.model.noise_params_mut();
		noise_params.rot_rot = rot_rot;
		noise_params.trans_rot = trans_rot;
		noise_params.rot_trans = rot_trans;
		noise_params.trans_trans = trans_trans;

		// godot_print!("noise params: {:?}", self.model.noise_params());
	}

	#[export]
	fn _physics_process(&mut self, owner: &Node2D, delta: f32) {
		if let (Some(est_pose), Some(last_pose)) = (self.est_pose.as_mut(), self.last_pose.as_mut()) {
			let cur_pose = owner.get_global_pose();
			let true_update = OdoUpdate2D::new(*last_pose, cur_pose, delta);
			let meas_motion = self.model.get_motion_measurement(&true_update);
			let meas_pose = meas_motion.apply_update(est_pose);
			*last_pose = cur_pose; // update last true pose
			*est_pose = meas_pose; // update estimated pose
		}
	}

	#[export]
	fn get_estimated_global_transform(&self, owner: &Node2D) -> Transform2D {
		self.est_pose
			.map(|pose| pose.into())
			.unwrap_or_else(|| owner.get_global_transform())
	}
}