use gdnative::prelude::*;
use crate::math::{Matrix2, Gaussian2D};
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



// odometry tracker with simulated noise
// A Node2D that tracks it's own position with simulated odometry noise
#[derive(NativeClass)]
#[inherit(Node2D)]
#[register_with(Self::register_signals)]
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

	fn register_signals(builder: &ClassBuilder<Self>) {
		builder.signal("motion_update")
			.with_param("measured_model", VariantType::Object)
			.with_param("est_pose", VariantType::Object)
			.done();
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
		if let (Some(est_pose), Some(last_pose)) = (self.est_pose.as_mut(), self.last_pose.as_mut()) {
			let cur_pose = owner.get_global_pose();
			let true_update = OdoUpdate2D::new(*last_pose, cur_pose, delta);
			let true_model = self.model.get_motion_model(&true_update);

			let meas_model = true_model.sample_motion_model();
			let meas_pose = meas_model.mean_motion().apply_update(est_pose);
			*last_pose = cur_pose; // update last true pose
			*est_pose = meas_pose; // update estimated pose

			owner.emit_signal("motion_update", &[Variant::new(meas_model), Variant::new(est_pose)]);
		}
	}

	#[export]
	fn load_settings(&mut self, _owner: &Node2D, settings: Ref<Object>) {
		let settings = unsafe { settings.assume_safe() };
		
		let motion_params = self.model.motion_params_mut();
		if let Some(value) = settings.get("speed_threshold").to::<f32>() {
			motion_params.speed_threshold = value;
		}
		if let Some(value) = settings.get("allow_reverse").to::<bool>() {
			motion_params.allow_reverse = value;
		}

		let noise_params = self.model.noise_params_mut();
		if let Some(value) = settings.get("rot_rot").to::<f32>() {
			noise_params.rot_rot = value;
		}
		if let Some(value) = settings.get("trans_rot").to::<f32>() {
			noise_params.trans_rot = value;
		}
		if let Some(value) = settings.get("rot_trans").to::<f32>() {
			noise_params.rot_trans = value;
		}
		if let Some(value) = settings.get("trans_trans").to::<f32>() {
			noise_params.trans_trans = value;
		}
	}

	#[export]
	fn get_estimated_global_transform(&self, owner: &Node2D) -> Transform2D {
		self.est_pose
			.map(|pose| pose.into())
			.unwrap_or_else(|| owner.get_global_transform())
	}
}



#[derive(ToVariant, FromVariant)]
pub struct GPSMeasurement {
	pub loc: Vector2,
	pub covar: Matrix2,
}

// #[methods]
impl GPSMeasurement { }


pub struct GPSModel {
	noise_model: Gaussian2D,
}

impl GPSModel {
	pub fn new(std_dev: f32) -> Self {
		let covar = &Matrix2::IDENTITY*std_dev.powi(2);
		Self {
			noise_model: Gaussian2D::new(Vector2::ZERO, covar),
		}
	}

	pub fn get_measurement(&self, true_loc: Vector2) -> GPSMeasurement {
		GPSMeasurement {
			loc: true_loc + self.noise_model.sample(),
			covar: *self.noise_model.covariance(),
		}
	}
}

#[derive(NativeClass)]
#[inherit(Node2D)]
pub struct GPS {
	model: GPSModel,
}

#[methods]
impl GPS {
	fn new(_owner: &Node2D) -> Self {
		Self { model: GPSModel::new(0.) }
	}

	#[export]
	fn load_noise_model(&mut self, _owner: &Node2D, noise_model: Ref<Object>) {
		let noise_model = unsafe { noise_model.assume_safe() };
		let std_dev = noise_model.get("std_dev").to::<f32>().unwrap();
		self.model = GPSModel::new(std_dev);
	}

	#[export]
	pub fn measure_position(&self, owner: &Node2D) -> GPSMeasurement {
		self.model.get_measurement(owner.position())
	}

	#[export]
	pub fn measure_global_position(&self, owner: &Node2D) -> GPSMeasurement {
		self.model.get_measurement(owner.global_position())
	}
}


