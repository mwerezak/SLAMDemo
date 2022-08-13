// Localization with odometry and GPS

use nalgebra::Matrix2;

use crate::motion_model::Pose2D;
use crate::motion_model::odometry::OdoMotion2D;
use crate::state_estimation::particle_filter::{
	Particle
};


struct GPSMeasurement {
	x: f32,
	y: f32,
	covar: Matrix2<f32>,
}

#[derive(Clone)]
struct DemoParticle {
	pose: Pose2D,
}

impl Particle<f32> for DemoParticle {
	type Update = OdoMotion2D;
	type Measurement = GPSMeasurement;
	fn update_state(&mut self, update: &OdoMotion2D) {
		self.pose = update.apply_update(&self.pose);
	}

	fn calc_weight(&self, _meas: &GPSMeasurement) -> f32 {
		todo!()
	}
}

