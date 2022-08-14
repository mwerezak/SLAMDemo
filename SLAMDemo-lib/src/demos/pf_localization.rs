// Localization with odometry and GPS
use gdnative::prelude::*;
use crate::math::{Gaussian2D, Gaussian, Matrix2};
use crate::motion_model::Pose2D;
use crate::motion_model::odometry::OdoMotionModel2D;
use crate::simulation::{GPSMeasurement};
use crate::state_estimation::particle_filter::{
	Particle, ParticleFilter
};


#[derive(Clone, Debug)]
struct DemoParticle {
	pub pose: Pose2D,
}

impl DemoParticle {
	fn new(pose: Pose2D) -> Self {
		Self { pose }
	}
}

impl Particle<f32> for DemoParticle {
	type Update = OdoMotionModel2D;
	type Measurement = GPSMeasurement;
	fn update_state(&mut self, update: &OdoMotionModel2D) {
		let motion = update.sample_motion();
		self.pose = motion.apply_update(&self.pose);
	}

	fn calc_weight(&self, meas: &GPSMeasurement) -> f32 {
		let gps_model = Gaussian2D::new(self.pose.loc, meas.covar);
		let weight = gps_model.probability_density(meas.loc);
		// godot_print!("loc: {:?}", self.pose.loc);
		// godot_print!("gps: {:?}", meas);
		// godot_print!("weight: {:?}", weight);
		weight
	}
}

type DemoParticleFilter = ParticleFilter<f32, DemoParticle>;


#[derive(NativeClass)]
#[inherit(Node)]
pub struct LocalizationFilter {
	pfilter: Option<DemoParticleFilter>,
	particle_count: usize,
}

#[methods]
impl LocalizationFilter {
	fn new(_owner: &Node) -> Self {
		Self {
			pfilter: None,
			particle_count: 10000,
		}
	}

	// reset the localization, assuming the given pose with absolute certainty
	// this must be called at least once to initialize the localization
	#[export]
	fn reset_pose_with_absolute_certainty(&mut self, _owner: &Node, true_pose: Transform2D) {
		self.pfilter = Some(DemoParticleFilter::new(
			self.particle_count, 
			|| DemoParticle { pose: true_pose.into() }
		));
	}

	#[export]
	fn reset_pose_with_uncertainty(&mut self, _owner: &Node, mean: Transform2D, loc_covar: Matrix2, rot_std_dev: f32) {
		let mean = Pose2D::from(mean);
		let loc_model = Gaussian2D::new(mean.loc, loc_covar);
		let rot_model = Gaussian::new(mean.rot, rot_std_dev);
		self.pfilter = Some(DemoParticleFilter::new(
			self.particle_count, 
			|| DemoParticle { pose: Pose2D {
				loc: loc_model.sample(),
				rot: rot_model.sample(),
			} }
		));
	}

	#[export]
	fn set_particle_count(&mut self, _owner: &Node, count: usize) {
		self.particle_count = count;
		if let Some(pfilter) = self.pfilter.as_mut() {
			pfilter.set_target_size(count);
		}
	}

	#[export]
	fn get_particle_count(&self, _owner: &Node) -> usize {
		match self.pfilter.as_ref() {
			Some(pfilter) => pfilter.size(),
			None => self.particle_count,
		}
	}

	#[export]
	fn get_particles(&self, _owner: &Node, max_count: usize) -> Option<Vec<(Pose2D, f32)>> {
		if let Some(pfilter) = self.pfilter.as_ref() {
			let particles = pfilter.particles().iter()
				.map(|p| p.pose);
			let weights = pfilter.weights().iter().copied()
				.chain(std::iter::repeat(0.));
			
			let mut data = particles.zip(weights)
				.filter(|(_, w)| w.is_finite())
				.collect::<Vec<(Pose2D, f32)>>();

			// sort in ascending order of weight, and keep only the top max_count elements
			data.sort_by(|(_, a), (_, b)| f32::partial_cmp(b, a).unwrap());
			data.truncate(max_count);
			return Some(data)
		}
		None
	}

	#[export]
	fn motion_update(&mut self, _owner: &Node, motion_model: OdoMotionModel2D) {
		if let Some(pfilter) = self.pfilter.as_mut() {
			pfilter.state_update(&motion_model)
		}
	}

	#[export]
	fn gps_update(&mut self, _owner: &Node, gps_meas: GPSMeasurement) {
		if let Some(pfilter) = self.pfilter.as_mut() {
			pfilter.measurement_update(&gps_meas)
		}
	}
}
