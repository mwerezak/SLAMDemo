use nalgebra::{SVector, SMatrix, vector, matrix};
use crate::math::Gaussian;
use crate::motion_model::Pose2D;
use crate::motion_model::odometry::*;


struct EKFOdoState {
	state: SVector<f32,3>,
	covar: SMatrix<f32,3,3>,
}

impl EKFOdoState {
	pub fn pack_exact(pose: &Pose2D) -> Self {
		Self { 
			state: vector![ pose.loc.x, pose.loc.y, pose.rot ],
			covar: SMatrix::repeat(0.0),
		}
	}

	pub fn pack_estimate(pose: &Pose2D, covar: SMatrix::<f32,3,3>) -> Self {
		Self { 
			state: vector![ pose.loc.x, pose.loc.y, pose.rot ],
			covar,
		}
	}

	pub fn unpack_pose(&self) -> Pose2D {
		Pose2D::new(self.state[0], self.state[1], self.state[2])
	}
}


struct GPSMeasurementModel {
	x: Gaussian,
	y: Gaussian,
}


struct EKFOdometry2D {
	state: EKFOdoState,

}

impl EKFOdometry2D {
	fn update(&mut self, control: &OdoMotionModel2D, meas: &GPSMeasurementModel) {
		let update_motion = control.sample_motion();
		let update_jacobian = {
			let cur_pose = self.state.unpack_pose();
			matrix![
				1., 0., -update_motion.trans*f32::sin(cur_pose.rot + update_motion.rot1);
				0., 1., -update_motion.trans*f32::cos(cur_pose.rot + update_motion.rot1);
				0., 0., 1.;
			]
		};

		let update_pose = update_motion.apply_update(&self.state.unpack_pose());
		let update_covar = (
			update_jacobian * self.state.covar * update_jacobian.transpose()
			+ 
		);

		let state_update = EKFOdoState::pack_estimate(&update_pose, update_covar);


	}
}