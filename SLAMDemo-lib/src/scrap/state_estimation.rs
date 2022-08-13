// use gdnative::prelude::*;

use nalgebra::{SVector, SMatrix, vector, matrix};
use crate::motion_model::Pose2D;

use crate::motion_model::odometry::*;
struct EKFilterOdometry2D {

}

impl EKFilterOdometry2D {
	fn update(&mut self, control: &OdoMotion2D, ) {

	}
}


// Unscented Kalman Filter
struct UKFilter {

}


impl UKFilter {
	fn update(&mut self) {

	}
}


// state transition: x[t] = g(u[t], x[t-1]) + eps[t]  
// measurement: z[t] = h(x[t]) + delt[t]


struct DemoState {
	state: SVector<f32, 3>,
	covar: SMatrix<f32, 3, 3>,
}

impl DemoState {
	pub fn pack_from(pose: &Pose2D) -> Self {
		Self {
			state: vector![ pose.loc.x, pose.loc.y, pose.rot ],
			covar: SMatrix::<f32, 3, 3>::repeat(0.0),
		}
	}

	pub fn unpack_pose(&self) -> Pose2D {
		Pose2D::new(self.state[0], self.state[1], self.state[2])
	}
}


use crate::motion_model::odometry::*;

fn demo_state_update(update: &OdoUpdate2D, prev_state: &DemoState) -> DemoState {
	todo!()
}