// Misc Godot interface code

use gdnative::prelude::*;
use crate::math::{Matrix2, Gaussian};
use crate::motion_model::Pose2D;


// Transparently convert Matrix2<->Transform2D at the Godot API layer
impl ToVariant for Matrix2 {
	fn to_variant(&self) -> Variant {
		let xform: Transform2D = (*self).into();
		xform.to_variant()
	}
}
impl FromVariant for Matrix2 {
	fn from_variant(o: &Variant) -> Result<Self, FromVariantError> {
		let xform = Transform2D::from_variant(o)?;
		Ok(xform.into())
	}
}


impl ToVariant for Gaussian {
	fn to_variant(&self) -> Variant {
		(self.mean(), self.std_dev()).to_variant()
	}
}
impl FromVariant for Gaussian {
	fn from_variant(o: &Variant) -> Result<Self, FromVariantError> {
		let (mean, std_dev) = o.try_to::<(f32, f32)>()?;
		Ok(Gaussian::new(mean, std_dev))
	}
}


impl ToVariant for Pose2D {
	fn to_variant(&self) -> Variant {
		Vector3::new(self.loc.x, self.loc.y, self.rot).to_variant()
	}
}
impl FromVariant for Pose2D {
	fn from_variant(o: &Variant) -> Result<Self, FromVariantError> {
		let v = o.try_to::<Vector3>()?;
		let pose = Pose2D {
			loc: Vector2::new(v.x, v.y),
			rot: v.z,
		};
		Ok(pose)
	}
}


/*
#[derive(NativeClass)]
#[inherit(Reference)]
#[no_constructor]
pub struct GaussData {
	mean: f32,
	std_dev: f32,
}

#[methods]
impl GaussData { }

impl Into<Gaussian> for &GaussData {
	fn into(self) -> Gaussian {
		Gaussian::new(self.mean, self.std_dev)
	}
}

impl From<&Gaussian> for GaussData {
	fn from(o: &Gaussian) -> Self {
		Self {
			mean: o.mean(),
			std_dev: o.std_dev(),
		}
	}
}

impl ToVariant for Gaussian {
	fn to_variant(&self) -> Variant {
		Instance::emplace(GaussData::from(self))
			.into_shared()
			.to_variant()
	}
}
impl FromVariant for Gaussian {
	fn from_variant(o: &Variant) -> Result<Self, FromVariantError> {
		let instance: Instance<GaussData, Shared> = o.try_to::<Instance<GaussData>>()?;
		let instance = unsafe { instance.assume_safe() };
		Ok(instance.map(|o, _owner| o.into()).unwrap())
	}
}
*/