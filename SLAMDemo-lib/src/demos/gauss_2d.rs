use gdnative::prelude::*;
use crate::math::Gaussian2D;


#[derive(NativeClass)]
#[inherit(Node)]
pub struct Gauss2D {
	dist: Option<Gaussian2D>,
}

#[methods]
impl Gauss2D {
	fn new(_owner: &Node) -> Self {
		Self {
			dist: None,
		}
	}

	#[export]
	fn _ready(&mut self, owner: &Node) {
		let dist_info = unsafe { 
			owner.get_node("Distribution")
				.expect("Distribution is required")
				.assume_safe()
		};
		let dist = dist_info.get("dist").to::<Transform2D>().unwrap();
		self.dist = Some(Gaussian2D::new(dist.origin, dist.into()));
	}

	#[export]
	fn sample(&self, _owner: &Node) -> Vector2 {
		self.dist.as_ref()
			.expect("distribution not initialized!")
			.sample()
	}
}