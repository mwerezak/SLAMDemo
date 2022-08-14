use std::ops::AddAssign;
use num_traits::Float;
use rand::rngs::ThreadRng;
use rand::distributions::uniform::SampleUniform;
use rand::distributions::{WeightedIndex, Distribution};


pub trait Particle<W: Float>: Clone {
	type Update;
	type Measurement;

	// implementations should update the current state in-place
	// by sampling from the state transition distribution
	// provided by Update
	fn update_state(&mut self, update: &Self::Update);

	// implementations should return the probability density
	// associated with the given Measurement conditioned on
	// the particle's state.
	fn calc_weight(&self, meas: &Self::Measurement) -> W;
}

#[derive(Debug)]
pub struct ParticleFilter<W, P>
where W: Float, P: Particle<W>
{
	num_particles: usize,
	weights: Vec<W>,
	particles: Vec<P>,
	rng: ThreadRng,
}

impl<P,W,U,Z> ParticleFilter<W, P>
where 
	// the extra trait bounds on W are needed for WeightedIndex
	W: Float + SampleUniform + Default + for<'a> AddAssign<&'a W>,
	P: Particle<W, Update=U, Measurement=Z>,
{
	pub fn new(num_particles: usize, f: impl Fn() -> P) -> Self {
		let mut particles = Vec::with_capacity(num_particles);
		for _ in 0..num_particles {
			particles.push(f());
		}

		Self {
			num_particles,
			weights: Vec::with_capacity(num_particles),
			particles,
			rng: rand::thread_rng(),
		}
	}

	pub fn size(&self) -> usize { self.particles.len() }
	pub fn target_size(&self) -> usize { self.num_particles }
	pub fn set_target_size(&mut self, num_particles: usize) {
		self.num_particles = num_particles;
	}

	pub fn state_update(&mut self, update: &P::Update) {
		for particle in self.particles.iter_mut() {
			particle.update_state(&update)
		}
	}

	pub fn measurement_update(&mut self, meas: &P::Measurement) {
		self.recalc_weights(meas);
		self.particles = self.weighted_sample(self.num_particles);
	}

	pub fn particles(&self) -> &[P] { &self.particles }
	pub fn weights(&self) -> &[W] { &self.weights }

	fn recalc_weights(&mut self, meas: &P::Measurement) {
		self.weights.resize(self.particles.len(), W::zero());
		for (idx, particle) in self.particles.iter().enumerate() {
			self.weights[idx] = particle.calc_weight(meas.clone());
		}
	}

	fn weighted_sample(&mut self, n: usize) -> Vec<P> {
		let sampler = WeightedIndex::new(&self.weights).unwrap();
		let mut resampled = Vec::with_capacity(self.num_particles);
		for _ in 0..n {
			let idx = sampler.sample(&mut self.rng);
			resampled.push(self.particles[idx].clone());
		}
		resampled
	}

}

