use std::ops::AddAssign;
use num_traits::Float;
use rand::rngs::ThreadRng;
use rand::distributions::uniform::SampleUniform;
use rand::distributions::{WeightedIndex, Distribution};


pub trait Particle<W: Float>: Clone {
	type Update;
	type Measurement;

	fn update_state(&mut self, update: Self::Update);
	fn calc_weight(&self, meas: Self::Measurement) -> W;
}

pub trait ParticleInit<P, W> 
where W: Float, P: Particle<W>
{
	fn make_particle(&self) -> P;
}


pub struct ParticleFilter<P, W>
where W: Float, P: Particle<W>
{
	num_particles: usize,
	weights: Vec<W>,
	particles: Vec<P>,
	rng: ThreadRng,
}

impl<P,W,U,Z> ParticleFilter<P, W>
where 
	// the extra trait bounds on W are needed for WeightedIndex
	W: Float + SampleUniform + Default + for<'a> AddAssign<&'a W>,
	P: Particle<W, Update=U, Measurement=Z>,
	U: Clone, Z: Clone,
{
	pub fn new(num_particles: usize, init: impl ParticleInit<P,W>) -> Self {
		let mut particles = Vec::with_capacity(num_particles);
		for _ in 0..num_particles {
			particles.push(init.make_particle());
		}

		Self {
			num_particles,
			weights: Vec::with_capacity(num_particles),
			particles,
			rng: rand::thread_rng(),
		}
	}

	pub fn state_update(&mut self, update: P::Update) {
		for particle in self.particles.iter_mut() {
			particle.update_state(update.clone())
		}
	}

	pub fn measurement_update(&mut self, meas: P::Measurement) {
		self.recalc_weights(meas);
		self.particles = self.weighted_sample(self.num_particles);
	}

	fn recalc_weights(&mut self, meas: P::Measurement) {
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

