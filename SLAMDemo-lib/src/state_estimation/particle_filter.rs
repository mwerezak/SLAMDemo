use std::iter::Sum;
use std::ops::AddAssign;
use num_traits::Float;
use rand::Rng;
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
pub enum ResamplePolicy {
	WeightedIndex,
	LowVariance,
}

#[derive(Debug)]
pub struct ParticleFilter<W, P>
where W: Float, P: Particle<W>
{
	num_particles: usize,
	weights: Vec<W>,
	particles: Vec<P>,
	resample_policy: ResamplePolicy,
	rng: ThreadRng,
}

impl<P,W,U,Z> ParticleFilter<W, P>
where 
	W: Float + SampleUniform + Default + AddAssign + for<'a> AddAssign<&'a W> + Sum,
	P: Particle<W, Update=U, Measurement=Z>,
{
	pub fn new(num_particles: usize, f: impl Fn() -> P) -> Self {
		Self::with_resample_policy(num_particles, ResamplePolicy::WeightedIndex, f)
	}

	pub fn with_resample_policy(num_particles: usize, resample_policy: ResamplePolicy, f: impl Fn() -> P) -> Self {
		if num_particles == 0 {
			panic!("num_particles must not be zero");
		}

		let mut particles = Vec::with_capacity(num_particles);
		for _ in 0..num_particles {
			particles.push(f());
		}

		Self {
			num_particles,
			weights: Vec::with_capacity(num_particles),
			particles,
			resample_policy,
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

	pub fn particles(&self) -> &[P] { &self.particles }
	pub fn weights(&self) -> &[W] { &self.weights }

	fn recalc_weights(&mut self, meas: &P::Measurement) {
		self.weights.resize(self.particles.len(), W::zero());
		for (idx, particle) in self.particles.iter().enumerate() {
			self.weights[idx] = particle.calc_weight(meas.clone());
		}
	}

	pub fn measurement_update(&mut self, meas: &P::Measurement) {
		self.recalc_weights(meas);
		self.particles = match self.resample_policy {
			ResamplePolicy::WeightedIndex => self.weighted_index_sample(self.num_particles),
			ResamplePolicy::LowVariance => self.low_variance_sample(self.num_particles),
		};
	}

	fn weighted_index_sample(&mut self, m: usize) -> Vec<P> {
		let sampler = WeightedIndex::new(&self.weights).unwrap();
		let mut resampled = Vec::with_capacity(self.num_particles);
		for _ in 0..m {
			let idx = sampler.sample(&mut self.rng);
			resampled.push(self.particles[idx].clone());
		}
		resampled
	}

	fn low_variance_sample(&mut self, m: usize) -> Vec<P> {
		let total_weight: W = self.weights.iter().copied().sum();
		let frac_width = total_weight/W::from(m).unwrap();
		let r = self.rng.gen_range(W::zero()..=frac_width);

		let mut idx = 0usize;
		let mut cum_weight = *self.weights.first().unwrap();
		let mut resampled = Vec::with_capacity(self.num_particles);
		for j in 0..m {
			let target = r + frac_width*W::from(j).unwrap();
			while cum_weight < target {
				idx += 1;
				cum_weight += &self.weights[idx];
			}
			resampled.push(self.particles[idx].clone());
		}
		resampled
	}
}

