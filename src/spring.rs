use crate::particle::Particle;

#[derive(Debug, Copy, Clone)]
pub struct Spring {
    pub particle_a: usize,
    pub particle_b: usize,
    pub rest_length: f32,
    pub stiffness: f32,
}

impl Spring {
    pub fn new(particle_a: usize, particle_b: usize, rest_length: f32, stiffness: f32) -> Self {
        Self {
            particle_a,
            particle_b,
            rest_length,
            stiffness,
        }
    }

    pub fn solve(&self, particles: &mut Vec<Particle>, dt: f32) {
        let split = particles.split_at_mut(self.particle_b);
        let particle_a = &mut split.0[self.particle_a];
        let particle_b = &mut split.1[0];

        let delta = particle_b.pos - particle_a.pos;
        let delta_length = delta.magnitude();
        if delta_length == 0.0 {
            return;
        }
        let delta_normal = delta / delta_length;

        let difference = (delta_length - self.rest_length) / delta_length;
        let impulse = delta_normal * difference * self.stiffness;

        particle_a.pos += impulse * dt;
        particle_b.pos -= impulse * dt;
    }
}
