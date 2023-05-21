use crate::particle::Particle;

#[derive(Debug, Copy, Clone)]
pub struct Spring {
    pub particle_a: usize,
    pub particle_b: usize,
    pub rest_length: f32,
    pub stiffness: f32,
    pub permanence_threshold: f32,
}

impl Spring {
    pub fn new(
        particle_a: usize,
        particle_b: usize,
        rest_length: f32,
        stiffness: f32,
        permanence_threshold: f32,
    ) -> Self {
        Self {
            particle_a,
            particle_b,
            rest_length,
            stiffness,
            permanence_threshold,
        }
    }

    pub fn solve(&mut self, particles: &mut Vec<Particle>, dt: f32) {
        let split = particles.split_at_mut(self.particle_b);
        let particle_a = &mut split.0[self.particle_a];
        let particle_b = &mut split.1[0];

        let delta = particle_a.pos - particle_b.pos;
        let delta_length = delta.magnitude();
        if delta_length == 0.0 {
            return;
        }

        let mut difference = delta_length - self.rest_length;
        // Permanent deformation
        if difference < self.permanence_threshold * self.rest_length {
            self.rest_length = delta_length;
            difference = delta_length - self.rest_length;
        }
        let impulse = delta.normalize() * difference * self.stiffness
            / (particle_a.inv_mass + particle_b.inv_mass);

        // TODO: Make this use the add_force method instead of directly modifying the velocity to account for mass
        particle_a.pos -= particle_a.inv_mass * impulse * dt;
        particle_b.pos += particle_b.inv_mass * impulse * dt;
    }
}
