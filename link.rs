use crate::physics::circle::Circle;
use crate::physics::particle::Particle;

#[derive(Debug, Copy, Clone)]
pub struct Link {
    pub particle_a: usize,
    pub particle_b: usize,
    pub target_distance: f32,
}

pub struct ParticleLink {
    pub link: Link,
}

impl ParticleLink {
    pub fn solve(&mut self, particles: &mut Vec<Particle>) {
        let split = particles.split_at_mut(self.link.particle_b);
        let particle_a = &mut split.0[self.link.particle_a];
        let particle_b = &mut split.1[0];
        let dist_vec = particle_a.pos - particle_b.pos;
        let dist = dist_vec.length();
        let normal = dist_vec.normalise();
        particle_a.pos -= normal * (dist - self.link.target_distance) * 0.5;
        particle_b.pos += normal * (dist - self.link.target_distance) * 0.5;
    }
}

pub struct CircleLink {
    pub link: Link,
}

impl CircleLink {
    pub fn solve(&mut self, circle: &mut Vec<Circle>) {

    }
}