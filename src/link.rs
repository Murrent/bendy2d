use crate::circle::Circle;
use crate::particle::Particle;

#[derive(Debug, Copy, Clone)]
pub struct Link {
    // particle_a needs to be lower than particle_b
    pub particle_a: usize,
    pub particle_b: usize,
    pub target_distance: f32,
}

#[derive(Debug, Copy, Clone)]
pub struct ParticleLink {
    pub link: Link,
}

impl ParticleLink {
    pub fn solve(&mut self, particles: &mut Vec<Particle>) {
        let split = particles.split_at_mut(self.link.particle_b);
        let particle_a = &mut split.0[self.link.particle_a];
        let particle_b = &mut split.1[0];
        let dist_vec = particle_a.pos - particle_b.pos;
        let dist = dist_vec.magnitude();
        // let normal = dist_vec.normalize();
        // particle_a.pos -= normal * (dist - self.link.target_distance) * 0.5;
        // particle_b.pos += normal * (dist - self.link.target_distance) * 0.5;
        let diff = (dist - self.link.target_distance)
            / (dist * (particle_a.inv_mass + particle_b.inv_mass));
        particle_a.pos -= particle_a.inv_mass * dist_vec * diff * 0.5;
        particle_b.pos += particle_b.inv_mass * dist_vec * diff * 0.5;
    }
}

#[derive(Debug, Copy, Clone)]
pub struct CircleLink {
    pub link: Link,
}

impl CircleLink {
    pub fn solve(&mut self, circles: &mut Vec<Circle>) {
        let split = circles.split_at_mut(self.link.particle_b);
        let circle_a = &mut split.0[self.link.particle_a];
        let circle_b = &mut split.1[0];
        let dist_vec = circle_a.point.pos - circle_b.point.pos;
        let dist = dist_vec.magnitude();
        let normal = dist_vec.normalize();
        let c_a_rad_sqr = circle_a.radius * circle_a.radius;
        let c_b_rad_sqr = circle_b.radius * circle_b.radius;
        let scale = 1.0 / (c_a_rad_sqr + c_b_rad_sqr);
        circle_a.point.pos -= normal * (dist - self.link.target_distance) * scale * c_b_rad_sqr;
        circle_b.point.pos += normal * (dist - self.link.target_distance) * scale * c_a_rad_sqr;
    }
}
