use vector2d::Vector2D;
use crate::circle::Circle;
use crate::particle::Particle;
use crate::polygon::Polygon;

#[derive(Debug, Copy, Clone)]
pub struct Link {
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
        let dist = dist_vec.length();
        let normal = dist_vec.normalise();
        particle_a.pos -= normal * (dist - self.link.target_distance) * 0.5;
        particle_b.pos += normal * (dist - self.link.target_distance) * 0.5;
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
        let dist = dist_vec.length();
        let normal = dist_vec.normalise();
        let c_a_rad_sqr = circle_a.radius * circle_a.radius;
        let c_b_rad_sqr = circle_b.radius * circle_b.radius;
        let scale = 1.0 / (c_a_rad_sqr + c_b_rad_sqr);
        circle_a.point.pos -= normal * (dist - self.link.target_distance) * scale * c_b_rad_sqr;
        circle_b.point.pos += normal * (dist - self.link.target_distance) * scale * c_a_rad_sqr;
    }
}

#[derive(Debug, Copy, Clone)]
pub struct PolygonLink {
    pub anchor: Vector2D<f32>,
    pub particle: usize,
    pub target_angle: f32,
    pub target_distance: f32,
}

impl PolygonLink {
    pub fn solve(&self, particle: &mut Particle, angle: f32, scale: f32) {
        let x = f32::cos(self.target_angle + angle) * self.target_distance * scale;
        let y = f32::sin(self.target_angle + angle) * self.target_distance * scale;
        let dist_vec = particle.pos - (self.anchor + Vector2D::new(x, y));
        let dist = dist_vec.length();
        let normal = dist_vec.normalise();
        particle.pos -= normal * dist;// * 0.5;
        //point_b.pos += normal * (dist - self.link.target_distance) * 0.5;
    }
}