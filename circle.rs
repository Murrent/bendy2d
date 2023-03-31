use crate::physics::particle::Particle;
use crate::physics::solver::Bounds;

#[derive(Debug, Copy, Clone)]
pub struct Circle {
    pub point: Particle,
    pub radius: f32,
}

impl Circle {

    pub fn solve_bounds(&mut self, bounds: &Bounds) {
        if self.point.pos.x < bounds.pos.x + self.radius {
            self.point.pos.x = bounds.pos.x + self.radius;
        } else if self.point.pos.x > bounds.pos.x + bounds.size.x - self.radius {
            self.point.pos.x = bounds.pos.x + bounds.size.x - self.radius;
        }
        if self.point.pos.y < bounds.pos.y + self.radius {
            self.point.pos.y = bounds.pos.y + self.radius;
        } else if self.point.pos.y > bounds.pos.y + bounds.size.y - self.radius {
            self.point.pos.y = bounds.pos.y + bounds.size.y - self.radius;
        }
    }

    pub fn solve_circle(&mut self, circle: &mut Circle) {

    }
}