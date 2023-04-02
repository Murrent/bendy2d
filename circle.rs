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
        let dist = self.point.pos - circle.point.pos;
        let dist_sqr = dist.length_squared();
        let radius_sum = self.radius + circle.radius;
        if dist_sqr < radius_sum * radius_sum  {
            let normal = dist.normalise();
            let overlap = radius_sum - dist_sqr.sqrt();
            let self_rad_sqr = self.radius * self.radius;
            let circle_rad_sqr = circle.radius * circle.radius;
            let scale = 1.0 / (self_rad_sqr + circle_rad_sqr);
            self.point.pos += normal * scale  * overlap * circle_rad_sqr;
            circle.point.pos -= normal * scale  * overlap * self_rad_sqr;
        }
    }
}
