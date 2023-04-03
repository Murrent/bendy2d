use crate::physics::solver::Bounds;
use vector2d::Vector2D;

#[derive(Debug, Copy, Clone)]
pub struct Particle {
    pub pos: Vector2D<f32>,
    pub prev_pos: Vector2D<f32>,
    acc: Vector2D<f32>,
}

impl Particle {
    pub fn new(pos: &Vector2D<f32>) -> Self {
        Self {
            pos: *pos,
            prev_pos: *pos,
            acc: Vector2D::new(0.0, 0.0),
        }
    }

    pub fn update(&mut self, dt: &f32) {
        let vel = self.pos - self.prev_pos;
        self.prev_pos = self.pos;
        self.pos = self.pos + vel + self.acc * *dt * *dt;
        self.acc = Vector2D { x: 0.0, y: 0.0 };
    }

    pub fn particle_bounds(&mut self, bounds: &Bounds) {
        if self.pos.x < bounds.pos.x {
            self.pos.x = bounds.pos.x;
        } else if self.pos.x > bounds.pos.x + bounds.size.x {
            self.pos.x = bounds.pos.x + bounds.size.x;
        }
        if self.pos.y < bounds.pos.y {
            self.pos.y = bounds.pos.y;
        } else if self.pos.y > bounds.pos.y + bounds.size.y {
            self.pos.y = bounds.pos.y + bounds.size.y;
        }
    }

    pub fn add_force(&mut self, force_x: &f32, force_y: &f32) {
        self.acc += Vector2D {
            x: *force_x,
            y: *force_y,
        };
    }

    pub fn add_force_v2(&mut self, force: &Vector2D<f32>) {
        self.acc += *force;
    }

    pub fn add_force_towards(&mut self, point: &Vector2D<f32>, force: &f32) {
        let dist: Vector2D<f32> = (*point - self.pos).normalise();
        self.acc += dist * *force;
    }
}
