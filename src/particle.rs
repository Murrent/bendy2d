use crate::solver::Bounds;
use nalgebra::{clamp, Vector2};

#[derive(Debug, Copy, Clone)]
pub struct Particle {
    pub pos: Vector2<f32>,
    pub prev_pos: Vector2<f32>,
    pub friction: f32,
    acc: Vector2<f32>,
}

impl Particle {
    pub fn new(pos: Vector2<f32>) -> Self {
        Self {
            pos,
            prev_pos: pos,
            friction: 1.0,
            acc: Vector2::new(0.0, 0.0),
        }
    }

    pub fn update(&mut self, dt: f32) {
        let vel = self.pos - self.prev_pos;
        self.prev_pos = self.pos;
        self.pos = self.pos + vel + self.acc * dt * dt;
        self.acc = Vector2::new(0.0, 0.0);
    }

    pub fn solve_bounds(&mut self, bounds: Bounds) {
        if self.pos.x < bounds.pos.x {
            let pen_x = (bounds.pos.x - self.pos.x).abs();
            self.pos.x = bounds.pos.x;
            let vel_y = self.prev_pos.y - self.pos.y;
            self.pos.y += clamp(pen_x * self.friction, 0.0, vel_y);
        } else if self.pos.x > bounds.pos.x + bounds.size.x {
            let pen_x = (self.pos.x - (bounds.pos.x + bounds.size.x)).abs();
            self.pos.x = bounds.pos.x + bounds.size.x;
            let vel_y = self.prev_pos.y - self.pos.y;
            self.pos.y += clamp(pen_x * self.friction, 0.0, vel_y);
        }
        if self.pos.y < bounds.pos.y {
            let pen_y = (bounds.pos.y - self.pos.y).abs();
            self.pos.y = bounds.pos.y;
            let vel_x = self.prev_pos.x - self.pos.x;
            self.pos.x += clamp(pen_y * self.friction, 0.0, vel_x);
        } else if self.pos.y > bounds.pos.y + bounds.size.y {
            let pen_y = (self.pos.y - (bounds.pos.y + bounds.size.y)).abs();
            self.pos.y = bounds.pos.y + bounds.size.y;
            let vel_x = self.prev_pos.x - self.pos.x;
            self.pos.x += clamp(pen_y * self.friction, 0.0, vel_x);
        }
    }

    pub fn add_force(&mut self, force_x: f32, force_y: f32) {
        self.acc += Vector2::new(force_x, force_y);
    }

    pub fn add_force_v2(&mut self, force: Vector2<f32>) {
        self.acc += force;
    }

    pub fn add_force_towards(&mut self, point: Vector2<f32>, force: f32) {
        let dist: Vector2<f32> = (point - self.pos).normalize();
        self.acc += dist * force;
    }
}
