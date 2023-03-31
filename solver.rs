use vector2d::Vector2D;
use crate::physics::particle::Particle;
use crate::physics::circle::Circle;

pub struct Solver {
    pub gravity: Vector2D<f32>,
    pub bounds: Bounds,
    pub bounds_active: bool,
    particles: Vec<Particle>,
    circles: Vec<Circle>,
    sub_steps: u16,
    sub_steps_multiplier: f32,
}

impl Solver {
    pub fn new() -> Self {
        Self {
            gravity: Vector2D::new(0.0, 98.2),
            particles: Vec::new(),
            circles: Vec::new(),
            bounds: Bounds {
                pos: Vector2D { x: 0.0, y: 0.0 },
                size: Vector2D { x: 100.0, y: 100.0 },
            },
            bounds_active: true,
            sub_steps: 1,
            sub_steps_multiplier: 0.0,
        }
    }

    pub fn add_particle(&mut self, pos: &Vector2D<f32>) {
        self.particles.push(Particle::new(pos));
    }

    pub fn add_circle(&mut self, circle: Circle) {
        self.circles.push(circle);
    }

    pub fn get_particles(&mut self) -> &mut Vec<Particle> {
        &mut self.particles
    }

    pub fn get_circles(&mut self) -> &mut Vec<Circle> {
        &mut self.circles
    }

    pub fn update(&mut self, dt: &f32) {
        self.sub_steps_multiplier = 1.0 / self.sub_steps as f32;
        let delta = *dt * self.sub_steps_multiplier;
        for i in 0..self.sub_steps {
            self.apply_gravity(&delta);
            self.solve_boundary_collisions();
            self.update_positions(&delta);
        }
    }

    fn update_positions(&mut self, dt: &f32) {
        for particle in self.particles.iter_mut() {
            particle.update(dt);
        }
        for circle in self.circles.iter_mut() {
            circle.point.update(dt);
        }
    }

    fn apply_gravity(&mut self, dt: &f32) {
        for particle in self.particles.iter_mut() {
            particle.add_force_v2(&(self.gravity * *dt));
        }
        for circle in self.circles.iter_mut() {
            circle.point.add_force_v2(&(self.gravity * *dt));
        }
    }

    fn solve_boundary_collisions(&mut self) {
        for particle in self.particles.iter_mut() {
            particle_bounds(particle, &self.bounds);
        }
        for circle in self.circles.iter_mut() {
            circle.solve_bounds(&self.bounds);
        }
    }

    // TODO: Fix brute force
    fn solve_dynamic_collisions(&mut self) {
        /*let length = self.circles.len();
        for i in 0..length {
            let circle_i = &mut self.circles[i];
            for j in i..length {
                let circle_j = &mut self.circles[j];
                circle_circle(circle_i, circle_j);
            }
        }*/
    }
}

pub struct Bounds {
    pub pos: Vector2D<f32>,
    pub size: Vector2D<f32>,
}

// Collision resolvers

fn particle_bounds(point: &mut Particle, bounds: &Bounds) {
    if point.pos.x < bounds.pos.x {
        point.pos.x = bounds.pos.x;
    } else if point.pos.x > bounds.pos.x + bounds.size.x {
        point.pos.x = bounds.pos.x + bounds.size.x;
    }
    if point.pos.y < bounds.pos.y {
        point.pos.y = bounds.pos.y;
    } else if point.pos.y > bounds.pos.y + bounds.size.y {
        point.pos.y = bounds.pos.y + bounds.size.y;
    }
}

fn circle_bounds(circle: &mut Circle, bounds: &Bounds) {
    if circle.point.pos.x < bounds.pos.x + circle.radius {
        circle.point.pos.x = bounds.pos.x + circle.radius;
    } else if circle.point.pos.x > bounds.pos.x + bounds.size.x - circle.radius {
        circle.point.pos.x = bounds.pos.x + bounds.size.x - circle.radius;
    }
    if circle.point.pos.y < bounds.pos.y + circle.radius {
        circle.point.pos.y = bounds.pos.y + circle.radius;
    } else if circle.point.pos.y > bounds.pos.y + bounds.size.y - circle.radius {
        circle.point.pos.y = bounds.pos.y + bounds.size.y - circle.radius;
    }
}

fn circle_circle(circle1: &mut Circle, circle2: &mut Circle) {

}