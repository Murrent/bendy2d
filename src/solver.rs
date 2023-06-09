use crate::circle::Circle;
use crate::link::{CircleLink, ParticleLink};
use crate::particle::Particle;
use crate::polygon::Polygon;
use nalgebra::Vector2;

pub enum ColliderType {
    Particle,
    Circle,
    Polygon,
}

#[derive(Debug, Copy, Clone)]
pub struct Bounds {
    pub pos: Vector2<f32>,
    pub size: Vector2<f32>,
}

#[derive(Debug, Clone)]
pub struct Solver {
    pub gravity: Vector2<f32>,
    pub bounds: Bounds,
    pub bounds_active: bool,
    particles: Vec<Particle>,
    particle_links: Vec<ParticleLink>,
    circles: Vec<Circle>,
    circle_links: Vec<CircleLink>,
    polygons: Vec<Polygon>,
    sub_steps: u16,
    sub_steps_multiplier: f32,
}

impl Solver {
    pub fn new() -> Self {
        Self {
            gravity: Vector2::new(0.0, 98.2),
            bounds: Bounds {
                pos: Vector2::new(0.0, 0.0),
                size: Vector2::new(100.0, 100.0),
            },
            bounds_active: true,
            particles: Vec::new(),
            particle_links: Vec::new(),
            circles: Vec::new(),
            circle_links: Vec::new(),
            polygons: Vec::new(),
            sub_steps: 1,
            sub_steps_multiplier: 0.0,
        }
    }

    pub fn add_particle(&mut self, pos: Vector2<f32>) {
        self.particles.push(Particle::new(pos));
    }
    pub fn add_circle(&mut self, circle: Circle) {
        self.circles.push(circle);
    }
    pub fn add_polygon(&mut self, polygon: Polygon) {
        self.polygons.push(polygon);
    }

    pub fn add_particle_link(&mut self, link: ParticleLink) {
        self.particle_links.push(link);
    }
    pub fn add_circle_link(&mut self, link: CircleLink) {
        self.circle_links.push(link);
    }

    pub fn get_particle_len(&self) -> usize {
        self.particles.len()
    }
    pub fn get_circles_len(&self) -> usize {
        self.circles.len()
    }
    pub fn get_polygons_len(&self) -> usize {
        self.polygons.len()
    }

    pub fn get_particle_links(&self) -> &Vec<ParticleLink> {
        &self.particle_links
    }
    pub fn get_circle_links(&self) -> &Vec<CircleLink> {
        &self.circle_links
    }

    pub fn get_particles(&self) -> &Vec<Particle> {
        &self.particles
    }
    pub fn get_circles(&self) -> &Vec<Circle> {
        &self.circles
    }
    pub fn get_polygons(&self) -> &Vec<Polygon> {
        &self.polygons
    }

    pub fn get_particle(&self, index: usize) -> Option<&Particle> {
        self.particles.get(index)
    }
    pub fn get_circle(&self, index: usize) -> Option<&Circle> {
        self.circles.get(index)
    }
    pub fn get_polygon(&self, index: usize) -> Option<&Polygon> {
        self.polygons.get(index)
    }

    pub fn update(&mut self, dt: f32) {
        self.sub_steps_multiplier = 1.0 / self.sub_steps as f32;
        let delta = dt * self.sub_steps_multiplier;
        for _ in 0..self.sub_steps {
            self.apply_gravity();
            self.apply_links();
            self.solve_dynamic_collisions();
            self.solve_boundary_collisions();
            self.update_positions(delta);
        }
    }

    fn update_positions(&mut self, dt: f32) {
        for particle in self.particles.iter_mut() {
            particle.update(dt);
        }
        for circle in self.circles.iter_mut() {
            circle.point.update(dt);
        }
        for polygon in self.polygons.iter_mut() {
            polygon.update(dt);
        }
    }

    fn apply_gravity(&mut self) {
        let gravity = self.gravity;
        for particle in self.particles.iter_mut() {
            particle.add_force_v2(gravity);
        }
        for circle in self.circles.iter_mut() {
            circle.point.add_force_v2(gravity);
        }
        for polygon in self.polygons.iter_mut() {
            polygon.add_force_v2(gravity);
        }
    }

    fn apply_links(&mut self) {
        for link in self.particle_links.iter_mut() {
            link.solve(&mut self.particles);
        }
        for link in self.circle_links.iter_mut() {
            link.solve(&mut self.circles);
        }
        for polygon in self.polygons.iter_mut() {
            polygon.solve_links();
        }
    }

    fn solve_boundary_collisions(&mut self) {
        for particle in self.particles.iter_mut() {
            particle.solve_bounds(self.bounds);
        }
        for circle in self.circles.iter_mut() {
            circle.solve_bounds(self.bounds);
        }
        for polygon in self.polygons.iter_mut() {
            polygon.solve_bounds(self.bounds);
        }
    }

    fn solve_dynamic_collisions(&mut self) {
        let length = self.circles.len();
        for i in 0..length {
            let start_index = i + 1;
            let split = self.circles.split_at_mut(start_index);
            let circle_i = &mut split.0[i];
            for j in start_index..length {
                let circle_j = &mut split.1[j - start_index];
                circle_i.solve_circle(circle_j);
            }
        }
        let length = self.polygons.len();
        for i in 0..length {
            let start_index = i + 1;
            let split = self.polygons.split_at_mut(start_index);
            let polygon_i = &mut split.0[i];
            for j in start_index..length {
                let polygon_j = &mut split.1[j - start_index];
                polygon_i.solve_polygon(polygon_j);
            }
        }
    }
}
