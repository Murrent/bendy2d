use nalgebra::Vector2;
use crate::link::{Link, ParticleLink, PolygonLink};
use crate::particle::Particle;
use crate::solver::Bounds;

#[derive(Debug, Clone)]
pub struct Polygon {
    pub points: Vec<Particle>,
    pub links: Vec<PolygonLink>,
    pub is_static: bool,
    pub center: Vector2<f32>,
    pub original_shape: Vec<Vector2<f32>>,
    pub start_angles: Vec<f32>,
    pub rotation: f32,
    pub desired_rotation: f32,
    pub scale: f32,
}

impl Polygon {
    pub fn circle(radius: f32, pos: Vector2<f32>, point_count: usize, is_static: bool) -> Self {
        let mut points = Vec::new();
        let mut links = Vec::new();
        let mut original_shape = Vec::new();
        let mut start_angles = Vec::new();
        let mut center = Vector2::new(0.0, 0.0);
        let mut angle = 0.0;
        for _ in 0..point_count {
            let x = radius * f32::cos(angle);
            let y = radius * f32::sin(angle);
            let point = Particle::new(pos + Vector2::new(x, y));
            points.push(point);
            original_shape.push(Vector2::new(x, y));
            start_angles.push(angle);
            center += point.pos;
            angle += 2.0 * std::f32::consts::PI / point_count as f32;
        }
        center /= point_count as f32;

        for i in 0..point_count {
            links.push(PolygonLink {
                anchor: center,
                particle: i,
                target_angle: start_angles[i],
                target_distance: radius,
            });
        }

        Self {
            points,
            links,
            is_static,
            center,
            original_shape,
            start_angles,
            rotation: 0.0,
            desired_rotation: 0.0,
            scale: 1.0,
        }
    }

    pub fn new(points: Vec<Vector2<f32>>, is_static: bool) -> Self {
        let mut particles = Vec::new();
        let mut links = Vec::new();
        let mut original_shape = Vec::new();
        let mut start_angles = Vec::new();
        let mut center = Vector2::new(0.0, 0.0);
        for point in &points {
            let particle = Particle::new(*point);
            particles.push(particle);
            original_shape.push(*point);
            center += particle.pos;
        }
        center /= points.len() as f32;

        for i in 0..points.len() {
            let point = points[i];
            let normal = (point - center).normalize();
            let angle = normal.y.atan2(normal.x);
            start_angles.push(angle);
            links.push(PolygonLink {
                anchor: center,
                particle: i,
                target_angle: angle,
                target_distance: (point - center).magnitude(),
            });
        }

        Self {
            points: particles,
            links,
            is_static,
            center,
            original_shape,
            start_angles,
            rotation: 0.0,
            desired_rotation: 0.0,
            scale: 1.0,
        }
    }

    pub fn update(&mut self, dt: f32) {
        if self.is_static {
            return;
        }
        for point in &mut self.points {
            point.update(dt);
        }

        self.center = Vector2::new(0.0, 0.0);
        for point in &self.points {
            self.center += point.pos;
        }
        self.center /= self.points.len() as f32;
        //
        // self.rotation = 0.0;
        // let mut total_angle = 0.0;
        // for i in 0..self.points.len() {
        //     let point = &self.points[i];
        //     //let original_angle = self.start_angles[i];
        //     // TODO: make this angle from atan2
        //     let normal = (point.pos - self.center).normalize();
        //     let angle = normal.y.atan2(normal.x);
        //     total_angle += angle;
        // }
        // self.rotation = total_angle / self.points.len() as f32;

    }

    pub fn solve_bounds(&mut self, bounds: Bounds) {
        for point in &mut self.points {
            point.solve_bounds(bounds);
        }
    }

    pub fn solve_links(&mut self) {
        for link in &mut self.links {
            link.anchor = self.center;
            link.solve(&mut self.points[link.particle], self.rotation, self.scale);
        }
    }

    pub fn add_force_v2(&mut self, force: Vector2<f32>) {
        for point in &mut self.points {
            point.add_force_v2(force);
        }
    }

    fn calc_center(&mut self) {
        self.center = Vector2::new(0.0, 0.0);
        for point in &self.points {
            self.center += point.pos;
        }
        self.center /= self.points.len() as f32;
    }
}