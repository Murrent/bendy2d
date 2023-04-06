use vector2d::Vector2D;
use crate::link::ParticleLink;
use crate::particle::Particle;

pub struct Polygon {
    pub points: Vec<Particle>,
    pub links: Vec<ParticleLink>, // might need to make a new type of link for polygons
    pub is_static: bool,
    pub center: Vector2D<f32>,
    pub original_shape: Vec<Vector2D<f32>>,
    pub rotation: f32,
    pub desired_rotation: f32,
    pub scale: f32,
}

impl Polygon {
    pub fn circle(radius: f32, pos: Vector2D<f32>, point_count: usize, is_static: bool) -> Self {
        let mut points = Vec::new();
        let mut links = Vec::new();
        let mut original_shape = Vec::new();
        let mut center = Vector2D::new(0.0, 0.0);
        let mut angle = 0.0;
        let angle_increment = 2.0 * std::f32::consts::PI / point_count as f32;
        for _ in 0..point_count {
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            let point = Particle::new(pos + Vector2D::new(x, y));
            points.push(point);
            original_shape.push(Vector2D::new(x, y));
            center += point.pos;
            angle += angle_increment;
        }
        center /= point_count as f32;
        for i in 0..point_count {
            let link = ParticleLink::new(&points[i], &points[(i + 1) % point_count], radius * 2.0);
            links.push(link);
        }
        Self {
            points,
            links,
            is_static,
            center,
            original_shape,
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
        self.center = Vector2D::new(0.0, 0.0);
        for point in &self.points {
            self.center += point.pos;
        }
        self.center /= self.points.len() as f32;
        self.rotation += (self.desired_rotation - self.rotation) * 0.1;
        for i in 0..self.points.len() {
            let mut point = self.points[i].pos - self.center;
            point = point.rotate(self.rotation);
            point *= self.scale;
            self.points[i].pos = point + self.center;
        }
    }

    pub fn apply_force(&mut self, force: Vector2D<f32>) {
        for point in &mut self.points {
            point.apply_force(force);
        }
    }
}