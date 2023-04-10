use nalgebra::Vector2;
use crate::link::{Link, ParticleLink, PolygonLink};
use crate::particle::Particle;
use crate::solver::Bounds;
use crate::common::line_intersection;

#[derive(Debug, Clone)]
pub struct Polygon {
    pub points: Vec<Particle>,
    pub polygon_links: Vec<PolygonLink>,
    pub particle_links: Vec<ParticleLink>,
    pub is_static: bool,
    pub center: Vector2<f32>,
    pub scale: f32,
}

impl Polygon {
    pub fn circle(radius: f32, pos: Vector2<f32>, point_count: usize, is_static: bool) -> Self {
        let mut particles = Vec::new();
        let mut polygon_links = Vec::new();
        let mut particle_links = Vec::new();
        let mut center = Vector2::new(0.0, 0.0);
        let mut angle = 0.0;
        for _ in 0..point_count {
            let x = radius * f32::cos(angle);
            let y = radius * f32::sin(angle);
            let point = Particle::new(pos + Vector2::new(x, y));
            particles.push(point);
            center += point.pos;
            angle += 2.0 * std::f32::consts::PI / point_count as f32;
        }
        center /= point_count as f32;

        for i in 0..point_count {
            polygon_links.push(PolygonLink {
                anchor: center,
                particle: i,
                target_distance: radius,
            });
            {
                let mut a_id = i;
                let mut b_id = (i + 2 * point_count / 3) % point_count;
                // Makes sure that a_id is always lower than b_id
                if a_id > b_id {
                    let temp = a_id;
                    a_id = b_id;
                    b_id = temp;
                }
                let particle_a = &particles[a_id];
                let particle_b = &particles[b_id];
                let dist_vec = particle_a.pos - particle_b.pos;
                particle_links.push(ParticleLink {
                    link: Link {
                        particle_a: a_id,
                        particle_b: b_id,
                        target_distance: dist_vec.magnitude(),
                    },
                });
            }
            {
                let mut a_id = i;
                let mut b_id = (i + point_count / 3) % point_count;
                // Makes sure that a_id is always lower than b_id
                if a_id > b_id {
                    let temp = a_id;
                    a_id = b_id;
                    b_id = temp;
                }
                let particle_a = &particles[a_id];
                let particle_b = &particles[b_id];
                let dist_vec = particle_a.pos - particle_b.pos;
                particle_links.push(ParticleLink {
                    link: Link {
                        particle_a: a_id,
                        particle_b: b_id,
                        target_distance: dist_vec.magnitude(),
                    },
                });
            }
        }

        Self {
            points: particles,
            polygon_links,
            particle_links,
            is_static,
            center,
            scale: 1.0,
        }
    }

    pub fn new(points: Vec<Vector2<f32>>, is_static: bool) -> Self {
        let mut particles = Vec::new();
        let mut polygon_links = Vec::new();
        let mut particle_links = Vec::new();
        let mut center = Vector2::new(0.0, 0.0);
        for point in &points {
            let particle = Particle::new(*point);
            particles.push(particle);
            center += particle.pos;
        }
        center /= points.len() as f32;

        for i in 0..points.len() {
            let point = points[i];
            polygon_links.push(PolygonLink {
                anchor: center,
                particle: i,
                target_distance: (point - center).magnitude(),
            });
            let mut a_id = i;
            let mut b_id = (i + 1) % particles.len();
            // Makes sure that a_id is always lower than b_id
            if a_id > b_id {
                let temp = a_id;
                a_id = b_id;
                b_id = temp;
            }
            let particle_a = &particles[a_id];
            let particle_b = &particles[b_id];
            let dist_vec = particle_a.pos - particle_b.pos;
            particle_links.push(ParticleLink {
                link: Link {
                    particle_a: a_id,
                    particle_b: b_id,
                    target_distance: dist_vec.magnitude(),
                },
            });
        }

        Self {
            points: particles,
            polygon_links,
            particle_links,
            is_static,
            center,
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

    pub fn solve_polygon(&mut self, other: &mut Polygon) {
        self.solve_polygon_single(other);
        other.solve_polygon_single(self);
    }

    pub fn solve_polygon_single(&mut self, other: &mut Polygon) {
        for i in 0..self.points.len() {
            let point_a = self.points[i];
            let b_id = (i + 1) % self.points.len();
            let point_b = self.points[b_id];
            for others_point in &mut other.points {
                let result = line_intersection((point_a.pos, point_b.pos), (others_point.pos, other.center));
                if let Some(intersection) = result {
                    let dist_to_a = (intersection - point_a.pos).magnitude();
                    let dist_to_b = (intersection - point_b.pos).magnitude();
                    let dist_to_intersection = (intersection - others_point.pos).magnitude();
                    let dist_a_b = dist_to_a + dist_to_b;
                    let force_a_b = 2.0 * dist_to_intersection / 3.0;
                    let force_others = dist_to_intersection / 3.0;
                    let force_a = force_a_b * dist_to_b / dist_a_b;
                    let force_b = force_a_b * dist_to_a / dist_a_b;
                    let normal_a_b = (point_b.pos - point_a.pos).normalize();
                    let perp_a_b = Vector2::new(-normal_a_b.y, normal_a_b.x);
                    {
                        let mut point_a = &mut self.points[i];
                        point_a.pos += perp_a_b * force_a;
                    }
                    {
                        let mut point_b = &mut self.points[b_id];
                        point_b.pos += perp_a_b * force_b;
                    }
                    {
                        let mut others_point = others_point;
                        others_point.pos += (intersection - others_point.pos).normalize() * force_others;
                    }
                }
            }
        }
    }

    pub fn solve_links(&mut self) {
        self.calc_center();
        for link in &mut self.polygon_links {
            link.anchor = self.center;
            //link.solve(&mut self.points[link.particle]);
        }
        for link in &mut self.particle_links {
            link.solve(&mut self.points);
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