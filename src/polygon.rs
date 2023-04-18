use crate::common::line_intersection;
use crate::link::{Link, ParticleLink};
use crate::particle::Particle;
use crate::solver::Bounds;
use nalgebra::Vector2;

#[derive(Debug, Clone)]
pub struct Polygon {
    pub particles: Vec<Particle>,
    pub particle_links: Vec<ParticleLink>,
    pub is_static: bool,
    pub center: Vector2<f32>,
    pub scale: f32,
}

impl Polygon {
    pub fn circle(radius: f32, pos: Vector2<f32>, point_count: usize, is_static: bool) -> Self {
        let mut particles = Vec::new();
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
            particles,
            particle_links,
            is_static,
            center,
            scale: 1.0,
        }
    }

    pub fn new(points: Vec<Vector2<f32>>, is_static: bool) -> Self {
        let mut particles = Vec::new();
        let mut particle_links = Vec::new();
        let mut center = Vector2::new(0.0, 0.0);
        for point in &points {
            let particle = Particle::new(*point);
            particles.push(particle);
            center += particle.pos;
        }
        center /= points.len() as f32;

        for i in 0..points.len() {
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
            particles,
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

        self.calc_center();
        for point in &mut self.particles {
            point.update(dt);
        }
    }

    pub fn solve_bounds(&mut self, bounds: Bounds) {
        for point in &mut self.particles {
            point.solve_bounds(bounds);
        }
    }

    pub fn solve_polygon(&mut self, other: &mut Polygon) {
        self.solve_polygon_single(other);
        other.solve_polygon_single(self);
    }

    pub fn solve_polygon_single(&mut self, other: &mut Polygon) {
        for i in 0..self.particles.len() {
            let point_a = self.particles[i];
            let b_id = (i + 1) % self.particles.len();
            let point_b = self.particles[b_id];
            for others_point in &mut other.particles {
                if let Some((new_a, new_b, new_others)) =
                    self.resolve_line_intersection(&point_a, &point_b, others_point, other.center)
                {
                    self.particles[i].pos = new_a;
                    self.particles[b_id].pos = new_b;
                    others_point.pos = new_others;
                }
            }
        }
    }

    fn resolve_line_intersection(
        &mut self,
        point_a: &Particle,
        point_b: &Particle,
        others_point: &mut Particle,
        other_center: Vector2<f32>,
    ) -> Option<(Vector2<f32>, Vector2<f32>, Vector2<f32>)> {
        let result =
            line_intersection((point_a.pos, point_b.pos), (others_point.pos, other_center));
        if let Some(intersection) = result {
            // The line normal
            let normal_line = (point_b.pos - point_a.pos).normalize();
            // The center point projected onto the line
            let center_proj = (normal_line.dot(&(self.center - intersection))
                / normal_line.dot(&normal_line))
                * normal_line;
            // The normal, inward facing from the line
            let normal_in = (self.center - (intersection + center_proj)).normalize();
            // We calculate the distance from the intersection point to a and b
            let dist_to_a = (intersection - point_a.pos).magnitude();
            let dist_to_b = (intersection - point_b.pos).magnitude();
            // The length of the line we intersect
            let dist_a_to_b = dist_to_a + dist_to_b;
            // The influence ratio of a and b
            let influence_a = dist_to_b / dist_a_to_b;
            let influence_b = dist_to_a / dist_a_to_b;
            // The vector between intersection and point
            let diff_int_to_point = intersection - others_point.pos;
            // diff_int_to_point projected onto the normal
            let intersection_on_normal =
                (normal_in.dot(&diff_int_to_point) / normal_in.dot(&normal_in)) * normal_in;
            // A third of the displacement
            let displace_third = intersection_on_normal / 3.0;
            // The displacement total for the line
            let displace_line = displace_third * 2.0;
            // The displacement for a and b
            let displacement_a = influence_a * displace_line;
            let displacement_b = influence_b * displace_line;
            // The new positions
            let new_a = point_a.pos - displacement_a;
            let new_b = point_b.pos - displacement_b;
            // The new position for the point
            let intersection_result = line_intersection(
                (point_a.pos, point_b.pos),
                (others_point.pos, others_point.pos - normal_in * 10000.0),
            );
            if let Some(new_point) = intersection_result {
                return Some((new_a, new_b, new_point));
            }
            return None;
        }
        None
    }

    pub fn solve_links(&mut self) {
        self.calc_center();
        for link in &mut self.particle_links {
            link.solve(&mut self.particles);
        }
    }

    pub fn add_force_v2(&mut self, force: Vector2<f32>) {
        for point in &mut self.particles {
            point.add_force_v2(force);
        }
    }

    fn calc_center(&mut self) {
        self.center = Vector2::new(0.0, 0.0);
        for point in &self.particles {
            self.center += point.pos;
        }
        self.center /= self.particles.len() as f32;
    }
}
