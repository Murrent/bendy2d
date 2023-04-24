use crate::common::{line_intersection, proj_a_on_b};
use crate::link::{Link, ParticleLink};
use crate::particle::Particle;
use crate::solver::Bounds;
use crate::spring::Spring;
use nalgebra::{clamp, Vector2};

#[derive(Debug, Clone)]
pub struct Polygon {
    pub particles: Vec<Particle>,
    pub particle_links: Vec<ParticleLink>,
    pub particle_springs: Vec<Spring>,
    pub is_static: bool,
    pub center: Vector2<f32>,
    pub scale: f32,
    pub pressure: f32,
}

impl Polygon {
    pub fn circle(
        radius: f32,
        pos: Vector2<f32>,
        point_count: usize,
        is_static: bool,
        stiffness: f32,
    ) -> Self {
        let mut particles = Vec::new();
        let mut particle_links = Vec::new();
        let mut particle_springs = Vec::new();
        let mut center = Vector2::new(0.0, 0.0);
        let mut angle = 0.0;
        for _ in 0..point_count {
            let x = radius * f32::cos(angle);
            let y = radius * f32::sin(angle);
            let mut point = Particle::new(pos + Vector2::new(x, y));
            point.set_mass(radius * radius * std::f32::consts::PI / point_count as f32);
            particles.push(point);
            center += point.pos;
            angle += 2.0 * std::f32::consts::PI / point_count as f32;
        }
        center /= point_count as f32;

        for i in 0..point_count {
            let mut a_id = i;
            if point_count > 3 {
                let mut b_id = (i + point_count / 2) % point_count;
                // Makes sure that a_id is always lower than b_id
                if a_id > b_id {
                    let temp = a_id;
                    a_id = b_id;
                    b_id = temp;
                }
                let particle_a = &particles[a_id];
                let particle_b = &particles[b_id];
                let dist_vec = particle_a.pos - particle_b.pos;
                particle_springs.push(Spring {
                    particle_a: a_id,
                    particle_b: b_id,
                    rest_length: dist_vec.magnitude(),
                    stiffness,
                });
            }
            a_id = i;
            let mut c_id = (i + 1) % point_count;
            if a_id > c_id {
                let temp = a_id;
                a_id = c_id;
                c_id = temp;
            }
            let particle_a = &particles[a_id];
            let particle_c = &particles[c_id];
            let dist_vec = particle_a.pos - particle_c.pos;
            particle_springs.push(Spring {
                particle_a: a_id,
                particle_b: c_id,
                rest_length: dist_vec.magnitude(),
                stiffness,
            });
        }

        Self {
            particles,
            particle_links,
            particle_springs,
            is_static,
            center,
            scale: 1.0,
            pressure: 1.0,
        }
    }

    pub fn new(points: Vec<Vector2<f32>>, is_static: bool) -> Self {
        let mut particles = Vec::new();
        let mut particle_links = Vec::new();
        let mut particle_springs = Vec::new();
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
            particle_springs.push(Spring {
                particle_a: a_id,
                particle_b: b_id,
                rest_length: dist_vec.magnitude(),
                stiffness: 5.0,
            });
        }

        Self {
            particles,
            particle_links,
            particle_springs,
            is_static,
            center,
            scale: 1.0,
            pressure: 1.0,
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
        for a_id in 0..self.particles.len() {
            let point_a = self.particles[a_id];
            let b_id = (a_id + 1) % self.particles.len();
            let point_b = self.particles[b_id];
            for others_point in &mut other.particles {
                let result =
                    line_intersection((point_a.pos, point_b.pos), (others_point.pos, other.center));
                if let Some(intersection) = result {
                    self.resolve_line_intersection(intersection, a_id, b_id, others_point);
                }
            }
        }
    }

    pub fn resolve_line_intersection(
        &mut self,
        intersection: Vector2<f32>,
        a_id: usize,
        b_id: usize,
        others_point: &mut Particle,
    ) {
        let point_a = &self.particles[a_id];
        let point_b = &self.particles[b_id];

        // The line normal
        let line_normalized = (point_b.pos - point_a.pos).normalize();
        // The center point projected onto the line
        let center_proj = (line_normalized.dot(&(self.center - intersection))
            / line_normalized.dot(&line_normalized))
            * line_normalized;
        // The normal, inward facing from the line
        let normal_in = (self.center - (intersection + center_proj)).normalize();
        // We project the current position of the point onto the normal
        let point_proj = (line_normalized.dot(&(others_point.pos - intersection))
            / line_normalized.dot(&line_normalized))
            * line_normalized;
        let real_intersection = intersection + point_proj;
        // We calculate the distance from the intersection point to a and b
        let dist_to_a = (real_intersection - point_a.pos).magnitude();
        let dist_to_b = (real_intersection - point_b.pos).magnitude();
        // The length of the line we intersect
        let dist_a_to_b = dist_to_a + dist_to_b;
        // The influence ratio of a and b
        let influence_a = dist_to_b / dist_a_to_b;
        let influence_b = dist_to_a / dist_a_to_b;
        // The penetration vector, between intersection and point
        let pen_vector = real_intersection - others_point.pos;
        // pen_vector projected onto the normal
        let pen_on_normal = (normal_in.dot(&pen_vector) / normal_in.dot(&normal_in)) * normal_in;
        // The impulse
        let impulse = pen_on_normal / (others_point.inv_mass + point_a.inv_mass + point_b.inv_mass);
        // A third of the displacement
        let displace_point = impulse * others_point.inv_mass;
        // The displacement total for the line
        let displace_line = impulse * (point_a.inv_mass + point_b.inv_mass);
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
            // let vel_after = others_point.prev_pos - others_point.pos;
            // let pen_on_line = proj_a_on_b(vel_after, line_normalized);
            // let pen_mag = pen_on_normal.magnitude();
            {
                // I think that the problem is that the friction is based on the direction of the point to the center
                // With friction, according to the definition from the paper we should somehow get the penetration
                // vector based on the previous position and the current position, instead of the current position and the center
                // TODO: implement a new collision detection algorithm that projects the penetration vector onto the closest line

                others_point.pos = new_point;
                // let vel_on_line = proj_a_on_b(vel_after, line_normalized);
                // let friction = clamp(pen_mag * others_point.friction, 0.0, vel_on_line.magnitude());
                // let friction_vec = pen_on_line * friction;
                // others_point.pos += friction_vec;
            }
            {
                let particle_a = &mut self.particles[a_id];
                particle_a.pos = new_a;
            }
            {
                let particle_b = &mut self.particles[b_id];
                particle_b.pos = new_b;
            }
        }
    }

    pub fn solve_links(&mut self) {
        self.calc_center();
        for link in &mut self.particle_links {
            link.solve(&mut self.particles);
        }
    }

    pub fn solve_springs(&mut self, dt: f32) {
        for spring in &mut self.particle_springs {
            spring.solve(&mut self.particles, dt);
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
