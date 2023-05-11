use crate::common::{aabb_vs_aabb, line_intersection, proj_a_on_b, proj_point_on_line};
use crate::link::{Link, ParticleLink};
use crate::particle::Particle;
use crate::solver::Bounds;
use crate::spring::Spring;
use nalgebra::{clamp, Vector2};

// TODO: remove this
#[derive(Debug, Copy, Clone)]
pub struct Collision {
    pub intersection: Vector2<f32>,
    pub point_a: Particle,
    pub point_b: Particle,
    pub point: Particle,
    pub center: Vector2<f32>,
    pub real_intersection: Vector2<f32>,
    pub dist_to_a: f32,
    pub dist_to_b: f32,
    pub dist_a_to_b: f32,
    pub influence_a: f32,
    pub influence_b: f32,
    pub pen_vector: Vector2<f32>,
    pub total_displacement: Vector2<f32>,
    pub displace_point: Vector2<f32>,
    pub displace_line: Vector2<f32>,
    pub displacement_a: Vector2<f32>,
    pub displacement_b: Vector2<f32>,
    pub new_a: Vector2<f32>,
    pub new_b: Vector2<f32>,
    pub new_point: Vector2<f32>,
}

#[derive(Debug, Clone)]
pub struct Polygon {
    pub particles: Vec<Particle>,
    pub particle_links: Vec<ParticleLink>,
    pub particle_springs: Vec<Spring>,
    pub is_static: bool,
    pub center: Vector2<f32>,
    pub scale: f32,
    pub pressure: f32,
    pub volume: f32,
    pub collisions: Vec<Collision>,
    pub bounds: Bounds,
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
            volume: radius * radius * std::f32::consts::PI,
            collisions: vec![],
            bounds: Bounds {
                pos: Vector2::new(0.0, 0.0),
                size: Vector2::new(0.0, 0.0),
            },
        }
    }

    pub fn new_box(pos: Vector2<f32>, angle: f32, size: Vector2<f32>, mass: f32, stiffness: f32, is_static: bool) -> Self {
        let mut particles = Vec::new();
        let mut particle_links = Vec::new();
        let mut particle_springs = Vec::new();
        let mut center = Vector2::new(0.0, 0.0);
        let mass = mass / 4.0;
        let point_count = 4;
        let x = size.x * 0.5;
        let y = size.y * 0.5;
        let mut point1 = Particle::new(pos + Vector2::new(x, y));
        let mut point2 = Particle::new(pos + Vector2::new(x, -y));
        let mut point3 = Particle::new(pos + Vector2::new(-x, -y));
        let mut point4 = Particle::new(pos + Vector2::new(-x, y));
        point1.set_mass(mass);
        point2.set_mass(mass);
        point3.set_mass(mass);
        point4.set_mass(mass);
        particles.push(point1);
        particles.push(point2);
        particles.push(point3);
        particles.push(point4);

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
            volume: mass,
            collisions: vec![],
            bounds: Bounds {
                pos: Vector2::new(0.0, 0.0),
                size: Vector2::new(0.0, 0.0),
            },
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
            volume: 1.0,
            collisions: vec![],
            bounds: Bounds {
                pos: Vector2::new(0.0, 0.0),
                size: Vector2::new(0.0, 0.0),
            },
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

        self.calc_center();
        self.update_bounds();
    }

    pub fn solve_bounds(&mut self, bounds: Bounds) {
        for point in &mut self.particles {
            point.solve_bounds(bounds);
        }
    }

    pub fn update_bounds(&mut self) {
        let mut particle_positions = self.particles.iter().map(|p| p.pos).collect::<Vec<_>>();
        particle_positions.sort_by(|a, b| a.x.partial_cmp(&b.x).unwrap());
        self.bounds.pos.x = particle_positions[0].x;
        self.bounds.size.x = particle_positions[particle_positions.len() - 1].x - self.bounds.pos.x;
        particle_positions.sort_by(|a, b| a.y.partial_cmp(&b.y).unwrap());
        self.bounds.pos.y = particle_positions[0].y;
        self.bounds.size.y = particle_positions[particle_positions.len() - 1].y - self.bounds.pos.y;
    }

    // pub fn solve_static_line(&mut self, line: (Vector2<f32>, Vector2<f32>)) {
    //     for i in 0..self.particles.len() {
    //         let result =
    //             line_intersection(line, (self.particles[i].pos, self.center));
    //
    //         if let Some(intersection) = result {
    //             if let Some(new_point) = self.resolve_static_line_intersection(intersection, i, line) {
    //                 self.particles[i].pos = new_point;
    //             }
    //         }
    //     }
    // }

    pub fn solve_polygon(&mut self, other: &mut Polygon) {
        if aabb_vs_aabb(self.bounds, other.bounds) {
            self.solve_polygon_single(other);
            other.solve_polygon_single(self);
            // This might not need to happen every aabb_vs_aabb check but it's fine for now
            self.update_bounds();
            other.update_bounds();
        }
    }

    pub fn solve_polygon_single(&mut self, other: &mut Polygon) {
        // We need to check if any of the points of the other polygon are inside this polygon
        // If we find one, project it on all the lines of this polygon and pick the closest one
        // Then resolve the collision
        // TODO: Debug this
        for others_point in &mut other.particles {
            let mut intersction_count = 0;
            for a_id in 0..self.particles.len() {
                let point_a = self.particles[a_id];
                let b_id = (a_id + 1) % self.particles.len();
                let point_b = self.particles[b_id];
                let result =
                    line_intersection((point_a.pos, point_b.pos), (others_point.pos, self.bounds.pos - Vector2::new(1.0, 1.0)));

                if result != None {
                    intersction_count += 1;
                }
            }

            if intersction_count % 2 == 1 {
                let mut closest_point: Option<(Vector2<f32>, Vector2<usize>)>= None;
                for a_id in 0..self.particles.len() {
                    let point_a = self.particles[a_id];
                    let b_id = (a_id + 1) % self.particles.len();
                    let point_b = self.particles[b_id];
                    let result = proj_point_on_line(others_point.pos, (point_a.pos, point_b.pos));

                    if let Some(closest) = closest_point {
                        if (result - others_point.pos).magnitude() < (closest.0 - others_point.pos).magnitude() {
                            closest_point = Some((result, Vector2::new(a_id, b_id)));
                        }
                    } else {
                        closest_point = Some((result, Vector2::new(a_id, b_id)));
                    }
                }
                if let Some(closest) = closest_point {
                    self.resolve_line_intersection(closest.0, closest.1.x, closest.1.y, others_point);
                }
            }
        }

        // for a_id in 0..self.particles.len() {
        //     let point_a = self.particles[a_id];
        //     let b_id = (a_id + 1) % self.particles.len();
        //     let point_b = self.particles[b_id];
        //     for others_point in &mut other.particles {
        //         let result =
        //             line_intersection((point_a.pos, point_b.pos), (others_point.pos, other.center));
        //
        //         if let Some(intersection) = result {
        //             self.resolve_line_intersection(intersection, a_id, b_id, others_point);
        //         }
        //     }
        // }
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

        let real_intersection = intersection;
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
        // The impulse
        let total_displacement =
            pen_vector / (others_point.inv_mass + point_a.inv_mass + point_b.inv_mass);
        // A third of the displacement
        let displace_point = total_displacement * (point_a.inv_mass + point_b.inv_mass);
        // The displacement total for the line
        let displace_line = -total_displacement * others_point.inv_mass;

        // // TEST!
        let qp_delta = -displace_point;
        let c_delta = (influence_a * influence_a + influence_b * influence_b);
        let delta_squared = qp_delta.dot(&qp_delta);
        let bottom = c_delta * delta_squared;
        if bottom == 0.0 {
            return;
        }
        let lambda = delta_squared / bottom;

        // let a_proportion = point_a.inv_mass / (point_a.inv_mass + point_b.inv_mass);
        // let b_proportion = point_b.inv_mass / (point_a.inv_mass + point_b.inv_mass);

        // The displacement for a and b, 0.5 because the mass is shared between them
        let displacement_a = lambda * influence_a * qp_delta * 0.5; //influence_a * displace_line;
        let displacement_b = lambda * influence_b * qp_delta * 0.5; //influence_b * displace_line;
                                                                    // The new positions
        let new_a = point_a.pos + displacement_a;
        let new_b = point_b.pos + displacement_b;
        let new_point = others_point.pos + displace_point;
        self.collisions.push(Collision {
            intersection,
            point_a: point_a.clone(),
            point_b: point_b.clone(),
            point: others_point.clone(),
            center: self.center,
            real_intersection,
            dist_to_a,
            dist_to_b,
            dist_a_to_b,
            influence_a,
            influence_b,
            pen_vector,
            total_displacement,
            displace_point,
            displace_line,
            displacement_a,
            displacement_b,
            new_a,
            new_b,
            new_point,
        });
        {
            // I think that the problem is that the friction is based on the direction of the point to the center
            // With friction, according to the definition from the paper we should somehow get the penetration
            // vector based on the previous position and the current position, instead of the current position and the center
            // TODO: implement a new collision detection algorithm that projects the penetration vector onto the closest line

            others_point.pos = new_point;
            // let vel_on_line = proj_a_on_b(vel_after, line_normalized);
            // let friction = clamp(
            //     pen_mag * others_point.friction,
            //     0.0,
            //     vel_on_line.magnitude(),
            // );
            // let friction_vec = vel_after * friction;
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

    // fn resolve_static_line_intersection(
    //     &self,
    //     intersection: Vector2<f32>,
    //     point_id: usize,
    //     line: (Vector2<f32>, Vector2<f32>),
    // ) -> Option<Vector2<f32>> {
    //     let (p1, p2) = line;
    //
    //     // The line normal
    //     let line_normalized = (p2 - p1).normalize();
    //     // The center point projected onto the line
    //     let center_proj = (line_normalized.dot(&(self.center - intersection))
    //         / line_normalized.dot(&line_normalized))
    //         * line_normalized;
    //     // The normal, inward facing from the line
    //     let normal_in = (self.center - (intersection + center_proj)).normalize();
    //     let point = &self.particles[point_id];
    //     // The new position for the point
    //     let intersection_result = line_intersection(
    //         (p1, p2),
    //         (point.pos, point.pos - normal_in * 10000.0),
    //     );
    //     if let Some(new_point) = intersection_result {
    //         {
    //             new_point;
    //         }
    //     }
    //     None
    // }

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

    pub fn gravity(&mut self, gravity_x: f32, gravity_y: f32) {
        for point in &mut self.particles {
            point.gravity(gravity_x, gravity_y);
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
