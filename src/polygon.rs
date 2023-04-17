use crate::common::{line_intersection, proj_point_on_line};
use crate::link::{Link, ParticleLink, PolygonLink};
use crate::particle::Particle;
use crate::solver::Bounds;
use nalgebra::{clamp, min, Vector2};

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

        self.calc_center();
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

    pub fn contains_point(&self, point: Vector2<f32>) -> bool {
        // // TODO: Check if polygon contains point
        // for i in 0..self.points.len() {
        //     let point_a = self.points[i];
        //     let b_id = (i + 1) % self.points.len();
        //     let point_b = self.points[b_id];
        // }
        false
    }

    pub fn solve_polygon_single(&mut self, other: &mut Polygon) {
        for i in 0..self.points.len() {
            let point_a = self.points[i];
            let b_id = (i + 1) % self.points.len();
            let point_b = self.points[b_id];
            //let mut closest_point = Option::<Vector2<f32>>::None;
            for others_point in &mut other.points {
                let result =
                    line_intersection((point_a.pos, point_b.pos), (others_point.pos, other.center));
                //     let new_point = proj_point_on_line(others_point.pos, (point_a.pos, point_b.pos));
                //     if let Some(point) = closest_point {
                //         let dist_old = (point - self.center).magnitude_squared();
                //         let dist_new = (new_point - self.center).magnitude_squared();
                //         if dist_old > dist_new {
                //             closest_point = Some(new_point);
                //         }
                //     }
                // }
                if let Some(intersection) = result {
                    //self.solve_point_line(others_point, intersection);
                    // The velocity of the point that is penetrating the other polygon
                    let vel_others = others_point.pos - others_point.prev_pos;
                    // The velocities of the line endpoints
                    let vel_a = point_a.pos - point_a.prev_pos;
                    let vel_b = point_b.pos - point_b.prev_pos;
                    // The velocity of the line
                    let vel_ab = vel_a + vel_b;
                    // The line normal
                    let normal_line = (point_b.pos - point_a.pos).normalize();
                    // The center point projected onto the line
                    let center_proj = (normal_line.dot(&(self.center - intersection))
                        / normal_line.dot(&normal_line))
                        * normal_line;
                    // The normal, inward facing from the line
                    let normal_in = (self.center - (intersection + center_proj)).normalize();
                    // The velocity of the line projected onto normal_in
                    let vel_ab_in =
                        (normal_in.dot(&vel_ab) / normal_in.dot(&normal_in)) * normal_in;
                    // The velocity of the point projected onto normal_in
                    let vel_others_in =
                        (normal_in.dot(&vel_others) / normal_in.dot(&normal_in)) * normal_in;
                    // Total collision velocity
                    let vel_total = vel_ab_in + vel_others_in;
                    // The velocity along the normal_in axis
                    let normal_vel = normal_in * normal_in.dot(&vel_total);
                    // Velocity distribution
                    let vel_third = normal_vel / 3.0;
                    let vel_line = vel_third * 2.0;
                    // We calculate the distance from the intersection point to a and b
                    let dist_to_a = (intersection - point_a.pos).magnitude();
                    let dist_to_b = (intersection - point_b.pos).magnitude();
                    // The length of the line we intersect
                    let dist_a_to_b = dist_to_a + dist_to_b;
                    // The influence ratio of a and b
                    let influence_a = dist_to_b / dist_a_to_b;
                    let influence_b = dist_to_a / dist_a_to_b;
                    // The new velocity of a and b
                    let new_vel_a = influence_a * vel_line;
                    let new_vel_b = influence_b * vel_line;
                    // The new velocity of point
                    let new_vel_point = vel_third;


                    //let dist_to_a_ratio = vel_line * influence_a;
                    //let dist_to_b_ratio = vel_line * influence_b;

                    // The vector between intersection and point
                    let diff_int_to_point = intersection - others_point.pos;
                    let intersection_on_normal =
                        (normal_in.dot(&diff_int_to_point) / normal_in.dot(&normal_in)) * normal_in;
                    let displace_point = intersection_on_normal / 3.0;
                    let displace_line = displace_point * 2.0;
                    let displacement_a = influence_a * displace_line;
                    let displacement_b = influence_b * displace_line;

                    {
                        let mut point_a = &mut self.points[i];
                        //point_a.prev_pos = point_a.pos + displacement_a;
                        point_a.pos -= displacement_a;
                    }
                    {
                        let mut point_b = &mut self.points[b_id];
                        //point_b.prev_pos = point_b.pos + displacement_b;
                        point_b.pos -= displacement_b;
                    }
                    {
                        let mut others_point = others_point;
                        //others_point.prev_pos = others_point.pos - displace_point;
                        others_point.pos += displace_point; // + diff_third * 2.0;
                    }
                    //------------------------------------------------------------------------------
                    // // We calculate the distance from the distance point to a and b
                    // let dist_to_a = (intersection - point_a.pos).magnitude();
                    // let dist_to_b = (intersection - point_b.pos).magnitude();
                    // // We also calculate the distance from the intersection point and the position of the penetrating point
                    // let diff_to_point = intersection - others_point.pos;
                    // let dist_to_intersection = diff_to_point.magnitude();
                    // // The distance between a and b
                    // let dist_a_b = dist_to_a + dist_to_b;
                    // // The velocity that should be applied to a and b
                    // let vel_a_b = 2.0 * dist_to_intersection / 3.0;
                    // // The velocity that the penetrating point should have
                    // // TODO: make sure that the point only loses velocity in the direction of the contact normal
                    // let vel_others = dist_to_intersection / 3.0;
                    // if dist_a_b == 0.0 {
                    //     continue;
                    // }
                    // let vel_a = vel_a_b * dist_to_b / dist_a_b;
                    // let vel_b = vel_a_b * dist_to_a / dist_a_b;
                    // let normal_a_b = (point_b.pos - point_a.pos).normalize();
                    // if normal_a_b.dot(&normal_a_b) == 0.0 {
                    //     continue;
                    // }
                    // let proj_point_n = (normal_a_b.dot(&diff_to_point) / normal_a_b.dot(&normal_a_b)) * normal_a_b;
                    // //let perp_a_b = Vector2::new(-normal_a_b.y, normal_a_b.x);
                    // let normal_out = (diff_to_point - proj_point_n).normalize();
                    // let reflection = diff_to_point - 2.0 * (diff_to_point.dot(&normal_out)) * normal_out;
                    // {
                    //     let mut point_a = &mut self.points[i];
                    //     let clamped_diff = clamp(normal_out * vel_a, Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0));
                    //     point_a.pos -= clamped_diff;
                    // }
                    // {
                    //     let mut point_b = &mut self.points[b_id];
                    //     let clamped_diff = clamp(normal_out * vel_b, Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0));
                    //     point_b.pos -= clamped_diff;
                    // }
                    // {
                    //     let mut others_point = others_point;
                    //     let clamped_diff = clamp(reflection * vel_others, Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0));
                    //     others_point.pos = intersection + clamped_diff;
                    // }
                }
            }
        }
    }

    // pub fn solve_point_line(&mut self, others_point: &mut Particle) {
    //     // The velocity of the point that is penetrating the other polygon
    //     let vel_others = others_point.pos - others_point.prev_pos;
    //     // The velocities of the line endpoints
    //     let vel_a = point_a.pos - point_a.prev_pos;
    //     let vel_b = point_b.pos - point_b.prev_pos;
    //     // The velocity of the line
    //     let vel_ab = vel_a + vel_b;
    //     // The line normal
    //     let normal_line = (point_b.pos - point_a.pos).normalize();
    //     // The center point projected onto the line
    //     let center_proj = (normal_line.dot(&(self.center - intersection))
    //         / normal_line.dot(&normal_line))
    //         * normal_line;
    //     // The normal, inward facing from the line
    //     let normal_in = (self.center - (intersection + center_proj)).normalize();
    //     // The velocity of the line projected onto normal_in
    //     let vel_ab_in = (normal_in.dot(&vel_ab)
    //         / normal_in.dot(&normal_in))
    //         * normal_in;
    //     // The velocity of the point projected onto normal_in
    //     let vel_others_in = (normal_in.dot(&vel_others)
    //         / normal_in.dot(&normal_in))
    //         * normal_in;
    //     // Total collision velocity
    //     let vel_total = vel_ab_in + vel_others_in;
    //     let vel_third = vel_total / 3.0;
    //     let vel_line = vel_third * 2.0;
    //     // We calculate the distance from the intersection point to a and b
    //     let dist_to_a = (intersection - point_a.pos).magnitude();
    //     let dist_to_b = (intersection - point_b.pos).magnitude();
    //     let dist_a_to_b = dist_to_a + dist_to_b;
    //     let influence_a = dist_to_a / dist_a_to_b;
    //     let influence_b = dist_to_b / dist_a_to_b;
    //     let dist_to_a_ratio = vel_line * influence_a;
    //     let dist_to_b_ratio = vel_line * influence_b;
    //
    //     let diff_to_point = intersection - others_point.pos;
    //     let dist_to_intersection = diff_to_point.magnitude();
    //     let diff_third = diff_to_point / 3.0;
    //     let diff_a = diff_third * influence_b;
    //     let diff_b = diff_third * influence_a;
    //
    //     {
    //         let mut point_a = &mut self.points[i];
    //         point_a.pos -= diff_third;
    //         point_a.prev_pos = point_a.pos + diff_a;
    //     }
    //     {
    //         let mut point_b = &mut self.points[b_id];
    //         point_b.pos -= diff_third;
    //         point_b.prev_pos = point_b.pos + diff_b;
    //     }
    //     {
    //         let mut others_point = others_point;
    //         others_point.pos += diff_third * 2.0;
    //         others_point.prev_pos = others_point.pos + vel_third;
    //     }
    //
    //     // // We calculate the distance from the distance point to a and b
    //     // let dist_to_a = (intersection - point_a.pos).magnitude();
    //     // let dist_to_b = (intersection - point_b.pos).magnitude();
    //     // // We also calculate the distance from the intersection point and the position of the penetrating point
    //     // let diff_to_point = intersection - others_point.pos;
    //     // let dist_to_intersection = diff_to_point.magnitude();
    //     // // The distance between a and b
    //     // let dist_a_b = dist_to_a + dist_to_b;
    //     // // The velocity that should be applied to a and b
    //     // let vel_a_b = 2.0 * dist_to_intersection / 3.0;
    //     // // The velocity that the penetrating point should have
    //     // // TODO: make sure that the point only loses velocity in the direction of the contact normal
    //     // let vel_others = dist_to_intersection / 3.0;
    //     // if dist_a_b == 0.0 {
    //     //     continue;
    //     // }
    //     // let vel_a = vel_a_b * dist_to_b / dist_a_b;
    //     // let vel_b = vel_a_b * dist_to_a / dist_a_b;
    //     // let normal_a_b = (point_b.pos - point_a.pos).normalize();
    //     // if normal_a_b.dot(&normal_a_b) == 0.0 {
    //     //     continue;
    //     // }
    //     // let proj_point_n = (normal_a_b.dot(&diff_to_point) / normal_a_b.dot(&normal_a_b)) * normal_a_b;
    //     // //let perp_a_b = Vector2::new(-normal_a_b.y, normal_a_b.x);
    //     // let normal_out = (diff_to_point - proj_point_n).normalize();
    //     // let reflection = diff_to_point - 2.0 * (diff_to_point.dot(&normal_out)) * normal_out;
    //     // {
    //     //     let mut point_a = &mut self.points[i];
    //     //     let clamped_diff = clamp(normal_out * vel_a, Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0));
    //     //     point_a.pos -= clamped_diff;
    //     // }
    //     // {
    //     //     let mut point_b = &mut self.points[b_id];
    //     //     let clamped_diff = clamp(normal_out * vel_b, Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0));
    //     //     point_b.pos -= clamped_diff;
    //     // }
    //     // {
    //     //     let mut others_point = others_point;
    //     //     let clamped_diff = clamp(reflection * vel_others, Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0));
    //     //     others_point.pos = intersection + clamped_diff;
    //     // }
    // }

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
