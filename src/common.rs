use crate::solver::Bounds;
use nalgebra::Vector2;

#[inline]
pub fn line_intersection(
    line1: (Vector2<f32>, Vector2<f32>),
    line2: (Vector2<f32>, Vector2<f32>),
) -> Option<Vector2<f32>> {
    let (p1, p2) = line1;
    let (p3, p4) = line2;
    let s1_x = p2.x - p1.x;
    let s1_y = p2.y - p1.y;
    let s2_x = p4.x - p3.x;
    let s2_y = p4.y - p3.y;

    let s = (-s1_y * (p1.x - p3.x) + s1_x * (p1.y - p3.y)) / (-s2_x * s1_y + s1_x * s2_y);
    let t = (s2_x * (p1.y - p3.y) - s2_y * (p1.x - p3.x)) / (-s2_x * s1_y + s1_x * s2_y);

    if s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0 {
        // Collision detected
        let i_x = p1.x + (t * s1_x);
        let i_y = p1.y + (t * s1_y);
        Some(Vector2::new(i_x, i_y))
    } else {
        None
    }
}

#[inline]
pub fn proj_a_on_b(a: Vector2<f32>, b: Vector2<f32>) -> Vector2<f32> {
    (a.dot(&b) / b.magnitude_squared()) * b
}

#[inline]
pub fn proj_a_on_b_clamped(a: Vector2<f32>, b: Vector2<f32>) -> Vector2<f32> {
    let proj = proj_a_on_b(a, b);
    if proj.xy() > b.xy() {
        b
    } else if proj.xy() < Vector2::zeros() {
        Vector2::zeros()
    } else {
        proj
    }
}

#[inline]
pub fn proj_point_on_line(point: Vector2<f32>, line: (Vector2<f32>, Vector2<f32>)) -> Vector2<f32> {
    let (l_start, l_end) = line;
    let line_vec = l_end - l_start;
    let point_vec = point - l_start;
    let proj = proj_a_on_b_clamped(point_vec, line_vec);
    l_start + proj
}

#[inline]
pub fn is_point_in_triangle(
    point: Vector2<f32>,
    triangle: (Vector2<f32>, Vector2<f32>, Vector2<f32>),
) -> bool {
    let (a, b, c) = triangle;
    let v0 = c - a;
    let v1 = b - a;
    let v2 = point - a;

    let dot00 = v0.dot(&v0);
    let dot01 = v0.dot(&v1);
    let dot02 = v0.dot(&v2);
    let dot11 = v1.dot(&v1);
    let dot12 = v1.dot(&v2);

    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);

    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    (u >= 0.0) && (v >= 0.0) && (u + v < 1.0)
}

#[inline]
pub fn aabb_vs_aabb(bounds1: Bounds, bounds2: Bounds) -> bool {
    bounds1.pos.x < bounds2.pos.x + bounds2.size.x
        && bounds1.pos.x + bounds1.size.x > bounds2.pos.x
        && bounds1.pos.y < bounds2.pos.y + bounds2.size.y
        && bounds1.pos.y + bounds1.size.y > bounds2.pos.y
}
