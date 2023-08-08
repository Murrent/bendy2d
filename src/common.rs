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

    let s = (s1_x * (p1.y - p3.y) - s1_y * (p1.x - p3.x)) / (-s2_x * s1_y + s1_x * s2_y);
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
    let scalar_proj = a.dot(&b) / b.magnitude();
    scalar_proj * b.normalize()
}

#[inline]
pub fn proj_a_on_b_clamped(a: Vector2<f32>, b: Vector2<f32>) -> Vector2<f32> {
    let b_magnitude = b.magnitude();
    let scalar_proj = a.dot(&b) / b_magnitude;
    let scalar_proj_clamped = scalar_proj.max(0.0).min(b_magnitude);
    scalar_proj_clamped * b.normalize()
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

#[inline]
pub fn is_point_in_polygon(point: &Vector2<f32>, polygon: &[Vector2<f32>]) -> Option<Vector2<f32>> {
    let mut is_inside = false;
    let mut closest_point = None;
    let mut min_distance = f32::MAX;
    let (mut j, mut i) = (polygon.len() - 1, 0);

    while i < polygon.len() {
        if ((polygon[i].y > point.y) != (polygon[j].y > point.y))
            && (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y)
            / (polygon[j].y - polygon[i].y)
            + polygon[i].x)
        {
            is_inside = !is_inside;
        }

        let curr_point = proj_point_on_line(*point, (polygon[i], polygon[j]));
        let curr_distance = (curr_point - *point).magnitude();
        if curr_distance < min_distance {
            min_distance = curr_distance;
            closest_point = Some(curr_point);
        }

        j = i;
        i += 1;
    }

    if is_inside {
        closest_point
    } else {
        None
    }
}
