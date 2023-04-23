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
