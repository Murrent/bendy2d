use nalgebra::Vector2;

// help me make an inline function that returns an option of whether two lines intersect
// and if they do, return the point of intersection
// Make line_intersection and inline function
#[inline]
pub fn line_intersection(line1: (Vector2<f32>, Vector2<f32>), line2: (Vector2<f32>, Vector2<f32>)) -> Option<Vector2<f32>> {
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