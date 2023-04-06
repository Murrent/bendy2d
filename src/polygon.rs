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
