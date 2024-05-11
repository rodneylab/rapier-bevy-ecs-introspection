use bevy_ecs::component::Component;
use macroquad::color::Color;
use rapier2d::{
    dynamics::RigidBodyHandle,
    na::{Complex, DVector, Isometry, Unit, Vector2},
};
use uom::si::f32::{Length, Velocity as VelocityUnit};

#[derive(Component, Debug, Default)]
pub struct Position(pub Vector2<Length>);

#[derive(Component, Debug, Default)]
pub struct Velocity(pub Vector2<VelocityUnit>);

#[derive(Component, Debug, Default)]
pub struct CircleCollider {
    pub radius: Length,
    pub physics_handle: Option<RigidBodyHandle>,
}

#[derive(Component, Debug, Default)]
pub struct CircleMesh {
    pub radius: Length,
    pub colour: Color,
}

#[derive(Component, Debug)]
pub struct HeightFieldCollider {
    pub heights: DVector<f32>,
    pub scale: Vector2<f32>,
    pub translation: Vector2<f32>,
}

#[derive(Component, Debug)]
pub struct CuboidCollider {
    pub half_extents: Vector2<f32>,
    pub position: Isometry<f32, Unit<Complex<f32>>, 2>,
    pub translation: Vector2<f32>,
}

#[derive(Component, Debug)]
pub struct Sensor;
