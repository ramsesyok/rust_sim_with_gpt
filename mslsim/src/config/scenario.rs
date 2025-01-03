// src/config/scenario.rs

use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct Scenario {
    pub missiles: Vec<MissileInstance>,
    pub radars: Vec<RadarInstance>,
    pub interceptors: Vec<InterceptorInstance>,
}

#[derive(Debug, Deserialize)]
pub struct MissileInstance {
    pub id: String,
    pub initial_position: [f64; 3],
    pub initial_velocity: [f64; 3],
    pub initial_pitch: f64,
}

#[derive(Debug, Deserialize)]
pub struct RadarInstance {
    pub id: String,
    pub position: [f64; 3],
}

#[derive(Debug, Deserialize)]
pub struct InterceptorInstance {
    pub id: String,
    pub initial_position: [f64; 3],
    pub initial_velocity: [f64; 3],
    pub initial_pitch: f64,
}
