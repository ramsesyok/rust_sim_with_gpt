use serde::Deserialize;
use serde_derive::Deserialize;
use std::fs::File;
use std::io::Read;
use std::error::Error;

// =======================
// Missile params
// =======================
#[derive(Debug, Deserialize, Clone)]
pub struct MissileParams {
    pub alpha: f64,
    pub cd: f64,
    pub area: f64,
    pub rho0: f64,
    pub h: f64,
    pub g: f64,
    pub alpha_filter: f64,
}

#[derive(Debug, Deserialize, Clone)]
pub struct MissileState {
    pub mass: f64,
    pub thrust: f64,
    pub theta: f64,
    pub psi: f64,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
}

// =======================
// Radar params
// =======================
#[derive(Debug, Deserialize, Clone)]
pub struct RadarParams {
    pub position: [f64; 3],
    pub direction: [f64; 3],
    pub range: f64,
    pub azimuth_range: f64,
    pub elevation_range: f64,
    pub period: f64,
}

// =======================
// Interceptor params
// =======================
#[derive(Debug, Deserialize, Clone)]
pub struct GuidanceConstants {
    pub n: f64,
}

#[derive(Debug, Deserialize, Clone)]
pub struct InterceptorParams {
    pub alpha: f64,
    pub cd: f64,
    pub area: f64,
    pub g: f64,
    pub thrust: f64,
    pub alpha_filter: f64,
    pub guidance_constants: GuidanceConstants,
}

#[derive(Debug, Deserialize, Clone)]
pub struct InterceptorState {
    pub mass: f64,
    pub thrust: f64,
    pub theta: f64,
    pub psi: f64,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub launched: bool,
}

// =======================
// シナリオ
// =======================
#[derive(Debug, Deserialize, Clone)]
pub struct Scenario {
    pub initial_conditions: InitialConditions,
}

#[derive(Debug, Deserialize, Clone)]
pub struct InitialConditions {
    pub missiles: Vec<MissileState>,
    pub interceptors: Vec<InterceptorState>,
}

// =======================
// YAML パース共通関数
// =======================
pub fn parse_yaml<T: for<'de> Deserialize<'de>>(path: &str) -> Result<T, Box<dyn Error>> {
    let mut file = File::open(path)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    let data: T = serde_yaml::from_str(&contents)?;
    Ok(data)
}