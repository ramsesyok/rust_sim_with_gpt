// src/simulation/load_parameters.rs

use std::error::Error;
use std::fs::File;
use serde_yaml::from_reader;

use crate::config::{parameters::MissileParameters, parameters::RadarParameters, parameters::InterceptorParameters, scenario::Scenario};

/// ミサイルパラメータの読み込み
pub fn load_missile_parameters(path: &str) -> Result<MissileParameters, Box<dyn Error>> {
    let file = File::open(path)?;
    let params: MissileParameters = from_reader(file)?;
    Ok(params)
}

/// レーダパラメータの読み込み
pub fn load_radar_parameters(path: &str) -> Result<RadarParameters, Box<dyn Error>> {
    let file = File::open(path)?;
    let params: RadarParameters = from_reader(file)?;
    Ok(params)
}

/// 迎撃ミサイルパラメータの読み込み
pub fn load_interceptor_parameters(path: &str) -> Result<InterceptorParameters, Box<dyn Error>> {
    let file = File::open(path)?;
    let params: InterceptorParameters = from_reader(file)?;
    Ok(params)
}

/// シナリオの読み込み
pub fn load_scenario(path: &str) -> Result<Scenario, Box<dyn Error>> {
    let file = File::open(path)?;
    let scenario: Scenario = from_reader(file)?;
    Ok(scenario)
}
