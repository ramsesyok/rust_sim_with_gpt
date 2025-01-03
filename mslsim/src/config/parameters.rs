// src/config/parameters.rs

use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub struct MissileParameters {
    pub mass_initial: f64, // 初期質量 (kg)
    pub fuel_consumption_rate: f64, // 燃料消費率 (kg/s)
    pub drag_coefficient: f64, // 抗力係数
    pub area: f64, // 投影面積 (m²)
    pub thrust: [f64; 3], // 推進力ベクトル (N)
}

#[derive(Debug, Deserialize, Clone)]
pub struct RadarParameters {
    pub azimuth_min: f64, // 方位角最小 (度)
    pub azimuth_max: f64, // 方位角最大 (度)
    pub elevation_min: f64, // 仰角最小 (度)
    pub elevation_max: f64, // 仰角最大 (度)
    pub detection_range: f64, // 探知距離 (m)
}

#[derive(Debug, Deserialize, Clone)]
pub struct InterceptorParameters {
    pub mass_initial: f64,                // 初期質量（kg）
    pub navigation_coefficient: f64, // 比例航法係数   
}
