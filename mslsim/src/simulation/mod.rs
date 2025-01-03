// src/simulation/mod.rs

pub mod load_parameters;
pub mod csv;
pub mod framework;
use crate::{Missile, Radar, Interceptor};
use crate::math::{AdamsBashforth2State, LowPassFilterState};

/// シミュレーションの全体状態を表す構造体
pub struct SimulationState {
    pub missiles: Vec<Missile>,
    pub radars: Vec<Radar>,
    pub interceptors: Vec<Interceptor>,
    pub integrators: Vec<AdamsBashforth2State>,        // 各ミサイルの積分器状態
    pub filters: Vec<LowPassFilterState>,             // 各ミサイルのフィルタ状態
    pub interceptor_filters: Vec<LowPassFilterState>, // 各迎撃ミサイルのフィルタ状態
}


