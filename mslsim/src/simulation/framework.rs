// src/simulation/framework.rs

use std::error::Error;

use crate::{Missile, Radar, Interceptor};
use crate::math::{AdamsBashforth2State, LowPassFilterState};
use crate::simulation::SimulationState;
use crate::config::parameters::{MissileParameters, RadarParameters, InterceptorParameters};
use crate::config::scenario::Scenario;

/// シミュレーションステートの初期化
pub fn initialize_simulation_state(
    missile_params: MissileParameters,
    radar_params: RadarParameters,
    interceptor_params: InterceptorParameters,
    scenario: Scenario,
) -> SimulationState {
    // ミサイルの初期化
    let missiles: Vec<Missile> = scenario
        .missiles
        .into_iter()
        .map(|m| Missile {
            id: m.id,
            position: m.initial_position,
            velocity: m.initial_velocity,
            pitch: m.initial_pitch,
            mass: missile_params.mass_initial,
        })
        .collect();

    // レーダの初期化
    let radars: Vec<Radar> = scenario
        .radars
        .into_iter()
        .map(|r| Radar {
            id: r.id,
            position: r.position,
            detection_range: radar_params.detection_range,
            azimuth_min: radar_params.azimuth_min,
            azimuth_max: radar_params.azimuth_max,
            elevation_min: radar_params.elevation_min,
            elevation_max: radar_params.elevation_max,
        })
        .collect();

    // 迎撃ミサイルの初期化
    let interceptors: Vec<Interceptor> = scenario
        .interceptors
        .into_iter()
        .map(|i| Interceptor {
            id: i.id,
            position: i.initial_position,
            velocity: i.initial_velocity,
            pitch: i.initial_pitch,
            mass: interceptor_params.mass_initial,
        })
        .collect();

    // 積分器とフィルタの初期状態
    let integrators: Vec<AdamsBashforth2State> =
        vec![AdamsBashforth2State { prev_f: None }; missiles.len()];
    let filters: Vec<LowPassFilterState> =
        vec![LowPassFilterState { previous: 0.0 }; missiles.len()];
    let interceptor_filters: Vec<LowPassFilterState> =
        vec![LowPassFilterState { previous: 0.0 }; interceptors.len()];

    SimulationState {
        missiles,
        radars,
        interceptors,
        integrators,
        filters,
        interceptor_filters,
    }
}

/// シミュレーションステップの実行
pub fn execute_simulation_step(
    state: &SimulationState,
    missile_params: &MissileParameters,
    interceptor_params: &InterceptorParameters,
    gravity: [f64; 3],
    dt: f64,
) -> Result<SimulationState, Box<dyn Error>> {
    // ミサイルの更新
    let (updated_missiles, updated_integrators, updated_filters) =
        crate::models::motion::update_missiles(state, missile_params, gravity, dt)?;

    // 迎撃ミサイルの更新
    let (updated_interceptors, updated_interceptor_filters) =
        crate::models::motion::update_interceptors(state, interceptor_params, dt)?;

    Ok(SimulationState {
        missiles: updated_missiles,
        radars: state.radars.clone(),
        interceptors: updated_interceptors,
        integrators: updated_integrators,
        filters: updated_filters,
        interceptor_filters: updated_interceptor_filters,
    })
}
