// src/models/motion.rs

use std::error::Error;
use crate::config::MissileParameters;
use crate::math::{adams_bashforth_2, AdamsBashforth2State, low_pass_filter, LowPassFilterState};
use crate::{Missile, Interceptor,Radar};
use crate::models::radar::detect;
use crate::simulation::SimulationState;
use crate::config::parameters::InterceptorParameters;

/// ミサイルの更新処理
pub fn update_missiles(
    state: &SimulationState,
    missile_params: &MissileParameters,
    gravity: [f64; 3],
    dt: f64,
) -> Result<(Vec<Missile>, Vec<AdamsBashforth2State>, Vec<LowPassFilterState>), Box<dyn Error>> {
    let (missiles, integrators, filters) = state
        .missiles
        .iter()
        .zip(state.integrators.iter())
        .zip(state.filters.iter())
        .map(|((missile, integrator), filter)| {
            // 高度に依存する大気密度の計算（簡略化）
            let altitude = missile.position[2].max(0.0);
            let air_density = standard_atmosphere_density(altitude);

            // 空気抵抗力の計算
            let drag = crate::models::missile::calculate_drag_force(
                &missile.velocity,
                air_density,
                missile_params.drag_coefficient,
                missile_params.area,
            );

            // 推進力の計算
            let thrust = crate::models::missile::calculate_thrust(&missile_params.thrust);

            // 重力力の計算
            let gravity_force = [
                0.0,
                0.0,
                missile.mass * gravity[2],
            ];

            // 合計力の計算
            let net_force = crate::models::missile::calculate_net_force(&thrust, &drag, &gravity_force);

            // 加速度の計算
            let acceleration = crate::models::missile::calculate_acceleration(&net_force, missile.mass);

            // Adams-Bashforth 2段法による積分
            let (new_integrator, new_velocity_component) =
                match adams_bashforth_2(integrator.clone(), missile.velocity[0], acceleration[0]) {
                    Ok(result) => result,
                    Err(_) => (integrator.clone(), missile.velocity[0]),
                };

            // ローパスフィルタの適用
            let (new_filter, filtered_velocity) =
                low_pass_filter(filter.clone(), new_velocity_component, 0.5);

            // 新しい速度の計算
            let new_velocity = [
                filtered_velocity,
                missile.velocity[1], // Y軸も同様に更新する場合、別途計算が必要
                missile.velocity[2], // Z軸も同様に更新する場合、別途計算が必要
            ];

            // 新しい位置の計算
            let new_position = crate::models::missile::update_position(&missile.position, &new_velocity, dt);

            // ピッチ角の更新（簡略化）
            let new_pitch = crate::models::missile::update_pitch(missile.pitch, missile.pitch); // 実際のピッチ角更新は推進力や重力に基づく計算が必要

            (
                Missile {
                    id: missile.id.clone(),
                    position: new_position,
                    velocity: new_velocity,
                    pitch: new_pitch,
                    mass: missile.mass - missile_params.fuel_consumption_rate * dt,
                },
                new_integrator,
                new_filter,
            )
        })
        .fold(
            (Vec::new(), Vec::new(), Vec::new()),
            |(mut missiles, mut integrators, mut filters), (m, i, f)| {
                missiles.push(m);
                integrators.push(i);
                filters.push(f);
                (missiles, integrators, filters)
            },
        );

    Ok((missiles, integrators, filters))
}

/// 迎撃ミサイルの更新処理
pub fn update_interceptors(
    state: &SimulationState,
    interceptor_params: &InterceptorParameters,
    dt: f64,
) -> Result<(Vec<Interceptor>, Vec<LowPassFilterState>), Box<dyn Error>> {
    let (interceptors, interceptor_filters) = state
        .interceptors
        .iter()
        .zip(state.interceptor_filters.iter())
        .map(|(interceptor, filter)| {
            // ターゲットミサイルの選定（例として最初のミサイルをターゲット）
            if let Some(target) = state.missiles.first() {
                match crate::models::interceptor::update_interceptor(
                    interceptor,
                    &target.position,
                    &target.velocity,
                    interceptor_params.navigation_coefficient,
                    dt,
                ) {
                    Ok(updated_interceptor) => {
                        // ローパスフィルタの適用
                        let (new_filter, _) =
                            low_pass_filter(filter.clone(), updated_interceptor.velocity[0], 0.5);
                        (updated_interceptor, new_filter)
                    }
                    Err(_) => (interceptor.clone(), filter.clone()),
                }
            } else {
                (interceptor.clone(), filter.clone())
            }
        })
        .fold(
            (Vec::new(), Vec::new()),
            |(mut interceptors, mut interceptor_filters), (m, f)| {
                interceptors.push(m);
                interceptor_filters.push(f);
                (interceptors, interceptor_filters)
            },
        );

    Ok((interceptors, interceptor_filters))
}

/// レーダーによる全探知処理
pub fn detect_all_radars(
    radars: &Vec<Radar>,
    missiles: &Vec<Missile>,
) -> Vec<(bool, [f64; 3])> {
    radars
        .iter()
        .map(|radar| {
            let detection = missiles.iter().any(|missile| detect(radar, missile));
            if detection {
                let detected_missile = missiles.iter().find(|m| detect(radar, m)).unwrap();
                (true, detected_missile.position)
            } else {
                (false, [0.0, 0.0, 0.0])
            }
        })
        .collect()
}

/// 標準大気モデルによる高度に依存する大気密度の計算（簡略化）
pub fn standard_atmosphere_density(altitude: f64) -> f64 {
    // 簡易的なモデル（実際の標準大気モデルを適用することを推奨）
    if altitude < 10000.0 {
        1.225 * (-0.00011856 * altitude + 1.0).exp()
    } else {
        0.0
    }
}
