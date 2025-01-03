// src/models/missile.rs



/// ミサイルの構造体
#[derive(Debug, Clone, PartialEq)]
pub struct Missile {
    pub id: String,
    pub position: [f64; 3], // [x, y, z] 座標
    pub velocity: [f64; 3], // [vx, vy, vz] 速度
    pub pitch: f64,         // ピッチ角（度）
    pub mass: f64,          // 質量（kg）
}

/// ミサイルのパラメータ構造体
#[derive(Debug, Clone, PartialEq)]
pub struct MissileParameters {
    pub thrust: [f64; 3],            // 推進力 [Fx, Fy, Fz]
    pub drag_coefficient: f64,       // 空気抵抗係数
    pub area: f64,                   // 空気抵抗面積（m²）
    pub fuel_consumption_rate: f64,  // 燃料消費率（kg/s）
    pub mass_initial: f64,           // 追加
}

/// 空気抵抗力を計算する純粋関数
///
/// # 引数
/// - `velocity`: ミサイルの速度ベクトル [vx, vy, vz]
/// - `air_density`: 大気密度（kg/m³）
/// - `drag_coefficient`: 空気抵抗係数
/// - `area`: 空気抵抗面積（m²）
///
/// # 戻り値
/// - 空気抵抗力ベクトル [Fx, Fy, Fz]
pub fn calculate_drag_force(
    velocity: &[f64; 3],
    air_density: f64,
    drag_coefficient: f64,
    area: f64,
) -> [f64; 3] {
    let speed = (velocity[0].powi(2) + velocity[1].powi(2) + velocity[2].powi(2)).sqrt();
    if speed == 0.0 {
        return [0.0, 0.0, 0.0];
    }
    let drag_magnitude = 0.5 * air_density * speed.powi(2) * drag_coefficient * area;
    [
        -drag_magnitude * (velocity[0] / speed),
        -drag_magnitude * (velocity[1] / speed),
        -drag_magnitude * (velocity[2] / speed),
    ]
}

/// 推進力を計算する純粋関数
///
/// # 引数
/// - `thrust`: 推進力ベクトル [Fx, Fy, Fz]
///
/// # 戻り値
/// - 推進力ベクトル [Fx, Fy, Fz]
pub fn calculate_thrust(thrust: &[f64; 3]) -> [f64; 3] {
    [thrust[0], thrust[1], thrust[2]]
}

/// 合計力を計算する純粋関数
///
/// # 引数
/// - `thrust`: 推進力ベクトル [Fx, Fy, Fz]
/// - `drag`: 空気抵抗力ベクトル [Fx, Fy, Fz]
/// - `gravity_force`: 重力力ベクトル [Fx, Fy, Fz]
///
/// # 戻り値
/// - 合計力ベクトル [Fx, Fy, Fz]
pub fn calculate_net_force(
    thrust: &[f64; 3],
    drag: &[f64; 3],
    gravity_force: &[f64; 3],
) -> [f64; 3] {
    [
        thrust[0] + drag[0] + gravity_force[0],
        thrust[1] + drag[1] + gravity_force[1],
        thrust[2] + drag[2] + gravity_force[2],
    ]
}

/// 加速度を計算する純粋関数
///
/// # 引数
/// - `net_force`: 合計力ベクトル [Fx, Fy, Fz]
/// - `mass`: ミサイルの質量（kg）
///
/// # 戻り値
/// - 加速度ベクトル [ax, ay, az]
pub fn calculate_acceleration(net_force: &[f64; 3], mass: f64) -> [f64; 3] {
    [
        net_force[0] / mass,
        net_force[1] / mass,
        net_force[2] / mass,
    ]
}

/// 速度を更新する純粋関数
///
/// # 引数
/// - `current_velocity`: 現在の速度ベクトル [vx, vy, vz]
/// - `acceleration`: 加速度ベクトル [ax, ay, az]
/// - `dt`: 時間ステップ（秒）
///
/// # 戻り値
/// - 更新後の速度ベクトル [vx, vy, vz]
pub fn _update_velocity(
    current_velocity: &[f64; 3],
    acceleration: &[f64; 3],
    dt: f64,
) -> [f64; 3] {
    [
        current_velocity[0] + acceleration[0] * dt,
        current_velocity[1] + acceleration[1] * dt,
        current_velocity[2] + acceleration[2] * dt,
    ]
}

/// 位置を更新する純粋関数
///
/// # 引数
/// - `current_position`: 現在の位置ベクトル [x, y, z]
/// - `velocity`: 速度ベクトル [vx, vy, vz]
/// - `dt`: 時間ステップ（秒）
///
/// # 戻り値
/// - 更新後の位置ベクトル [x, y, z]
pub fn update_position(
    current_position: &[f64; 3],
    velocity: &[f64; 3],
    dt: f64,
) -> [f64; 3] {
    [
        current_position[0] + velocity[0] * dt,
        current_position[1] + velocity[1] * dt,
        current_position[2] + velocity[2] * dt,
    ]
}

/// ピッチ角を更新する純粋関数（簡略化）
///
/// # 引数
/// - `current_pitch`: 現在のピッチ角（度）
/// - `new_pitch`: 新しいピッチ角（度）
///
/// # 戻り値
/// - 更新後のピッチ角（度）
pub fn update_pitch(_current_pitch: f64, new_pitch: f64) -> f64 {
    new_pitch // 実際のロジックに基づいて計算することが望ましい
}

/// テスト
#[cfg(test)]
mod tests {
    use crate::Missile;
    use crate::config::MissileParameters;
    use crate::math::{AdamsBashforth2State, LowPassFilterState};
    use crate::simulation::SimulationState;
    use crate::models::motion::update_missiles;

    #[test]
    fn test_update_missiles() {
        let missile_params = MissileParameters {
            thrust: [5000.0, 0.0, 0.0],
            drag_coefficient: 0.3,
            area: 1.0,
            fuel_consumption_rate: 10.0, // kg/s
            mass_initial: 5000.0, // 追加
        };

        let gravity = [0.0, 0.0, -9.81];
        let dt = 0.1;

        let missile = Missile {
            id: "missile1".to_string(),
            position: [0.0, 0.0, 0.0],
            velocity: [100.0, 0.0, 50.0],
            pitch: 45.0,
            mass: 5000.0,
        };

        let integrator = AdamsBashforth2State { prev_f: None };
        let filter = LowPassFilterState { previous: 0.0 };

        let state = SimulationState {
            missiles: vec![missile.clone()],
            radars: vec![],
            interceptors: vec![],
            integrators: vec![integrator.clone()],
            filters: vec![filter.clone()],
            interceptor_filters: vec![],
        };

        let updated_state = update_missiles(&state, &missile_params, gravity, dt).unwrap();

        // ミサイルの数が1であることを確認
        assert_eq!(updated_state.0.len(), 1);
        // 積分器の数が1であることを確認
        assert_eq!(updated_state.1.len(), 1);
        // フィルタの数が1であることを確認
        assert_eq!(updated_state.2.len(), 1);

        // 具体的な値の検証（ここでは簡略化）
        let updated_missile = &updated_state.0[0];
        assert!(updated_missile.mass < missile.mass); // 燃料が減少していること
    }
}
