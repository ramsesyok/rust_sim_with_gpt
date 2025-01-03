use serde_derive::Deserialize;
use crate::math::integrator::AdamsBashforthIntegrator;
use crate::math::low_pass_filter::LowPassFilter;

/// ミサイルのパラメータ
#[derive(Clone, Debug, Deserialize)]
pub struct MissileParams {
    pub alpha: f64,        // 燃料消費率係数
    pub cd: f64,           // 空気抵抗係数
    pub area: f64,         // 断面積 [m^2]
    pub rho0: f64,         // 大気密度の基準値 [kg/m^3]
    pub h: f64,            // 大気密度のスケール高度 [m]
    pub g: f64,            // 重力加速度 [m/s^2]
    pub alpha_filter: f64, // ローパスフィルタalpha
}

/// ミサイルの動的状態
#[derive(Clone, Debug, Deserialize)]
pub struct MissileState {
    pub mass: f64,
    pub thrust: f64,
    pub theta: f64,
    pub psi: f64,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
}

/// ミサイル本体 (パラメータ & 状態)
#[derive(Clone, Debug)]
pub struct Missile {
    pub params: MissileParams,
    pub state: MissileState,
}

/// ベクトル演算用ヘルパー関数
fn vector_norm(v: &[f64; 3]) -> f64 {
    (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt()
}

/// 加速度計算
fn calculate_acceleration(params: &MissileParams, state: &MissileState) -> [f64; 3] {
    // 大気密度
    let rho = params.rho0 * (-state.position[2] / params.h).exp();

    // 速度ノルム
    let speed = vector_norm(&state.velocity);
    if speed < 1e-9 {
        // 速度がほぼ0なら抗力0
        // (厳密には速度0でも抗力方向は定義できるが、NaN回避のためこうする)
        let gravity = [0.0, 0.0, -params.g * state.mass];
        let thrust_vec = [
            state.thrust * state.theta.cos() * state.psi.cos(),
            state.thrust * state.theta.cos() * state.psi.sin(),
            state.thrust * state.theta.sin(),
        ];
        return [
            thrust_vec[0] + gravity[0],
            thrust_vec[1] + gravity[1],
            thrust_vec[2] + gravity[2],
        ];
    }

    // 抗力
    let drag = 0.5 * rho * params.cd * params.area * speed * speed;
    let drag_vec = [
        -drag * (state.velocity[0] / speed),
        -drag * (state.velocity[1] / speed),
        -drag * (state.velocity[2] / speed),
    ];

    // 重力
    let gravity_vec = [0.0, 0.0, -params.g * state.mass];

    // 推力
    let thrust_vec = [
        state.thrust * state.theta.cos() * state.psi.cos(),
        state.thrust * state.theta.cos() * state.psi.sin(),
        state.thrust * state.theta.sin(),
    ];

    [
        thrust_vec[0] + drag_vec[0] + gravity_vec[0],
        thrust_vec[1] + drag_vec[1] + gravity_vec[1],
        thrust_vec[2] + drag_vec[2] + gravity_vec[2],
    ]
}

/// 質量更新
fn update_mass(params: &MissileParams, state: &MissileState, dt: f64) -> f64 {
    let new_mass = state.mass - params.alpha * state.thrust * dt;
    if new_mass > 0.0 {
        new_mass
    } else {
        0.0
    }
}

/// ミサイルの状態を更新 (Adams-Bashforth 2段法 + ローパスフィルタ)
pub fn update_missile(
    params: &MissileParams,
    state: &MissileState,
    integrators: &mut [AdamsBashforthIntegrator; 3],
    filters: &mut [LowPassFilter; 3],
    dt: f64,
) -> MissileState {
    // 加速度
    let acc = calculate_acceleration(params, state);

    // 速度更新 (AB2段法)
    let mut new_velocity = [0.0; 3];
    for i in 0..3 {
        new_velocity[i] = integrators[i].integrate(acc[i] / state.mass, dt, state.velocity[i]);
    }

    // ローパスフィルタ適用
    for i in 0..3 {
        new_velocity[i] = filters[i].apply(new_velocity[i]);
    }

    // 位置更新
    let new_position = [
        state.position[0] + new_velocity[0] * dt,
        state.position[1] + new_velocity[1] * dt,
        state.position[2] + new_velocity[2] * dt,
    ];

    // 質量更新
    let new_mass = update_mass(params, state, dt);

    // 新しい状態
    MissileState {
        mass: new_mass,
        thrust: state.thrust, // 必要に応じて制御
        theta: state.theta,   // 必要に応じて制御
        psi: state.psi,       // 必要に応じて制御
        position: new_position,
        velocity: new_velocity,
    }
}

/// 衝突判定 (z <= 0)
pub fn check_collision(state: &MissileState) -> bool {
    state.position[2] <= 0.0
}