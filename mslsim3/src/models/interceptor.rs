use serde_derive::Deserialize;
use crate::math::integrator::AdamsBashforthIntegrator;
use crate::math::low_pass_filter::LowPassFilter;
use crate::models::missile::MissileState;

#[derive(Clone, Debug, Deserialize)]
pub struct GuidanceConstants {
    pub n: f64, // 比例航法定数
}

#[derive(Clone, Debug, Deserialize)]
pub struct InterceptorParams {
    pub alpha: f64,
    pub cd: f64,
    pub area: f64,
    pub g: f64,
    pub thrust: f64,
    pub alpha_filter: f64,
    pub guidance_constants: GuidanceConstants,
}

#[derive(Clone, Debug, Deserialize)]
pub struct InterceptorState {
    pub mass: f64,
    pub thrust: f64,
    pub theta: f64,
    pub psi: f64,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub launched: bool,
}

#[derive(Clone, Debug)]
pub struct Interceptor {
    pub params: InterceptorParams,
    pub state: InterceptorState,
}

// ベクトル計算
fn vector_sub(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
}
fn vector_norm(v: &[f64; 3]) -> f64 {
    (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt()
}
fn vector_normalize(v: &[f64; 3]) -> [f64; 3] {
    let n = vector_norm(v);
    if n < 1e-9 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0]/n, v[1]/n, v[2]/n]
    }
}

/// 比例航法 (最簡易版: lambda_dot=0として誘導加速度=0にし、実装サンプル用とする)
fn guidance(_state: &InterceptorState, target_pos: &[f64; 3], constants: &GuidanceConstants) -> [f64; 3] {
    // ここでは、あまり詳しく実装しないサンプル
    // もし本格的にやるなら LOS角速度を計算し a_guidance = N * V_rel * lambda_dot * ...
    let rel = vector_sub(target_pos, &_state.position);
    let rel_u = vector_normalize(&rel);
    // ダミーで少しだけ誘導加速度を加える
    // (N が小さいほど誘導が弱い)
    [
        constants.n * rel_u[0],
        constants.n * rel_u[1],
        constants.n * rel_u[2],
    ]
}

/// 迎撃ミサイルの運動更新
pub fn update_interceptor(
    params: &InterceptorParams,
    state: &InterceptorState,
    target_pos: &[f64; 3],
    integrators: &mut [AdamsBashforthIntegrator; 3],
    filters: &mut [LowPassFilter; 3],
    dt: f64,
) -> InterceptorState {
    if !state.launched {
        return state.clone();
    }

    // 誘導加速度
    let a_guidance = guidance(state, target_pos, &params.guidance_constants);

    // 速度ノルム
    let speed = vector_norm(&state.velocity);

    // 抗力
    let drag = 0.5 * params.cd * params.area * speed * speed;
    let drag_vec = if speed < 1e-9 {
        [0.0, 0.0, 0.0]
    } else {
        [
            -drag * (state.velocity[0] / speed),
            -drag * (state.velocity[1] / speed),
            -drag * (state.velocity[2] / speed),
        ]
    };

    // 重力
    let gravity_vec = [0.0, 0.0, -params.g * state.mass];

    // 推力ベクトル
    let thrust_vec = [
        state.thrust * state.theta.cos() * state.psi.cos(),
        state.thrust * state.theta.cos() * state.psi.sin(),
        state.thrust * state.theta.sin(),
    ];

    // 合力 = 推力 + 抗力 + 重力 + 誘導
    let total_fx = thrust_vec[0] + drag_vec[0] + gravity_vec[0] + a_guidance[0]*state.mass;
    let total_fy = thrust_vec[1] + drag_vec[1] + gravity_vec[1] + a_guidance[1]*state.mass;
    let total_fz = thrust_vec[2] + drag_vec[2] + gravity_vec[2] + a_guidance[2]*state.mass;

    // 加速度
    let ax = total_fx / state.mass;
    let ay = total_fy / state.mass;
    let az = total_fz / state.mass;

    // 速度更新
    let new_vx = integrators[0].integrate(ax, dt, state.velocity[0]);
    let new_vy = integrators[1].integrate(ay, dt, state.velocity[1]);
    let new_vz = integrators[2].integrate(az, dt, state.velocity[2]);

    let mut new_velocity = [new_vx, new_vy, new_vz];
    // ローパス
    for i in 0..3 {
        new_velocity[i] = filters[i].apply(new_velocity[i]);
    }

    // 位置更新
    let new_position = [
        state.position[0] + new_velocity[0] * dt,
        state.position[1] + new_velocity[1] * dt,
        state.position[2] + new_velocity[2] * dt,
    ];

    // 質量更新 (燃料消費)
    // 例:  m' = m - alpha * thrust * dt
    let new_mass = {
        let m = state.mass - params.alpha * state.thrust * dt;
        if m < 0.0 { 0.0 } else { m }
    };

    InterceptorState {
        mass: new_mass,
        thrust: state.thrust,  // 必要に応じて制御
        theta: state.theta,    // 誘導で変化させてもよい
        psi: state.psi,        // 同上
        position: new_position,
        velocity: new_velocity,
        launched: state.launched,
    }
}

/// 迎撃ミサイルを発射状態にする
pub fn launch_interceptor(state: &InterceptorState) -> InterceptorState {
    InterceptorState {
        launched: true,
        ..*state
    }
}

/// 迎撃判定 (ミサイルとの距離が閾値以下であれば迎撃成功)
pub fn check_interception(
    interceptor: &InterceptorState,
    missile: &MissileState,
    intercept_dist: f64
) -> bool {
    let diff = vector_sub(&interceptor.position, &missile.position);
    let dist = vector_norm(&diff);
    dist <= intercept_dist
}