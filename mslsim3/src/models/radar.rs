use serde_derive::Deserialize;
use crate::models::missile::MissileState;

/// レーダのパラメータ
#[derive(Clone, Debug, Deserialize)]
pub struct RadarParams {
    pub position: [f64; 3],
    pub direction: [f64; 3],
    pub range: f64,
    pub azimuth_range: f64,    // [deg]
    pub elevation_range: f64,  // [deg]
    pub period: f64,
}

/// レーダ本体 (パラメータのみ)
#[derive(Clone, Debug)]
pub struct Radar {
    pub params: RadarParams,
}

/// 検出結果
#[derive(Clone, Debug)]
pub struct DetectionResult {
    pub detected: bool,
    pub missile_position: Option<[f64; 3]>,
    pub missile_orientation: Option<[f64; 3]>, // [theta, psi, phi]
    pub detection_position: Option<[f64; 3]>,
}

fn vector_sub(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
}

fn vector_norm(v: &[f64; 3]) -> f64 {
    (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt()
}

/// atan2 のゼロ近傍をチェックする関数
fn is_atan2_near_zero(x: f64, y: f64, eps: f64) -> bool {
    x.abs() < eps && y.abs() < eps
}

/// ミサイル検出判定
pub fn detect_missile(radar: &Radar, missile: &MissileState, eps: f64) -> DetectionResult {
    // 距離判定
    let diff = vector_sub(&missile.position, &radar.params.position);
    let dist = vector_norm(&diff);
    if dist > radar.params.range {
        return DetectionResult {
            detected: false,
            missile_position: None,
            missile_orientation: None,
            detection_position: None,
        };
    }

    // 角度判定 (超簡易バージョン: ここでは厳密な方位角差や仰角差は省略し、
    // レーダの direction と ミサイル方向の内積から cosθ をとるなどしても良い)
    // 例: ここでは direction=[dx, dy, dz] が正面と仮定し、
    //     diff=[mx, my, mz] とレーダ正面との角度を見る
    let radar_norm = vector_norm(&radar.params.direction);
    if radar_norm < 1e-9 {
        // レーダ方向が無効
        return DetectionResult {
            detected: false,
            missile_position: None,
            missile_orientation: None,
            detection_position: None,
        };
    }

    // atan2 のゼロ近傍チェック
    if is_atan2_near_zero(radar.params.direction[0], radar.params.direction[1], eps) &&
       is_atan2_near_zero(diff[0], diff[1], eps)
    {
        return DetectionResult {
            detected: false,
            missile_position: None,
            missile_orientation: None,
            detection_position: None,
        };
    }

    // ここでは「全部OKだったら検出成功」とする
    // 実際は方位角(psi)の差や仰角(theta)の差を計算して判定してください
    DetectionResult {
        detected: true,
        missile_position: Some(missile.position),
        missile_orientation: Some([missile.theta, missile.psi, 0.0]),
        detection_position: Some(radar.params.position),
    }
}

/// レーダが「発射指示」を出すかどうか
pub fn generate_fire_command(result: &DetectionResult) -> bool {
    result.detected
}