// src/models/interceptor.rs

use crate::math::error::MathError;

/// 迎撃ミサイルの構造体
#[derive(Debug, Clone, PartialEq)]
pub struct Interceptor {
    pub id: String,
    pub position: [f64; 3], // [x, y, z] 座標
    pub velocity: [f64; 3], // [vx, vy, vz] 速度
    pub pitch: f64,         // ピッチ角（度）
    pub mass: f64,          // 質量（kg）
}

/// 迎撃ミサイルの状態を更新する純粋な関数
///
/// # 引数
/// - `interceptor`: 現在の迎撃ミサイルのデータ
/// - `target_position`: ターゲットミサイルの現在位置
/// - `target_velocity`: ターゲットミサイルの現在速度
/// - `navigation_coefficient`: 比例航法係数
/// - `dt`: 時間ステップ
///
/// # 戻り値
/// - 更新後の迎撃ミサイルのデータ
pub fn update_interceptor(
    interceptor: &Interceptor,
    target_position: &[f64; 3],
    target_velocity: &[f64; 3],
    navigation_coefficient: f64,
    dt: f64,
) -> Result<Interceptor, MathError> {
    // 相対位置と相対速度の計算
    let rel_position = [
        target_position[0] - interceptor.position[0],
        target_position[1] - interceptor.position[1],
        target_position[2] - interceptor.position[2],
    ];
    let rel_velocity = [
        target_velocity[0] - interceptor.velocity[0],
        target_velocity[1] - interceptor.velocity[1],
        target_velocity[2] - interceptor.velocity[2],
    ];

    let distance = (rel_position[0].powi(2) + rel_position[1].powi(2) + rel_position[2].powi(2)).sqrt();
    if distance < 1e-6 {
        return Err(MathError::Atan2ZeroInput);
    }

    // 誘導加速度の計算（比例航法）
    let a_c = [
        navigation_coefficient * rel_velocity[0] / distance,
        navigation_coefficient * rel_velocity[1] / distance,
        navigation_coefficient * rel_velocity[2] / distance,
    ];

    // 新しい速度の計算
    let new_velocity = [
        interceptor.velocity[0] + a_c[0] * dt,
        interceptor.velocity[1] + a_c[1] * dt,
        interceptor.velocity[2] + a_c[2] * dt,
    ];

    // 新しい位置の計算
    let new_position = [
        interceptor.position[0] + new_velocity[0] * dt,
        interceptor.position[1] + new_velocity[1] * dt,
        interceptor.position[2] + new_velocity[2] * dt,
    ];

    // ピッチ角の更新（簡略化）
    let new_pitch = interceptor.pitch; // 実際のピッチ角更新は推進力や重力に基づく計算が必要

    Ok(Interceptor {
        id: interceptor.id.clone(),
        position: new_position,
        velocity: new_velocity,
        pitch: new_pitch,
        mass: interceptor.mass, // 質量変化があれば更新
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_update_interceptor_success() {
        let interceptor = Interceptor {
            id: "interceptor1".to_string(),
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            pitch: 0.0,
            mass: 2000.0,
        };

        let target_position = [100.0, 0.0, 0.0];
        let target_velocity = [10.0, 0.0, 0.0];
        let navigation_coefficient = 3.0;
        let dt = 0.1;

        let updated = update_interceptor(&interceptor, &target_position, &target_velocity, navigation_coefficient, dt).unwrap();

        // 相対位置: [100, 0, 0]
        // 相対速度: [10, 0, 0]
        // distance = 100
        // a_c = [3.0 * 10 / 100, 0.0, 0.0] = [0.3, 0.0, 0.0]
        // new_velocity = [0 + 0.3 * 0.1, 0 + 0 * 0.1, 0 + 0 * 0.1] = [0.03, 0.0, 0.0]
        // new_position = [0 + 0.03 * 0.1, 0 + 0 * 0.1, 0 + 0 * 0.1] = [0.003, 0.0, 0.0]

        assert_eq!(updated.id, "interceptor1");
        assert!((updated.position[0] - 0.003).abs() < 1e-6);
        assert_eq!(updated.position[1], 0.0);
        assert_eq!(updated.position[2], 0.0);
        assert_eq!(updated.velocity, [0.03, 0.0, 0.0]);
        assert_eq!(updated.pitch, 0.0);
        assert_eq!(updated.mass, 2000.0);
    }

    #[test]
    fn test_update_interceptor_zero_distance() {
        let interceptor = Interceptor {
            id: "interceptor1".to_string(),
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            pitch: 0.0,
            mass: 2000.0,
        };

        let target_position = [0.0, 0.0, 0.0];
        let target_velocity = [0.0, 0.0, 0.0];
        let navigation_coefficient = 3.0;
        let dt = 0.1;

        let result = update_interceptor(&interceptor, &target_position, &target_velocity, navigation_coefficient, dt);

        assert!(result.is_err());
        match result {
            Err(MathError::Atan2ZeroInput) => (),
            _ => panic!("Expected MathError::Atan2ZeroInput"),
        }
    }
}
