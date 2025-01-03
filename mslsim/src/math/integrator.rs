// src/math/integrator.rs

use crate::math::error::MathError;

/// Adams-Bashforth 2段法の積分器の状態
#[derive(Debug, Clone, PartialEq)]
pub struct AdamsBashforth2State {
    pub prev_f: Option<f64>,
}

/// Adams-Bashforth 2段法による積分
///
/// # 引数
/// - `state`: 現在の積分器の状態
/// - `current_y`: 現在のyの値
/// - `current_f`: 現在のf(x, y)の値
///
/// # 戻り値
/// - 更新後の積分器の状態
/// - 次のyの値
pub fn adams_bashforth_2(
    state: AdamsBashforth2State,
    current_y: f64,
    current_f: f64,
) -> Result<(AdamsBashforth2State, f64), MathError> {
    match state.prev_f {
        Some(prev_f) => {
            let y_next = current_y + (0.1 / 2.0) * (3.0 * current_f - prev_f);
            let new_state = AdamsBashforth2State {
                prev_f: Some(current_f),
            };
            Ok((new_state, y_next))
        }
        None => {
            // 初回ステップではEuler法で計算
            let y_next = current_y + current_f * 0.1;
            let new_state = AdamsBashforth2State {
                prev_f: Some(current_f),
            };
            Ok((new_state, y_next))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// test_adams_bashforth_2_initial_step 
    /// 初回ステップでは、前の f 値が None であるため、Euler法を使用して y_next を計算します。
    /// 期待される y_next は 0.0 + 2.0 * 0.1 = 0.2 です。
    #[test]
    fn test_adams_bashforth_2_initial_step() {
        let initial_state = AdamsBashforth2State { prev_f: None };
        let current_y = 0.0;
        let current_f = 2.0;
        let result = adams_bashforth_2(initial_state.clone(), current_y, current_f).unwrap();
        
        let expected_state = AdamsBashforth2State { prev_f: Some(2.0) };
        let expected_y_next = 0.0 + 2.0 * 0.1; // Euler法: y_next = y + f * dt = 0 + 2*0.1 = 0.2

        assert_eq!(result.0, expected_state);
        assert!((result.1 - expected_y_next).abs() < 1e-6);
    }

    /// test_adams_bashforth_2_subsequent_step
    /// 2回目以降のステップでは、前の f 値が Some(1.5) であるため、Adams-Bashforth 2段法を使用して y_next を計算します。
    /// 期待される y_next は 0.2 + (0.1 / 2.0) * (3.0 * 2.5 - 1.5) = 0.2 + 0.05 * 6.0 = 0.5 です。
    #[test]
    fn test_adams_bashforth_2_subsequent_step() {
        let initial_state = AdamsBashforth2State { prev_f: Some(1.5) };
        let current_y = 0.2;
        let current_f = 2.5;
        let result = adams_bashforth_2(initial_state.clone(), current_y, current_f).unwrap();
        
        let expected_state = AdamsBashforth2State { prev_f: Some(2.5) };
        let expected_y_next = 0.2 + (0.1 / 2.0) * (3.0 * 2.5 - 1.5); // y_next = 0.2 + 0.05*(7.5 -1.5)=0.2 + 0.05*6=0.2 +0.3=0.5

        assert_eq!(result.0, expected_state);
        assert!((result.1 - expected_y_next).abs() < 1e-6);
    }

    /// test_adams_bashforth_2_error_handling
    /// 現在の実装では特定のエラー条件がないため、正常に動作することを確認します。
    #[test]
    fn test_adams_bashforth_2_error_handling() {
        // 現在の実装では特定のエラー条件がないため、ここではエラーが発生しないことを確認
        let initial_state = AdamsBashforth2State { prev_f: Some(1.0) };
        let current_y = 1.0;
        let current_f = 3.0;
        let result = adams_bashforth_2(initial_state.clone(), current_y, current_f);
        
        assert!(result.is_ok());
    }
}
