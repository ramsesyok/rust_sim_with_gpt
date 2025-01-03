// src/math/filter.rs

/// 一階ローパスフィルタの状態
#[derive(Debug, Clone, PartialEq)]
pub struct LowPassFilterState {
    pub previous: f64,
}

/// 一階ローパスフィルタ
///
/// # 引数
/// - `state`: 現在のフィルタの状態
/// - `input`: 入力値
/// - `alpha`: フィルタ係数
///
/// # 戻り値
/// - 更新後のフィルタの状態
/// - フィルタ後の値
pub fn low_pass_filter(
    state: LowPassFilterState,
    input: f64,
    alpha: f64,
) -> (LowPassFilterState, f64) {
    let filtered = alpha * input + (1.0 - alpha) * state.previous;
    let new_state = LowPassFilterState {
        previous: filtered,
    };
    (new_state, filtered)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_low_pass_filter_initial_step() {
        let initial_state = LowPassFilterState { previous: 0.0 };
        let input = 10.0;
        let alpha = 0.5;
        let (new_state, filtered) = low_pass_filter(initial_state.clone(), input, alpha);
        
        let expected_filtered = 0.5 * 10.0 + 0.5 * 0.0; // 5.0
        let expected_state = LowPassFilterState { previous: 5.0 };
        
        assert_eq!(filtered, expected_filtered);
        assert_eq!(new_state, expected_state);
    }

    #[test]
    fn test_low_pass_filter_subsequent_steps() {
        let initial_state = LowPassFilterState { previous: 5.0 };
        let input = 15.0;
        let alpha = 0.3;
        let (new_state, filtered) = low_pass_filter(initial_state.clone(), input, alpha);
        
        let expected_filtered = 0.3 * 15.0 + 0.7 * 5.0; // 4.5 + 3.5 = 8.0
        let expected_state = LowPassFilterState { previous: 8.0 };
        
        assert_eq!(filtered, expected_filtered);
        assert_eq!(new_state, expected_state);
    }

    #[test]
    fn test_low_pass_filter_zero_alpha() {
        let initial_state = LowPassFilterState { previous: 2.0 };
        let input = 10.0;
        let alpha = 0.0;
        let (new_state, filtered) = low_pass_filter(initial_state.clone(), input, alpha);
        
        let expected_filtered = 0.0 * 10.0 + 1.0 * 2.0; // 2.0
        let expected_state = LowPassFilterState { previous: 2.0 };
        
        assert_eq!(filtered, expected_filtered);
        assert_eq!(new_state, expected_state);
    }

    #[test]
    fn test_low_pass_filter_full_alpha() {
        let initial_state = LowPassFilterState { previous: 3.0 };
        let input = 7.0;
        let alpha = 1.0;
        let (new_state, filtered) = low_pass_filter(initial_state.clone(), input, alpha);
        
        let expected_filtered = 1.0 * 7.0 + 0.0 * 3.0; // 7.0
        let expected_state = LowPassFilterState { previous: 7.0 };
        
        assert_eq!(filtered, expected_filtered);
        assert_eq!(new_state, expected_state);
    }
}
