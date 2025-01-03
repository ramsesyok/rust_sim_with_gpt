/// 1次遅れローパスフィルタ
#[derive(Clone, Debug)]
pub struct LowPassFilter {
    filtered: f64,
    alpha: f64,
}

impl LowPassFilter {
    pub fn new(alpha: f64) -> Self {
        LowPassFilter {
            filtered: 0.0,
            alpha,
        }
    }

    /// 入力値 input に対して、y_filtered(t) = alpha * input + (1-alpha) * y_filtered(t-1)
    pub fn apply(&mut self, input: f64) -> f64 {
        self.filtered = self.alpha * input + (1.0 - self.alpha) * self.filtered;
        self.filtered
    }
}