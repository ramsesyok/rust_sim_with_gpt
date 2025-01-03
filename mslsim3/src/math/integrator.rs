/// Adams-Bashforth 2段法によるステートフルな数値積分器
#[derive(Clone, Debug)]
pub struct AdamsBashforthIntegrator {
    previous_f: f64,
}

impl AdamsBashforthIntegrator {
    pub fn new() -> Self {
        AdamsBashforthIntegrator { previous_f: 0.0 }
    }

    /// 現在の微分値 (current_f) と前の微分値 (previous_f) を用いて、
    /// 次の状態 y_{n+1} を返す。
    /// ここでは「y_{n}」は呼び出し側から渡されるので、差分だけを加える設計にする。
    pub fn integrate(&mut self, current_f: f64, dt: f64, y_n: f64) -> f64 {
        let y_next = y_n + (dt / 2.0) * (3.0 * current_f - self.previous_f);
        self.previous_f = current_f;
        y_next
    }
}