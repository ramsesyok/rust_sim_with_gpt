// src/math/error.rs

use thiserror::Error;

#[derive(Error, Debug)]
pub enum MathError {
    #[error("atan2 の入力がゼロに近すぎます。")]
    Atan2ZeroInput,
    // 他の数値計算エラーを追加可能
}
