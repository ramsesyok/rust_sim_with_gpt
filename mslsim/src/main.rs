// src/main.rs

use std::error::Error;
use std::io::Write;

use simulation::load_parameters::*;
use simulation::csv::*;
use simulation::framework::*;
use models::missile::Missile;
use models::radar::Radar;
use models::interceptor::Interceptor;


mod simulation;
mod models;
mod math;
mod config;

fn main() -> Result<(), Box<dyn Error>> {
    // 設定とシナリオの読み込み
    let missile_params = load_missile_parameters("config/missile_parameters.yaml")?;
    let radar_params = load_radar_parameters("config/radar_parameters.yaml")?;
    let interceptor_params = load_interceptor_parameters("config/interceptor_parameters.yaml")?;
    let scenario = load_scenario("config/scenario.yaml")?;

    // エンティティの初期化
    let mut state = initialize_simulation_state(
        missile_params.clone(),
        radar_params,
        interceptor_params.clone(),
        scenario,
    );

    // CSV出力の設定
    let mut writer: Box<dyn Write> = setup_csv_output("output/simulation_results.csv", &state)?;

    // 重力加速度の定義
    let gravity = [0.0, 0.0, -9.81];
    let dt = 0.1;
    let cycles = 1000;

    // シミュレーションのメインループ
    for cycle in 0..cycles {
        let time = cycle as f64 * dt;

        // シミュレーションステップの実行
        state = execute_simulation_step(&state, &missile_params, &interceptor_params, gravity, dt)?;

        // レーダーの探知処理
        let radar_detections = models::motion::detect_all_radars(&state.radars, &state.missiles);

        // CSV行の作成と書き込み
        let row = create_csv_row(
            &time,
            &state.missiles,
            &state.interceptors,
            &state.radars,
            &radar_detections,
        );
        writer.write_all(row.as_bytes())?;
    }

    Ok(())
}

