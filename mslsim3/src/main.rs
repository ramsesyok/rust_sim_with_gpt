use std::error::Error;
use std::fs::File;
use std::io::Write;

mod math;
mod models;
mod utils;

use math::integrator::AdamsBashforthIntegrator;
use math::low_pass_filter::LowPassFilter;
use models::interceptor::{check_interception, launch_interceptor, Interceptor};
use models::missile::{check_collision as check_missile_collision, Missile};
use models::radar::{detect_missile, generate_fire_command, Radar};
use utils::yaml_parser::{parse_yaml, InterceptorParams, MissileParams, RadarParams, Scenario};

fn main() -> Result<(), Box<dyn Error>> {
    // === 1. YAMLファイルからパラメータとシナリオを読み込む ===
    let missile_params: MissileParams = parse_yaml("config/missile_params.yaml")?;
    let radar_params: RadarParams = parse_yaml("config/radar_params.yaml")?;
    let interceptor_params: InterceptorParams = parse_yaml("config/interceptor_params.yaml")?;
    let scenario: Scenario = parse_yaml("config/scenario.yaml")?;

    // === 2. 初期化 ===
    // シミュレーション用のオブジェクトを生成
    let mut missiles: Vec<Missile> = scenario
        .initial_conditions
        .missiles
        .iter()
        .map(|init_state| Missile {
            params: missile_params.clone(),
            state: init_state.clone(),
        })
        .collect();

    let radar = Radar {
        params: radar_params.clone(),
    };

    let mut interceptors: Vec<Interceptor> = scenario
        .initial_conditions
        .interceptors
        .iter()
        .map(|init_state| Interceptor {
            params: interceptor_params.clone(),
            state: init_state.clone(),
        })
        .collect();

    // タイムステップ dt
    let mut dt = 0.1; // デフォルト
                      // 必要に応じて scenario.yaml などから dt を設定しても良い

    // Adams-Bashforth およびローパスフィルタ用のインスタンス
    // ※ ミサイル数や迎撃ミサイル数に応じて生成する
    let mut missile_integrators: Vec<[AdamsBashforthIntegrator; 3]> = (0..missiles.len())
        .map(|_| core::array::from_fn(|_| AdamsBashforthIntegrator::new()))
        .collect();

    let mut missile_filters: Vec<[LowPassFilter; 3]> = vec![
        [
            LowPassFilter::new(missile_params.alpha_filter),
            LowPassFilter::new(missile_params.alpha_filter),
            LowPassFilter::new(missile_params.alpha_filter)
        ];
        missiles.len()
    ];

    let mut interceptor_integrators: Vec<[AdamsBashforthIntegrator; 3]> = (0..interceptors.len())
        .map(|_| core::array::from_fn(|_| AdamsBashforthIntegrator::new()))
        .collect();

    let mut interceptor_filters: Vec<[LowPassFilter; 3]> = vec![
        [
            LowPassFilter::new(interceptor_params.alpha_filter),
            LowPassFilter::new(interceptor_params.alpha_filter),
            LowPassFilter::new(interceptor_params.alpha_filter)
        ];
        interceptors.len()
    ];

    // CSV出力ファイルを準備
    let mut file = File::create("output/simulation_results.csv")?;
    // CSVヘッダー
    writeln!(
        file,
        "time[s],missile_id,missile_x[m],missile_y[m],missile_z[m],radar_detected,interceptor_id,interceptor_x[m],interceptor_y[m],interceptor_z[m]"
    )?;

    // === 3. タイムループ ===
    let mut time = 0.0;
    let mut running = true;

    while running {
        // 各オブジェクトがまだ「終了条件」に達していないか確認しつつ進める
        // 今回はサンプルとして、一定時間を超えたら強制的にシミュレーション終了
        if time > 2000.0 {
            break;
        }

        running = false; // 全て終了していればループを抜ける

        // ===== (1) ミサイルの更新 =====
        for (i, missile) in missiles.iter_mut().enumerate() {
            // すでに地表衝突 or 迎撃されている場合は更新不要
            if check_missile_collision(&missile.state) {
                continue;
            }

            // まだ生存中ならフラグを true にする
            running = true;

            // Adams-Bashforth & ローパスを用いて更新
            missile.state = models::missile::update_missile(
                &missile.params,
                &missile.state,
                &mut missile_integrators[i],
                &mut missile_filters[i],
                dt,
            );
        }

        // ===== (2) レーダ演算 (探知 & 発射指示) =====
        // 0.1 s周期で探知するとあるので、簡易的に dt が 0.1 前後なら毎ステップチェック
        let mut detections = Vec::new();
        for (missile_id, missile) in missiles.iter().enumerate() {
            // 衝突 (終了) のミサイルはスキップ
            if check_missile_collision(&missile.state) {
                continue;
            }
            // 探知を試みる
            let detection_result = detect_missile(&radar, &missile.state, 1e-6);
            if detection_result.detected {
                // 発射指示
                let fire_command = generate_fire_command(&detection_result);
                detections.push((missile_id, detection_result, fire_command));
            }
        }

        // ===== (3) 迎撃ミサイルの更新 =====
        // レーダが探知した場合、発射フラグをオンにする
        for (i, interceptor) in interceptors.iter_mut().enumerate() {
            if !interceptor.state.launched {
                // まだ発射していない → レーダからの指示があれば発射
                if let Some((_mid, _dres, fire_command)) =
                    detections.iter().find(|(_, _, fire)| *fire)
                {
                    if *fire_command {
                        interceptor.state = launch_interceptor(&interceptor.state);
                    }
                }
            }
            // インターセプタの運動更新
            // ここでは最も近いミサイルを狙うなど、シンプルなロジックにする
            // (複数ミサイルがあるときは誘導ターゲットを決める必要がある)
            if interceptor.state.launched {
                // とりあえず最初のミサイルを追尾
                if let Some(target_missile) = missiles.get(0) {
                    interceptor.state = models::interceptor::update_interceptor(
                        &interceptor.params,
                        &interceptor.state,
                        &target_missile.state.position,
                        &mut interceptor_integrators[i],
                        &mut interceptor_filters[i],
                        dt,
                    );
                    // 迎撃成功判定
                    let intercept_distance = 50.0; // 適当な判定距離
                    if check_interception(
                        &interceptor.state,
                        &target_missile.state,
                        intercept_distance,
                    ) {
                        println!(
                            "Interceptor {} has intercepted Missile 0 at t={:.2} s",
                            i, time
                        );
                        // 迎撃成功 → ミサイルを強制的に地表衝突扱いにするなど
                        // ここでは簡単に z=0 にして衝突状態とします
                        missiles[0].state.position[2] = 0.0;
                    }
                }
            }
        }

        // ===== (4) CSVログ出力 =====
        // ミサイルごとに行を出力 (本来はまとめて出してもよい)
        for (missile_id, missile) in missiles.iter().enumerate() {
            let detected = if detections.iter().any(|(mid, _, _)| *mid == missile_id) {
                "true"
            } else {
                "false"
            };
            // 1つ目の迎撃ミサイルの位置だけを記録する例
            // (本来は複数インターセプタもループで出力する)
            let (interceptor_id, ix, iy, iz) = if let Some(intc) = interceptors.get(0) {
                (
                    0,
                    intc.state.position[0],
                    intc.state.position[1],
                    intc.state.position[2],
                )
            } else {
                // 未定義
                (-1, 0.0, 0.0, 0.0)
            };
            writeln!(
                file,
                "{:.3},{},{:.3},{:.3},{:.3},{},{},{:.3},{:.3},{:.3}",
                time,
                missile_id,
                missile.state.position[0],
                missile.state.position[1],
                missile.state.position[2],
                detected,
                interceptor_id,
                ix,
                iy,
                iz
            )?;
        }

        // 時間経過
        time += dt;
    }

    println!("Simulation finished. Results saved to output/simulation_results.csv");
    Ok(())
}
