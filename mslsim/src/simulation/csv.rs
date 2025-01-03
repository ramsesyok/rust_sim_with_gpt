// src/simulation/csv.rs

use std::error::Error;
use std::io::Write;
use std::fs::File;
use std::io::BufWriter;

use crate::{Missile, Radar, Interceptor};
use crate::simulation::SimulationState;

/// CSV出力の設定とヘッダーの書き込み
pub fn setup_csv_output(
    path: &str,
    state: &SimulationState,
) -> Result<Box<dyn Write>, Box<dyn Error>> {
    let output_file = File::create(path)?;
    let mut writer = BufWriter::new(output_file);
    write_csv_header(&mut writer, state)?;
    Ok(Box::new(writer))
}


/// CSVヘッダーの書き込み
pub fn write_csv_header<W: Write>(
    writer: &mut W,
    state: &SimulationState,
) -> Result<(), std::io::Error> {
    let mut header = String::from("time(s),");

    // ミサイルのヘッダー
    for missile in &state.missiles {
        header.push_str(&format!(
            "{0}_x(m),{0}_y(m),{0}_z(m),{0}_pitch(deg),",
            missile.id
        ));
    }

    // 迎撃ミサイルのヘッダー
    for interceptor in &state.interceptors {
        header.push_str(&format!(
            "{0}_x(m),{0}_y(m),{0}_z(m),{0}_pitch(deg),",
            interceptor.id
        ));
    }

    // レーダのヘッダー
    for radar in &state.radars {
        header.push_str(&format!(
            "{0}_detected(bool),{0}_detect_x(m),{0}_detect_y(m),{0}_detect_z(m),",
            radar.id
        ));
    }

    header.push('\n');
    writer.write_all(header.as_bytes())?;
    Ok(())
}


/// CSV行の作成
pub fn create_csv_row(
    time: &f64,
    missiles: &Vec<Missile>,
    interceptors: &Vec<Interceptor>,
    _radars: &Vec<Radar>,
    radar_detections: &Vec<(bool, [f64; 3])>,
) -> String {
    let mut row = format!("{},", time);

    // ミサイルの状態
    for missile in missiles {
        row.push_str(&format!(
            "{},{},{},{},",
            missile.position[0],
            missile.position[1],
            missile.position[2],
            missile.pitch
        ));
    }

    // 迎撃ミサイルの状態
    for interceptor in interceptors {
        row.push_str(&format!(
            "{},{},{},{},",
            interceptor.position[0],
            interceptor.position[1],
            interceptor.position[2],
            interceptor.pitch
        ));
    }

    // レーダの探知状況
    for detection in radar_detections {
        row.push_str(&format!(
            "{},{},{},{},",
            detection.0,
            detection.1[0],
            detection.1[1],
            detection.1[2]
        ));
    }

    row.push('\n');
    row
}
