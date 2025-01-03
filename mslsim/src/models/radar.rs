// src/models/radar.rs

use crate::Missile;

/// レーダの構造体
#[derive(Debug, Clone, PartialEq)]
pub struct Radar {
    pub id: String,
    pub position: [f64; 3],
    pub detection_range: f64,
    pub azimuth_min: f64,    // 度単位
    pub azimuth_max: f64,    // 度単位
    pub elevation_min: f64,  // 度単位
    pub elevation_max: f64,  // 度単位
}

/// ミサイルを探知するか判定する関数
///
/// # 引数
/// - `radar`: レーダのデータ
/// - `missile`: ミサイルのデータ
///
/// # 戻り値
/// - ミサイルがレーダーの探知範囲および角度範囲内にある場合は`true`、それ以外は`false`
pub fn detect(radar: &Radar, missile: &Missile) -> bool {
    // 相対位置ベクトルの計算
    let rel_position = [
        missile.position[0] - radar.position[0],
        missile.position[1] - radar.position[1],
        missile.position[2] - radar.position[2],
    ];

    // 距離の計算
    let distance = (rel_position[0].powi(2) + rel_position[1].powi(2) + rel_position[2].powi(2)).sqrt();
    if distance > radar.detection_range {
        return false;
    }

    // 方位角の計算（度単位）
    let azimuth_rad = rel_position[1].atan2(rel_position[0]);
    let mut azimuth_deg = azimuth_rad.to_degrees();
    if azimuth_deg < 0.0 {
        azimuth_deg += 360.0;
    }

    // 仰角の計算（度単位）
    let horizontal_distance = (rel_position[0].powi(2) + rel_position[1].powi(2)).sqrt();
    let elevation_rad = rel_position[2].atan2(horizontal_distance);
    let elevation_deg = elevation_rad.to_degrees();

    // 方位角および仰角の範囲チェック
    let azimuth_in_range = if radar.azimuth_min <= radar.azimuth_max {
        azimuth_deg >= radar.azimuth_min && azimuth_deg <= radar.azimuth_max
    } else {
        // 角度が360度を跨ぐ場合の処理
        azimuth_deg >= radar.azimuth_min || azimuth_deg <= radar.azimuth_max
    };

    let elevation_in_range = elevation_deg >= radar.elevation_min && elevation_deg <= radar.elevation_max;

    azimuth_in_range && elevation_in_range
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Missile;

    #[test]
    fn test_radar_detection_within_range_and_angles() {
        let radar = Radar {
            id: "radar1".to_string(),
            position: [0.0, 0.0, 0.0],
            detection_range: 1000.0,
            azimuth_min: 0.0,
            azimuth_max: 90.0,
            elevation_min: -10.0,
            elevation_max: 10.0,
        };

        let missile = Missile {
            id: "missile1".to_string(),
            position: [500.0, 500.0, 0.0], // azimuth = 45°, elevation = 0°
            velocity: [100.0, 0.0, 50.0],
            pitch: 45.0,
            mass: 5000.0,
        };

        assert!(detect(&radar, &missile));
    }

    #[test]
    fn test_radar_detection_out_of_distance() {
        let radar = Radar {
            id: "radar1".to_string(),
            position: [0.0, 0.0, 0.0],
            detection_range: 1000.0,
            azimuth_min: 0.0,
            azimuth_max: 90.0,
            elevation_min: -10.0,
            elevation_max: 10.0,
        };

        let missile = Missile {
            id: "missile1".to_string(),
            position: [1000.0, 1000.0, 0.0], // distance = ~1414.2 > 1000
            velocity: [100.0, 0.0, 50.0],
            pitch: 45.0,
            mass: 5000.0,
        };

        assert!(!detect(&radar, &missile));
    }

    #[test]
    fn test_radar_detection_out_of_azimuth() {
        let radar = Radar {
            id: "radar1".to_string(),
            position: [0.0, 0.0, 0.0],
            detection_range: 1000.0,
            azimuth_min: 0.0,
            azimuth_max: 90.0,
            elevation_min: -10.0,
            elevation_max: 10.0,
        };

        let missile = Missile {
            id: "missile1".to_string(),
            position: [-500.0, 500.0, 0.0], // azimuth = 135° > 90°
            velocity: [100.0, 0.0, 50.0],
            pitch: 45.0,
            mass: 5000.0,
        };

        assert!(!detect(&radar, &missile));
    }

    #[test]
    fn test_radar_detection_out_of_elevation() {
        let radar = Radar {
            id: "radar1".to_string(),
            position: [0.0, 0.0, 0.0],
            detection_range: 1000.0,
            azimuth_min: 0.0,
            azimuth_max: 360.0,
            elevation_min: -10.0,
            elevation_max: 10.0,
        };

        let missile = Missile {
            id: "missile1".to_string(),
            position: [500.0, 500.0, 200.0], // elevation = ~19.1° > 10°
            velocity: [100.0, 0.0, 50.0],
            pitch: 45.0,
            mass: 5000.0,
        };

        assert!(!detect(&radar, &missile));
    }

    #[test]
    fn test_radar_detection_azimuth_wrap_around() {
        let radar = Radar {
            id: "radar1".to_string(),
            position: [0.0, 0.0, 0.0],
            detection_range: 1000.0,
            azimuth_min: 350.0,
            azimuth_max: 10.0,
            elevation_min: -10.0,
            elevation_max: 10.0,
        };

        // azimuth = 5° (within 350-10°)
        let missile1 = Missile {
            id: "missile1".to_string(),
            position: [100.0, -17.3648178, 0.0], // ~5°
            velocity: [100.0, 0.0, 50.0],
            pitch: 45.0,
            mass: 5000.0,
        };
        assert!(detect(&radar, &missile1));

        // azimuth = 355° (within 350-10°)
        let missile2 = Missile {
            id: "missile2".to_string(),
            position: [100.0, 29.0482216, 0.0], // ~355°
            velocity: [100.0, 0.0, 50.0],
            pitch: 45.0,
            mass: 5000.0,
        };
        assert!(detect(&radar, &missile2));

        // azimuth = 20° (outside 350-10°)
        let missile3 = Missile {
            id: "missile3".to_string(),
            position: [100.0, -34.202014, 0.0], // ~20°
            velocity: [100.0, 0.0, 50.0],
            pitch: 45.0,
            mass: 5000.0,
        };
        assert!(!detect(&radar, &missile3));
    }
}
