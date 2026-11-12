use point_lio::{
    Config as LioConfig, ImuSample, LidarPoint, PointLio, Scan, UnitQuaternion, Vector3,
};
use terrain_map::{map_updates::*, Config, Mapper};

fn imu(time: f64) -> ImuSample {
    ImuSample {
        time,
        angular_velocity: Vector3::zeros(),
        acceleration: Vector3::z() * 9.81,
    }
}
fn lio(capacity: usize) -> PointLio {
    let mut lio = PointLio::new_with_source(
        LioConfig {
            voxel_size: 0.1,
            map_capacity: capacity,
            ..LioConfig::default()
        },
        41,
    )
    .unwrap();
    for i in 0..100 {
        lio.push_imu(imu(f64::from(i) * 0.005)).unwrap();
    }
    lio
}
fn scan(frame: usize, y: f64) -> Scan {
    let start = 0.5 + frame as f64 * 0.1;
    Scan {
        start_time: start,
        end_time: start + 0.1,
        points: (0..49)
            .map(|i| LidarPoint {
                position: Vector3::new(
                    1.05,
                    y + 0.05 + f64::from(i % 7) * 0.12,
                    0.05 + f64::from(i / 7) * 0.12,
                ),
                intensity: 1.,
                offset: 0.05,
            })
            .collect(),
    }
}
fn feed(lio: &mut PointLio, frame: usize) {
    let begin = if frame == 0 { 100 } else { 101 + frame * 20 };
    for i in begin..=120 + frame * 20 {
        lio.push_imu(imu(i as f64 * 0.005)).unwrap();
    }
}

// 实际Point-LIO的小容量淘汰不会删除地形障碍；第二面墙仍来自筛点前观测。
#[test]
fn real_ivox_eviction_does_not_remove_terrain_geometry() {
    let mut source = lio(5);
    let mut terrain = Mapper::new(Config::default()).unwrap();
    terrain
        .apply_update(&MapUpdate::Reset(source.observation_reset()))
        .unwrap();
    feed(&mut source, 0);
    let first = source.process_scan(&scan(0, 0.)).unwrap();
    assert_eq!(first.map_voxels, 5);
    assert_eq!(first.observations.point_count(), 49);
    terrain
        .apply_update(&MapUpdate::Observations(first.observations.clone()))
        .unwrap();
    let old_key = first.observations.voxels[0].key;
    let before = terrain.snapshot().surfaces.voxel_count();
    assert!(before > 5);
    feed(&mut source, 1);
    let second = source.process_scan(&scan(1, 4.)).unwrap();
    assert_eq!(second.map_voxels, 5);
    let out = terrain
        .apply_update(&MapUpdate::Observations(second.observations))
        .unwrap();
    assert!(out.surfaces.voxel(old_key).is_some());
    assert!(out.surfaces.voxel_count() > before);
}

// 缓存无需插入新点时依然输出本帧全部观测，输出序号不因等待或重置混淆。
#[test]
fn repeated_scans_emit_observations_even_without_map_growth() {
    let mut source = lio(1000);
    assert!(matches!(
        source.process_scan(&scan(0, 0.)),
        Err(point_lio::Error::NeedMoreImu)
    ));
    assert_eq!(source.observation_reset().next_sequence, 1);
    feed(&mut source, 0);
    let first = source.process_scan(&scan(0, 0.)).unwrap();
    feed(&mut source, 1);
    let second = source.process_scan(&scan(1, 0.)).unwrap();
    assert_eq!(first.map_points, second.map_points);
    assert_eq!(second.observations.point_count(), 49);
    assert_eq!(second.observations.sequence, 2);
    assert_eq!(source.observation_reset().next_sequence, 3);
    let old_session = second.observations.session;
    source.reset();
    let reset = source.observation_reset();
    assert_eq!(reset.session.source, old_session.source);
    assert_eq!(reset.session.epoch, old_session.epoch + 1);
    assert_eq!(reset.next_sequence, 1);
}

// 逐点原点必须包含外参平移及采样时刻旋转，不能整帧使用末状态位置。
#[test]
fn acquisition_origins_include_extrinsics_and_motion() {
    let extrinsic = Vector3::new(0.4, 0.1, 0.2);
    let mut source = PointLio::new(LioConfig {
        lidar_to_imu_translation: extrinsic,
        lidar_to_imu_rotation: UnitQuaternion::from_euler_angles(0., 0., 0.2),
        ..LioConfig::default()
    })
    .unwrap();
    for i in 0..100 {
        source.push_imu(imu(f64::from(i) * 0.005)).unwrap();
    }
    for i in 100..=120 {
        let mut s = imu(f64::from(i) * 0.005);
        s.angular_velocity.z = 1.;
        source.push_imu(s).unwrap();
    }
    let input = Scan {
        start_time: 0.5,
        end_time: 0.6,
        points: vec![
            LidarPoint {
                position: Vector3::x(),
                intensity: 1.,
                offset: 0.01,
            },
            LidarPoint {
                position: Vector3::x(),
                intensity: 1.,
                offset: 0.09,
            },
        ],
    };
    let output = source.process_scan(&input).unwrap();
    let mut points: Vec<_> = output
        .observations
        .voxels
        .iter()
        .flat_map(|v| v.points.iter())
        .collect();
    points.sort_by(|a, b| a.time.total_cmp(&b.time));
    let to_vec = |p: [f64; 3]| Vector3::new(p[0], p[1], p[2]);
    assert!((to_vec(points[0].origin) - to_vec(points[1].origin)).norm() > 0.001);
    let final_origin = output.state.lidar_point_to_world(&Vector3::zeros());
    assert!((to_vec(points[0].origin) - final_origin).norm() > 0.001);
    for point in points {
        assert!((to_vec(point.origin).norm() - extrinsic.norm()).abs() < 1e-9);
        assert!(((to_vec(point.endpoint) - to_vec(point.origin)).norm() - 1.).abs() < 1e-9);
        assert!(output
            .world_points
            .iter()
            .any(|p| (*p - to_vec(point.endpoint)).norm() < 1e-14));
    }
}
