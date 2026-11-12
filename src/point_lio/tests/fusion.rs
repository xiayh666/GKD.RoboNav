use point_lio::{Config, Error, ImuSample, LidarPoint, PointLio, Scan, UnitQuaternion, Vector3};

fn sample(time: f64) -> ImuSample {
    ImuSample {
        time,
        angular_velocity: Vector3::zeros(),
        acceleration: Vector3::z() * 9.81,
    }
}

fn initialized(config: Config) -> PointLio {
    let count = config.initialization_samples;
    let mut lio = PointLio::new(config).unwrap();
    for i in 0..count {
        lio.push_imu(sample(i as f64 * 0.005)).unwrap();
    }
    lio
}

fn room() -> Vec<Vector3<f64>> {
    let mut points = Vec::new();
    for a in -6..=6 {
        for b in -6..=6 {
            let a = a as f64 * 0.25;
            let b = b as f64 * 0.25;
            points.push(Vector3::new(a, b, -1.5));
            points.push(Vector3::new(3., a, b));
            points.push(Vector3::new(a, 3., b));
        }
    }
    points
}

// 无先验地图启动，点云在首帧建立地图，后续静止帧能匹配且位姿不漂移。
#[test]
fn stationary_room_bootstraps_and_matches() {
    let mut lio = initialized(Config::default());
    assert_eq!(lio.map_len(), 0);
    for i in 100..=140 {
        lio.push_imu(sample(i as f64 * 0.005)).unwrap();
    }
    for frame in 0..2 {
        let start = 0.5 + frame as f64 * 0.1;
        let scan = Scan {
            start_time: start,
            end_time: start + 0.1,
            points: room()
                .into_iter()
                .map(|p| LidarPoint {
                    position: p,
                    intensity: 1.,
                    offset: 0.05,
                })
                .collect(),
        };
        let result = lio.process_scan(&scan).unwrap();
        assert!(result.state.position.norm() < 1e-8);
        assert!(result.state.orientation.angle() < 1e-8);
        assert!(result.map_points > 100);
        if frame == 0 {
            assert_eq!(result.matched_points, 0);
        } else {
            assert!(
                result.matched_points > 100,
                "matches {}",
                result.matched_points
            );
        }
    }
}

// 等待 IMU、无效点和倒序数据不能修改滤波状态、消耗队列或污染地图。
#[test]
fn rejected_scans_are_transactional_and_retryable() {
    let mut lio = initialized(Config::default());
    let scan = Scan {
        start_time: 0.5,
        end_time: 0.6,
        points: vec![],
    };
    assert_eq!(lio.process_scan(&scan).unwrap_err(), Error::NeedMoreImu);
    assert_eq!(lio.state().unwrap().time, 0.495);
    for i in 100..=120 {
        lio.push_imu(sample(i as f64 * 0.005)).unwrap();
    }
    let len = lio.buffered_imu_len();
    let mut bad = scan.clone();
    bad.points.push(LidarPoint {
        position: Vector3::new(f64::NAN, 0., 0.),
        intensity: 0.,
        offset: 0.01,
    });
    assert_eq!(lio.process_scan(&bad).unwrap_err(), Error::InvalidScan);
    assert_eq!(lio.buffered_imu_len(), len);
    assert_eq!(lio.map_len(), 0);
    lio.process_scan(&scan).unwrap();
    assert_eq!(
        lio.process_scan(&scan).unwrap_err(),
        Error::NonMonotonicTime
    );
    assert_eq!(lio.push_imu(sample(0.6)), Err(Error::NonMonotonicTime));
    lio.reset();
    assert!(lio.state().is_none());
    assert_eq!(lio.map_len(), 0);
    assert_eq!(lio.process_scan(&scan).unwrap_err(), Error::NotInitialized);
}

// IMU 饱和轴必须被跳过，其他轴仍参与观测；不得把饱和值当作真实角速度。
#[test]
fn saturation_axes_are_excluded() {
    let mut lio = initialized(Config::default());
    let mut imu = sample(0.5);
    imu.angular_velocity.x = 40.;
    imu.acceleration.y = 120.;
    lio.push_imu(imu).unwrap();
    let out = lio
        .process_scan(&Scan {
            start_time: 0.5,
            end_time: 0.5,
            points: vec![],
        })
        .unwrap();
    assert_eq!(out.rejected_saturated_axes, 2);
    assert!(out.state.angular_velocity.norm() < 1e-10);
    assert!(out.state.velocity.norm() < 1e-10);
}

// 静止初始化须支持倒置 IMU，拒绝运动窗口，并允许继续收集后重试。
#[test]
fn initialization_checks_motion_and_handles_inverted_imu() {
    let c = Config {
        initialization_samples: 4,
        ..Config::default()
    };
    let mut lio = PointLio::new(c).unwrap();
    for i in 0..4 {
        let mut imu = sample(i as f64 * 0.005);
        imu.angular_velocity.z = 1.;
        let result = lio.push_imu(imu);
        if i == 3 {
            assert_eq!(result, Err(Error::NotStationary));
        } else {
            result.unwrap();
        }
    }
    assert!(lio.state().is_none());
    for i in 4..8 {
        let mut imu = sample(i as f64 * 0.005);
        imu.acceleration.z = -9.81;
        lio.push_imu(imu).unwrap();
    }
    let state = lio.state().unwrap();
    assert!((state.orientation * state.specific_force + state.gravity).norm() < 1e-8);
    assert!(
        (state
            .orientation
            .to_rotation_matrix()
            .matrix()
            .determinant()
            - 1.)
            .abs()
            < 1e-10
    );
}

// 中断的 IMU 覆盖必须显式报错，不能在空档中静默外推。
#[test]
fn imu_gap_is_rejected_without_consuming_data() {
    let mut lio = initialized(Config::default());
    lio.push_imu(sample(0.7)).unwrap();
    let out = lio.process_scan(&Scan {
        start_time: 0.5,
        end_time: 0.7,
        points: vec![],
    });
    assert_eq!(out.unwrap_err(), Error::ImuGap);
    assert_eq!(lio.buffered_imu_len(), 1);
}

// 逐点采样时间和非零外参在旋转/平移时仍应生成一致世界点并维持轨迹。
#[test]
fn moving_room_tracks_per_point_motion_with_extrinsics() {
    let c = Config {
        voxel_size: 0.2,
        neighbor_radius: 0.8,
        lidar_to_imu_translation: Vector3::new(0.12, -0.04, 0.08),
        lidar_to_imu_rotation: UnitQuaternion::from_euler_angles(0.05, -0.08, 0.1),
        ..Config::default()
    };
    let mut lio = initialized(c.clone());
    let base = 0.495;
    let truth = |time: f64| {
        let t = time - base;
        (
            Vector3::new(0.2 * t * t, 0., 0.),
            UnitQuaternion::from_euler_angles(0., 0., 0.2 * t),
        )
    };
    for i in 100..=300 {
        let time = i as f64 * 0.005;
        let (_, r) = truth(time);
        lio.push_imu(ImuSample {
            time,
            angular_velocity: Vector3::new(0., 0., 0.2),
            acceleration: r.inverse() * Vector3::new(0.4, 0., 9.81),
        })
        .unwrap();
    }
    let landmarks = room();
    let mut total_matches = 0;
    let mut last_error = 0.;
    for frame in 0..10 {
        let start = 0.5 + frame as f64 * 0.1;
        let points = landmarks
            .iter()
            .enumerate()
            .map(|(i, world)| {
                let offset = 0.099 * i as f64 / (landmarks.len() - 1) as f64;
                let (p, r) = truth(start + offset);
                let in_imu = r.inverse() * (world - p);
                LidarPoint {
                    offset,
                    intensity: 1.,
                    position: c.lidar_to_imu_rotation.inverse()
                        * (in_imu - c.lidar_to_imu_translation),
                }
            })
            .collect();
        let out = lio
            .process_scan(&Scan {
                start_time: start,
                end_time: start + 0.1,
                points,
            })
            .unwrap();
        let (p, r) = truth(start + 0.1);
        last_error = (out.state.position - p).norm();
        assert!(
            last_error < 0.08,
            "frame {frame}: position error {last_error}"
        );
        assert!(out.state.orientation.angle_to(&r) < 0.04);
        total_matches += out.matched_points;
        assert!(out.covariance.diagonal().min() >= -1e-10);
    }
    assert!(
        total_matches > 1000,
        "matches {total_matches}, position error {last_error}"
    );
}

// 多实例必须隔离状态，非法配置不能进入地图整数索引和矩阵计算。
#[test]
fn configuration_and_instance_isolation() {
    assert!(matches!(
        PointLio::new(Config {
            voxel_size: 0.,
            ..Config::default()
        }),
        Err(Error::InvalidConfig)
    ));
    for size in [0.0, -0.1, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            PointLio::new(Config {
                filter_size_map: size,
                ..Config::default()
            }),
            Err(Error::InvalidConfig)
        ));
    }
    let a = initialized(Config::default());
    let mut b = initialized(Config::default());
    b.reset();
    assert!(a.state().is_some());
    assert!(b.state().is_none());
}

// 对比纯惯性传播，激光必须能抑制运行后引入的加计误差，而非只生成地图。
#[test]
fn lidar_updates_reduce_inertial_drift() {
    let mut fused = initialized(Config::default());
    let mut inertial = initialized(Config::default());
    for frame in 0..20 {
        for i in 0..20 {
            let mut imu = sample((100 + frame * 20 + i) as f64 * 0.005);
            if frame > 0 {
                imu.acceleration.x = 0.2;
            }
            fused.push_imu(imu.clone()).unwrap();
            inertial.push_imu(imu).unwrap();
        }
        let time = (120 + frame * 20) as f64 * 0.005;
        // End at the final available sample, before the next frame's IMU event.
        let start = (100 + frame * 20) as f64 * 0.005;
        let scan = Scan {
            start_time: start,
            end_time: time - 0.005,
            points: room()
                .into_iter()
                .map(|p| LidarPoint {
                    position: p,
                    intensity: 1.,
                    offset: 0.05,
                })
                .collect(),
        };
        fused.process_scan(&scan).unwrap();
        inertial
            .process_scan(&Scan {
                points: vec![],
                ..scan
            })
            .unwrap();
    }
    let error_fused = fused.state().unwrap().position.norm();
    let error_inertial = inertial.state().unwrap().position.norm();
    assert!(error_inertial > 0.1, "inertial drift {error_inertial}");
    assert!(
        error_fused < error_inertial * 0.25,
        "laser correction: fused {error_fused}, inertial {error_inertial}"
    );
}

// 队列满时样本不能被接收，消费扫描后同一时间戳应可重试。
#[test]
fn bounded_imu_queue_supports_retry() {
    let c = Config {
        imu_buffer_capacity: 2,
        ..Config::default()
    };
    let mut lio = initialized(c);
    lio.push_imu(sample(0.5)).unwrap();
    lio.push_imu(sample(0.505)).unwrap();
    assert_eq!(lio.push_imu(sample(0.51)), Err(Error::ImuBufferFull));
    lio.process_scan(&Scan {
        start_time: 0.5,
        end_time: 0.505,
        points: vec![],
    })
    .unwrap();
    lio.push_imu(sample(0.51)).unwrap();
    assert_eq!(lio.buffered_imu_len(), 1);
}

// 同时刻点必须联合线性化，交换输入顺序不能改变该次位姿和协方差更新。
#[test]
fn simultaneous_point_order_does_not_change_posterior() {
    let mut a = initialized(Config::default());
    let mut b = initialized(Config::default());
    for i in 100..=140 {
        a.push_imu(sample(i as f64 * 0.005)).unwrap();
        b.push_imu(sample(i as f64 * 0.005)).unwrap();
    }
    let bootstrap = Scan {
        start_time: 0.5,
        end_time: 0.6,
        points: room()
            .into_iter()
            .map(|position| LidarPoint {
                position,
                offset: 0.05,
                intensity: 1.,
            })
            .collect(),
    };
    a.process_scan(&bootstrap).unwrap();
    b.process_scan(&bootstrap).unwrap();
    let mut next = Scan {
        start_time: 0.6,
        end_time: 0.7,
        points: room()
            .into_iter()
            .map(|p| LidarPoint {
                position: p - Vector3::new(0.02, -0.01, 0.015),
                offset: 0.05,
                intensity: 1.,
            })
            .collect(),
    };
    let forward = a.process_scan(&next).unwrap();
    next.points.reverse();
    let reversed = b.process_scan(&next).unwrap();
    assert!(forward.matched_points > 100);
    assert_eq!(forward.matched_points, reversed.matched_points);
    assert!((forward.state.position - reversed.state.position).norm() < 1e-10);
    assert!(
        forward
            .state
            .orientation
            .angle_to(&reversed.state.orientation)
            < 1e-10
    );
    assert!((forward.covariance - reversed.covariance).norm() < 1e-10);
}

// 稀疏首帧不能把未成形的地图交给增量筛选，跨帧积累到阈值后一次建图。
#[test]
fn sparse_startup_accumulates_before_map_matching() {
    let mut lio = initialized(Config::default());
    for i in 100..=140 {
        lio.push_imu(sample(i as f64 * 0.005)).unwrap();
    }
    for frame in 0..2 {
        let start = 0.5 + frame as f64 * 0.1;
        let points = (0..5)
            .map(|i| LidarPoint {
                position: Vector3::new(0.01 + i as f64 * 0.01, 0.01 + frame as f64 * 0.01, 1.51),
                intensity: 1.,
                offset: 0.05,
            })
            .collect();
        let out = lio
            .process_scan(&Scan {
                start_time: start,
                end_time: start + 0.1,
                points,
            })
            .unwrap();
        assert_eq!(out.matched_points, 0);
        assert_eq!(out.map_points, if frame == 0 { 0 } else { 10 });
    }
    assert_eq!(lio.map_voxel_len(), 1);
}
