use point_lio::{Config, ImuSample, LidarPoint, PointLio, Scan, UnitQuaternion, Vector3};
use std::time::Instant;

const DT: f64 = 0.005;
const START: f64 = 0.5;

fn imu(time: f64) -> ImuSample {
    ImuSample {
        time,
        angular_velocity: Vector3::zeros(),
        acceleration: Vector3::z() * 9.81,
    }
}

fn estimator(config: Config) -> PointLio {
    let mut lio = PointLio::new(config).unwrap();
    for i in 0..100 {
        lio.push_imu(imu(i as f64 * DT)).unwrap();
    }
    lio
}

fn landmarks(count: usize, floor_only: bool) -> Vec<Vector3<f64>> {
    (0..count)
        .map(|i| {
            let a = ((i / 3) % 37) as f64 * 0.15 - 2.7;
            let b = ((i / 111) % 37) as f64 * 0.15 - 2.7;
            match if floor_only { 0 } else { i % 3 } {
                0 => Vector3::new(a, b, -1.5),
                1 => Vector3::new(3., a, b),
                _ => Vector3::new(a, 3., b),
            }
        })
        .collect()
}

fn scan(
    frame: usize,
    world: &[Vector3<f64>],
    pose: impl Fn(f64) -> UnitQuaternion<f64>,
    noise: bool,
) -> Scan {
    let start = START + frame as f64 * 0.1;
    let points = world
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let offset = 0.099 * i as f64 / world.len().max(1) as f64;
            let mut position = pose(start + offset).inverse() * p;
            if noise {
                let t = frame as f64 * 0.31 + i as f64 * 1.73;
                position += Vector3::new(t.sin(), (t * 1.3).sin(), (t * 1.7).cos()) * 0.01;
            }
            LidarPoint {
                position,
                offset,
                intensity: 1.,
            }
        })
        .collect();
    Scan {
        start_time: start,
        end_time: start + 0.1,
        points,
    }
}

fn feed(lio: &mut PointLio, frame: usize, make: impl Fn(f64) -> ImuSample) {
    // Retain the previous frame's end IMU; never submit duplicate timestamps.
    let first = if frame == 0 { 100 } else { 101 + frame * 20 };
    for tick in first..=120 + frame * 20 {
        lio.push_imu(make(tick as f64 * DT)).unwrap();
    }
}

// 60 秒含确定性传感器噪声的静止回放，检查漂移、匹配和队列/地图容量。
#[test]
fn sixty_seconds_stationary_with_sensor_noise() {
    stationary_run(300);
}

// 使用默认地图容量重复相同输入，区分容量压力与其他累计误差。
#[test]
fn sixty_seconds_stationary_default_map_with_sensor_noise() {
    stationary_run(Config::default().map_capacity);
}

fn stationary_run(capacity: usize) {
    let config = Config {
        map_capacity: capacity,
        ..Config::default()
    };
    let mut lio = estimator(config);
    let world = landmarks(999, false);
    let mut max_position: f64 = 0.;
    let mut max_angle: f64 = 0.;
    let mut min_matches = usize::MAX;
    for frame in 0..600 {
        feed(&mut lio, frame, |time| {
            let mut s = imu(time);
            s.angular_velocity =
                Vector3::new((time * 13.).sin(), (time * 17.).cos(), (time * 19.).sin()) * 0.001;
            s.acceleration +=
                Vector3::new((time * 11.).sin(), (time * 7.).cos(), (time * 23.).sin()) * 0.02;
            s
        });
        let out = lio
            .process_scan(&scan(frame, &world, |_| UnitQuaternion::identity(), true))
            .unwrap();
        max_position = max_position.max(out.state.position.norm());
        max_angle = max_angle.max(out.state.orientation.angle());
        if frame > 0 {
            min_matches = min_matches.min(out.matched_points);
        }
        assert!(out.map_voxels <= capacity);
        assert!(lio.buffered_imu_len() <= 1);
        assert!(out.covariance.iter().all(|x| x.is_finite()));
    }
    println!("stationary_60s: capacity={capacity}, max_position_m={max_position:.6}, max_angle_rad={max_angle:.6}, min_matches={min_matches}, map={}", lio.map_len());
    assert!(max_position < 0.15, "stationary drift {max_position} m");
    assert!(max_angle < 0.05, "stationary angle {max_angle} rad");
    assert!(min_matches > 100, "only {min_matches} matches");
}

// 实际旋转时裁剪陀螺量程，对比不饱和控制组，验证激光能否维持航向。
#[test]
fn rotating_with_clipped_gyro_and_lidar() {
    let mut clipped = estimator(Config {
        gyro_saturation: 0.3,
        ..Config::default()
    });
    let mut control = estimator(Config::default());
    let world = landmarks(1665, false);
    let yaw = |time: f64| {
        let t = (time - START).max(0.);
        0.6 * (t - (1. - (-2. * t).exp()) / 2.)
    };
    let rotation = |time| UnitQuaternion::from_euler_angles(0., 0., yaw(time));
    let mut rejected = 0;
    let mut max_error: f64 = 0.;
    let mut control_error: f64 = 0.;
    for frame in 0..30 {
        let make = |time| {
            let mut s = imu(time);
            s.angular_velocity.z = 0.6 * (1. - (-2. * (time - START)).exp());
            s
        };
        feed(&mut control, frame, make);
        feed(&mut clipped, frame, |time| {
            let mut s = make(time);
            s.angular_velocity.z = s.angular_velocity.z.min(0.3);
            s
        });
        let input = scan(frame, &world, rotation, false);
        let a = clipped.process_scan(&input).unwrap();
        let b = control.process_scan(&input).unwrap();
        let truth = rotation(input.end_time);
        max_error = max_error.max(a.state.orientation.angle_to(&truth));
        control_error = control_error.max(b.state.orientation.angle_to(&truth));
        rejected += a.rejected_saturated_axes;
    }
    println!("clipped_rotation_3s: max_angle_rad={max_error:.6}, control_max_angle_rad={control_error:.6}, rejected_axes={rejected}");
    assert!(rejected > 100);
    assert!(
        max_error < 0.15,
        "clipped gyro tracking error {max_error} rad"
    );
    assert!(control_error < 0.05);
}

// 单一地面无法约束平面内位移；刻画这种退化，避免把有匹配数当成定位可靠。
#[test]
fn floor_only_does_not_observe_tangential_drift() {
    let mut lio = estimator(Config::default());
    // Use distinct returns on a 2D surface. The old helper repeated each point
    // three times, so the five nearest could contain only two unique points.
    let world: Vec<_> = (0..1665)
        .map(|i| {
            Vector3::new(
                (i % 37) as f64 * 0.15 - 2.7,
                (i / 37) as f64 * 0.15 - 2.7,
                -1.5,
            )
        })
        .collect();
    let mut matches = 0;
    for frame in 0..30 {
        feed(&mut lio, frame, |time| {
            let mut s = imu(time);
            if frame > 0 {
                s.acceleration.x = 0.2;
            }
            s
        });
        let out = lio
            .process_scan(&scan(frame, &world, |_| UnitQuaternion::identity(), false))
            .unwrap();
        matches += out.matched_points;
    }
    let s = lio.state().unwrap();
    println!(
        "floor_degeneracy_3s: x_drift_m={:.6}, z_error_m={:.6}, matches={matches}",
        s.position.x,
        s.position.z.abs()
    );
    assert!(matches > 1000);
    assert!(s.position.z.abs() < 0.05);
    assert!(
        s.position.x.abs() > 0.1,
        "tangential motion is unobservable from one plane"
    );
}

// 雷达连续 0.5 秒没有回波后恢复，检查接口可继续融合且重新获得匹配。
#[test]
fn lidar_returns_after_half_second_without_points() {
    let mut lio = estimator(Config::default());
    let world = landmarks(1665, false);
    let mut recovery_matches = 0;
    for frame in 0..25 {
        feed(&mut lio, frame, |time| {
            let mut s = imu(time);
            if frame >= 5 {
                s.acceleration.x = 0.1;
            }
            s
        });
        let mut input = scan(frame, &world, |_| UnitQuaternion::identity(), false);
        if (5..10).contains(&frame) {
            input.points.clear();
        }
        let out = lio.process_scan(&input).unwrap();
        if frame == 10 {
            recovery_matches = out.matched_points;
        }
    }
    let error = lio.state().unwrap().position.norm();
    println!("lidar_recovery: first_matches={recovery_matches}, final_position_m={error:.6}");
    assert!(recovery_matches > 100);
    assert!(error < 0.15);
}

// 仅在显式请求时测量完整逐点处理耗时，不将机器速度写成易波动的单元断言。
#[test]
#[ignore = "manual release-mode timing measurement"]
fn processing_time_by_scan_size() {
    for count in [1_000, 10_000, 20_000] {
        let mut lio = estimator(Config::default());
        let world = landmarks(count, false);
        let mut timings = Vec::new();
        let mut matches = 0;
        for frame in 0..6 {
            feed(&mut lio, frame, imu);
            let input = scan(frame, &world, |_| UnitQuaternion::identity(), false);
            let start = Instant::now();
            let result = lio.process_scan(&input).unwrap();
            let ms = start.elapsed().as_secs_f64() * 1000.;
            if frame > 0 {
                timings.push(ms);
                matches += result.matched_points;
            }
        }
        timings.sort_by(f64::total_cmp);
        println!("timing: points={count}, samples={}, median_ms={:.3}, min_ms={:.3}, max_ms={:.3}, matches_per_frame={:.0}, meets_10hz_in_all_samples={}",
            timings.len(), timings[timings.len()/2], timings[0], timings[timings.len()-1], matches as f64 / 5., timings.iter().all(|ms| *ms <= 100.));
        assert!(matches > 0);
    }
}
