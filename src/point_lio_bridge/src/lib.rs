//! C ABI v1; the opaque handle is owned by one ROS adapter worker.
use point_lio::{Config, Error, ImuSample, LidarPoint, PointLio, Scan, UnitQuaternion, Vector3};
use std::{
    ffi::c_void,
    panic::{catch_unwind, AssertUnwindSafe},
};

#[repr(C)]
pub struct PlioConfig {
    pub translation: [f64; 3],
    pub quaternion: [f64; 4],
    pub min_range: f64,
    pub max_range: f64,
    pub voxel_size: f64,
    pub filter_size_map: f64,
    pub map_capacity: u64,
}
#[repr(C)]
pub struct PlioImu {
    pub time: f64,
    pub gyro: [f64; 3],
    pub acceleration: [f64; 3],
}
#[repr(C)]
pub struct PlioPoint {
    pub position: [f64; 3],
    pub offset: f64,
    pub intensity: f32,
}
#[repr(C)]
pub struct PlioOutput {
    pub time: f64,
    pub position: [f64; 3],
    pub quaternion: [f64; 4],
    pub velocity_body: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub pose_covariance: [f64; 36],
    pub twist_covariance: [f64; 36],
    pub matches: u64,
    pub map_points: u64,
    pub map_voxels: u64,
}
fn status(error: Error) -> i32 {
    match error {
        Error::NotInitialized => 1,
        Error::NotStationary => 2,
        Error::NeedMoreImu => 3,
        Error::NonMonotonicTime => 4,
        Error::ImuGap => 5,
        Error::ImuBufferFull => 6,
        Error::NumericalFailure => 8,
        _ => 7,
    }
}
fn guarded(f: impl FnOnce() -> Result<(), Error>) -> i32 {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(())) => 0,
        Ok(Err(e)) => status(e),
        Err(_) => 9,
    }
}
#[no_mangle]
pub extern "C" fn plio_abi_version() -> u32 {
    1
}
/// # Safety
/// `config` points to a valid PlioConfig for this call.
#[no_mangle]
pub unsafe extern "C" fn plio_create(config: *const PlioConfig) -> *mut c_void {
    catch_unwind(AssertUnwindSafe(|| {
        let c = unsafe { config.as_ref() }?;
        if c.quaternion
            .iter()
            .chain(c.translation.iter())
            .any(|x| !x.is_finite())
        {
            return None;
        }
        let norm = c.quaternion.iter().map(|x| x * x).sum::<f64>();
        if (norm - 1.).abs() > 1e-8 {
            return None;
        }
        let rotation = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            c.quaternion[3],
            c.quaternion[0],
            c.quaternion[1],
            c.quaternion[2],
        ));
        let config = Config {
            lidar_to_imu_translation: Vector3::from(c.translation),
            lidar_to_imu_rotation: rotation,
            min_range: c.min_range,
            max_range: c.max_range,
            voxel_size: c.voxel_size,
            filter_size_map: c.filter_size_map,
            map_capacity: usize::try_from(c.map_capacity).ok()?,
            ..Config::default()
        };
        PointLio::new(config)
            .ok()
            .map(|p| Box::into_raw(Box::new(p)).cast::<c_void>())
    }))
    .ok()
    .flatten()
    .unwrap_or(std::ptr::null_mut())
}
/// # Safety
/// `handle` is a live uniquely owned handle from plio_create, or null.
#[no_mangle]
pub unsafe extern "C" fn plio_destroy(handle: *mut c_void) {
    if !handle.is_null() {
        let _ = catch_unwind(AssertUnwindSafe(|| {
            drop(unsafe { Box::from_raw(handle.cast::<PointLio>()) })
        }));
    }
}
/// # Safety
/// `handle` is live and exclusively borrowed for this call.
#[no_mangle]
pub unsafe extern "C" fn plio_reset(handle: *mut c_void) -> i32 {
    guarded(|| {
        let p = unsafe { handle.cast::<PointLio>().as_mut() }.ok_or(Error::InvalidConfig)?;
        p.reset();
        Ok(())
    })
}
/// # Safety
/// Live exclusive handle, valid imu pointer for the duration of this call.
#[no_mangle]
pub unsafe extern "C" fn plio_push_imu(handle: *mut c_void, imu: *const PlioImu) -> i32 {
    guarded(|| {
        let p = unsafe { handle.cast::<PointLio>().as_mut() }.ok_or(Error::InvalidConfig)?;
        let s = unsafe { imu.as_ref() }.ok_or(Error::InvalidImu)?;
        p.push_imu(ImuSample {
            time: s.time,
            angular_velocity: Vector3::from(s.gyro),
            acceleration: Vector3::from(s.acceleration),
        })
    })
}
/// # Safety
/// Live exclusive handle; time is a valid writable f64 pointer.
#[no_mangle]
pub unsafe extern "C" fn plio_state_time(handle: *mut c_void, time: *mut f64) -> i32 {
    guarded(|| {
        let p = unsafe { handle.cast::<PointLio>().as_ref() }.ok_or(Error::InvalidConfig)?;
        let t = unsafe { time.as_mut() }.ok_or(Error::InvalidConfig)?;
        *t = p.state().ok_or(Error::NotInitialized)?.time;
        Ok(())
    })
}
/// # Safety
/// Live exclusive handle. points is a valid nonaliasing array of count elements
/// (may be null for zero count); output is writable and does not alias inputs.
#[no_mangle]
pub unsafe extern "C" fn plio_scan(
    handle: *mut c_void,
    start: f64,
    end: f64,
    points: *const PlioPoint,
    count: usize,
    output: *mut PlioOutput,
) -> i32 {
    guarded(|| {
        let p = unsafe { handle.cast::<PointLio>().as_mut() }.ok_or(Error::InvalidConfig)?;
        let out = unsafe { output.as_mut() }.ok_or(Error::InvalidScan)?;
        if count > 100_000 || (count > 0 && points.is_null()) {
            return Err(Error::InvalidScan);
        }
        let points = if count == 0 {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(points, count) }
        };
        let scan = Scan {
            start_time: start,
            end_time: end,
            points: points
                .iter()
                .map(|x| LidarPoint {
                    position: Vector3::from(x.position),
                    offset: x.offset,
                    intensity: x.intensity,
                })
                .collect(),
        };
        let r = p.process_scan(&scan)?;
        let s = &r.state;
        let rot = s.orientation.to_rotation_matrix();
        let rot = rot.matrix();
        let body_velocity = s.orientation.inverse() * s.velocity;
        let mut j = nalgebra::SMatrix::<f64, 6, 30>::zeros();
        for i in 0..3 {
            j[(i, i)] = 1.;
            for k in 0..3 {
                j[(i + 3, k + 3)] = rot[(i, k)];
            }
        }
        let pose_cov = j * r.covariance * j.transpose();
        j.fill(0.);
        let v = body_velocity;
        let skew = [[0., -v.z, v.y], [v.z, 0., -v.x], [-v.y, v.x, 0.]];
        for i in 0..3 {
            for k in 0..3 {
                j[(i, k + 3)] = skew[i][k];
                j[(i, k + 12)] = rot[(k, i)];
            }
            j[(i + 3, i + 15)] = 1.;
        }
        let twist_cov = j * r.covariance * j.transpose();
        let mut pose_covariance = [0.; 36];
        let mut twist_covariance = [0.; 36];
        for i in 0..6 {
            for k in 0..6 {
                pose_covariance[i * 6 + k] = pose_cov[(i, k)];
                twist_covariance[i * 6 + k] = twist_cov[(i, k)];
            }
        }
        *out = PlioOutput {
            time: s.time,
            position: s.position.into(),
            quaternion: [
                s.orientation.i,
                s.orientation.j,
                s.orientation.k,
                s.orientation.w,
            ],
            velocity_body: body_velocity.into(),
            angular_velocity: s.angular_velocity.into(),
            pose_covariance,
            twist_covariance,
            matches: r.matched_points as u64,
            map_points: r.map_points as u64,
            map_voxels: r.map_voxels as u64,
        };
        Ok(())
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config() -> PlioConfig {
        PlioConfig {
            translation: [-0.011, -0.02329, 0.04412],
            quaternion: [0., 0., 0., 1.],
            min_range: 0.5,
            max_range: 30.,
            voxel_size: 0.3,
            filter_size_map: 0.3,
            map_capacity: 100000,
        }
    }
    fn sample(i: usize) -> PlioImu {
        PlioImu {
            time: i as f64 * 0.005,
            gyro: [0.; 3],
            acceleration: [0., 0., 9.81],
        }
    }
    // ABI结构大小及对齐必须与C++头文件一致，禁止Rust默认布局跨语言。
    #[test]
    fn c_layout_is_fixed() {
        assert_eq!(std::mem::size_of::<PlioConfig>(), 96);
        assert_eq!(std::mem::size_of::<PlioImu>(), 56);
        assert_eq!(std::mem::size_of::<PlioPoint>(), 40);
        assert_eq!(std::mem::size_of::<PlioOutput>(), 712);
        assert_eq!(std::mem::align_of::<PlioOutput>(), 8);
    }
    // 非单位外参、空句柄和越界长度应返回错误，不能解引用或越过ABI崩溃。
    #[test]
    fn rejects_invalid_config_and_pointers() {
        unsafe {
            let mut c = config();
            c.quaternion = [0.; 4];
            assert!(plio_create(&c).is_null());
            assert!(plio_create(std::ptr::null()).is_null());
            assert_eq!(plio_reset(std::ptr::null_mut()), 7);
            let p = plio_create(&config());
            assert!(!p.is_null());
            let mut out = std::mem::MaybeUninit::<PlioOutput>::uninit();
            assert_eq!(
                plio_scan(p, 0., 1., std::ptr::null(), 100001, out.as_mut_ptr()),
                7
            );
            assert_eq!(plio_push_imu(p, std::ptr::null()), 7);
            plio_destroy(p);
            plio_destroy(std::ptr::null_mut());
        }
    }
    // 初始化、缺IMU重试和reset保留Rust核心事务语义，不制造伪位姿输出。
    #[test]
    fn initialization_coverage_and_reset() {
        unsafe {
            let p = plio_create(&config());
            let mut time = 0.;
            assert_eq!(plio_state_time(p, &mut time), 1);
            for i in 0..100 {
                assert_eq!(plio_push_imu(p, &sample(i)), 0);
            }
            assert_eq!(plio_state_time(p, &mut time), 0);
            assert_eq!(time, 0.495);
            let mut out = std::mem::MaybeUninit::<PlioOutput>::uninit();
            assert_eq!(
                plio_scan(p, 0.5, 0.6, std::ptr::null(), 0, out.as_mut_ptr()),
                3
            );
            assert_eq!(plio_state_time(p, &mut time), 0);
            assert_eq!(time, 0.495);
            for i in 100..=120 {
                assert_eq!(plio_push_imu(p, &sample(i)), 0);
            }
            assert_eq!(
                plio_scan(p, 0.5, 0.6, std::ptr::null(), 0, out.as_mut_ptr()),
                0
            );
            assert_eq!(out.assume_init().time, 0.6);
            assert_eq!(plio_reset(p), 0);
            assert_eq!(plio_state_time(p, &mut time), 1);
            plio_destroy(p);
        }
    }
    // ABI与直接调用同一Rust核心的状态必须一致，协方差保持有限、对称。
    #[test]
    fn matches_direct_core_with_extrinsics() {
        unsafe {
            let c = config();
            let p = plio_create(&c);
            let mut direct = PointLio::new(Config {
                min_range: c.min_range,
                max_range: c.max_range,
                lidar_to_imu_translation: Vector3::from(c.translation),
                ..Config::default()
            })
            .unwrap();
            for i in 0..=120 {
                let s = sample(i);
                assert_eq!(plio_push_imu(p, &s), 0);
                direct
                    .push_imu(ImuSample {
                        time: s.time,
                        angular_velocity: Vector3::from(s.gyro),
                        acceleration: Vector3::from(s.acceleration),
                    })
                    .unwrap();
            }
            let points: Vec<_> = (0..49)
                .map(|i| PlioPoint {
                    position: [3., (i % 7) as f64 * 0.2 - 0.6, (i / 7) as f64 * 0.2 - 0.6],
                    offset: 0.05,
                    intensity: 1.,
                })
                .collect();
            let mut out = std::mem::MaybeUninit::<PlioOutput>::uninit();
            assert_eq!(
                plio_scan(p, 0.5, 0.6, points.as_ptr(), points.len(), out.as_mut_ptr()),
                0
            );
            let out = out.assume_init();
            let result = direct
                .process_scan(&Scan {
                    start_time: 0.5,
                    end_time: 0.6,
                    points: points
                        .iter()
                        .map(|p| LidarPoint {
                            position: Vector3::from(p.position),
                            offset: p.offset,
                            intensity: p.intensity,
                        })
                        .collect(),
                })
                .unwrap();
            assert_eq!(out.position, <[f64; 3]>::from(result.state.position));
            assert_eq!(out.map_points, result.map_points as u64);
            for cov in [out.pose_covariance, out.twist_covariance] {
                for i in 0..6 {
                    assert!(cov[i * 6 + i] >= 0.);
                    for j in 0..6 {
                        assert!(cov[i * 6 + j].is_finite());
                        assert!((cov[i * 6 + j] - cov[j * 6 + i]).abs() < 1e-12);
                    }
                }
            }
            assert_eq!(out.quaternion, [0., 0., 0., 1.]);
            plio_destroy(p);
        }
    }
}
