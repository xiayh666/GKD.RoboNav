use nalgebra::{UnitQuaternion, Vector3};
use std::{error, fmt};

#[derive(Clone, Debug)]
pub struct ImuSample {
    pub time: f64,
    pub angular_velocity: Vector3<f64>,
    /// Specific force, including the stationary gravity reaction.
    pub acceleration: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct LidarPoint {
    /// Coordinates in the LiDAR frame at this point's acquisition time.
    pub position: Vector3<f64>,
    pub intensity: f32,
    /// Seconds relative to Scan::start_time (not milliseconds).
    pub offset: f64,
}

#[derive(Clone, Debug)]
pub struct Scan {
    pub start_time: f64,
    /// Explicit end time, including when the scan has no valid returns.
    pub end_time: f64,
    /// May be unordered; the pipeline sorts a private copy by acquisition time.
    pub points: Vec<LidarPoint>,
}

#[derive(Clone, Debug)]
pub struct ScanResult {
    pub state: crate::State,
    pub covariance: crate::Covariance,
    /// Each accepted return transformed using its own acquisition-time pose.
    pub world_points: Vec<Vector3<f64>>,
    /// All range-accepted returns grouped before localization-map thinning.
    pub observations: std::sync::Arc<map_updates::ObservationBatch>,
    pub matched_points: usize,
    /// Actual stored returns; multiple points may share a voxel.
    pub map_points: usize,
    /// Occupied voxels, bounded by Config::map_capacity.
    pub map_voxels: usize,
    pub rejected_saturated_axes: usize,
}

/// Noise values are variances. Process noise follows upstream's Q * dt² rule.
#[derive(Clone, Debug)]
pub struct Config {
    pub gravity_magnitude: f64,
    pub initialization_samples: usize,
    /// Accumulate startup returns before enabling local-map matching.
    pub initial_map_min_points: usize,
    pub initialization_max_gyro: f64,
    pub initialization_max_accel_std: f64,
    pub max_imu_gap: f64,
    /// Rotation/translation of LiDAR coordinates into the IMU frame.
    pub lidar_to_imu_rotation: UnitQuaternion<f64>,
    pub lidar_to_imu_translation: Vector3<f64>,
    pub estimate_extrinsics: bool,
    pub gyro_saturation: f64,
    pub accel_saturation: f64,
    pub gyro_measurement_variance: f64,
    pub accel_measurement_variance: f64,
    pub velocity_process_variance: f64,
    pub gyro_process_variance: f64,
    pub accel_process_variance: f64,
    pub gyro_bias_process_variance: f64,
    pub accel_bias_process_variance: f64,
    pub lidar_variance: f64,
    pub plane_threshold: f64,
    /// Upstream's range > match_s * point_plane_distance² gate.
    pub match_s: f64,
    pub min_range: f64,
    pub max_range: f64,
    /// iVox index resolution (upstream ivox_grid_resolution), in meters.
    pub voxel_size: f64,
    /// Map insertion occupancy cell size (upstream filter_size_map), in meters.
    pub filter_size_map: f64,
    /// Distance cap inside the center + 26 neighboring voxel search.
    pub neighbor_radius: f64,
    /// Voxel budget, not a bound on the number of stored points.
    pub map_capacity: usize,
    pub imu_buffer_capacity: usize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            gravity_magnitude: 9.81,
            initialization_samples: 100,
            initial_map_min_points: 10,
            initialization_max_gyro: 0.1,
            initialization_max_accel_std: 0.15,
            max_imu_gap: 0.05,
            lidar_to_imu_rotation: UnitQuaternion::identity(),
            lidar_to_imu_translation: Vector3::zeros(),
            estimate_extrinsics: false,
            gyro_saturation: 35.0,
            accel_saturation: 100.0,
            gyro_measurement_variance: 0.01,
            accel_measurement_variance: 0.1,
            velocity_process_variance: 20.0,
            gyro_process_variance: 0.1,
            accel_process_variance: 0.1,
            gyro_bias_process_variance: 0.0001,
            accel_bias_process_variance: 0.0001,
            lidar_variance: 0.01,
            plane_threshold: 0.1,
            match_s: 81.0,
            min_range: 0.1,
            max_range: 100.0,
            voxel_size: 0.3,
            filter_size_map: 0.3,
            neighbor_radius: 5.0,
            map_capacity: 100_000,
            imu_buffer_capacity: 20_000,
        }
    }
}

impl Config {
    pub(crate) fn validate(&self) -> Result<(), Error> {
        let positive = [
            self.gravity_magnitude,
            self.initialization_max_gyro,
            self.initialization_max_accel_std,
            self.max_imu_gap,
            self.gyro_saturation,
            self.accel_saturation,
            self.gyro_measurement_variance,
            self.accel_measurement_variance,
            self.lidar_variance,
            self.plane_threshold,
            self.match_s,
            self.max_range,
            self.voxel_size,
            self.filter_size_map,
            self.neighbor_radius,
        ];
        let noise = [
            self.velocity_process_variance,
            self.gyro_process_variance,
            self.accel_process_variance,
            self.gyro_bias_process_variance,
            self.accel_bias_process_variance,
        ];
        if positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || noise.iter().any(|v| !v.is_finite() || *v < 0.0)
            || !self.min_range.is_finite()
            || self.min_range < 0.0
            || self.min_range >= self.max_range
            || self.initialization_samples < 2
            || self.initial_map_min_points < 5
            || self.map_capacity < 5
            || self.imu_buffer_capacity == 0
            || !self.lidar_to_imu_translation.iter().all(|v| v.is_finite())
            || !self
                .lidar_to_imu_rotation
                .coords
                .iter()
                .all(|v| v.is_finite())
            || (self.lidar_to_imu_rotation.norm_squared() - 1.0).abs() > 1e-8
        {
            return Err(Error::InvalidConfig);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Error {
    InvalidConfig,
    InvalidImu,
    InvalidScan,
    NonMonotonicTime,
    NotInitialized,
    NotStationary,
    NeedMoreImu,
    ImuGap,
    ImuBufferFull,
    NumericalFailure,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::InvalidConfig => "invalid filter or map configuration",
            Self::InvalidImu => "IMU sample contains a non-finite value",
            Self::InvalidScan => "scan contains an invalid point or time interval",
            Self::NonMonotonicTime => "sensor timestamp moved backwards or repeated",
            Self::NotInitialized => "stationary IMU initialization is incomplete",
            Self::NotStationary => {
                "initialization window is moving, saturated, or has invalid gravity"
            }
            Self::NeedMoreImu => "buffer IMU through the scan end before processing",
            Self::ImuGap => "IMU coverage has a gap larger than max_imu_gap",
            Self::ImuBufferFull => "IMU buffer full; process scans or reset before retrying",
            Self::NumericalFailure => "filter innovation or covariance is numerically invalid",
        })
    }
}

impl error::Error for Error {}
