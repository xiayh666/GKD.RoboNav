use crate::{
    filter::Filter, map::VoxelMap, Config, Covariance, Error, ImuSample, Scan, ScanResult, State,
};
use map_updates::{Observation, ObservationBatch, ObservationReset, Pose, SessionId};
use nalgebra::Vector3;
use std::{collections::VecDeque, sync::Arc};

/// An independent estimator. Call sequentially, or protect the whole instance
/// with a caller-owned mutex. No global state or background threads.
pub struct PointLio {
    config: Config,
    filter: Option<Filter>,
    initialization: Vec<ImuSample>,
    imu: VecDeque<ImuSample>,
    last_imu_time: Option<f64>,
    map: VoxelMap,
    initial_map_points: Vec<Vector3<f64>>,
    session: SessionId,
    sequence: u64,
}

impl PointLio {
    pub fn new(config: Config) -> Result<Self, Error> {
        Self::new_with_source(config, 0)
    }

    /// Assign distinct source IDs when multiple estimators feed consumers.
    pub fn new_with_source(config: Config, source: u64) -> Result<Self, Error> {
        config.validate()?;
        Ok(Self {
            map: VoxelMap::new(config.voxel_size, config.map_capacity),
            config,
            filter: None,
            initialization: Vec::new(),
            imu: VecDeque::new(),
            last_imu_time: None,
            initial_map_points: Vec::new(),
            session: SessionId { source, epoch: 0 },
            sequence: 0,
        })
    }

    /// Explicit late-join/resync boundary; does not reset filter or iVox.
    /// Call and deliver before the next process_scan, in the same producer order.
    pub fn observation_reset(&self) -> ObservationReset {
        ObservationReset {
            session: self.session,
            next_sequence: self
                .sequence
                .checked_add(1)
                .expect("observation sequence exhausted"),
        }
    }

    pub fn state(&self) -> Option<&State> {
        self.filter.as_ref().map(|f| &f.state)
    }
    pub fn covariance(&self) -> Option<&Covariance> {
        self.filter.as_ref().map(|f| &f.covariance)
    }
    pub fn map_len(&self) -> usize {
        self.map.len()
    }
    pub fn map_voxel_len(&self) -> usize {
        self.map.voxels()
    }

    pub fn buffered_imu_len(&self) -> usize {
        self.imu.len()
    }

    /// Reset all state and map; required after a sensor clock restart.
    pub fn reset(&mut self) {
        self.session.epoch = self
            .session
            .epoch
            .checked_add(1)
            .expect("observation epoch exhausted");
        self.sequence = 0;
        self.filter = None;
        self.initialization.clear();
        self.imu.clear();
        self.last_imu_time = None;
        self.initial_map_points.clear();
        self.map = VoxelMap::new(self.config.voxel_size, self.config.map_capacity);
    }

    /// Before initialization, collects a stationary window. A rejected window
    /// is cleared and returns NotStationary; subsequent samples retry startup.
    /// Afterwards buffers samples until a covered scan is submitted.
    /// A full buffer returns ImuBufferFull without accepting the sample.
    pub fn push_imu(&mut self, sample: ImuSample) -> Result<(), Error> {
        if !sample.time.is_finite()
            || !sample.angular_velocity.iter().all(|v| v.is_finite())
            || !sample.acceleration.iter().all(|v| v.is_finite())
        {
            return Err(Error::InvalidImu);
        }
        if self.last_imu_time.is_some_and(|t| sample.time <= t) {
            return Err(Error::NonMonotonicTime);
        }
        if self.filter.is_none() {
            if self
                .last_imu_time
                .is_some_and(|t| sample.time - t > self.config.max_imu_gap)
            {
                self.initialization.clear();
            }
            self.last_imu_time = Some(sample.time);
            self.initialization.push(sample);
            if self.initialization.len() < self.config.initialization_samples {
                return Ok(());
            }
            let result = self.initialize();
            self.initialization.clear();
            result
        } else {
            if self.imu.len() >= self.config.imu_buffer_capacity {
                return Err(Error::ImuBufferFull);
            }
            self.last_imu_time = Some(sample.time);
            self.imu.push_back(sample);
            Ok(())
        }
    }

    fn initialize(&mut self) -> Result<(), Error> {
        let n = self.initialization.len() as f64;
        let mean_a = self
            .initialization
            .iter()
            .map(|s| s.acceleration)
            .sum::<Vector3<f64>>()
            / n;
        let mean_g = self
            .initialization
            .iter()
            .map(|s| s.angular_velocity)
            .sum::<Vector3<f64>>()
            / n;
        let std_a = (self
            .initialization
            .iter()
            .map(|s| (s.acceleration - mean_a).norm_squared())
            .sum::<f64>()
            / n)
            .sqrt();
        let c = &self.config;
        if std_a > c.initialization_max_accel_std
            || (mean_a.norm() - c.gravity_magnitude).abs() > c.initialization_max_accel_std * 3.0
            || self.initialization.iter().any(|s| {
                s.angular_velocity.norm() > c.initialization_max_gyro
                    || s.angular_velocity
                        .iter()
                        .any(|v| v.abs() >= 0.99 * c.gyro_saturation)
                    || s.acceleration
                        .iter()
                        .any(|v| v.abs() >= 0.99 * c.accel_saturation)
            })
        {
            return Err(Error::NotStationary);
        }
        let time = self.initialization.last().unwrap().time;
        self.filter = Some(Filter::new(State::initial(c, time, mean_a, mean_g), c));
        Ok(())
    }

    /// Requires IMU coverage through end_time. Invalid/waiting scans leave all
    /// state untouched and can be retried after more IMU arrives. Numerical
    /// failures also roll back filter/queue; map commits only after success.
    pub fn process_scan(&mut self, scan: &Scan) -> Result<ScanResult, Error> {
        let filter = self.filter.as_ref().ok_or(Error::NotInitialized)?;
        if !scan.start_time.is_finite()
            || !scan.end_time.is_finite()
            || scan.end_time < scan.start_time
        {
            return Err(Error::InvalidScan);
        }
        if scan.start_time < filter.state.time - 1e-9 {
            return Err(Error::NonMonotonicTime);
        }
        if scan.end_time <= filter.state.time {
            return Err(Error::NonMonotonicTime);
        }
        for p in &scan.points {
            if !p.offset.is_finite()
                || p.offset < 0.0
                || scan.start_time + p.offset > scan.end_time + 1e-9
                || !p.position.iter().all(|v| v.is_finite())
                || !p.intensity.is_finite()
                || !p.position.norm_squared().is_finite()
            {
                return Err(Error::InvalidScan);
            }
        }
        if self.last_imu_time.is_none_or(|t| t < scan.end_time - 1e-9) {
            return Err(Error::NeedMoreImu);
        }
        // Check the future sample as well, so a long gap cannot hide at frame end.
        let mut previous = filter.state.time;
        for sample in &self.imu {
            if sample.time - previous > self.config.max_imu_gap + 1e-12 {
                return Err(Error::ImuGap);
            }
            previous = sample.time;
            if previous >= scan.end_time {
                break;
            }
        }
        let mut points: Vec<_> = scan
            .points
            .iter()
            .filter(|p| {
                let range = p.position.norm();
                range >= self.config.min_range && range <= self.config.max_range
            })
            .collect();
        points.sort_by(|a, b| a.offset.total_cmp(&b.offset));

        // Work on a filter copy and read the queue by index. No partial commits.
        let mut next = filter.clone();
        let mut consumed = 0;
        let mut rejected = 0;
        let mut matched = 0;
        let mut world_points = Vec::with_capacity(points.len());
        let mut observed_points = Vec::with_capacity(points.len());
        let mut neighbors = Vec::with_capacity(points.len());
        let mut index = 0;
        while index < points.len() {
            let begin = index;
            let offset = points[index].offset;
            while index < points.len() && points[index].offset == offset {
                index += 1;
            }
            self.advance(
                &mut next,
                scan.start_time + offset,
                &mut consumed,
                &mut rejected,
            )?;
            let group = &points[begin..index];
            let mut observations = Vec::with_capacity(group.len());
            for point in group {
                let world = next.state.lidar_point_to_world(&point.position);
                let nearby = self.map.closest(world, self.config.neighbor_radius);
                if let Some((normal, offset)) =
                    VoxelMap::fit_plane(&nearby, self.config.plane_threshold)
                {
                    observations.push((point.position, normal, offset));
                }
                neighbors.push(nearby);
            }
            matched += next.observe_planes(&observations, &self.config)?;
            for point in group {
                let world = next.state.lidar_point_to_world(&point.position);
                let origin = next.state.lidar_point_to_world(&Vector3::zeros());
                observed_points.push(Observation {
                    time: scan.start_time + point.offset,
                    origin: [origin.x, origin.y, origin.z],
                    endpoint: [world.x, world.y, world.z],
                });
                world_points.push(world);
            }
        }
        self.advance(&mut next, scan.end_time, &mut consumed, &mut rejected)?;
        // Build from every accepted observation BEFORE MapIncremental filtering.
        // This does not expose cache evictions as environment deletions.
        let observations = Arc::new(
            ObservationBatch::from_points(
                self.observation_reset(),
                scan.start_time,
                scan.end_time,
                Pose {
                    position: [
                        next.state.position.x,
                        next.state.position.y,
                        next.state.position.z,
                    ],
                    orientation: [
                        next.state.orientation.i,
                        next.state.orientation.j,
                        next.state.orientation.k,
                        next.state.orientation.w,
                    ],
                },
                self.config.voxel_size,
                observed_points,
            )
            .map_err(|_| Error::InvalidScan)?,
        );
        if self.map.voxels() == 0 {
            self.initial_map_points.extend_from_slice(&world_points);
            if self.initial_map_points.len() >= self.config.initial_map_min_points {
                for point in self.initial_map_points.drain(..) {
                    self.map.insert(point);
                }
            }
        } else {
            self.map
                .insert_frame(&world_points, &neighbors, self.config.filter_size_map);
        }
        self.imu.drain(..consumed);
        let result = ScanResult {
            state: next.state.clone(),
            covariance: next.covariance,
            world_points,
            matched_points: matched,
            map_points: self.map.len(),
            map_voxels: self.map.voxels(),
            rejected_saturated_axes: rejected,
            observations: Arc::clone(&observations),
        };
        self.sequence = observations.sequence;
        self.filter = Some(next);
        Ok(result)
    }

    fn advance(
        &self,
        filter: &mut Filter,
        time: f64,
        consumed: &mut usize,
        rejected: &mut usize,
    ) -> Result<(), Error> {
        while let Some(sample) = self.imu.get(*consumed).filter(|s| s.time <= time) {
            filter.predict_to(sample.time, &self.config)?;
            *rejected += filter.observe_imu(sample, &self.config)?;
            *consumed += 1;
        }
        filter.predict_to(time, &self.config)
    }
}
