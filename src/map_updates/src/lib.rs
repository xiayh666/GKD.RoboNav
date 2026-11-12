//! Observation updates, not mutations of a localization cache. All coordinates
//! are metres in one gravity-aligned odometry frame per session.
use std::{collections::BTreeMap, error, fmt, sync::Arc};

pub type Point3 = [f64; 3];

/// The caller assigns a distinct source for each estimator; reset advances epoch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SessionId {
    pub source: u64,
    pub epoch: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VoxelKey(pub i64, pub i64, pub i64);
impl VoxelKey {
    pub fn from_point(p: Point3, resolution: f64) -> Result<Self, InvalidUpdate> {
        if !resolution.is_finite() || resolution <= 0. {
            return Err(InvalidUpdate);
        }
        let q = p.map(|x| (x / resolution).floor());
        if q.iter()
            .any(|x| !x.is_finite() || *x < -9223372036854775808. || *x >= 9223372036854775808.)
        {
            return Err(InvalidUpdate);
        }
        Ok(Self(q[0] as i64, q[1] as i64, q[2] as i64))
    }
    pub fn center(self, resolution: f64) -> Point3 {
        [self.0, self.1, self.2].map(|v| (v as f64 + 0.5) * resolution)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub position: Point3,
    /// Unit quaternion, xyzw; body/IMU to world.
    pub orientation: [f64; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct Observation {
    pub time: f64,
    /// World position of LiDAR at acquisition time, including its extrinsic offset.
    pub origin: Point3,
    pub endpoint: Point3,
}

#[derive(Clone, Debug)]
pub struct ObservedVoxel {
    pub key: VoxelKey,
    pub points: Arc<[Observation]>,
}

/// Every range-accepted return, grouped before iVox map-insertion filtering.
/// An absent voxel is NOT evidence of empty space; no cache eviction operations.
#[derive(Clone, Debug)]
pub struct ObservationBatch {
    pub session: SessionId,
    pub sequence: u64,
    pub start_time: f64,
    pub end_time: f64,
    pub pose: Pose,
    pub resolution: f64,
    pub voxels: Arc<[ObservedVoxel]>,
}

#[derive(Clone, Copy, Debug)]
pub struct ObservationReset {
    pub session: SessionId,
    /// Supports late join/resync without resetting the estimator. The receiver
    /// discards its geometry and accepts this sequence next, starting unknown.
    pub next_sequence: u64,
}

#[derive(Clone, Debug)]
pub enum MapUpdate {
    Reset(ObservationReset),
    Observations(Arc<ObservationBatch>),
}

#[derive(Clone, Copy, Debug)]
pub struct InvalidUpdate;
impl fmt::Display for InvalidUpdate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("invalid observation update")
    }
}
impl error::Error for InvalidUpdate {}

impl ObservationBatch {
    pub fn from_points(
        reset: ObservationReset,
        start_time: f64,
        end_time: f64,
        pose: Pose,
        resolution: f64,
        points: impl IntoIterator<Item = Observation>,
    ) -> Result<Self, InvalidUpdate> {
        let mut groups: BTreeMap<VoxelKey, Vec<Observation>> = BTreeMap::new();
        for point in points {
            groups
                .entry(VoxelKey::from_point(point.endpoint, resolution)?)
                .or_default()
                .push(point);
        }
        let result = Self {
            session: reset.session,
            sequence: reset.next_sequence,
            start_time,
            end_time,
            pose,
            resolution,
            voxels: groups
                .into_iter()
                .map(|(key, points)| ObservedVoxel {
                    key,
                    points: points.into(),
                })
                .collect(),
        };
        result.validate()?;
        Ok(result)
    }
    pub fn point_count(&self) -> usize {
        self.voxels
            .iter()
            .fold(0usize, |n, v| n.saturating_add(v.points.len()))
    }
    pub fn validate(&self) -> Result<(), InvalidUpdate> {
        if self.sequence == 0
            || !self.start_time.is_finite()
            || !self.end_time.is_finite()
            || self.end_time < self.start_time
            || !self.resolution.is_finite()
            || self.resolution <= 0.
            || self
                .pose
                .position
                .iter()
                .chain(&self.pose.orientation)
                .any(|v| !v.is_finite())
            || (self.pose.orientation.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-6
        {
            return Err(InvalidUpdate);
        }
        let mut previous = None;
        for voxel in self.voxels.iter() {
            if previous.is_some_and(|k| k >= voxel.key) || voxel.points.is_empty() {
                return Err(InvalidUpdate);
            }
            previous = Some(voxel.key);
            for p in voxel.points.iter() {
                if !p.time.is_finite()
                    || p.time < self.start_time - 1e-9
                    || p.time > self.end_time + 1e-9
                    || p.origin.iter().chain(&p.endpoint).any(|v| !v.is_finite())
                    || VoxelKey::from_point(p.endpoint, self.resolution)? != voxel.key
                {
                    return Err(InvalidUpdate);
                }
            }
        }
        Ok(())
    }
}
