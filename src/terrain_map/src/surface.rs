use crate::{Config, Error};
use map_updates::{Observation, ObservationBatch, ObservationReset, Pose, SessionId, VoxelKey};
use std::{collections::HashMap, sync::Arc};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamStatus {
    AwaitingReset,
    Ready,
    NeedsReset { expected: u64, received: u64 },
}

#[derive(Clone, Debug)]
pub struct SurfaceVoxel {
    /// Bounded recent samples; each retains its acquisition time. Geometry
    /// algorithms must use live_samples() to exclude individually stale samples.
    pub samples: Arc<[Observation]>,
    pub last_seen: f64,
}

/// Geometry view using the producer's keys/resolution, independent of iVox LRU.
/// Cells share observation arrays when no merging or sampling is necessary.
#[derive(Clone)]
pub struct SurfaceSnapshot {
    cells: HashMap<VoxelKey, Arc<SurfaceVoxel>>,
    pub session: Option<SessionId>,
    pub next_sequence: u64,
    pub status: StreamStatus,
    pub resolution: Option<f64>,
    pub time: Option<f64>,
    pub pose: Option<Pose>,
    max_age: f64,
    epochs: HashMap<u64, u64>,
}
impl SurfaceSnapshot {
    pub(crate) fn new(config: &Config) -> Self {
        Self {
            cells: HashMap::new(),
            session: None,
            next_sequence: 1,
            status: StreamStatus::AwaitingReset,
            resolution: None,
            time: None,
            pose: None,
            max_age: config.max_age,
            epochs: HashMap::new(),
        }
    }
    pub fn voxel_count(&self) -> usize {
        self.cells.len()
    }
    pub fn voxel(&self, key: VoxelKey) -> Option<&Arc<SurfaceVoxel>> {
        self.cells.get(&key)
    }
    pub fn observed(&self) -> impl Iterator<Item = (VoxelKey, &Arc<SurfaceVoxel>)> {
        self.cells.iter().map(|(k, v)| (*k, v))
    }
    pub fn live_samples(&self, key: VoxelKey) -> impl Iterator<Item = &Observation> {
        let cutoff = self.time.unwrap_or(f64::INFINITY) - self.max_age;
        self.cells
            .get(&key)
            .into_iter()
            .flat_map(|v| v.samples.iter())
            .filter(move |p| p.time >= cutoff)
    }
    pub(crate) fn reset(&self, reset: ObservationReset, config: &Config) -> Result<Self, Error> {
        if reset.next_sequence == 0 {
            return Err(Error::InvalidUpdate);
        }
        if let Some(current) = self.session {
            if current.source == reset.session.source && reset.session.epoch < current.epoch {
                return Err(Error::SessionMismatch);
            }
            if current == reset.session && reset.next_sequence < self.next_sequence {
                return Err(Error::OutOfOrder);
            }
        }
        if self.session != Some(reset.session) {
            if self
                .epochs
                .get(&reset.session.source)
                .is_some_and(|epoch| reset.session.epoch <= *epoch)
            {
                return Err(Error::SessionMismatch);
            }
            if !self.epochs.contains_key(&reset.session.source) && self.epochs.len() >= 1024 {
                return Err(Error::Capacity);
            }
        }
        let mut next = Self::new(config);
        next.epochs = self.epochs.clone();
        next.epochs
            .insert(reset.session.source, reset.session.epoch);
        next.session = Some(reset.session);
        next.next_sequence = reset.next_sequence;
        next.status = StreamStatus::Ready;
        Ok(next)
    }
    pub(crate) fn invalidate(&mut self, expected: u64, received: u64) {
        self.cells.clear();
        self.status = StreamStatus::NeedsReset { expected, received };
    }
    pub(crate) fn prepare(&self, batch: &ObservationBatch, config: &Config) -> Result<Self, Error> {
        if self.session.is_none() {
            return Err(Error::AwaitingReset);
        }
        if self.session != Some(batch.session) {
            return Err(Error::SessionMismatch);
        }
        if matches!(self.status, StreamStatus::NeedsReset { .. }) {
            return Err(Error::NeedsReset);
        }
        if batch.point_count() > config.max_points {
            return Err(Error::Capacity);
        }
        batch.validate().map_err(|_| Error::InvalidUpdate)?;
        if batch.sequence < self.next_sequence {
            return Err(Error::OutOfOrder);
        }
        if batch.sequence > self.next_sequence {
            return Err(Error::SequenceGap {
                expected: self.next_sequence,
                received: batch.sequence,
            });
        }
        if self.resolution.is_some_and(|r| r != batch.resolution) {
            return Err(Error::InvalidUpdate);
        }
        if self.time.is_some_and(|t| batch.end_time <= t) {
            return Err(Error::NonMonotonicTime);
        }
        let inside = |k: VoxelKey| {
            let p = k.center(batch.resolution);
            (0..3).all(|i| (p[i] - batch.pose.position[i]).abs() <= config.half_extent[i])
        };
        let cutoff = batch.end_time - config.max_age;
        let mut next = self.clone();
        next.cells
            .retain(|k, v| inside(*k) && v.last_seen >= cutoff);
        for group in batch.voxels.iter().filter(|v| inside(v.key)) {
            let old = next.cells.get(&group.key);
            let samples = if old.is_none()
                && group.points.len() <= config.max_surface_samples_per_voxel
                && group.points.iter().all(|p| p.time >= cutoff)
            {
                Arc::clone(&group.points)
            } else {
                let mut samples: Vec<_> = old
                    .into_iter()
                    .flat_map(|v| v.samples.iter())
                    .chain(group.points.iter())
                    .filter(|p| p.time >= cutoff)
                    .copied()
                    .collect();
                samples.sort_by(|a, b| a.time.total_cmp(&b.time));
                if samples.len() > config.max_surface_samples_per_voxel {
                    samples.drain(..samples.len() - config.max_surface_samples_per_voxel);
                }
                samples.into()
            };
            if !samples.is_empty() {
                next.cells.insert(
                    group.key,
                    Arc::new(SurfaceVoxel {
                        samples,
                        last_seen: batch.end_time,
                    }),
                );
            }
            if next.cells.len() > config.max_surface_voxels {
                return Err(Error::Capacity);
            }
        }
        next.next_sequence = batch.sequence.checked_add(1).ok_or(Error::InvalidUpdate)?;
        next.time = Some(batch.end_time);
        next.pose = Some(batch.pose);
        next.resolution = Some(batch.resolution);
        Ok(next)
    }
}
