use std::{error, fmt};

pub type Point3 = [f64; 3];

/// A return and its acquisition-time sensor origin, both in one gravity-aligned
/// metric frame. No transform or motion compensation is performed by this crate.
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Point3,
    pub endpoint: Point3,
}

#[derive(Clone, Debug)]
pub struct Frame {
    /// Strictly increasing sensor time, in seconds.
    pub time: f64,
    /// Center of the local map, not a substitute for per-return ray origins.
    pub center: Point3,
    pub rays: Vec<Ray>,
}

#[derive(Clone, Debug)]
pub struct Config {
    pub resolution: f64,
    pub half_extent: Point3,
    pub max_age: f64,
    pub max_range: f64,
    pub max_blocks: usize,
    pub max_points: usize,
    pub max_ray_voxels: usize,
    pub max_update_voxels: usize,
    pub input_capacity: usize,
    /// Private Rayon pool, never the global pool. Frames commit sequentially.
    pub cpu_threads: usize,
    pub max_surface_voxels: usize,
    pub max_surface_samples_per_voxel: usize,
    /// Optional ray evidence alongside producer-keyed geometry. Default off.
    pub integrate_observation_rays: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            resolution: 0.1,
            half_extent: [15.0, 15.0, 5.0],
            max_age: 30.0,
            max_range: 30.0,
            max_blocks: 8192,
            max_points: 100_000,
            max_ray_voxels: 2048,
            max_update_voxels: 1_000_000,
            input_capacity: 2,
            cpu_threads: 2,
            max_surface_voxels: 200_000,
            max_surface_samples_per_voxel: 256,
            integrate_observation_rays: false,
        }
    }
}

impl Config {
    pub(crate) fn validate(&self) -> Result<(), Error> {
        if [self.resolution, self.max_age, self.max_range]
            .into_iter()
            .chain(self.half_extent)
            .any(|v| !v.is_finite() || v <= 0.0)
            || [
                self.max_blocks,
                self.max_points,
                self.max_ray_voxels,
                self.max_update_voxels,
                self.input_capacity,
                self.cpu_threads,
                self.max_surface_voxels,
                self.max_surface_samples_per_voxel,
            ]
            .contains(&0)
            || self.cpu_threads > 64
            || self.input_capacity > 1024
        {
            return Err(Error::InvalidConfig);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Error {
    InvalidConfig,
    InvalidFrame,
    InvalidUpdate,
    AwaitingReset,
    SessionMismatch,
    OutOfOrder,
    SequenceGap { expected: u64, received: u64 },
    NeedsReset,
    MixedInput,
    NonMonotonicTime,
    Capacity,
    InvalidAnnotation,
    UnknownAnnotation,
    Busy,
    Closed,
    NoRuntime,
    Worker(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}
impl error::Error for Error {}

/// Axis-aligned approximate region in the map frame. A hint is not a detection.
#[derive(Clone, Debug, PartialEq)]
pub struct Annotation {
    pub id: u64,
    pub min: Point3,
    pub max: Point3,
    pub kind: AnnotationKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnnotationKind {
    TunnelHint,
    Forbidden,
}

impl Annotation {
    pub(crate) fn validate(&self) -> Result<(), Error> {
        if (0..3).any(|i| {
            !self.min[i].is_finite() || !self.max[i].is_finite() || self.min[i] >= self.max[i]
        }) {
            return Err(Error::InvalidAnnotation);
        }
        Ok(())
    }
    pub fn contains(&self, point: Point3) -> bool {
        (0..3).all(|i| point[i] >= self.min[i] && point[i] <= self.max[i])
    }
}
