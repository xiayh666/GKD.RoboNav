//! ROS-independent Point-LIO output-state filter and online point-cloud mapping.
//!
//! All timestamps use seconds on one monotonic sensor clock, positions use metres,
//! angular velocities rad/s and accelerations m/s². See the crate README for the
//! input contract and differences from the author's implementation.

mod filter;
mod map;
mod pipeline;
mod types;

pub use filter::{Covariance, State, STATE_DIM};
pub use nalgebra::{UnitQuaternion, Vector3};
pub use pipeline::PointLio;
pub use types::{Config, Error, ImuSample, LidarPoint, Scan, ScanResult};

/// Shared output protocol; independent of nalgebra, ROS and filter internals.
pub use map_updates;
