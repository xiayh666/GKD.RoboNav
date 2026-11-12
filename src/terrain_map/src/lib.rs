//! Sparse local voxel mapping with bounded asynchronous admission and frame-local
//! CPU parallelism. Geometry and tunnel classification are not yet implemented.
//!
//! Inputs are motion-compensated returns and acquisition-time ray origins in one
//! gravity-aligned metric frame. Unknown space is never treated as observed free.

mod mapper;
mod runtime;
mod types;
mod voxel;

pub use mapper::{MapSnapshot, Mapper};
pub use runtime::{MapHandle, Receipt, RejectedFrame, TerrainRuntime};
pub use types::{Annotation, AnnotationKind, Config, Error, Frame, Point3, Ray};
pub use voxel::{Occupancy, Voxel, VoxelKey, VoxelSnapshot};
