use crate::{Annotation, AnnotationKind, Config, Error, Frame, Point3, VoxelSnapshot};
use crate::{Ray, StreamStatus, SurfaceSnapshot};
use map_updates::{MapUpdate, SessionId};
use std::sync::Arc;

/// A complete consistent state, not a delta. Geometry analysis is not yet present.
#[derive(Clone)]
pub struct MapSnapshot {
    pub revision: u64,
    pub map: VoxelSnapshot,
    /// Primary geometry view from upstream voxel observations.
    pub surfaces: SurfaceSnapshot,
    pub annotation_version: u64,
    pub annotations: Arc<[Annotation]>,
}

impl MapSnapshot {
    /// Only explicit manual forbiddance. False is NOT proof of traversability.
    pub fn manually_forbidden(&self, point: Point3) -> bool {
        self.annotations
            .iter()
            .any(|a| a.kind == AnnotationKind::Forbidden && a.contains(point))
    }
}

/// Synchronous owner. A private bounded pool parallelizes work within one frame.
pub struct Mapper {
    pool: rayon::ThreadPool,
    config: Config,
    surfaces: SurfaceSnapshot,
    map: VoxelSnapshot,
    revision: u64,
    annotation_version: u64,
    annotations: Arc<[Annotation]>,
}

impl Mapper {
    pub fn new(config: Config) -> Result<Self, Error> {
        config.validate()?;
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(config.cpu_threads)
            .thread_name(|i| format!("terrain-cpu-{i}"))
            .build()
            .map_err(|e| Error::Worker(e.to_string()))?;
        Ok(Self {
            pool,
            map: VoxelSnapshot::new(config.clone()),
            surfaces: SurfaceSnapshot::new(&config),
            config,
            revision: 0,
            annotation_version: 0,
            annotations: Arc::from([]),
        })
    }
    pub fn update(&mut self, frame: &Frame) -> Result<Arc<MapSnapshot>, Error> {
        if self.surfaces.session.is_some() {
            return Err(Error::MixedInput);
        }
        self.map.update(frame, &self.pool)?;
        self.revision += 1;
        Ok(self.snapshot())
    }
    /// Primary input. Reset is explicit; gaps invalidate geometry and require resync.
    pub fn apply_update(&mut self, update: &MapUpdate) -> Result<Arc<MapSnapshot>, Error> {
        match update {
            MapUpdate::Reset(reset) => {
                let next = self.surfaces.reset(*reset, &self.config)?;
                // A changed odometry session invalidates annotations in its old frame.
                if self.surfaces.session != Some(reset.session) && !self.annotations.is_empty() {
                    self.annotations = Arc::from([]);
                    self.annotation_version += 1;
                }
                self.surfaces = next;
                self.map = VoxelSnapshot::new(self.config.clone());
            }
            MapUpdate::Observations(batch) => {
                let next = match self.surfaces.prepare(batch, &self.config) {
                    Ok(next) => next,
                    Err(Error::SequenceGap { expected, received }) => {
                        self.surfaces.invalidate(expected, received);
                        self.map = VoxelSnapshot::new(self.config.clone());
                        self.revision += 1;
                        return Err(Error::SequenceGap { expected, received });
                    }
                    Err(error) => return Err(error),
                };
                if self.config.integrate_observation_rays {
                    let frame = Frame {
                        time: batch.end_time,
                        center: batch.pose.position,
                        rays: batch
                            .voxels
                            .iter()
                            .flat_map(|v| v.points.iter())
                            .map(|p| Ray {
                                origin: p.origin,
                                endpoint: p.endpoint,
                            })
                            .collect(),
                    };
                    self.map.update(&frame, &self.pool)?;
                }
                self.surfaces = next;
            }
        }
        self.revision += 1;
        Ok(self.snapshot())
    }
    pub fn stream_status(&self) -> StreamStatus {
        self.surfaces.status
    }

    pub fn upsert_annotation(&mut self, annotation: Annotation) -> Result<Arc<MapSnapshot>, Error> {
        self.upsert_annotation_for(self.surfaces.session, annotation)
    }
    pub fn upsert_annotation_for(
        &mut self,
        session: Option<SessionId>,
        annotation: Annotation,
    ) -> Result<Arc<MapSnapshot>, Error> {
        if self.surfaces.session != session {
            return Err(Error::SessionMismatch);
        }

        annotation.validate()?;
        let mut items = self.annotations.to_vec();
        if let Some(item) = items.iter_mut().find(|a| a.id == annotation.id) {
            *item = annotation;
        } else {
            if items.len() >= 1024 {
                return Err(Error::Capacity);
            }
            items.push(annotation);
        }
        items.sort_by_key(|a| a.id);
        self.annotations = items.into();
        self.annotation_version += 1;
        self.revision += 1;
        Ok(self.snapshot())
    }
    pub fn remove_annotation(&mut self, id: u64) -> Result<Arc<MapSnapshot>, Error> {
        self.remove_annotation_for(self.surfaces.session, id)
    }
    pub fn remove_annotation_for(
        &mut self,
        session: Option<SessionId>,
        id: u64,
    ) -> Result<Arc<MapSnapshot>, Error> {
        if self.surfaces.session != session {
            return Err(Error::SessionMismatch);
        }

        if !self.annotations.iter().any(|a| a.id == id) {
            return Err(Error::UnknownAnnotation);
        }
        self.annotations = self
            .annotations
            .iter()
            .filter(|a| a.id != id)
            .cloned()
            .collect();
        self.annotation_version += 1;
        self.revision += 1;
        Ok(self.snapshot())
    }
    pub fn snapshot(&self) -> Arc<MapSnapshot> {
        Arc::new(MapSnapshot {
            revision: self.revision,
            map: self.map.clone(),
            surfaces: self.surfaces.clone(),
            annotation_version: self.annotation_version,
            annotations: Arc::clone(&self.annotations),
        })
    }
}
