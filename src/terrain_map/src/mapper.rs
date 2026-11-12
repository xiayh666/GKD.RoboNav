use crate::{Annotation, AnnotationKind, Config, Error, Frame, Point3, VoxelSnapshot};
use std::sync::Arc;

/// A complete consistent state, not a delta. Geometry analysis is not yet present.
#[derive(Clone)]
pub struct MapSnapshot {
    pub revision: u64,
    pub map: VoxelSnapshot,
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
            map: VoxelSnapshot::new(config),
            revision: 0,
            annotation_version: 0,
            annotations: Arc::from([]),
        })
    }
    pub fn update(&mut self, frame: &Frame) -> Result<Arc<MapSnapshot>, Error> {
        self.map.update(frame, &self.pool)?;
        self.revision += 1;
        Ok(self.snapshot())
    }
    pub fn upsert_annotation(&mut self, annotation: Annotation) -> Result<Arc<MapSnapshot>, Error> {
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
            annotation_version: self.annotation_version,
            annotations: Arc::clone(&self.annotations),
        })
    }
}
