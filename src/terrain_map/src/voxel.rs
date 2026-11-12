use crate::{Config, Error, Frame, Point3, Ray};
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;

const SIDE: i32 = 8;
const COUNT: usize = 512;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VoxelKey(pub i32, pub i32, pub i32);

impl VoxelKey {
    fn axes(self) -> [i32; 3] {
        [self.0, self.1, self.2]
    }
    fn block(self) -> Self {
        Self(
            self.0.div_euclid(SIDE),
            self.1.div_euclid(SIDE),
            self.2.div_euclid(SIDE),
        )
    }
    fn offset(self) -> usize {
        (self.0.rem_euclid(SIDE)
            + SIDE * self.1.rem_euclid(SIDE)
            + SIDE * SIDE * self.2.rem_euclid(SIDE)) as usize
    }
    pub fn center(self, resolution: f64) -> Point3 {
        self.axes().map(|v| (f64::from(v) + 0.5) * resolution)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Occupancy {
    Unknown,
    Free,
    Occupied,
}

#[derive(Clone, Copy, Debug)]
pub struct Voxel {
    /// Bounded evidence score, not a calibrated probability.
    pub evidence: i8,
    pub last_seen: f64,
}
impl Default for Voxel {
    fn default() -> Self {
        Self {
            evidence: 0,
            last_seen: f64::NEG_INFINITY,
        }
    }
}

#[derive(Clone)]
struct Block {
    voxels: [Voxel; COUNT],
    last_seen: f64,
}
impl Default for Block {
    fn default() -> Self {
        Self {
            voxels: [Voxel::default(); COUNT],
            last_seen: f64::NEG_INFINITY,
        }
    }
}

/// Immutable block snapshot; unchanged blocks are shared, updated blocks use COW.
#[derive(Clone)]
pub struct VoxelSnapshot {
    config: Config,
    blocks: HashMap<VoxelKey, Arc<Block>>,
    pub time: Option<f64>,
    pub center: Point3,
    pub version: u64,
}

impl VoxelSnapshot {
    pub(crate) fn new(config: Config) -> Self {
        Self {
            config,
            blocks: HashMap::new(),
            time: None,
            center: [0.; 3],
            version: 0,
        }
    }
    pub fn resolution(&self) -> f64 {
        self.config.resolution
    }
    pub fn block_count(&self) -> usize {
        self.blocks.len()
    }
    pub fn key(&self, point: Point3) -> Option<VoxelKey> {
        key(point, self.config.resolution).ok()
    }
    pub fn occupancy(&self, point: Point3) -> Occupancy {
        self.key(point)
            .map_or(Occupancy::Unknown, |k| self.occupancy_at(k))
    }
    pub fn occupancy_at(&self, key: VoxelKey) -> Occupancy {
        self.voxel(key)
            .map_or(Occupancy::Unknown, |v| match v.evidence.cmp(&0) {
                std::cmp::Ordering::Greater => Occupancy::Occupied,
                std::cmp::Ordering::Less => Occupancy::Free,
                std::cmp::Ordering::Equal => Occupancy::Unknown,
            })
    }
    pub fn voxel(&self, key: VoxelKey) -> Option<Voxel> {
        let time = self.time?;
        if !inside(key, self.center, &self.config) {
            return None;
        }
        let voxel = self.blocks.get(&key.block())?.voxels[key.offset()];
        (time - voxel.last_seen <= self.config.max_age).then_some(voxel)
    }
    /// Observed non-stale cells, including zero-score unknown cells. Order unspecified.
    pub fn observed(&self) -> impl Iterator<Item = (VoxelKey, Voxel)> + '_ {
        self.blocks.iter().flat_map(move |(block_key, block)| {
            block.voxels.iter().enumerate().filter_map(move |(i, _)| {
                let k = VoxelKey(
                    block_key.0 * SIDE + (i % 8) as i32,
                    block_key.1 * SIDE + ((i / 8) % 8) as i32,
                    block_key.2 * SIDE + (i / 64) as i32,
                );
                self.voxel(k).map(|v| (k, v))
            })
        })
    }
    pub(crate) fn update(&mut self, frame: &Frame, pool: &rayon::ThreadPool) -> Result<(), Error> {
        validate_frame(frame, &self.config)?;
        if self.time.is_some_and(|t| frame.time <= t) {
            return Err(Error::NonMonotonicTime);
        }
        let config = &self.config;
        // Each worker owns its evidence sets. Hash iteration order cannot alter
        // the score: one hit/miss per cell per frame, and a hit wins over misses.
        let updates = pool.install(|| {
            frame
                .rays
                .par_chunks(256)
                .map(|rays| {
                    let mut delta = Delta::default();
                    for ray in rays {
                        trace(*ray, frame.center, config, &mut delta)?;
                    }
                    Ok(delta)
                })
                .try_reduce(Delta::default, |mut a, b| {
                    a.merge(b);
                    if a.cells > config.max_update_voxels {
                        return Err(Error::Capacity);
                    }
                    Ok(a)
                })
        })?;
        if updates.cells > config.max_update_voxels {
            return Err(Error::Capacity);
        }
        let live = |k: &VoxelKey, b: &Arc<Block>| {
            frame.time - b.last_seen <= config.max_age && block_inside(*k, frame.center, config)
        };
        let mut blocks: HashSet<_> = self
            .blocks
            .iter()
            .filter(|(k, b)| live(k, b))
            .map(|(k, _)| *k)
            .collect();
        blocks.extend(updates.blocks.keys().copied());
        if blocks.len() > config.max_blocks {
            return Err(Error::Capacity);
        }
        // All fallible input/capacity work completes before the state commit.
        self.blocks.retain(|k, b| live(k, b));
        for (block_key, bits) in updates.blocks {
            let block = Arc::make_mut(
                self.blocks
                    .entry(block_key)
                    .or_insert_with(|| Arc::new(Block::default())),
            );
            for word in 0..8 {
                let mut changed = bits.free[word] | bits.hits[word];
                while changed != 0 {
                    let bit = changed.trailing_zeros() as usize;
                    changed &= changed - 1;
                    let hit = bits.hits[word] & (1 << bit) != 0;
                    let voxel = &mut block.voxels[word * 64 + bit];
                    if frame.time - voxel.last_seen > config.max_age {
                        voxel.evidence = 0;
                    }
                    voxel.evidence = (voxel.evidence + if hit { 2 } else { -1 }).clamp(-8, 8);
                    voxel.last_seen = frame.time;
                }
            }
            block.last_seen = frame.time;
        }
        self.center = frame.center;
        self.time = Some(frame.time);
        self.version += 1;
        Ok(())
    }
}

#[derive(Default)]
struct Bits {
    free: [u64; 8],
    hits: [u64; 8],
}

#[derive(Default)]
struct Delta {
    blocks: HashMap<VoxelKey, Bits>,
    cells: usize,
}
impl Delta {
    fn insert(&mut self, cell: VoxelKey, hit: bool) {
        let bits = self.blocks.entry(cell.block()).or_default();
        let offset = cell.offset();
        let word = offset / 64;
        let mask = 1 << (offset % 64);
        let before = bits.free[word] | bits.hits[word];
        if hit {
            bits.hits[word] |= mask;
        } else {
            bits.free[word] |= mask;
        }
        self.cells += usize::from(before & mask == 0);
    }
    fn merge(&mut self, other: Self) {
        for (key, b) in other.blocks {
            let a = self.blocks.entry(key).or_default();
            for word in 0..8 {
                let before = (a.free[word] | a.hits[word]).count_ones();
                a.free[word] |= b.free[word];
                a.hits[word] |= b.hits[word];
                self.cells += ((a.free[word] | a.hits[word]).count_ones() - before) as usize;
            }
        }
    }
}

fn key(p: Point3, resolution: f64) -> Result<VoxelKey, Error> {
    let q = p.map(|v| (v / resolution).floor());
    if q.iter().any(|v| !v.is_finite() || v.abs() > 1_000_000_000.) {
        return Err(Error::InvalidFrame);
    }
    Ok(VoxelKey(q[0] as i32, q[1] as i32, q[2] as i32))
}
fn inside(key: VoxelKey, center: Point3, c: &Config) -> bool {
    let p = key.center(c.resolution);
    (0..3).all(|i| (p[i] - center[i]).abs() <= c.half_extent[i])
}
fn block_inside(key: VoxelKey, center: Point3, c: &Config) -> bool {
    let p = key.axes().map(|v| (f64::from(v) * 8. + 4.) * c.resolution);
    (0..3).all(|i| (p[i] - center[i]).abs() <= c.half_extent[i] + 4. * c.resolution)
}
fn validate_frame(f: &Frame, c: &Config) -> Result<(), Error> {
    if !f.time.is_finite() || f.time < 0. || f.rays.len() > c.max_points {
        return Err(Error::InvalidFrame);
    }
    key(f.center, c.resolution)?;
    for ray in &f.rays {
        key(ray.origin, c.resolution)?;
        key(ray.endpoint, c.resolution)?;
    }
    Ok(())
}

fn trace(ray: Ray, center: Point3, c: &Config, delta: &mut Delta) -> Result<(), Error> {
    let direction: Point3 = std::array::from_fn(|i| ray.endpoint[i] - ray.origin[i]);
    let distance = direction.iter().map(|x| x * x).sum::<f64>().sqrt();
    let clipped = distance > c.max_range;
    let endpoint = if clipped {
        std::array::from_fn(|i| ray.origin[i] + direction[i] * c.max_range / distance)
    } else {
        ray.endpoint
    };
    let direction: Point3 = std::array::from_fn(|i| endpoint[i] - ray.origin[i]);
    let mut axes = key(ray.origin, c.resolution)?.axes();
    let end = key(endpoint, c.resolution)?;
    let step: [i32; 3] = direction.map(|d| {
        if d > 0. {
            1
        } else if d < 0. {
            -1
        } else {
            0
        }
    });
    let dt = direction.map(|d| {
        if d == 0. {
            f64::INFINITY
        } else {
            c.resolution / d.abs()
        }
    });
    let mut next: Point3 = std::array::from_fn(|i| {
        if step[i] == 0 {
            f64::INFINITY
        } else {
            let edge = f64::from(axes[i] + i32::from(step[i] > 0)) * c.resolution;
            (edge - ray.origin[i]) / direction[i]
        }
    });
    for _ in 0..c.max_ray_voxels {
        let cell = VoxelKey(axes[0], axes[1], axes[2]);
        if cell == end {
            if inside(cell, center, c) {
                if clipped {
                    delta.insert(cell, false);
                } else {
                    delta.insert(cell, true);
                }
            }
            return Ok(());
        }
        if inside(cell, center, c) {
            delta.insert(cell, false);
        }
        let minimum = next.into_iter().fold(f64::INFINITY, f64::min);
        // Negative-direction endpoints on exact boundaries can have a floor key
        // different from the infinitesimal continuation of the ray. Stop at t=1
        // before stepping beyond the measured segment.
        if minimum >= 1.0 - 1e-12 {
            if inside(end, center, c) {
                if clipped {
                    delta.insert(end, false);
                } else {
                    delta.insert(end, true);
                }
            }
            return Ok(());
        }
        // Simultaneous boundary crossings do not visit zero-length edge cells.
        for i in 0..3 {
            if next[i] <= minimum + 1e-12 {
                axes[i] += step[i];
                next[i] += dt[i];
            }
        }
        if delta.cells > c.max_update_voxels {
            return Err(Error::Capacity);
        }
    }
    Err(Error::Capacity)
}
