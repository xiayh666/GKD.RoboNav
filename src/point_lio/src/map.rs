use nalgebra::{SMatrix, SVector, Vector3};
use std::collections::HashMap;

type Key = (i64, i64, i64);

struct Node {
    points: Vec<Vector3<f64>>,
    previous: Option<Key>,
    next: Option<Key>,
}

/// Incremental voxel map: preserve points within a voxel, touch on insertion,
/// evict the least recently written voxel. Search the 26 adjacent cells.
/// This follows the author's DEFAULT iVox node and MapIncremental contracts.
pub(crate) struct VoxelMap {
    size: f64,
    capacity: usize,
    cells: HashMap<Key, Node>,
    head: Option<Key>,
    tail: Option<Key>,
    points: usize,
}

impl VoxelMap {
    pub fn new(size: f64, capacity: usize) -> Self {
        Self {
            size,
            capacity,
            cells: HashMap::new(),
            head: None,
            tail: None,
            points: 0,
        }
    }

    fn key(&self, p: Vector3<f64>) -> Key {
        (
            (p.x / self.size).floor() as i64,
            (p.y / self.size).floor() as i64,
            (p.z / self.size).floor() as i64,
        )
    }

    pub fn len(&self) -> usize {
        self.points
    }
    pub fn voxels(&self) -> usize {
        self.cells.len()
    }

    fn touch(&mut self, key: Key) {
        if self.head == Some(key) {
            return;
        }
        let node = &self.cells[&key];
        let (previous, next) = (node.previous, node.next);
        if let Some(p) = previous {
            self.cells.get_mut(&p).unwrap().next = next;
        }
        if let Some(n) = next {
            self.cells.get_mut(&n).unwrap().previous = previous;
        } else {
            self.tail = previous;
        }
        let old = self.head;
        if let Some(h) = old {
            self.cells.get_mut(&h).unwrap().previous = Some(key);
        }
        let node = self.cells.get_mut(&key).unwrap();
        node.previous = None;
        node.next = old;
        self.head = Some(key);
        if self.tail.is_none() {
            self.tail = Some(key);
        }
    }

    pub fn insert(&mut self, p: Vector3<f64>) {
        let key = self.key(p);
        if let Some(node) = self.cells.get_mut(&key) {
            node.points.push(p);
            self.points += 1;
            self.touch(key);
            return;
        }
        if self.cells.len() == self.capacity {
            let key = self.tail.unwrap();
            let old = self.cells.remove(&key).unwrap();
            self.points -= old.points.len();
            self.tail = old.previous;
            if let Some(t) = self.tail {
                self.cells.get_mut(&t).unwrap().next = None;
            } else {
                self.head = None;
            }
        }
        let old = self.head;
        self.cells.insert(
            key,
            Node {
                points: vec![p],
                previous: None,
                next: old,
            },
        );
        if let Some(h) = old {
            self.cells.get_mut(&h).unwrap().previous = Some(key);
        } else {
            self.tail = Some(key);
        }
        self.head = Some(key);
        self.points += 1;
    }

    /// The five closest points within the iVox 26-neighborhood and distance cap.
    pub fn closest(&self, p: Vector3<f64>, radius: f64) -> Vec<Vector3<f64>> {
        let centre = self.key(p);
        let mut nearest: Vec<(f64, Vector3<f64>)> = Vec::with_capacity(6);
        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    let Some(kx) = centre.0.checked_add(x) else {
                        continue;
                    };
                    let Some(ky) = centre.1.checked_add(y) else {
                        continue;
                    };
                    let Some(kz) = centre.2.checked_add(z) else {
                        continue;
                    };
                    if let Some(node) = self.cells.get(&(kx, ky, kz)) {
                        for q in &node.points {
                            let d = (q - p).norm_squared();
                            if d >= radius * radius {
                                continue;
                            }
                            let at = nearest.partition_point(|v| v.0 <= d);
                            if at < 5 {
                                nearest.insert(at, (d, *q));
                                if nearest.len() > 5 {
                                    nearest.pop();
                                }
                            }
                        }
                    }
                }
            }
        }
        nearest.into_iter().map(|(_, q)| q).collect()
    }

    /// Author's esti_plane: solve A*n = -1 with column-pivoted QR, normalize,
    /// and validate all five neighbors; use Eigen-style basic solutions when rank deficient.
    pub fn fit_plane(points: &[Vector3<f64>], threshold: f64) -> Option<(Vector3<f64>, f64)> {
        if points.len() != 5 {
            return None;
        }
        let a = SMatrix::<f64, 5, 3>::from_fn(|i, j| points[i][j]);
        let qr = a.col_piv_qr();
        let r = qr.r();
        let rank_threshold = r[(0, 0)].abs() * f64::EPSILON * 3.0;
        let rank = r
            .diagonal()
            .iter()
            .take_while(|v| v.abs() > rank_threshold)
            .count();
        if rank == 0 {
            return None;
        }
        let b = qr.q().transpose() * SVector::<f64, 5>::repeat(-1.0);
        let mut n = Vector3::zeros();
        // Match Eigen ColPivHouseholderQR's basic solution for rank-deficient
        // systems. As upstream, acceptance still requires all five residuals.
        for i in (0..rank).rev() {
            let tail: f64 = (i + 1..rank).map(|j| r[(i, j)] * n[j]).sum();
            n[i] = (b[i] - tail) / r[(i, i)];
        }
        qr.p().inv_permute_rows(&mut n);
        let length = n.norm();
        if !length.is_finite() || length < 1e-12 {
            return None;
        }
        let normal = n / length;
        let offset = 1.0 / length;
        if points
            .iter()
            .any(|q| (normal.dot(q) + offset).abs() > threshold)
        {
            return None;
        }
        Some((normal, offset))
    }

    /// Frame decisions use neighbors cached before the frame is inserted.
    pub fn insert_frame(
        &mut self,
        world: &[Vector3<f64>],
        neighbors: &[Vec<Vector3<f64>>],
        filter_size_map: f64,
    ) {
        let bootstrap = self.cells.is_empty();
        for (point, nearby) in world.iter().zip(neighbors) {
            let centre = point.map(|v| (v / filter_size_map).floor() + 0.5) * filter_size_map;
            let occupied = nearby.iter().any(|q| {
                let d = (q - centre).abs();
                d.x < filter_size_map * 0.5
                    && d.y < filter_size_map * 0.5
                    && d.z < filter_size_map * 0.5
            });
            if bootstrap || !occupied {
                self.insert(*point);
            }
        }
    }

    #[cfg(test)]
    fn plane(&self, p: Vector3<f64>, radius: f64, threshold: f64) -> Option<(Vector3<f64>, f64)> {
        Self::fit_plane(&self.closest(p, radius), threshold)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // iVox 初始节点必须保留同体素的多点，才能从密集扫描中构造局部平面。
    #[test]
    fn initial_voxel_keeps_multiple_returns() {
        let mut map = VoxelMap::new(1.0, 5);
        let points = vec![
            Vector3::new(0.1, 0.1, 0.5),
            Vector3::new(0.2, 0.1, 0.5),
            Vector3::new(0.1, 0.2, 0.5),
            Vector3::new(0.2, 0.2, 0.5),
            Vector3::new(0.3, 0.2, 0.5),
        ];
        for p in &points {
            map.insert(*p);
        }
        assert_eq!(map.len(), 5);
        assert_eq!(map.voxels(), 1);
        let neighbors = map.closest(Vector3::new(0.15, 0.15, 0.5), 5.0);
        assert_eq!(neighbors.len(), 5);
        assert!(VoxelMap::fit_plane(&neighbors, 0.01).is_some());
        map.insert_frame(&points, &vec![neighbors; 5], 1.0);
        assert_eq!(map.len(), 5);
    }

    // 插点筛选格可比索引格大或小，负坐标仍按 floor 分格。
    #[test]
    fn insertion_grid_is_independent_of_index_grid() {
        for sign in [1.0, -1.0] {
            for (index_size, insertion_size, expected_points, expected_voxels) in
                [(0.2, 0.5, 1, 1), (0.5, 0.2, 2, 1)]
            {
                let mut map = VoxelMap::new(index_size, 100);
                let old = Vector3::repeat(sign * 0.1);
                let new = Vector3::repeat(sign * 0.3);
                map.insert(old);
                let neighbors = map.closest(new, 5.0);
                assert_eq!(neighbors, vec![old]);
                map.insert_frame(&[new], &[neighbors], insertion_size);
                assert_eq!(map.len(), expected_points);
                assert_eq!(map.voxels(), expected_voxels);
            }
        }
    }

    // 重写过的体素应保留，淘汰未被重写的旧体素，双向链接不得循环或丢失。
    #[test]
    fn eviction_tracks_last_write_not_first_insertion() {
        let mut map = VoxelMap::new(1.0, 3);
        for x in [0.1, 1.1, 2.1, 0.2, 3.1] {
            map.insert(Vector3::new(x, 0.1, 0.1));
        }
        assert!(map.cells.contains_key(&(0, 0, 0)));
        assert!(!map.cells.contains_key(&(1, 0, 0)));
        let mut key = map.head;
        let mut seen = Vec::new();
        while let Some(k) = key {
            assert!(!seen.contains(&k));
            seen.push(k);
            key = map.cells[&k].next;
        }
        assert_eq!(seen.len(), map.voxels());
        assert_eq!(seen.last().copied(), map.tail);
    }

    // 官方 QR 平面约束在非退化邻居中成立，共线邻居不能形成有效约束。
    #[test]
    fn qr_plane_and_degenerate_neighbors() {
        let mut map = VoxelMap::new(0.3, 100);
        for x in -2..=2 {
            for y in -2..=2 {
                map.insert(Vector3::new(x as f64 * 0.2, y as f64 * 0.2, -1.5));
            }
        }
        let (n, d) = map
            .plane(Vector3::new(0.05, 0.05, -1.4), 0.5, 0.01)
            .unwrap();
        assert!(n.z.abs() > 0.999);
        assert!((d.abs() - 1.5).abs() < 1e-10);
        let mut line = VoxelMap::new(0.1, 100);
        for x in -3..=3 {
            line.insert(Vector3::new(x as f64 * 0.2, 0., 0.));
        }
        assert!(line.plane(Vector3::zeros(), 0.5, 0.01).is_none());
    }

    // 地图容量和插入顺序同步有界，不能靠无限积累点云运行。
    #[test]
    fn capacity_is_bounded() {
        let mut map = VoxelMap::new(0.1, 5);
        for i in 0..100 {
            map.insert(Vector3::new(i as f64, 0., 0.));
        }
        assert_eq!(map.len(), 5);
        assert_eq!(map.voxels(), 5);
        assert!(!map.cells.contains_key(&(0, 0, 0)));
    }
}
