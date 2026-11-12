use terrain_map::*;

fn config() -> Config {
    Config {
        resolution: 1.,
        half_extent: [20.; 3],
        max_range: 50.,
        ..Config::default()
    }
}
fn ray(x: f64) -> Ray {
    Ray {
        origin: [0.5; 3],
        endpoint: [x, 0.5, 0.5],
    }
}
fn frame(time: f64, rays: Vec<Ray>) -> Frame {
    Frame {
        time,
        center: [0.; 3],
        rays,
    }
}
fn cells(snapshot: &MapSnapshot) -> Vec<(VoxelKey, i8)> {
    let mut cells: Vec<_> = snapshot
        .map
        .observed()
        .map(|(k, v)| (k, v.evidence))
        .collect();
    cells.sort();
    cells
}

// 空闲仅来自实际射线，负坐标和角点穿越必须按 floor 正确分格。
#[test]
fn ray_evidence_unknown_negative_and_diagonal() {
    let mut mapper = Mapper::new(config()).unwrap();
    let out = mapper
        .update(&frame(
            1.,
            vec![
                ray(3.5),
                ray(-2.5),
                Ray {
                    origin: [0.5; 3],
                    endpoint: [3.5; 3],
                },
            ],
        ))
        .unwrap();
    assert_eq!(out.map.occupancy([1.5, 0.5, 0.5]), Occupancy::Free);
    assert_eq!(out.map.occupancy([3.5, 0.5, 0.5]), Occupancy::Occupied);
    assert_eq!(out.map.occupancy([-2.5, 0.5, 0.5]), Occupancy::Occupied);
    assert_eq!(out.map.occupancy([1.5; 3]), Occupancy::Free);
    assert_eq!(out.map.occupancy([3.5; 3]), Occupancy::Occupied);
    assert_eq!(out.map.occupancy([0.5, 1.5, 0.5]), Occupancy::Unknown);
    assert_eq!(out.map.key([-0.01; 3]), Some(VoxelKey(-1, -1, -1)));
}

// 超量程回波只清除已截断射线，不能在截断处凭空制造障碍。
#[test]
fn range_limit_does_not_create_a_hit() {
    let mut mapper = Mapper::new(Config {
        max_range: 2.,
        ..config()
    })
    .unwrap();
    let out = mapper.update(&frame(1., vec![ray(9.5)])).unwrap();
    assert_eq!(out.map.occupancy([2.5, 0.5, 0.5]), Occupancy::Free);
    assert_eq!(out.map.occupancy([3.5, 0.5, 0.5]), Occupancy::Unknown);
}

// 同帧回波优先于穿透空闲，重复点和并行合并顺序不改变证据。
#[test]
fn parallel_updates_equal_serial_with_hit_precedence() {
    let mut a = Mapper::new(Config {
        cpu_threads: 1,
        ..config()
    })
    .unwrap();
    let mut b = Mapper::new(Config {
        cpu_threads: 4,
        ..config()
    })
    .unwrap();
    let mut rays = vec![ray(1.5); 1000];
    rays.extend(vec![ray(3.5); 1000]);
    rays.extend((0..2048).map(|i| Ray {
        origin: [0.15, -0.15, 1.05],
        endpoint: [
            f64::from(i % 37) - 18.2,
            f64::from(i % 23) - 11.1,
            f64::from(i % 7) - 3.2,
        ],
    }));
    let out = a.update(&frame(1., rays.clone())).unwrap();
    rays.reverse();
    let other = b.update(&frame(1., rays)).unwrap();
    assert_eq!(cells(&out), cells(&other));
    assert_eq!(out.map.voxel(VoxelKey(1, 0, 0)).unwrap().evidence, 2);
}

// 参数、混合坏帧、非递增时刻和容量拒绝均不得提交部分地图。
#[test]
fn rejected_updates_preserve_version_and_state() {
    assert!(matches!(
        Mapper::new(Config {
            cpu_threads: 0,
            ..config()
        }),
        Err(Error::InvalidConfig)
    ));
    let mut mapper = Mapper::new(config()).unwrap();
    let old = mapper.update(&frame(1., vec![ray(1.5)])).unwrap();
    assert!(matches!(
        mapper.update(&frame(1., vec![])),
        Err(Error::NonMonotonicTime)
    ));
    let mut invalid = vec![ray(9.5); 1000];
    invalid.push(ray(f64::NAN));
    assert!(matches!(
        mapper.update(&frame(2., invalid)),
        Err(Error::InvalidFrame)
    ));
    assert_eq!(mapper.snapshot().revision, old.revision);
    assert_eq!(cells(&mapper.snapshot()), cells(&old));
    let mut limited = Mapper::new(Config {
        max_blocks: 1,
        ..config()
    })
    .unwrap();
    assert!(matches!(
        limited.update(&frame(1., vec![ray(17.5)])),
        Err(Error::Capacity)
    ));
    assert_eq!(limited.snapshot().map.block_count(), 0);
    assert_eq!(limited.snapshot().revision, 0);
    // 射线步数限制同样必须拒绝整帧，不能悄悄截断回波。
    let mut short = Mapper::new(Config {
        max_ray_voxels: 2,
        ..config()
    })
    .unwrap();
    assert!(matches!(
        short.update(&frame(1., vec![ray(3.5)])),
        Err(Error::Capacity)
    ));
    assert_eq!(short.snapshot().revision, 0);
}

// 历史快照不可随新帧修改，连续空闲证据可消除旧障碍。
#[test]
fn snapshots_are_immutable_and_free_observations_clear_old_hits() {
    let mut mapper = Mapper::new(config()).unwrap();
    let old = mapper.update(&frame(1., vec![ray(1.5)])).unwrap();
    for time in 2..6 {
        mapper
            .update(&frame(f64::from(time), vec![ray(3.5)]))
            .unwrap();
    }
    assert_eq!(old.map.occupancy([1.5, 0.5, 0.5]), Occupancy::Occupied);
    assert_eq!(
        mapper.snapshot().map.occupancy([1.5, 0.5, 0.5]),
        Occupancy::Free
    );
}

// 过期和局部范围之外都查询为未知，不能继续作为自由空间使用。
#[test]
fn aging_and_local_window_prune_observations() {
    let mut mapper = Mapper::new(Config {
        max_age: 1.,
        half_extent: [4.; 3],
        ..config()
    })
    .unwrap();
    mapper.update(&frame(1., vec![ray(1.5)])).unwrap();
    let expired = mapper.update(&frame(3., vec![])).unwrap();
    assert_eq!(expired.map.block_count(), 0);
    assert_eq!(expired.map.occupancy([1.5, 0.5, 0.5]), Occupancy::Unknown);
    mapper.update(&frame(4., vec![ray(1.5)])).unwrap();
    let mut moved = frame(5., vec![]);
    moved.center = [50.; 3];
    let out = mapper.update(&moved).unwrap();
    assert_eq!(out.map.block_count(), 0);
}

// 人工提示不是识别结论，强制禁行可独立生效且版本与地图版本分开。
#[test]
fn manual_annotations_are_distinct_from_detection() {
    let mut mapper = Mapper::new(config()).unwrap();
    let a = Annotation {
        id: 1,
        min: [-1.; 3],
        max: [1.; 3],
        kind: AnnotationKind::TunnelHint,
    };
    let hint = mapper.upsert_annotation(a.clone()).unwrap();
    assert!(!hint.manually_forbidden([0.; 3]));
    let forbidden = mapper
        .upsert_annotation(Annotation {
            kind: AnnotationKind::Forbidden,
            ..a
        })
        .unwrap();
    assert!(forbidden.manually_forbidden([0.; 3]));
    assert_eq!(forbidden.map.version, 0);
    assert_eq!(forbidden.annotation_version, 2);
    assert_eq!(forbidden.annotations.len(), 1);
    let removed = mapper.remove_annotation(1).unwrap();
    assert!(!removed.manually_forbidden([0.; 3]));
    assert!(matches!(
        mapper.remove_annotation(1),
        Err(Error::UnknownAnnotation)
    ));
    assert!(matches!(
        mapper.upsert_annotation(Annotation {
            id: 2,
            min: [0.; 3],
            max: [f64::NAN; 3],
            kind: AnnotationKind::Forbidden
        }),
        Err(Error::InvalidAnnotation)
    ));
    assert_eq!(mapper.snapshot().revision, 3);
}

// 负方向回波落在精确格边界时不能继续步进越过终点，误报容量耗尽。
#[test]
fn negative_endpoint_on_grid_boundary_terminates() {
    let mut mapper = Mapper::new(config()).unwrap();
    let out = mapper
        .update(&frame(
            1.,
            vec![Ray {
                origin: [0.05, 0.05, 1.05],
                endpoint: [-9.42, 3.35, -2.],
            }],
        ))
        .unwrap();
    assert_eq!(out.map.occupancy([-9.42, 3.35, -2.]), Occupancy::Occupied);
}

// 同块其他格的新观测不能刷新旧格的寿命；过期证据重置后重新累计。
#[test]
fn stale_cell_inside_live_block_is_unknown() {
    let mut mapper = Mapper::new(Config {
        max_age: 1.,
        ..config()
    })
    .unwrap();
    mapper.update(&frame(1., vec![ray(1.5)])).unwrap();
    let unrelated = Ray {
        origin: [2.5, 2.5, 2.5],
        endpoint: [3.5, 2.5, 2.5],
    };
    let out = mapper.update(&frame(3., vec![unrelated])).unwrap();
    assert_eq!(out.map.block_count(), 1);
    assert_eq!(out.map.occupancy([1.5, 0.5, 0.5]), Occupancy::Unknown);
    let new = mapper.update(&frame(4., vec![ray(1.5)])).unwrap();
    assert_eq!(new.map.voxel(VoxelKey(1, 0, 0)).unwrap().evidence, 2);
}
