use std::sync::Arc;
use terrain_map::{
    map_updates::*, Annotation, AnnotationKind, Config, Error, Mapper, StreamStatus, TerrainRuntime,
};

fn reset() -> ObservationReset {
    ObservationReset {
        session: SessionId {
            source: 9,
            epoch: 0,
        },
        next_sequence: 1,
    }
}
fn batch(sequence: u64) -> Arc<ObservationBatch> {
    Arc::new(
        ObservationBatch::from_points(
            ObservationReset {
                next_sequence: sequence,
                ..reset()
            },
            sequence as f64 - 0.1,
            sequence as f64,
            Pose {
                position: [0.; 3],
                orientation: [0., 0., 0., 1.],
            },
            0.3,
            [Observation {
                time: sequence as f64,
                origin: [0.; 3],
                endpoint: [1.05, 0.05, 0.05],
            }],
        )
        .unwrap(),
    )
}
fn start(config: Config) -> Mapper {
    let mut mapper = Mapper::new(config).unwrap();
    mapper.apply_update(&MapUpdate::Reset(reset())).unwrap();
    mapper
}
fn annotation() -> Annotation {
    Annotation {
        id: 1,
        min: [0.; 3],
        max: [2.; 3],
        kind: AnnotationKind::Forbidden,
    }
}

// 使用上游键及分辨率，首批样本可直接共享Arc；默认不重新积分射线。
#[test]
fn consumes_grouped_geometry_without_revoxelizing_or_ray_mapping() {
    let mut mapper = start(Config {
        resolution: 0.07,
        ..Config::default()
    });
    let input = batch(1);
    let out = mapper
        .apply_update(&MapUpdate::Observations(Arc::clone(&input)))
        .unwrap();
    assert_eq!(out.surfaces.resolution, Some(0.3));
    assert_eq!(out.surfaces.voxel_count(), 1);
    let cell = out.surfaces.voxel(input.voxels[0].key).unwrap();
    assert!(Arc::ptr_eq(&cell.samples, &input.voxels[0].points));
    assert_eq!(out.map.block_count(), 0);
    assert_eq!(out.surfaces.next_sequence, 2);
}

// 每次重观测都更新表面证据，历史快照保持不变且每格样本上限有效。
#[test]
fn repeated_observations_are_retained_independently_of_cache_changes() {
    let mut mapper = start(Config {
        max_surface_samples_per_voxel: 2,
        ..Config::default()
    });
    let old = mapper
        .apply_update(&MapUpdate::Observations(batch(1)))
        .unwrap();
    for i in 2..=4 {
        mapper
            .apply_update(&MapUpdate::Observations(batch(i)))
            .unwrap();
    }
    let key = batch(1).voxels[0].key;
    assert_eq!(old.surfaces.voxel(key).unwrap().samples.len(), 1);
    let latest = mapper.snapshot();
    let samples = &latest.surfaces.voxel(key).unwrap().samples;
    assert_eq!(samples.len(), 2);
    assert_eq!(samples[0].time, 3.);
    assert_eq!(samples[1].time, 4.);
}

// 丢序必须清空可用视图并阻止融合；同会话显式重同步保留人工标注。
#[test]
fn gap_invalidates_geometry_until_explicit_resync() {
    let mut mapper = start(Config::default());
    mapper
        .apply_update(&MapUpdate::Observations(batch(1)))
        .unwrap();
    mapper.upsert_annotation(annotation()).unwrap();
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Observations(batch(3))),
        Err(Error::SequenceGap {
            expected: 2,
            received: 3
        })
    ));
    assert_eq!(mapper.snapshot().surfaces.voxel_count(), 0);
    assert!(matches!(
        mapper.stream_status(),
        StreamStatus::NeedsReset { .. }
    ));
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Observations(batch(2))),
        Err(Error::NeedsReset)
    ));
    mapper
        .apply_update(&MapUpdate::Reset(ObservationReset {
            next_sequence: 4,
            ..reset()
        }))
        .unwrap();
    let out = mapper
        .apply_update(&MapUpdate::Observations(batch(4)))
        .unwrap();
    assert_eq!(out.surfaces.status, StreamStatus::Ready);
    assert_eq!(out.annotations.len(), 1);
    // 迟到的旧重置和旧批次不可清空当前健康视图。
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Reset(reset())),
        Err(Error::OutOfOrder)
    ));
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Observations(batch(1))),
        Err(Error::OutOfOrder)
    ));
    assert_eq!(mapper.snapshot().surfaces.voxel_count(), 1);
}

// 更换里程计会话不能沿用旧世界系地图或人工框；旧会话晚到消息应拒绝。
#[test]
fn session_reset_invalidates_old_coordinates_and_annotations() {
    let mut mapper = start(Config::default());
    mapper.upsert_annotation(annotation()).unwrap();
    mapper
        .apply_update(&MapUpdate::Observations(batch(1)))
        .unwrap();
    let new = ObservationReset {
        session: SessionId {
            epoch: 1,
            ..reset().session
        },
        next_sequence: 1,
    };
    let out = mapper.apply_update(&MapUpdate::Reset(new)).unwrap();
    assert_eq!(out.annotations.len(), 0);
    assert_eq!(out.surfaces.voxel_count(), 0);
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Observations(batch(2))),
        Err(Error::SessionMismatch)
    ));
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Reset(reset())),
        Err(Error::SessionMismatch)
    ));
}

// 非法键或容量拒绝不能推进序号或部分提交；调用方可修正同一序号后重试。
#[test]
fn invalid_and_over_budget_batches_are_transactional() {
    let mut mapper = start(Config {
        max_surface_voxels: 1,
        ..Config::default()
    });
    let mut invalid = (*batch(1)).clone();
    invalid.voxels = vec![ObservedVoxel {
        key: VoxelKey(999, 0, 0),
        points: Arc::clone(&invalid.voxels[0].points),
    }]
    .into();
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Observations(Arc::new(invalid))),
        Err(Error::InvalidUpdate)
    ));
    assert_eq!(mapper.snapshot().surfaces.next_sequence, 1);
    let too_many = ObservationBatch::from_points(
        reset(),
        0.,
        1.,
        batch(1).pose,
        0.3,
        [
            Observation {
                time: 1.,
                origin: [0.; 3],
                endpoint: [1.05, 0.05, 0.05],
            },
            Observation {
                time: 1.,
                origin: [0.; 3],
                endpoint: [2.05, 0.05, 0.05],
            },
        ],
    )
    .unwrap();
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Observations(Arc::new(too_many))),
        Err(Error::Capacity)
    ));
    assert_eq!(mapper.snapshot().surfaces.voxel_count(), 0);
    mapper
        .apply_update(&MapUpdate::Observations(batch(1)))
        .unwrap();
    assert_eq!(mapper.snapshot().surfaces.next_sequence, 2);
}

// 空间范围/老化独立于上游缓存；开启射线时才维护额外自由空间证据。
#[test]
fn surface_expiration_and_optional_free_space() {
    let mut mapper = start(Config {
        max_age: 0.5,
        integrate_observation_rays: true,
        ..Config::default()
    });
    let out = mapper
        .apply_update(&MapUpdate::Observations(batch(1)))
        .unwrap();
    assert!(out.map.block_count() > 0);
    let mut empty = (*batch(2)).clone();
    empty.voxels = Arc::from([]);
    let out = mapper
        .apply_update(&MapUpdate::Observations(Arc::new(empty)))
        .unwrap();
    assert_eq!(out.surfaces.voxel_count(), 0);
}

// 即使本次回执是丢序错误，watch也必须公布失效状态，不能继续保留健康结果。
#[tokio::test(flavor = "current_thread")]
async fn async_stream_publishes_gap_health_and_preserves_retry_payload() {
    let runtime = TerrainRuntime::start(Config {
        input_capacity: 1,
        ..Config::default()
    })
    .unwrap();
    let handle = runtime.handle();
    let receipt = handle.try_submit_update(MapUpdate::Reset(reset())).unwrap();
    let rejected = handle
        .try_submit_update(MapUpdate::Observations(batch(1)))
        .err()
        .unwrap();
    assert_eq!(rejected.error, Error::Busy);
    receipt.wait().await.unwrap();
    handle
        .submit_update(rejected.update)
        .await
        .unwrap()
        .wait()
        .await
        .unwrap();
    let mut changes = handle.subscribe();
    changes.borrow_and_update();
    assert!(matches!(
        handle
            .submit_update(MapUpdate::Observations(batch(3)))
            .await
            .unwrap()
            .wait()
            .await,
        Err(Error::SequenceGap { .. })
    ));
    changes.changed().await.unwrap();
    assert!(matches!(
        changes.borrow().surfaces.status,
        StreamStatus::NeedsReset { .. }
    ));
    runtime.shutdown().await.unwrap();
}

// 切换生产者后，旧源的迟到Reset也不能重新激活旧坐标系。
#[test]
fn retired_source_reset_cannot_roll_back_current_session() {
    let mut mapper = start(Config::default());
    let other = ObservationReset {
        session: SessionId {
            source: 10,
            epoch: 0,
        },
        next_sequence: 1,
    };
    mapper.apply_update(&MapUpdate::Reset(other)).unwrap();
    assert!(matches!(
        mapper.apply_update(&MapUpdate::Reset(reset())),
        Err(Error::SessionMismatch)
    ));
    let refreshed = ObservationReset {
        session: SessionId {
            epoch: 1,
            ..reset().session
        },
        next_sequence: 1,
    };
    mapper.apply_update(&MapUpdate::Reset(refreshed)).unwrap();
    assert_eq!(mapper.snapshot().surfaces.session, Some(refreshed.session));
}

// UI仍显示旧坐标系时排在Reset之后的命令，不能误应用到新地图。
#[tokio::test(flavor = "current_thread")]
async fn queued_annotation_is_bound_to_its_displayed_session() {
    let runtime = TerrainRuntime::start(Config {
        input_capacity: 4,
        ..Config::default()
    })
    .unwrap();
    let handle = runtime.handle();
    handle
        .submit_update(MapUpdate::Reset(reset()))
        .await
        .unwrap()
        .wait()
        .await
        .unwrap();
    let new = ObservationReset {
        session: SessionId {
            epoch: 1,
            ..reset().session
        },
        next_sequence: 1,
    };
    let receipt = handle.try_submit_update(MapUpdate::Reset(new)).unwrap();
    assert!(matches!(
        handle.upsert_annotation(annotation()).await,
        Err(Error::SessionMismatch)
    ));
    receipt.wait().await.unwrap();
    assert!(handle.latest().annotations.is_empty());
    handle
        .upsert_annotation_in_session(new.session, annotation())
        .await
        .unwrap();
    runtime.shutdown().await.unwrap();
}
