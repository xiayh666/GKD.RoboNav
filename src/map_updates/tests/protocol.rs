use map_updates::*;
use std::sync::Arc;

fn make(points: Vec<Observation>) -> ObservationBatch {
    ObservationBatch::from_points(
        ObservationReset {
            session: SessionId {
                source: 1,
                epoch: 0,
            },
            next_sequence: 1,
        },
        0.,
        1.,
        Pose {
            position: [0.; 3],
            orientation: [0., 0., 0., 1.],
        },
        0.5,
        points,
    )
    .unwrap()
}
fn point(x: f64) -> Observation {
    Observation {
        time: 0.5,
        origin: [0.; 3],
        endpoint: [x, 0., 0.],
    }
}

// 协议按floor分组并稳定排序；同格重复回波仍是观测，不能被当作缓存重复点删除。
#[test]
fn groups_negative_coordinates_and_preserves_reobservations() {
    let batch = make(vec![point(1.), point(-0.1), point(1.1)]);
    assert_eq!(batch.voxels.len(), 2);
    assert_eq!(batch.voxels[0].key, VoxelKey(-1, 0, 0));
    assert_eq!(batch.voxels[1].points.len(), 2);
    assert_eq!(batch.point_count(), 3);
}

// 非法时间、原点、姿态或键不得通过公共协议验证。
#[test]
fn rejects_invalid_coordinates_times_and_pose() {
    let mut batch = make(vec![point(1.)]);
    batch.pose.orientation = [0.; 4];
    assert!(batch.validate().is_err());
    batch.pose.orientation = [0., 0., 0., 1.];
    batch.sequence = 0;
    assert!(batch.validate().is_err());
    batch.sequence = 1;
    batch.voxels = vec![ObservedVoxel {
        key: VoxelKey(99, 0, 0),
        points: Arc::from([point(1.)]),
    }]
    .into();
    assert!(batch.validate().is_err());
    assert!(VoxelKey::from_point([f64::NAN; 3], 0.1).is_err());
    assert!(VoxelKey::from_point([1e300; 3], 0.1).is_err());
    assert!(VoxelKey::from_point([0.; 3], 0.).is_err());
    let mut p = point(1.);
    p.time = 2.;
    assert!(ObservationBatch::from_points(
        ObservationReset {
            session: batch.session,
            next_sequence: 1
        },
        0.,
        1.,
        batch.pose,
        0.5,
        [p]
    )
    .is_err());
    p.time = 0.5;
    p.origin[0] = f64::NAN;
    assert!(ObservationBatch::from_points(
        ObservationReset {
            session: batch.session,
            next_sequence: 1
        },
        0.,
        1.,
        batch.pose,
        0.5,
        [p]
    )
    .is_err());
}
