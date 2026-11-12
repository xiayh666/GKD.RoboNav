//! Fill the default 200k surface-voxel budget; check transactional overflow and TTL.
use std::{error::Error, sync::Arc, time::Instant};
use terrain_map::{map_updates::*, Config, Mapper};
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().collect();
    let points: usize = args.get(2).map(String::as_str).unwrap_or("10000").parse()?;
    let frames: usize = args.get(3).map(String::as_str).unwrap_or("120").parse()?;
    assert_eq!(points, 10000);
    assert!(frames >= 20);
    let mut mapper = Mapper::new(Config::default())?;
    let session = SessionId {
        source: 3,
        epoch: 0,
    };
    mapper.apply_update(&MapUpdate::Reset(ObservationReset {
        session,
        next_sequence: 1,
    }))?;
    let pose = Pose {
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
    };
    let batch =
        |sequence: u64, first: usize, count: usize| -> Result<ObservationBatch, InvalidUpdate> {
            let time = sequence as f64 * 0.1;
            ObservationBatch::from_points(
                ObservationReset {
                    session,
                    next_sequence: sequence,
                },
                time - 0.1,
                time,
                pose,
                0.3,
                (first..first + count).map(|i| Observation {
                    time,
                    origin: [0.; 3],
                    endpoint: [
                        (i % 100) as f64 * 0.3 - 14.85,
                        ((i / 100) % 100) as f64 * 0.3 - 14.85,
                        (i / 10000) as f64 * 0.3 - 2.85,
                    ],
                }),
            )
        };
    println!("frame,lio_ms,export_rebuild_ms,admission_ms,ack_ms,e2e_ms,scheduled_ms,pending,terrain_voxels,samples,ivox_points,matches,position_error_m,angle_error_rad");
    for frame in 0..frames {
        let begin = Instant::now();
        let input = batch(frame as u64 + 1, frame % 20 * points, points)?;
        let export = begin.elapsed().as_secs_f64() * 1000.;
        let clock = Instant::now();
        let s = mapper.apply_update(&MapUpdate::Observations(Arc::new(input)))?;
        let commit = clock.elapsed().as_secs_f64() * 1000.;
        let total = begin.elapsed().as_secs_f64() * 1000.;
        assert_eq!(s.surfaces.voxel_count(), ((frame + 1) * points).min(200000));
        println!(
            "{frame},0,{export:.6},0,{commit:.6},{total:.6},{total:.6},0,{},0,0,0,0,0",
            s.surfaces.voxel_count()
        );
    }
    let retained = mapper.snapshot();
    let seq = frames as u64 + 1;
    let overflow = MapUpdate::Observations(Arc::new(batch(seq, 200000, 1)?));
    assert!(matches!(
        mapper.apply_update(&overflow),
        Err(terrain_map::Error::Capacity)
    ));
    let after = mapper.snapshot();
    assert_eq!(after.revision, retained.revision);
    assert_eq!(after.surfaces.next_sequence, seq);
    assert_eq!(after.surfaces.voxel_count(), 200000);
    mapper.apply_update(&MapUpdate::Observations(Arc::new(batch(seq, 0, 1)?)))?;
    let time = seq as f64 * 0.1 + 31.;
    let empty = ObservationBatch::from_points(
        ObservationReset {
            session,
            next_sequence: seq + 1,
        },
        time,
        time,
        pose,
        0.3,
        [],
    )?;
    assert_eq!(
        mapper
            .apply_update(&MapUpdate::Observations(Arc::new(empty)))?
            .surfaces
            .voxel_count(),
        0
    );
    assert_eq!(retained.surfaces.voxel_count(), 200000);
    eprintln!("capacity=200000 overflow_transactional=true retry_same_sequence=true ttl_clear=true retained_snapshot_unchanged=true");
    Ok(())
}
