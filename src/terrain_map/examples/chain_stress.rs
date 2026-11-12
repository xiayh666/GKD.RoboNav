//! Repeatable synthetic stress harness. CSV timings use monotonic clocks.
//! `chain`: actual LIO + bounded runtime; `direct`/`burst`: isolate terrain/export.
use point_lio::{
    Config as LioConfig, ImuSample, LidarPoint, PointLio, Scan, UnitQuaternion, Vector3,
};
use std::{
    error::Error,
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};
use terrain_map::{
    map_updates::{MapUpdate, Observation, ObservationBatch, ObservationReset, Pose, SessionId},
    Config, Mapper, StreamStatus, TerrainRuntime,
};
type AnyError = Box<dyn Error + Send + Sync>;
fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.
}
fn imu(time: f64, moving: bool) -> ImuSample {
    let t = (time - 0.5).max(0.);
    ImuSample {
        time,
        angular_velocity: Vector3::new(
            (time * 13.).sin() * 0.001,
            (time * 17.).cos() * 0.001,
            if moving { 0.06 * (0.3 * t).cos() } else { 0. } + (time * 19.).sin() * 0.001,
        ),
        acceleration: Vector3::z() * 9.81
            + Vector3::new((time * 11.).sin(), (time * 7.).cos(), (time * 23.).sin()) * 0.02,
    }
}
fn rotation(time: f64, moving: bool) -> UnitQuaternion<f64> {
    UnitQuaternion::from_euler_angles(
        0.,
        0.,
        if moving {
            0.2 * (0.3 * (time - 0.5).max(0.)).sin()
        } else {
            0.
        },
    )
}
fn landmarks(n: usize) -> Vec<Vector3<f64>> {
    let w = ((n as f64 / 3.).sqrt().ceil() as usize).max(1);
    (0..n)
        .map(|i| {
            let j = i / 3;
            let a = ((j % w) as f64 + 0.5) / w as f64 * 5.4 - 2.7;
            let b = ((j / w) as f64 + 0.5) / w as f64 * 5.4 - 2.7;
            match i % 3 {
                0 => Vector3::new(a, b, -1.5),
                1 => Vector3::new(3., a, b),
                _ => Vector3::new(a, 3., b),
            }
        })
        .collect()
}
fn scan(frame: usize, world: &[Vector3<f64>], moving: bool) -> Scan {
    let start = 0.5 + frame as f64 * 0.1;
    Scan {
        start_time: start,
        end_time: start + 0.1,
        points: world
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let offset = 0.099 * i as f64 / world.len() as f64;
                let t = frame as f64 * 0.31 + i as f64 * 1.73;
                LidarPoint {
                    position: rotation(start + offset, moving).inverse() * p
                        + Vector3::new(t.sin(), (t * 1.3).sin(), (t * 1.7).cos()) * 0.01,
                    offset,
                    intensity: 1.,
                }
            })
            .collect(),
    }
}
fn header() {
    println!("frame,lio_ms,export_rebuild_ms,admission_ms,ack_ms,e2e_ms,scheduled_ms,pending,terrain_voxels,samples,ivox_points,matches,position_error_m,angle_error_rad");
}
#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), AnyError> {
    let a: Vec<_> = std::env::args().collect();
    let mode = a.get(1).map(String::as_str).unwrap_or("chain");
    let n: usize = a.get(2).map(String::as_str).unwrap_or("10000").parse()?;
    let frames: usize = a.get(3).map(String::as_str).unwrap_or("600").parse()?;
    let hz: f64 = a.get(4).map(String::as_str).unwrap_or("10").parse()?;
    let threads: usize = a.get(5).map(String::as_str).unwrap_or("2").parse()?;
    let moving = a.get(6).is_some_and(|v| v == "moving");
    assert!(n > 0 && frames > 0 && hz.is_finite() && hz >= 0.);
    let config = Config {
        cpu_threads: threads,
        ..Config::default()
    };
    header();
    if mode == "direct" || mode == "burst" || mode == "lifecycle" {
        return isolated(mode, n, frames, hz, config).await;
    }
    assert_eq!(mode, "chain");
    let runtime = TerrainRuntime::start(config)?;
    let handle = runtime.handle();
    let mut source = PointLio::new_with_source(LioConfig::default(), 1)?;
    for i in 0..100 {
        source.push_imu(ImuSample {
            time: i as f64 * 0.005,
            angular_velocity: Vector3::zeros(),
            acceleration: Vector3::z() * 9.81,
        })?;
    }
    handle
        .submit_update(MapUpdate::Reset(source.observation_reset()))
        .await
        .map_err(|e| e.error)?
        .wait()
        .await?;
    let world = Arc::new(landmarks(n));
    let origin = Instant::now();
    let pending = Arc::new(AtomicUsize::new(0));
    let mut tasks = Vec::new();
    let mut busy = 0;
    for frame in 0..frames {
        let scheduled = if hz > 0. {
            origin + Duration::from_secs_f64(frame as f64 / hz)
        } else {
            Instant::now()
        };
        if hz > 0. {
            tokio::time::sleep_until(scheduled.into()).await;
        }
        let begin = Instant::now();
        let world = Arc::clone(&world);
        let (returned, out, lio_ms) =
            tokio::task::spawn_blocking(move || -> Result<_, point_lio::Error> {
                let input = scan(frame, &world, moving); // Generation excluded from LIO timer, included in e2e.
                let clock = Instant::now();
                let first = if frame == 0 { 100 } else { 101 + frame * 20 };
                for tick in first..=120 + frame * 20 {
                    source.push_imu(imu(tick as f64 * 0.005, moving))?;
                }
                let out = source.process_scan(&input)?;
                Ok((source, out, ms(clock.elapsed())))
            })
            .await??;
        source = returned;
        assert!(out.covariance.iter().all(|v| v.is_finite()));
        assert!(source.buffered_imu_len() <= 1);
        assert_eq!(out.observations.point_count(), n);
        let pos = out.state.position.norm();
        let angle = out
            .state
            .orientation
            .angle_to(&rotation(out.state.time, moving));
        let points = out.map_points;
        let matches = out.matched_points;
        let submit = Instant::now();
        let receipt = match handle.try_submit_update(MapUpdate::Observations(out.observations)) {
            Ok(r) => r,
            Err(e) => {
                assert_eq!(e.error, terrain_map::Error::Busy);
                busy += 1;
                handle.submit_update(e.update).await.map_err(|e| e.error)?
            }
        };
        let admission = ms(submit.elapsed());
        let admitted = Instant::now();
        let count = pending.fetch_add(1, Ordering::SeqCst) + 1;
        let pending = Arc::clone(&pending);
        tasks.push(tokio::spawn(async move {
            let snapshot=receipt.wait().await?;let finished=Instant::now();pending.fetch_sub(1,Ordering::SeqCst);
            assert_eq!(snapshot.surfaces.status,StreamStatus::Ready);
            assert_eq!(snapshot.surfaces.next_sequence,frame as u64+2);
            let samples=if frame%100==0 || frame+1==frames {snapshot.surfaces.observed().map(|(_,v)|{assert!(v.samples.len()<=256);v.samples.len()}).sum::<usize>()} else {0};
            println!("{frame},{lio_ms:.6},0,{admission:.6},{:.6},{:.6},{:.6},{count},{},{samples},{points},{matches},{pos:.9},{angle:.9}",ms(finished.duration_since(admitted)),ms(finished.duration_since(begin)),ms(finished.saturating_duration_since(scheduled)),snapshot.surfaces.voxel_count());
            Ok::<_,terrain_map::Error>(())
        }));
        // Do not retain completed task outputs for the whole soak.
        if tasks.len() >= 100 {
            for t in tasks.drain(..) {
                t.await??;
            }
        }
    }
    for t in tasks {
        t.await??;
    }
    let end = handle.latest();
    assert_eq!(end.surfaces.next_sequence, frames as u64 + 1);
    assert_eq!(pending.load(Ordering::SeqCst), 0);
    runtime.shutdown().await?;
    eprintln!("mode=chain frames={frames} points={n} hz={hz} threads={threads} busy={busy} elapsed_s={:.3}",origin.elapsed().as_secs_f64());
    Ok(())
}
async fn isolated(
    mode: &str,
    n: usize,
    frames: usize,
    hz: f64,
    config: Config,
) -> Result<(), AnyError> {
    let session = SessionId {
        source: 2,
        epoch: 0,
    };
    let reset = ObservationReset {
        session,
        next_sequence: 1,
    };
    let mut pose = Pose {
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
    };
    let world = landmarks(n);
    let origin = Instant::now();
    let mut busy = 0;
    let mut mapper = Mapper::new(config.clone())?;
    mapper.apply_update(&MapUpdate::Reset(reset))?;
    let runtime = TerrainRuntime::start(config)?;
    let handle = runtime.handle();
    handle
        .submit_update(MapUpdate::Reset(reset))
        .await
        .map_err(|e| e.error)?
        .wait()
        .await?;
    let mut tasks = Vec::new();
    let pending = Arc::new(AtomicUsize::new(0));
    for frame in 0..frames {
        let scheduled = if hz > 0. {
            origin + Duration::from_secs_f64(frame as f64 / hz)
        } else {
            Instant::now()
        };
        if hz > 0. {
            tokio::time::sleep_until(scheduled.into()).await;
        }
        let begin = Instant::now();
        let start = frame as f64 * 0.1;
        // Same grouping/validation routine as LIO export; diagnostic, separate workload.
        let clock = Instant::now();
        if mode == "lifecycle" {
            pose.position[0] = frame as f64 * 0.05;
        }
        let batch = Arc::new(ObservationBatch::from_points(
            ObservationReset {
                session,
                next_sequence: frame as u64 + 1,
            },
            start,
            start + 0.1,
            pose,
            0.3,
            world.iter().enumerate().map(|(i, p)| Observation {
                time: start + 0.099 * i as f64 / n as f64,
                origin: [0.; 3],
                endpoint: [p.x + pose.position[0], p.y, p.z],
            }),
        )?);
        let export = ms(clock.elapsed());
        if mode == "direct" || mode == "lifecycle" {
            let clock = Instant::now();
            let s = mapper.apply_update(&MapUpdate::Observations(batch))?;
            let commit = ms(clock.elapsed());
            let samples = if frame % 100 == 0 || frame + 1 == frames {
                s.surfaces.observed().map(|(_, v)| v.samples.len()).sum()
            } else {
                0
            };
            println!(
                "{frame},0,{export:.6},0,{commit:.6},{:.6},{:.6},0,{},{samples},0,0,0,0",
                ms(begin.elapsed()),
                ms(scheduled.elapsed()),
                s.surfaces.voxel_count()
            );
        } else {
            let submit = Instant::now();
            let receipt = match handle.try_submit_update(MapUpdate::Observations(batch)) {
                Ok(r) => r,
                Err(e) => {
                    assert_eq!(e.error, terrain_map::Error::Busy);
                    busy += 1;
                    handle.submit_update(e.update).await.map_err(|e| e.error)?
                }
            };
            let admission = ms(submit.elapsed());
            let admitted = Instant::now();
            let count = pending.fetch_add(1, Ordering::SeqCst) + 1;
            let p = Arc::clone(&pending);
            tasks.push(tokio::spawn(async move {
                let s = receipt.wait().await?;
                let end = Instant::now();
                p.fetch_sub(1, Ordering::SeqCst);
                assert_eq!(s.surfaces.next_sequence, frame as u64 + 2);
                println!(
                    "{frame},0,{export:.6},{admission:.6},{:.6},{:.6},{:.6},{count},{},0,0,0,0,0",
                    ms(end.duration_since(admitted)),
                    ms(end.duration_since(begin)),
                    ms(end.duration_since(scheduled)),
                    s.surfaces.voxel_count()
                );
                Ok::<_, terrain_map::Error>(())
            }));
            if tasks.len() >= 100 {
                for t in tasks.drain(..) {
                    t.await??;
                }
            }
        }
    }
    for t in tasks {
        t.await??;
    }
    if mode == "lifecycle" {
        let retained = mapper.snapshot();
        let count = retained.surfaces.voxel_count();
        assert!(count > 0);
        for (key, v) in retained.surfaces.observed() {
            assert!(v.samples.len() <= 256);
            assert!((key.center(0.3)[0] - pose.position[0]).abs() <= 15.);
        }
        let time = frames as f64 * 0.1 + 31.;
        let empty = ObservationBatch::from_points(
            ObservationReset {
                session,
                next_sequence: frames as u64 + 1,
            },
            time,
            time,
            pose,
            0.3,
            [],
        )?;
        let cleared = mapper.apply_update(&MapUpdate::Observations(Arc::new(empty)))?;
        assert_eq!(cleared.surfaces.voxel_count(), 0);
        assert_eq!(retained.surfaces.voxel_count(), count);
        eprintln!("lifecycle: window_and_sample_bounds=true expired_voxels={count} after_ttl=0 retained_snapshot_unchanged=true");
    }
    runtime.shutdown().await?;
    eprintln!(
        "mode={mode} frames={frames} points={n} hz={hz} busy={busy} elapsed_s={:.3}",
        origin.elapsed().as_secs_f64()
    );
    Ok(())
}
