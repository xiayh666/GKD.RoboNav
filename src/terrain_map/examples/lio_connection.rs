//! Synthetic end-to-end example: ordered LIO computation on spawn_blocking,
//! Arc observation delivery, and terrain updates on its existing bounded runtime.
use point_lio::{Config as LioConfig, ImuSample, LidarPoint, PointLio, Scan, Vector3};
use terrain_map::{map_updates::MapUpdate, Config, TerrainRuntime};

fn imu(time: f64) -> ImuSample {
    ImuSample {
        time,
        angular_velocity: Vector3::zeros(),
        acceleration: Vector3::z() * 9.81,
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let runtime = TerrainRuntime::start(Config::default())?;
    let handle = runtime.handle();
    let mut source = PointLio::new_with_source(
        LioConfig {
            map_capacity: 5,
            ..LioConfig::default()
        },
        1,
    )?;
    handle
        .submit_update(MapUpdate::Reset(source.observation_reset()))
        .await
        .map_err(|e| e.error)?
        .wait()
        .await?;
    for frame in 0..3 {
        let (returned, output) = tokio::task::spawn_blocking(move || {
            if frame == 0 {
                for i in 0..100 {
                    source.push_imu(imu(f64::from(i) * 0.005))?;
                }
            }
            let begin = if frame == 0 { 100 } else { 101 + frame * 20 };
            for i in begin..=120 + frame * 20 {
                source.push_imu(imu(f64::from(i) * 0.005))?;
            }
            let start = 0.5 + f64::from(frame) * 0.1;
            let scan = Scan {
                start_time: start,
                end_time: start + 0.1,
                points: (0..49)
                    .map(|i| LidarPoint {
                        position: Vector3::new(
                            1.05,
                            f64::from(frame) * 2. + 0.05 + f64::from(i % 7) * 0.31,
                            0.05 + f64::from(i / 7) * 0.31,
                        ),
                        intensity: 1.,
                        offset: 0.05,
                    })
                    .collect(),
            };
            let output = source.process_scan(&scan)?;
            Ok::<_, point_lio::Error>((source, output))
        })
        .await??;
        source = returned;
        let sequence = output.observations.sequence;
        let ivox_cells = output.map_voxels;
        let map = handle
            .submit_update(MapUpdate::Observations(output.observations))
            .await
            .map_err(|e| e.error)?
            .wait()
            .await?;
        println!(
            "sequence={sequence}, ivox_voxels={ivox_cells}, terrain_voxels={}, status={:?}",
            map.surfaces.voxel_count(),
            map.surfaces.status
        );
    }
    runtime.shutdown().await?;
    Ok(())
}
