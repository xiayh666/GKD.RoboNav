use terrain_map::{Annotation, AnnotationKind, Config, Frame, Ray, TerrainRuntime};

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let runtime = TerrainRuntime::start(Config::default())?;
    let handle = runtime.handle();
    for i in 1..=3 {
        let input = Frame {
            time: f64::from(i) * 0.1,
            center: [0.; 3],
            rays: vec![Ray {
                origin: [0.05, 0.05, 0.55],
                endpoint: [2.05, 0.05, 0.55],
            }],
        };
        let receipt = handle
            .submit(input)
            .await
            .map_err(|rejected| rejected.error)?;
        let map = receipt.wait().await?;
        println!(
            "revision={}, map_version={}, blocks={}",
            map.revision,
            map.map.version,
            map.map.block_count()
        );
    }
    let map = handle
        .upsert_annotation(Annotation {
            id: 1,
            min: [1., -1., 0.],
            max: [4., 1., 2.],
            kind: AnnotationKind::TunnelHint,
        })
        .await?;
    println!(
        "annotation_version={}, hints={}, automatic_tunnel_detection=not_implemented",
        map.annotation_version,
        map.annotations.len()
    );
    runtime.shutdown().await?;
    Ok(())
}
