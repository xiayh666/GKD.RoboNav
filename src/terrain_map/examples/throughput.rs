//! Synthetic volumetric ray workload; excludes input generation and pool creation.
//! cargo run --release -p terrain-map --example throughput -- 10000 5
use std::time::Instant;
use terrain_map::{Config, Frame, Mapper, Ray};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let count: usize = args.next().map_or(Ok(10_000), |s| s.parse())?;
    let frames: usize = args.next().map_or(Ok(5), |s| s.parse())?;
    if count == 0 || count > 100_000 || frames == 0 {
        return Err("points 1..100000, frames > 0 required".into());
    }
    let rays: Vec<_> = (0..count)
        .map(|i| {
            let yaw = i as f64 * 2.399963229728653;
            let z = 4. * ((i * 7919 % 10007) as f64 / 10007.) - 2.;
            Ray {
                origin: [0.05, 0.05, 1.05],
                endpoint: [10. * yaw.cos(), 10. * yaw.sin(), z],
            }
        })
        .collect();
    for threads in [1, 2, 4] {
        let mut mapper = Mapper::new(Config {
            cpu_threads: threads,
            ..Config::default()
        })?;
        let mut times = Vec::with_capacity(frames);
        let mut blocks = 0;
        for i in 0..=frames {
            let frame = Frame {
                time: i as f64 * 0.1,
                center: [0.; 3],
                rays: rays.clone(),
            };
            let begin = Instant::now();
            let output = mapper.update(&frame)?;
            let ms = begin.elapsed().as_secs_f64() * 1000.;
            blocks = output.map.block_count();
            if i > 0 {
                times.push(ms);
            }
        }
        times.sort_by(f64::total_cmp);
        println!("points={count}, threads={threads}, samples={frames}, median_ms={:.3}, min_ms={:.3}, max_ms={:.3}, blocks={blocks}",
            times[times.len()/2],times[0],times[times.len()-1]);
    }
    Ok(())
}
