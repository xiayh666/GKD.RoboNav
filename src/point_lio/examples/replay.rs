//! `cargo run --release -p point-lio --example replay -- events.csv`
//! No path: run a synthetic stationary-room demonstration.
use point_lio::{Config, ImuSample, LidarPoint, PointLio, Scan, ScanResult, Vector3};
use std::{
    error::Error,
    fs::File,
    io::{self, BufRead, BufReader, Write},
};

fn output(result: &ScanResult, out: &mut impl Write) -> io::Result<()> {
    let s = &result.state;
    let q = s.orientation.quaternion();
    writeln!(
        out,
        "{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{}",
        s.time,
        s.position.x,
        s.position.y,
        s.position.z,
        q.i,
        q.j,
        q.k,
        q.w,
        result.matched_points,
        result.map_points
    )
}

fn replay(
    input: impl BufRead,
    lio: &mut PointLio,
    out: &mut impl Write,
    ros_f32: bool,
) -> Result<(), Box<dyn Error>> {
    let mut points = Vec::<(f64, LidarPoint)>::new();
    for (line_index, line) in input.lines().enumerate() {
        let line = line?;
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let fields: Vec<_> = line.split(',').map(str::trim).collect();
        let parse = |i: usize| -> Result<f64, Box<dyn Error>> {
            Ok(fields.get(i).ok_or("missing CSV column")?.parse()?)
        };
        let result: Result<(), Box<dyn Error>> =
            (|| {
                match fields[0] {
                    "I" if fields.len() == 8 => {
                        lio.push_imu(ImuSample {
                            time: parse(1)?,
                            angular_velocity: Vector3::new(parse(2)?, parse(3)?, parse(4)?),
                            acceleration: Vector3::new(parse(5)?, parse(6)?, parse(7)?),
                        })?;
                    }
                    "P" if fields.len() == 6 => {
                        points.push((
                            parse(1)?,
                            LidarPoint {
                                position: Vector3::new(parse(2)?, parse(3)?, parse(4)?),
                                intensity: parse(5)? as f32,
                                offset: 0.0,
                            },
                        ));
                    }
                    "S" if fields.len() == 3 => {
                        let start_time = parse(1)?;
                        let end_time = parse(2)?;
                        let scan = Scan {
                            start_time,
                            end_time,
                            points: points
                                .drain(..)
                                .map(|(time, mut p)| {
                                    p.offset = time - start_time;
                                    if ros_f32 {
                                        p.position = p.position.map(|v| v as f32 as f64);
                                        p.offset = (p.offset as f32 * 1000.0) as f64 / 1000.0;
                                    }
                                    p
                                })
                                .collect(),
                        };
                        output(&lio.process_scan(&scan)?, out)?;
                    }
                    _ => return Err(
                        "expected I,time,gx,gy,gz,ax,ay,az / P,time,x,y,z,intensity / S,start,end"
                            .into(),
                    ),
                }
                Ok(())
            })();
        result.map_err(|e| format!("line {}: {e}", line_index + 1))?;
    }
    if !points.is_empty() {
        return Err("uncommitted points: missing final S record".into());
    }
    Ok(())
}

fn demo(lio: &mut PointLio, out: &mut impl Write) -> Result<(), Box<dyn Error>> {
    for i in 0..100 {
        lio.push_imu(ImuSample {
            time: i as f64 * 0.005,
            angular_velocity: Vector3::zeros(),
            acceleration: Vector3::z() * 9.81,
        })?;
    }
    for frame in 0..3 {
        for i in 0..=20 {
            if frame > 0 && i == 0 {
                continue;
            }
            lio.push_imu(ImuSample {
                time: (100 + frame * 20 + i) as f64 * 0.005,
                angular_velocity: Vector3::zeros(),
                acceleration: Vector3::z() * 9.81,
            })?;
        }
        let mut points = Vec::new();
        for x in -6..=6 {
            for y in -6..=6 {
                points.push(LidarPoint {
                    position: Vector3::new(x as f64 * 0.25, y as f64 * 0.25, -1.5),
                    offset: 0.05,
                    intensity: 1.,
                });
            }
        }
        let start = 0.5 + frame as f64 * 0.1;
        output(
            &lio.process_scan(&Scan {
                start_time: start,
                end_time: start + 0.1,
                points,
            })?,
            out,
        )?;
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    // Replace identity extrinsics and noise defaults with your calibrated values.
    let mut config = Config::default();
    let mut path = None;
    let mut ros_f32 = false;
    for arg in std::env::args().skip(1) {
        if let Some(value) = arg.strip_prefix("--map-capacity=") {
            config.map_capacity = value.parse()?;
        } else if let Some(value) = arg.strip_prefix("--gyro-saturation=") {
            config.gyro_saturation = value.parse()?;
        } else if arg == "--ros-f32" {
            ros_f32 = true;
        } else if arg.starts_with('-') || path.is_some() {
            return Err(format!("unknown argument: {arg}").into());
        } else {
            path = Some(arg);
        }
    }
    let mut lio = PointLio::new(config)?;
    let mut out = io::BufWriter::new(io::stdout().lock());
    writeln!(out, "time,x,y,z,qx,qy,qz,qw,matches,map_points")?;
    if let Some(path) = path {
        replay(
            BufReader::new(File::open(path)?),
            &mut lio,
            &mut out,
            ros_f32,
        )?;
    } else {
        demo(&mut lio, &mut out)?;
    }
    Ok(())
}
