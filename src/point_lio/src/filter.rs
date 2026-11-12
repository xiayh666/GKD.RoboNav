use crate::{Config, Error, ImuSample};
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, UnitQuaternion, Vector3};

pub const STATE_DIM: usize = 30;
pub type Covariance = SMatrix<f64, STATE_DIM, STATE_DIM>;
type Tangent = SVector<f64, STATE_DIM>;

/// Right-local SO(3) errors. Block order matches upstream state_output:
/// p, R, R_LI, t_LI, v, omega, specific_force, gravity, bg, ba.
#[derive(Clone, Debug)]
pub struct State {
    pub time: f64,
    pub position: Vector3<f64>,
    pub orientation: UnitQuaternion<f64>,
    pub lidar_to_imu_rotation: UnitQuaternion<f64>,
    pub lidar_to_imu_translation: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub specific_force: Vector3<f64>,
    pub gravity: Vector3<f64>,
    pub gyro_bias: Vector3<f64>,
    pub accel_bias: Vector3<f64>,
}

pub(crate) fn hat(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0., -v.z, v.y, v.z, 0., -v.x, -v.y, v.x, 0.)
}

/// SO(3) right Jacobian, including the stable small-angle limit.
fn right_jacobian(v: Vector3<f64>) -> Matrix3<f64> {
    let t2 = v.norm_squared();
    let a = hat(v);
    let (b, c) = if t2 < 1e-8 {
        (0.5 - t2 / 24.0, 1.0 / 6.0 - t2 / 120.0)
    } else {
        let t = t2.sqrt();
        ((1.0 - t.cos()) / t2, (t - t.sin()) / (t2 * t))
    };
    Matrix3::identity() - a * b + a * a * c
}

impl State {
    pub(crate) fn initial(c: &Config, time: f64, acc: Vector3<f64>, _gyro: Vector3<f64>) -> Self {
        // World z points up; yaw is intentionally unobservable at startup.
        let up = Vector3::z() * c.gravity_magnitude;
        let orientation = UnitQuaternion::rotation_between(&acc, &up).unwrap_or_else(|| {
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
        });
        Self {
            time,
            position: Vector3::zeros(),
            orientation,
            lidar_to_imu_rotation: c.lidar_to_imu_rotation,
            lidar_to_imu_translation: c.lidar_to_imu_translation,
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            specific_force: orientation.inverse() * up,
            gravity: -up,
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
        }
    }

    pub fn lidar_point_to_world(&self, p: &Vector3<f64>) -> Vector3<f64> {
        self.orientation * (self.lidar_to_imu_rotation * p + self.lidar_to_imu_translation)
            + self.position
    }

    fn apply(&mut self, dx: &Tangent) {
        self.position += dx.fixed_rows::<3>(0);
        self.orientation *= UnitQuaternion::from_scaled_axis(dx.fixed_rows::<3>(3).into_owned());
        self.lidar_to_imu_rotation *=
            UnitQuaternion::from_scaled_axis(dx.fixed_rows::<3>(6).into_owned());
        self.lidar_to_imu_translation += dx.fixed_rows::<3>(9);
        self.velocity += dx.fixed_rows::<3>(12);
        self.angular_velocity += dx.fixed_rows::<3>(15);
        self.specific_force += dx.fixed_rows::<3>(18);
        self.gravity += dx.fixed_rows::<3>(21);
        self.gyro_bias += dx.fixed_rows::<3>(24);
        self.accel_bias += dx.fixed_rows::<3>(27);
    }
}

#[derive(Clone)]
pub(crate) struct Filter {
    pub state: State,
    pub covariance: Covariance,
    covariance_time: f64,
}

impl Filter {
    pub fn new(state: State, c: &Config) -> Self {
        let mut p = Covariance::identity() * 0.01;
        for i in 21..24 {
            p[(i, i)] = 0.0001;
        }
        for i in 24..30 {
            p[(i, i)] = 0.001;
        }
        if !c.estimate_extrinsics {
            for i in 6..12 {
                p[(i, i)] = 0.0;
            }
        }
        Self {
            covariance_time: state.time,
            state,
            covariance: p,
        }
    }

    pub fn predict_to(&mut self, time: f64, _c: &Config) -> Result<(), Error> {
        let dt = time - self.state.time;
        if dt < -1e-9 {
            return Err(Error::NonMonotonicTime);
        }
        if dt <= 0.0 {
            return Ok(());
        }
        let r = self.state.orientation.to_rotation_matrix().into_inner();
        let phi = self.state.angular_velocity * dt;
        // Preserve upstream first-order nominal propagation (not a frame ICP).
        let acceleration = r * self.state.specific_force + self.state.gravity;
        self.state.position += self.state.velocity * dt;
        self.state.velocity += acceleration * dt;
        self.state.orientation *= UnitQuaternion::from_scaled_axis(phi);
        self.state.time = time;
        self.check()
    }

    /// Covariance has its own clock: upstream's prop_at_freq_of_imu=true.
    /// Evaluate at the propagated IMU-time state, not at every LiDAR point.
    fn propagate_covariance_to(&mut self, time: f64, c: &Config) -> Result<(), Error> {
        let dt = time - self.covariance_time;
        if dt < -1e-9 {
            return Err(Error::NonMonotonicTime);
        }
        if dt <= 0.0 {
            return Ok(());
        }
        let s = &self.state;
        let r = s.orientation.to_rotation_matrix().into_inner();
        let mut f = Covariance::identity();
        let eye = Matrix3::identity();
        // Discretized output-state Jacobian from Estimator::df_dx_output.
        f.fixed_view_mut::<3, 3>(0, 12).copy_from(&(eye * dt));
        let phi = s.angular_velocity * dt;
        f.fixed_view_mut::<3, 3>(3, 3).copy_from(
            &UnitQuaternion::from_scaled_axis(-phi)
                .to_rotation_matrix()
                .into_inner(),
        );
        f.fixed_view_mut::<3, 3>(3, 15)
            .copy_from(&(right_jacobian(phi) * dt));
        f.fixed_view_mut::<3, 3>(12, 3)
            .copy_from(&(-r * hat(s.specific_force) * dt));
        f.fixed_view_mut::<3, 3>(12, 18).copy_from(&(r * dt));
        f.fixed_view_mut::<3, 3>(12, 21).copy_from(&(eye * dt));
        let mut q = Covariance::zeros();
        for (start, variance) in [
            (12, c.velocity_process_variance),
            (15, c.gyro_process_variance),
            (18, c.accel_process_variance),
            (24, c.gyro_bias_process_variance),
            (27, c.accel_bias_process_variance),
        ] {
            for i in start..start + 3 {
                q[(i, i)] = variance * dt * dt;
            }
        }
        self.covariance = f * self.covariance * f.transpose() + q;
        self.covariance_time = time;
        self.check()
    }

    fn check(&self) -> Result<(), Error> {
        let s = &self.state;
        let finite = [
            &s.position,
            &s.velocity,
            &s.angular_velocity,
            &s.specific_force,
            &s.gravity,
            &s.gyro_bias,
            &s.accel_bias,
            &s.lidar_to_imu_translation,
        ]
        .iter()
        .all(|v| v.iter().all(|x| x.is_finite()));
        if !finite
            || !s.orientation.coords.iter().all(|v| v.is_finite())
            || !s.lidar_to_imu_rotation.coords.iter().all(|v| v.is_finite())
            || !self.covariance.iter().all(|v| v.is_finite())
            || self.covariance.diagonal().iter().any(|v| *v < -1e-10)
        {
            return Err(Error::NumericalFailure);
        }
        Ok(())
    }

    // Upstream applies a single local error increment and P - K H P.
    // No additional manifold covariance reset is inserted into that algorithm.
    fn update(&mut self, h: Tangent, residual: f64, variance: f64) -> Result<(), Error> {
        let ph = self.covariance * h;
        let innovation = h.dot(&ph) + variance;
        if !innovation.is_finite() || innovation <= 0.0 {
            return Err(Error::NumericalFailure);
        }
        self.covariance -= ph * ph.transpose() / innovation;
        self.covariance = (self.covariance + self.covariance.transpose()) * 0.5;
        self.state.apply(&(ph * (residual / innovation)));
        self.check()
    }

    pub fn observe_imu(&mut self, sample: &ImuSample, c: &Config) -> Result<usize, Error> {
        self.propagate_covariance_to(sample.time, c)?;
        let mut h = SMatrix::<f64, 6, 30>::zeros();
        let mut residual = SVector::<f64, 6>::zeros();
        let mut noise = SMatrix::<f64, 6, 6>::zeros();
        let mut rejected = 0;
        for axis in 0..6 {
            let gyro = axis < 3;
            let k = axis % 3;
            let measurement = if gyro {
                sample.angular_velocity[k]
            } else {
                sample.acceleration[k]
            };
            let limit = if gyro {
                c.gyro_saturation
            } else {
                c.accel_saturation
            };
            noise[(axis, axis)] = if gyro {
                c.gyro_measurement_variance
            } else {
                c.accel_measurement_variance
            };
            if measurement.abs() >= limit * 0.99 {
                rejected += 1;
                continue;
            }
            h[(axis, 15 + axis)] = 1.0;
            h[(axis, 24 + axis)] = 1.0;
            let predicted = if gyro {
                self.state.angular_velocity[k] + self.state.gyro_bias[k]
            } else {
                self.state.specific_force[k] + self.state.accel_bias[k]
            };
            residual[axis] = measurement - predicted;
        }
        let ph = self.covariance * h.transpose();
        let innovation = h * ph + noise;
        let gain = innovation
            .cholesky()
            .ok_or(Error::NumericalFailure)?
            .solve(&ph.transpose())
            .transpose();
        self.covariance -= gain * ph.transpose();
        self.covariance = (self.covariance + self.covariance.transpose()) * 0.5;
        self.state.apply(&(gain * residual));
        self.check()?;
        Ok(rejected)
    }

    /// All simultaneous laser residuals are linearized at one predicted state.
    pub fn observe_planes(
        &mut self,
        observations: &[(Vector3<f64>, Vector3<f64>, f64)],
        c: &Config,
    ) -> Result<usize, Error> {
        let mut rows = Vec::new();
        for &(p, normal, offset) in observations {
            let distance = normal.dot(&self.state.lidar_point_to_world(&p)) + offset;
            if p.norm() > c.match_s * distance * distance {
                rows.push((
                    self.plane_jacobian(p, normal, c.estimate_extrinsics),
                    -distance,
                ));
            }
        }
        let count = rows.len();
        if count == 0 {
            return Ok(0);
        }
        if count == 1 {
            self.update(rows[0].0, rows[0].1, c.lidar_variance)?;
            return Ok(1);
        }
        let h = DMatrix::from_fn(count, 30, |i, j| rows[i].0[j]);
        let z = DVector::from_iterator(count, rows.iter().map(|r| r.1));
        let p = DMatrix::from_column_slice(30, 30, self.covariance.as_slice());
        // Solve in measurement space for small groups and state space for large
        // groups; both use variance consistently, without inverting singular
        // fixed-extrinsic prior blocks.
        let ph = &p * h.transpose();
        let (dx, next) = if count < 30 {
            let innovation = &h * &ph + DMatrix::identity(count, count) * c.lidar_variance;
            let gain = innovation
                .cholesky()
                .ok_or(Error::NumericalFailure)?
                .solve(&ph.transpose())
                .transpose();
            (&gain * z, &p - gain * ph.transpose())
        } else {
            let factor = DMatrix::identity(30, 30) + &p * h.transpose() * &h / c.lidar_variance;
            let lu = factor.lu();
            let next = lu.solve(&p).ok_or(Error::NumericalFailure)?;
            let dx = &next * h.transpose() * z / c.lidar_variance;
            (dx, next)
        };
        let increment = Tangent::from_column_slice(dx.as_slice());
        self.covariance = Covariance::from_column_slice(next.as_slice());
        self.covariance = (self.covariance + self.covariance.transpose()) * 0.5;
        self.state.apply(&increment);
        self.check()?;
        Ok(count)
    }

    pub(crate) fn plane_jacobian(
        &self,
        p: Vector3<f64>,
        n: Vector3<f64>,
        estimate: bool,
    ) -> Tangent {
        let s = &self.state;
        let imu_p = s.lidar_to_imu_rotation * p + s.lidar_to_imu_translation;
        let local_n = s.orientation.inverse() * n;
        let mut h = Tangent::zeros();
        h.fixed_rows_mut::<3>(0).copy_from(&n);
        h.fixed_rows_mut::<3>(3).copy_from(&imu_p.cross(&local_n));
        if estimate {
            h.fixed_rows_mut::<3>(6)
                .copy_from(&p.cross(&(s.lidar_to_imu_rotation.inverse() * local_n)));
            h.fixed_rows_mut::<3>(9).copy_from(&local_n);
        }
        h
    }

    #[cfg(test)]
    pub fn observe_plane(
        &mut self,
        p: Vector3<f64>,
        normal: Vector3<f64>,
        offset: f64,
        c: &Config,
    ) -> Result<bool, Error> {
        let distance = normal.dot(&self.state.lidar_point_to_world(&p)) + offset;
        if p.norm() <= c.match_s * distance * distance {
            return Ok(false);
        }
        let h = self.plane_jacobian(p, normal, c.estimate_extrinsics);
        self.update(h, -distance, c.lidar_variance)?;
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 直接编译作者 ESEKF 与动力学/IMU 函数生成的金标准，核对完整状态和 P。
    #[test]
    fn output_filter_matches_official_cpp_oracle() {
        let reference: Vec<Vec<f64>> = include_str!("../tests/data/official_filter.csv")
            .lines()
            .map(|line| line.split(',').map(|x| x.parse().unwrap()).collect())
            .collect();
        let c = Config::default();
        let mut f = Filter::new(
            State::initial(&c, 0., Vector3::z() * 9.81, Vector3::zeros()),
            &c,
        );
        let mut at = 0;
        let mut max_error: f64 = 0.;
        for tick in 1..=100 {
            let time = tick as f64 * 0.005;
            f.predict_to(time - 0.0025, &c).unwrap();
            f.predict_to(time, &c).unwrap();
            let mut angular_velocity = Vector3::new(
                0.02 * (time * 7.).sin(),
                -0.01 * (time * 3.).cos(),
                0.3 * time,
            );
            if (60..80).contains(&tick) {
                angular_velocity.z = 40.;
            }
            f.observe_imu(
                &ImuSample {
                    time,
                    angular_velocity,
                    acceleration: Vector3::new(
                        0.1 * (time * 5.).sin(),
                        0.04 * (time * 11.).cos(),
                        9.81,
                    ),
                },
                &c,
            )
            .unwrap();
            if tick % 4 == 0 {
                f.observe_planes(
                    &[
                        (Vector3::new(1., 0.5, 0.4), Vector3::z(), -0.4),
                        (Vector3::new(1., 0.5, 0.4), Vector3::x(), -1.),
                    ],
                    &c,
                )
                .unwrap();
            }
            if at == reference.len() || (reference[at][0] - time).abs() > 1e-12 {
                continue;
            }
            let s = &f.state;
            let mut values = Vec::new();
            values.extend_from_slice(s.position.as_slice());
            values.extend_from_slice(s.orientation.coords.as_slice());
            values.extend_from_slice(s.lidar_to_imu_rotation.coords.as_slice());
            for v in [
                &s.lidar_to_imu_translation,
                &s.velocity,
                &s.angular_velocity,
                &s.specific_force,
                &s.gravity,
                &s.gyro_bias,
                &s.accel_bias,
            ] {
                values.extend_from_slice(v.as_slice());
            }
            values.extend_from_slice(f.covariance.as_slice());
            assert_eq!(values.len() + 1, reference[at].len());
            for (column, (actual, expected)) in values.iter().zip(&reference[at][1..]).enumerate() {
                max_error = max_error.max((actual - expected).abs());
                assert!(
                    (actual - expected).abs() < 1e-9,
                    "tick {tick} column {column}: Rust {actual}, C++ {expected}"
                );
            }
            at += 1;
        }
        assert_eq!(at, reference.len());
        println!("official_filter_oracle: checkpoints={at}, max_abs_error={max_error:e}");
    }

    // 同一 IMU 间隔被任意多个点时刻切分，不能改变该间隔的过程噪声。
    #[test]
    fn covariance_does_not_depend_on_lidar_time_subdivision() {
        let c = Config::default();
        let mut s = State::initial(&c, 0., Vector3::z() * 9.81, Vector3::zeros());
        s.angular_velocity.z = 0.3;
        let mut coarse = Filter::new(s.clone(), &c);
        let mut dense = Filter::new(s, &c);
        let before = dense.covariance;
        for i in 1..=1000 {
            dense.predict_to(i as f64 * 0.000005, &c).unwrap();
        }
        assert_eq!(dense.covariance, before);
        coarse.predict_to(0.005, &c).unwrap();
        let imu = ImuSample {
            time: 0.005,
            angular_velocity: Vector3::new(0., 0., 0.3),
            acceleration: Vector3::z() * 9.81,
        };
        coarse.observe_imu(&imu, &c).unwrap();
        dense.observe_imu(&imu, &c).unwrap();
        assert!((coarse.covariance - dense.covariance).norm() < 1e-12);
        assert!(coarse.state.orientation.angle_to(&dense.state.orientation) < 1e-12);
    }

    // 联合观测在小批量和大批量分支都必须遵守同一个解析 Kalman 解。
    #[test]
    fn simultaneous_planes_match_analytic_posterior() {
        let c = Config::default();
        for count in [2, 29, 30, 128] {
            let s = State::initial(&c, 0., Vector3::z() * 9.81, Vector3::zeros());
            let mut f = Filter::new(s, &c);
            let observations = vec![(Vector3::z(), Vector3::z(), -1.1); count];
            assert_eq!(f.observe_planes(&observations, &c).unwrap(), count);
            let expected_position = 0.1 * count as f64 / (count as f64 + 1.0);
            let expected_variance = 0.01 / (count as f64 + 1.0);
            assert!((f.state.position.z - expected_position).abs() < 1e-12);
            assert!((f.covariance[(2, 2)] - expected_variance).abs() < 1e-12);
        }
    }

    // 点到平面右扰动雅可比必须与数值微分一致，包含在线外参六维。
    #[test]
    fn plane_jacobian_matches_finite_difference() {
        let c = Config::default();
        let mut s = State::initial(&c, 0., Vector3::z() * 9.81, Vector3::zeros());
        s.orientation = UnitQuaternion::from_euler_angles(0.3, -0.2, 0.5);
        s.lidar_to_imu_rotation = UnitQuaternion::from_euler_angles(-0.1, 0.2, 0.4);
        s.lidar_to_imu_translation = Vector3::new(0.1, -0.2, 0.3);
        let f = Filter::new(s, &c);
        let p = Vector3::new(2., 1., -0.4);
        let n = Vector3::new(0.2, 0.5, 1.).normalize();
        let h = f.plane_jacobian(p, n, true);
        for i in 0..30 {
            let mut plus = f.state.clone();
            let mut minus = f.state.clone();
            let mut d = Tangent::zeros();
            d[i] = 1e-6;
            plus.apply(&d);
            minus.apply(&(-d));
            let numeric =
                n.dot(&(plus.lidar_point_to_world(&p) - minus.lidar_point_to_world(&p))) / 2e-6;
            assert!(
                (numeric - h[i]).abs() < 1e-7,
                "axis {i}: {numeric} != {}",
                h[i]
            );
        }
    }

    // 持续预测和观测后，协方差应对称半正定且固定外参不能漂移。
    #[test]
    fn covariance_stays_psd_and_fixed_extrinsics_stay_fixed() {
        let c = Config::default();
        let s = State::initial(&c, 0., Vector3::z() * 9.81, Vector3::zeros());
        let mut f = Filter::new(s, &c);
        for i in 1..101 {
            let time = i as f64 * 0.005;
            f.predict_to(time, &c).unwrap();
            f.observe_imu(
                &ImuSample {
                    time,
                    acceleration: Vector3::z() * 9.81,
                    angular_velocity: Vector3::zeros(),
                },
                &c,
            )
            .unwrap();
            f.observe_plane(Vector3::new(1., 1., 1.), Vector3::z(), -1., &c)
                .unwrap();
        }
        assert!((f.covariance - f.covariance.transpose()).norm() < 1e-10);
        assert!(f.covariance.symmetric_eigen().eigenvalues.min() > -1e-10);
        assert_eq!(f.state.lidar_to_imu_rotation, c.lidar_to_imu_rotation);
        assert_eq!(f.state.lidar_to_imu_translation, c.lidar_to_imu_translation);
    }
}
