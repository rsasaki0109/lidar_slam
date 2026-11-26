//! Closed-loop active SLAM environment (docs/roadmap/active-slam-2026-11.md, P5).
//!
//! A kinematic differential-drive robot carries a Livox Mid-360 (RobotNativeEngine's model:
//! the real scan pattern, range noise, per-return emission times) and an IMU through a static
//! world of boxes. A controller process drives it over stdin / stdout in lockstep, one LiDAR
//! frame (0.1 s) per request, so a run is deterministic and never runs ahead of the SLAM.
//!
//! The messages match what `livox_ros_driver2` publishes for a real Mid-360: points in the
//! sensor frame (x forward, z up) at their own emission time, fields x, y, z, intensity (f32),
//! tag, line (u8), timestamp (f64, absolute ns); IMU in the same frame, angular velocity in
//! rad/s and acceleration in g.
//!
//! Protocol (text requests, one per line):
//!
//! * `step <v_m_s> <yaw_rate_rad_s>`: hold the command for one frame. Reply: one JSON line
//!   (`frame`, `stamp_ns`, `points`, `imu`, `truth` = [x, y, yaw] at the frame end,
//!   `collided`), then `points` x 26 bytes (the driver's point layout, little endian), then
//!   `imu` x 56 bytes (stamp i64 ns, gyro xyz f64, accel xyz f64).
//! * `quit`.
//!
//! The world file is JSON in a z-up frame (metres): `{"boxes": [{"min": [x, y, z], "max":
//! [x, y, z]}, ...], "start": [x, y, yaw], "seed": 1}`. The floor is part of the world (a box).
//!
//! RobotNativeEngine's world is y-up; this program maps z-up (x, y, z) to y-up (x, z, -y).

use std::f64::consts::FRAC_PI_2;
use std::io::{self, BufRead, BufWriter, Write};

use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{PhysicsError, PhysicsWorldId, RaycastHit, RaycastQuery};
use rne_sensor::{
    livox_mid360_spec, sample_livox_mid360, LidarRaycaster, LidarSweep, LivoxMid360Pattern,
    SensorNoiseKey, LIVOX_MID360_FRAME_PERIOD_S,
};
use rne_world::Transform3;
use serde::Deserialize;

const IMU_RATE_HZ: u32 = 200;
const SENSOR_HEIGHT_M: f64 = 0.5;
const ROBOT_RADIUS_M: f64 = 0.25;
const MAX_SPEED_M_S: f64 = 1.0;
const MAX_YAW_RATE_RAD_S: f64 = 1.0;
const MAX_ACCEL_M_S2: f64 = 1.0;
const GRAVITY_M_S2: f64 = 9.80665;
const START_TIME_NS: i64 = 1_700_000_000_000_000_000;
const GYRO_NOISE_RAD_S: f64 = 0.002;
const ACCEL_NOISE_M_S2: f64 = 0.02;

#[derive(Deserialize)]
struct BoxSpec {
    min: [f64; 3],
    max: [f64; 3],
}

#[derive(Deserialize)]
struct WorldSpec {
    boxes: Vec<BoxSpec>,
    start: [f64; 3],
    #[serde(default = "default_seed")]
    seed: u64,
    /// Scales the IMU noise and bias (1: the defaults; 0: an ideal IMU).
    #[serde(default = "default_noise_scale")]
    imu_noise_scale: f64,
}

fn default_noise_scale() -> f64 {
    1.0
}

fn default_seed() -> u64 {
    1
}

/// z-up -> RobotNativeEngine's y-up.
fn to_y_up(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[2], -p[1])
}

/// Axis-aligned boxes (in y-up) as a `LidarRaycaster`.
struct BoxWorld {
    boxes: Vec<(Vec3, Vec3, Entity)>,
    // z-up footprints of the boxes that reach above the floor, for collision checks.
    obstacles_2d: Vec<([f64; 2], [f64; 2])>,
}

impl BoxWorld {
    fn hit(lo: Vec3, hi: Vec3, query: &RaycastQuery) -> Option<(f64, Vec3)> {
        let o = [query.origin_m.x, query.origin_m.y, query.origin_m.z];
        let d = [query.direction.x, query.direction.y, query.direction.z];
        let lo = [lo.x, lo.y, lo.z];
        let hi = [hi.x, hi.y, hi.z];
        let mut t_near = 0.0_f64;
        let mut t_far = query.max_distance_m;
        let mut normal = Vec3::ZERO;
        for axis in 0..3 {
            if d[axis].abs() < 1e-12 {
                if o[axis] < lo[axis] || o[axis] > hi[axis] {
                    return None;
                }
                continue;
            }
            let mut t0 = (lo[axis] - o[axis]) / d[axis];
            let mut t1 = (hi[axis] - o[axis]) / d[axis];
            let mut sign = -1.0;
            if t0 > t1 {
                std::mem::swap(&mut t0, &mut t1);
                sign = 1.0;
            }
            if t0 > t_near {
                t_near = t0;
                let mut n = [0.0; 3];
                n[axis] = sign;
                normal = Vec3::new(n[0], n[1], n[2]);
            }
            t_far = t_far.min(t1);
            if t_near > t_far {
                return None;
            }
        }
        if t_near <= 0.0 {
            return None; // the origin is inside (or on) the box
        }
        Some((t_near, normal))
    }

    fn collides(&self, x: f64, y: f64) -> bool {
        self.obstacles_2d.iter().any(|(lo, hi)| {
            let cx = x.clamp(lo[0], hi[0]);
            let cy = y.clamp(lo[1], hi[1]);
            (x - cx).hypot(y - cy) < ROBOT_RADIUS_M
        })
    }
}

impl LidarRaycaster for BoxWorld {
    fn lidar_raycast(
        &self,
        _: PhysicsWorldId,
        query: RaycastQuery,
    ) -> Result<Vec<RaycastHit>, PhysicsError> {
        let mut hits: Vec<RaycastHit> = self
            .boxes
            .iter()
            .filter_map(|(lo, hi, entity)| {
                Self::hit(*lo, *hi, &query).map(|(t, normal)| RaycastHit {
                    entity: *entity,
                    point_m: query.origin_m + query.direction * t,
                    normal,
                    distance_m: t,
                })
            })
            .collect();
        hits.sort_by(|a, b| {
            a.distance_m
                .total_cmp(&b.distance_m)
                .then(a.entity.cmp(&b.entity))
        });
        Ok(hits)
    }
}

/// splitmix64 + Box-Muller: deterministic IMU noise.
struct Noise(u64);

impl Noise {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn gaussian(&mut self) -> f64 {
        let u1 = ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64;
        let u2 = ((self.next_u64() >> 11) as f64) / (1u64 << 53) as f64;
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

#[derive(Clone, Copy)]
struct State {
    x: f64,
    y: f64,
    yaw: f64,
    v: f64,
}

/// The sensor's pose in y-up: the z-up sensor frame (x forward, z up) rotated by yaw, then
/// mapped to y-up.
fn sensor_transform(state: &State) -> Transform3 {
    let rotation = Quat::from_rotation_x(-FRAC_PI_2) * Quat::from_rotation_z(state.yaw);
    Transform3::from_translation_rotation(to_y_up([state.x, state.y, SENSOR_HEIGHT_M]), rotation)
}

struct Env {
    world: World,
    boxes: BoxWorld,
    spec: rne_sensor::LidarSpec,
    pattern: LivoxMid360Pattern,
    seed: u64,
    state: State,
    frame: u64,
    noise: Noise,
    gyro_bias: [f64; 3],
    imu_noise_scale: f64,
}

struct Frame {
    stamp_ns: i64,
    points: Vec<u8>,
    point_count: usize,
    imu: Vec<u8>,
    imu_count: usize,
    collided: bool,
}

impl Env {
    fn new(spec_json: &str) -> Result<Self, String> {
        let world_spec: WorldSpec =
            serde_json::from_str(spec_json).map_err(|e| format!("world file: {e}"))?;
        let mut world = World::new();
        let mut boxes = Vec::new();
        let mut obstacles_2d = Vec::new();
        for (index, b) in world_spec.boxes.iter().enumerate() {
            let entity = spawn_named(&mut world, format!("box{index}"));
            let a = to_y_up(b.min);
            let c = to_y_up(b.max);
            boxes.push((a.min(c), a.max(c), entity));
            // Obstacle for the robot when it rises above 5 cm and is not below the sensor's
            // floor (the floor box tops out at z = 0).
            if b.max[2] > 0.05 && b.min[2] < SENSOR_HEIGHT_M + 0.5 {
                obstacles_2d.push(([b.min[0], b.min[1]], [b.max[0], b.max[1]]));
            }
        }
        let mut noise = Noise(world_spec.seed ^ 0xA5A5_5A5A_1234_5678);
        let gyro_bias = [
            0.001 * noise.gaussian(),
            0.001 * noise.gaussian(),
            0.001 * noise.gaussian(),
        ];
        Ok(Self {
            world,
            boxes: BoxWorld {
                boxes,
                obstacles_2d,
            },
            spec: livox_mid360_spec(),
            pattern: LivoxMid360Pattern::new(),
            seed: world_spec.seed,
            state: State {
                x: world_spec.start[0],
                y: world_spec.start[1],
                yaw: world_spec.start[2],
                v: 0.0,
            },
            frame: 0,
            noise,
            gyro_bias,
            imu_noise_scale: world_spec.imu_noise_scale,
        })
    }

    fn step(&mut self, v_cmd: f64, w_cmd: f64) -> Frame {
        let v_cmd = v_cmd.clamp(-MAX_SPEED_M_S, MAX_SPEED_M_S);
        let w = w_cmd.clamp(-MAX_YAW_RATE_RAD_S, MAX_YAW_RATE_RAD_S);
        let start = self.state;
        let substeps = (IMU_RATE_HZ as f64 * LIVOX_MID360_FRAME_PERIOD_S).round() as u32;
        let dt = LIVOX_MID360_FRAME_PERIOD_S / substeps as f64;
        let frame_start_ns =
            START_TIME_NS + (self.frame as f64 * LIVOX_MID360_FRAME_PERIOD_S * 1e9).round() as i64;
        let mut imu = Vec::with_capacity(substeps as usize * 56);
        let mut collided = false;
        for k in 0..substeps {
            let s = self.state;
            // Speed follows the command within the acceleration limit.
            let dv = (v_cmd - s.v).clamp(-MAX_ACCEL_M_S2 * dt, MAX_ACCEL_M_S2 * dt);
            let v = s.v + dv;
            let accel_forward = dv / dt;
            let yaw = s.yaw + w * dt;
            let mid_yaw = s.yaw + 0.5 * w * dt;
            let x = s.x + v * dt * mid_yaw.cos();
            let y = s.y + v * dt * mid_yaw.sin();
            if self.boxes.collides(x, y) {
                collided = true;
                self.state = State { v: 0.0, yaw, ..s };
            } else {
                self.state = State { x, y, yaw, v };
            }
            let v_now = self.state.v;
            // Body-frame specific force: forward acceleration, centripetal v * w, gravity.
            let accel = [
                if collided { 0.0 } else { accel_forward },
                v_now * w,
                GRAVITY_M_S2,
            ];
            let gyro = [0.0, 0.0, w];
            let stamp_ns = frame_start_ns + ((k + 1) as f64 * dt * 1e9).round() as i64;
            imu.extend_from_slice(&stamp_ns.to_le_bytes());
            for (rate, bias) in gyro.iter().zip(self.gyro_bias) {
                let value =
                    rate + self.imu_noise_scale * (bias + GYRO_NOISE_RAD_S * self.noise.gaussian());
                imu.extend_from_slice(&value.to_le_bytes());
            }
            for value in accel {
                let noisy = (value
                    + self.imu_noise_scale * ACCEL_NOISE_M_S2 * self.noise.gaussian())
                    / GRAVITY_M_S2;
                imu.extend_from_slice(&noisy.to_le_bytes());
            }
        }

        let sweep = LidarSweep::new(sensor_transform(&start), sensor_transform(&self.state));
        let cloud = sample_livox_mid360(
            &self.boxes,
            PhysicsWorldId::DEFAULT,
            &self.world,
            &sweep,
            &self.spec,
            &self.pattern,
            self.frame,
            None,
            SensorNoiseKey::new(self.seed, self.spec.seed, 1, self.frame),
        );
        let mut points = Vec::with_capacity(cloud.points_m.len() * 26);
        let mut count = 0;
        // In emission order, as the driver publishes them (stable: ties keep ray order).
        let mut order: Vec<usize> = (0..cloud.points_m.len()).collect();
        order.sort_by(|&a, &b| {
            let ta = cloud.timestamps_s.get(a).copied().unwrap_or(0.0);
            let tb = cloud.timestamps_s.get(b).copied().unwrap_or(0.0);
            ta.total_cmp(&tb)
        });
        for index in order {
            let world_point = &cloud.points_m[index];
            let time_s = cloud.timestamps_s.get(index).copied().unwrap_or(0.0);
            let pose = sweep.pose_at(time_s / LIVOX_MID360_FRAME_PERIOD_S);
            let local = pose.rotation.inverse() * (*world_point - pose.translation);
            let intensity = cloud.intensities.get(index).copied().unwrap_or(0.0) * 100.0;
            let line = cloud.channel_indices.get(index).copied().unwrap_or(0) as u8;
            let stamp = frame_start_ns as f64 + (time_s * 1e9).round();
            points.extend_from_slice(&(local.x as f32).to_le_bytes());
            points.extend_from_slice(&(local.y as f32).to_le_bytes());
            points.extend_from_slice(&(local.z as f32).to_le_bytes());
            points.extend_from_slice(&intensity.to_le_bytes());
            points.push(0); // tag
            points.push(line);
            points.extend_from_slice(&stamp.to_le_bytes());
            count += 1;
        }
        self.frame += 1;
        Frame {
            stamp_ns: frame_start_ns,
            points,
            point_count: count,
            imu_count: imu.len() / 56,
            imu,
            collided,
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: rne_active_slam_env <world.json>");
        std::process::exit(2);
    }
    let spec = std::fs::read_to_string(&args[1]).unwrap_or_else(|e| {
        eprintln!("read {}: {e}", args[1]);
        std::process::exit(2);
    });
    let mut env = Env::new(&spec).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(2);
    });
    let stdin = io::stdin();
    let mut out = BufWriter::new(io::stdout().lock());
    for line in stdin.lock().lines() {
        let line = line.expect("stdin");
        let fields: Vec<&str> = line.split_whitespace().collect();
        match fields.as_slice() {
            ["step", v, w] => {
                let (Ok(v), Ok(w)) = (v.parse::<f64>(), w.parse::<f64>()) else {
                    eprintln!("bad step: {line}");
                    std::process::exit(2);
                };
                let frame = env.step(v, w);
                let s = env.state;
                writeln!(
                    out,
                    "{{\"frame\":{},\"stamp_ns\":{},\"points\":{},\"imu\":{},\"truth\":[{:.9},{:.9},{:.9}],\"collided\":{}}}",
                    env.frame - 1,
                    frame.stamp_ns,
                    frame.point_count,
                    frame.imu_count,
                    s.x,
                    s.y,
                    s.yaw,
                    frame.collided
                )
                .expect("stdout");
                out.write_all(&frame.points).expect("stdout");
                out.write_all(&frame.imu).expect("stdout");
                out.flush().expect("stdout");
            }
            ["quit"] => break,
            [] => {}
            _ => {
                eprintln!("unknown request: {line}");
                std::process::exit(2);
            }
        }
    }
}
