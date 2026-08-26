# state_fusion_backend

`state_fusion_backend` is the FusionLynx state-estimation backend. It subscribes
to the topics already published by the current workspace and publishes the
single fused odometry authority:

```text
/motioncore/wheel_odom  -> Wheel Between factor
/motioncore/imu/data    -> LIO health: static and yaw consistency
/motioncore/imu/mag     -> LIO health: magnetic-field stability
/fast_lio/odom          -> FAST-LIO LiDAR pose relative factor

/odom                   -> fused odometry
odom -> base_link       -> fused TF authority
```

The node uses GTSAM 4.2 `IncrementalFixedLagSmoother`. In the current v1
contract, MotionCore IMU and MAG are subscribed and timestamp-checked only to
assess FAST-LIO health. They do not directly change the localization estimate.
The graph therefore remains Wheel + FAST-LIO + optional planar soft prior.
IMU preintegration and a calibrated yaw-only MAG factor remain future
experiments; MAG never constrains Z.

FAST-LIO estimates its IMU/body state internally, then publishes
`/fast_lio/odom` as `camera_init -> lidar_link` using the configured
LiDAR-to-IMU extrinsic. The fusion backend converts that observation with
`lio_lidar_to_base_*` and keeps the final output contract as
`odom -> base_link`; the robot model supplies the static
`base_link -> lidar_link` and `base_link -> body` transforms.

## LIO adaptive handling

The LIO input is not low-pass filtered. For each FAST-LIO odometry sample the
backend builds a quality context from the current wheel increment, buffered
MotionCore IMU samples, the wheel+state prediction, and the LIO pose covariance
when its diagonal is valid.

The graph uses the relative transform between consecutive FAST-LIO poses.
FAST-LIO's absolute frame is used only to initialize the graph and to align a
recovered frontend. This prevents a drifting frontend origin from becoming a
new absolute prior at every fixed-lag update.

- Innovation is evaluated with a six-dimensional pose residual and NIS.
- FAST-LIO covariance is used when valid, but the configured LIO position and
  rotation sigmas are applied as a minimum floor. This prevents
  filter-internal, near-zero covariance from turning centimeter-level
  repeatability error into false health failures or an overconfident graph
  factor.
- Wheel and IMU motion consistency are evaluated independently of absolute
  displacement, so consistent small motion is retained.
- A valid magnetic-field norm is tracked across consecutive observations.
  Sudden magnetic disturbance increases the LIO health scale; no magnetic
  heading factor is created.
- Static detection requires wheel motion and IMU gyro thresholds to remain
  satisfied for `static_min_duration`.
- During a confirmed stop, LIO covariance is enlarged and a finite-covariance
  pose anchor is held at the stop-entry estimate; the current Pose3-only graph
  also keeps the relative planar motion prior active. An explicit zero-velocity
  factor is reserved for the future IMU-preintegration state.
- `DEGRADED` keeps the LIO factor with enlarged covariance and Huber protection.
- `FAILED` omits only the LIO factor; the fixed-lag graph continues with Wheel,
  the planar motion prior, and fallback pose observability.
- Recovery enters `RECOVERING`, aligns the first stable LIO pose to the current
  fused state, and only then lets the recovered LIO pose re-enter normally.

Diagnostics expose `lio_state`, `lio_nis`, innovation norm, wheel/IMU/MAG
consistency, confidence, covariance source, and static state. The current
implementation uses diagonal covariance because FAST-LIO may publish an
all-zero or otherwise invalid covariance matrix; per-DOF covariance policy
remains an extension point.

## LIO state-transition logging

The backend logs LIO health transitions rather than printing every LIO frame.
The transition record includes the ROS timestamp, previous and current state,
health reason, factor acceptance, alignment requirement, static detection,
covariance scale, NIS, innovation norm, and wheel/IMU/MAG consistency scores.

The severity follows the operational consequence:

- `INFO`: initialization or recovery to `HEALTHY`;
- `WARN`: `DEGRADED` or `RECOVERING`, including rejected or down-weighted LIO;
- `ERROR`: `FAILED`, including LIO timeout or unrecoverable tracking loss.

Repeated observations in the same state are suppressed. High-rate per-frame
details remain available through `/motioncore/diagnostics`, so logging does not
become a second high-frequency data path or alter the Wheel/LIO timing contract.
The logging policy and fields are documented in
`docs/fusionlynx_lio_logging.md`.

## Output timing

The callbacks only validate and enqueue measurements. A single timer-owned
`process_pending()` path selects the next monotonic event and is the only
place allowed to update GTSAM or publish the fused odometry. Profiles C-G use
FAST-LIO measurement timestamps as the primary state clock and attach the
latest causal wheel sample at or before that timestamp. Profiles A and B
process their only enabled source directly.

When FAST-LIO is absent or its steady receive age exceeds `sensor_timeout`,
profiles C-G advance on wheel timestamps and allow wheel prediction output.
At startup, the watchdog grants one `sensor_timeout` interval for the first
FAST-LIO sample before enabling that fallback.
During healthy LIO operation, the backend does not publish wheel timestamps
ahead of the latest LIO event; this bounded policy preserves causal ordering
when Raspberry Pi transport delay causes a previously stamped LIO message to
arrive late. A later LIO sample is aligned to the current fused state before
relative LIO factors resume.

The output stamp remains the selected measurement timestamp. Receive time is
used only by the watchdog, so network delay is not silently converted into a
new measurement time. Samples that arrive outside `reorder_window`, are
future-dated beyond `max_future_stamp`, or would move the cross-sensor event
clock backwards are rejected.

## Fusion profiles

`fusion_profile` implements the Vault `config_fusion_matrix`:

| Profile | Wheel | FAST-LIO | LIO health | Planar prior | Kernel |
| --- | ---: | ---: | ---: | ---: | --- |
| A | yes | no | no | no | Gaussian |
| B | no | yes | no | no | Gaussian |
| C | yes | yes | no | yes | Gaussian |
| D | yes | yes | no | yes | Huber |
| E | yes | yes | yes | yes | Huber |
| F | yes | yes | yes | yes | Huber + static anchor |
| G | yes | yes | yes | no | Huber + health |

Use `fusion_profile:=A` through `G` for an experiment override. Leave it
unset to use the individual switches in YAML (`default`/`custom` semantics).
The launch layer also accepts explicit `wheel_odom_topic` and
`fast_lio_odom_topic` overrides.

Wheel samples more than `max_future_stamp` ahead of the ROS clock are rejected.
This is a guard against a broken upstream timestamp mapping; it is not a
replacement for fixing the source timestamp. The MotionCore driver therefore
uses host receive time by default, while mapped device timestamps remain an
explicit experimental option.

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch state_fusion_backend fusion.launch.py
# Example: Vault experiment A with alternate topic names
ros2 launch state_fusion_backend fusion.launch.py \
  fusion_profile:=A \
  wheel_odom_topic:=/motioncore/wheel_odom \
  fast_lio_odom_topic:=/fast_lio/odom
```

This is the first executable skeleton. The implementation deliberately keeps
observation conversion, health supervision, graph construction and ROS output
in separate modules so IMU preintegration, visual odometry, GNSS and
loop-closure factors can be added without changing the existing Wheel/LIO
contracts.
