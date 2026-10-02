# FusionLynx SLAM Parameter Measurement Plan

## 1. Scope

This document defines how to measure, validate, and record the parameters in:

```text
src/fusionlynx/fusionlynx_slam/config/fusionlynx_slam.yaml
```

The file is the system profile passed to `state_fusion_backend_node`. It does
not contain the FAST-LIO frontend parameters. FAST-LIO continues to use its
package-owned UniLidar configuration, and RTAB-Map parameters are supplied by
`fusionlynx_mapping`.

The current production data path is:

```text
MotionCore wheel odometry
    + FAST-LIO /fast_lio/odom
    -> state_fusion_backend
    -> /odom and odom -> base_link
    -> RTAB-Map
```

The measurement order is mandatory:

```text
interface and TF validation
    -> timestamp validation
    -> extrinsic validation
    -> motion and sensor noise measurement
    -> fixed-lag and robust-kernel selection
    -> health-threshold calibration
    -> independent validation
```

Do not tune estimator noise to compensate for an incorrect frame, timestamp,
or sensor calibration. Those defects must be fixed first.

## 2. Parameter Classes

Every parameter belongs to one of four classes:

| Class | Meaning | Required action |
|---|---|---|
| Contract | Topic, frame, ownership, and enable/disable policy | Verify against the running graph; do not statistically tune |
| Calibration | Physical extrinsics, sensor noise, motion-model uncertainty | Measure from hardware or recorded data |
| Design sweep | Fixed-lag length, robust loss, covariance scaling | Sweep against accuracy, continuity, and compute cost |
| Compatibility | Declared for a future factor adapter but inactive in the current `Pose3` backend | Keep disabled and document as inactive |

The current system uses wheel and FAST-LIO factors in a fixed-lag `Pose3`
graph. MotionCore IMU and magnetic data are consumed by the LIO health
monitor when health monitoring is enabled, but they are not direct
localization factors in the current backend.

## 3. Parameter Inventory

### 3.1 Contract parameters

These values are not determined by trajectory optimization. They are checked
against the ROS 2 graph, URDF/Xacro, and the architecture contract.

| Parameters | Current role | Verification |
|---|---|---|
| `wheel_odom_topic` | Wheel odometry input | `ros2 topic info -v` and one-message inspection |
| `imu_topic` | Health-monitoring IMU input | Confirm publisher, QoS, timestamp, and frame |
| `mag_topic` | Health-monitoring magnetic input | Confirm publisher, QoS, timestamp, and frame |
| `fastlio_topic` | FAST-LIO pose observation | Confirm `/fast_lio/odom` and covariance layout |
| `fused_odom_topic` | Authoritative fused odometry | Verify exactly one `/odom` publisher |
| `diagnostics_topic` | Diagnostic output | Verify diagnostic publisher and update rate |
| `wheel_odom_frame` | Wheel odometry parent frame | Compare `header.frame_id` |
| `wheel_child_frame` | Wheel odometry child frame | Compare `child_frame_id` |
| `lio_world_frame` | FAST-LIO pose parent frame | Compare `/fast_lio/odom.header.frame_id` |
| `lio_child_frame` | FAST-LIO pose child frame | Compare `/fast_lio/odom.child_frame_id` |
| `odom_frame` | Fused odometry frame | Verify `/odom.header.frame_id` |
| `base_link_frame` | Fused robot body frame | Verify `/odom.child_frame_id` and TF |
| `publish_tf` | Dynamic TF ownership | Verify only `state_fusion_backend` publishes `odom -> base_link` |
| `fusion_profile` | Preset switch for graph composition | Verify the selected profile matches the experiment |
| `use_wheel_odom`, `use_fast_lio` | Factor source switches | Record the active ablation condition |
| `use_lio_health`, `use_static_anchor` | Health and static-anchor switches | Record explicitly for each experiment |

The following checks are required before any numerical calibration:

```bash
ros2 topic info -v /motioncore/wheel_odom
ros2 topic info -v /fast_lio/odom
ros2 topic echo /motioncore/wheel_odom --once
ros2 topic echo /fast_lio/odom --once
ros2 topic info -v /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_tools view_frames
```

Acceptance criteria:

1. `/odom` has one authoritative publisher.
2. `state_fusion_backend` is the only dynamic `odom -> base_link` publisher.
3. FAST-LIO does not publish a competing dynamic odometry TF.
4. The frame IDs in the messages match the YAML values.
5. The FAST-LIO covariance uses the standard ROS `Odometry` order.
6. No timestamp is negative, non-monotonic, or unexpectedly in the future.

### 3.2 Physical extrinsics

| Parameters | Status | Measurement method |
|---|---|---|
| `lio_lidar_to_base_xyz` | Active calibration | Measure LiDAR origin relative to `base_link` origin |
| `lio_lidar_to_base_rpy` | Active calibration | Determine LiDAR axes relative to the robot body axes |

The transform is used as:

\[
T_{\mathrm{camera\_init},\mathrm{base}}
=
T_{\mathrm{camera\_init},\mathrm{lidar}}
T_{\mathrm{lidar},\mathrm{base}}.
\]

Measure the translation from the mechanical reference datums, not from the
visual center of the sensor housing. Record:

- sensor mounting holes or bracket datum;
- `base_link` origin definition;
- positive direction of each axis;
- lever-arm uncertainty;
- mounting repeatability after removal and reinstallation.

For rotation:

1. Align the robot on a level floor.
2. Mark the forward, left, and upward axes of `base_link`.
3. Measure LiDAR yaw relative to the forward axis.
4. Verify roll and pitch with a precision level or point-cloud plane fit.
5. Confirm the sign convention by rotating the robot in place and checking the
   direction of the FAST-LIO pose increment.

Validation must use both TF and motion:

- compare the explicit conversion with the URDF static transform;
- drive straight and verify that lateral displacement is not introduced;
- rotate in place and verify that the LiDAR lever arm does not create a
  false translation;
- compare the transformed LiDAR pose with
  `fast_lio_base_test.launch.py`.

Report the final transform with an uncertainty estimate. If the explicit
configuration and the URDF disagree, stop calibration and resolve the
ownership conflict before collecting accuracy results.

### 3.3 Time and buffering parameters

| Parameters | How to measure | Selection rule |
|---|---|---|
| `frequency` | Measure timer period, output rate, and callback processing time | Choose a rate that meets the output requirement without timer overruns |
| `sensor_timeout` | Measure receive age and inter-arrival gaps for FAST-LIO and wheel odometry | Set above the normal 99th-percentile receive age plus margin, below the maximum acceptable stale duration |
| `max_time_delay` | Compare event timestamps after clock alignment | Set above the 99th-percentile cross-sensor pairing error plus timestamp uncertainty |
| `max_future_stamp` | Measure positive message age relative to node time | Set above the maximum normal future skew plus margin; reject larger values |
| `queue_size` | Measure input rates, burst size, and buffer occupancy | Set above the largest expected burst and `rate * reorder_window` for each input |
| `reorder_window` | Measure out-of-order arrival delay | Set to the 99th or 99.9th percentile of late-arrival delay, with a bounded latency budget |
| `clock_offset_tolerance` | Estimate clock offset using common motion events and message stamps | Set above the maximum absolute clock offset plus uncertainty |
| `diagnostics_publish_rate` | Measure diagnostic CPU cost and required observability | Use a rate sufficient to observe state transitions without affecting estimation |

Use monotonic receive time and message header time as separate quantities.
Never estimate sensor delay by comparing only two message header timestamps if
the devices use independent clocks.

For every recorded bag, compute:

\[
\Delta t_{\mathrm{arrival}}
=
t_{\mathrm{receive}}-t_{\mathrm{header}},
\]

\[
\Delta t_{\mathrm{pair}}
=
t_{\mathrm{wheel}}-t_{\mathrm{lio}},
\]

and the out-of-order delay:

\[
\Delta t_{\mathrm{late}}
=
t_{\mathrm{processed}}-t_{\mathrm{arrival\ order}}.
\]

Use the median, 95th, 99th, and maximum values. Record dropped messages,
buffer overflows, rejected old messages, future-stamp rejections, and timeout
events. A valid setting must have no buffer overflow during the target
workload and must not hide a persistent clock error.

Recommended initial sweep:

```text
frequency:          25, 50, 75 Hz
sensor_timeout:     measured p99 + margin
max_time_delay:     measured p99 + margin
reorder_window:     measured p99 late-arrival delay + margin
queue_size:         calculated lower bound, then one burst-size margin
```

### 3.4 Fixed-lag and graph-computation parameters

| Parameters | Measurement procedure | Acceptance criteria |
|---|---|---|
| `window_duration` | Sweep 2, 4, 6, 8, and 12 s on identical bags | Select the shortest window after which ATE/RPE and recovery behavior stabilize |
| `robust_kernel` | Compare `gaussian`, `huber`, and `none` on nominal and injected-outlier data | Use the option with the best error-continuity trade-off; do not choose from nominal data alone |
| `robust_kernel_scale` | Analyze whitened residuals and perform a local sweep around the 90th-99th percentile of accepted residuals | Outliers should be down-weighted without suppressing valid lateral motion |
| `lio_degraded_scale` | Sweep the covariance multiplier while recording factor decisions and fused trajectory | Degraded LIO must not dominate wheel constraints, while normal LIO remains useful |
| `recovery_stable_count` | Measure the number of consecutive stable LIO frames needed after a failure | Choose the smallest count that prevents re-entry on transient recovery spikes |

`window_duration` must be evaluated together with:

- output latency;
- update duration;
- CPU utilization;
- memory usage;
- number of active states and factors;
- ATE and RPE;
- behavior during LIO gaps;
- recovery alignment jump.

The chosen window is not the one with the lowest error in a single sequence.
It is the smallest window that is stable across the validation sequences and
fits the processing budget.

Because the current backend uses an incremental fixed-lag smoother, increasing
the window can improve local consistency while increasing marginalization and
optimization cost. Report both accuracy and resource usage.

### 3.5 Wheel factor noise

| Parameters | Meaning | Measurement method |
|---|---|---|
| `wheel_x_sigma` | Wheel relative-translation uncertainty in the forward direction | Compare wheel increments with an external reference under straight motion |
| `wheel_y_sigma` | Wheel relative-translation uncertainty in the lateral direction | Compare wheel increments with an external reference under pure lateral motion |
| `wheel_yaw_sigma` | Wheel relative-yaw uncertainty | Compare wheel yaw increments with an external reference under in-place rotation |

Use the following motion matrix:

1. straight forward motion;
2. straight backward motion;
3. pure lateral motion;
4. clockwise rotation;
5. counter-clockwise rotation;
6. diagonal motion;
7. combined translation and rotation;
8. repeated start-stop cycles;
9. wheel-slip and rapid acceleration trials.

For each motion type, align the wheel trajectory to the reference and compute
the relative increment error over the same time interval:

\[
e_{\mathrm{wheel},k}
=
\operatorname{Log}\left(
\Delta T_{\mathrm{reference},k}^{-1}
\Delta T_{\mathrm{wheel},k}
\right).
\]

Estimate a robust one-sigma scale for each constrained degree of freedom using
the median absolute deviation:

\[
\hat{\sigma}=1.4826\,\operatorname{median}\left(
|e-\operatorname{median}(e)|
\right).
\]

Add a repeatability and slip margin. Do not use a single low-speed run to set
`wheel_y_sigma`; lateral slip is an expected error mode of a mecanum platform.
The final value should be conservative enough that the wheel factor does not
force FAST-LIO to follow an incorrect lateral model.

### 3.6 LIO factor noise

| Parameters | Meaning | Measurement method |
|---|---|---|
| `lio_position_sigma` | Minimum system-level translational standard deviation for LIO factors | Measure relative FAST-LIO error against a trusted reference |
| `lio_rotation_sigma` | Minimum system-level rotational standard deviation for LIO factors | Measure relative FAST-LIO rotation error against a trusted reference |
| `lio_z_sigma_scale` | Additional uncertainty multiplier for vertical motion | Estimate the vertical-to-horizontal error ratio and validate on flat and non-flat terrain |
| `fallback_pose_sigma` | Weak prior used when neither primary motion factor is available | Verify that it does not over-constrain wheel-only prediction or create an artificial pose hold |

FAST-LIO covariance must be inspected before using it as a measurement
covariance. The backend already treats the configured LIO sigmas as a floor
because a frontend covariance can be overconfident for the external fused
trajectory.

For each sequence:

1. align FAST-LIO to the reference frame;
2. compute relative pose errors at the same event times;
3. separate translation, roll, pitch, yaw, and \(z\);
4. compute robust scale and bias;
5. inspect the reported FAST-LIO covariance;
6. compare predicted covariance with empirical squared error;
7. set the configured floor no smaller than the repeatability-supported scale.

Do not estimate LIO noise from the fused `/odom`; that would make the
measurement model self-referential. Use motion capture, RTK, a surveyed
reference trajectory, or a carefully defined independent reference.

For `lio_z_sigma_scale`, use:

\[
s_z=
\frac{\hat{\sigma}_z}{\hat{\sigma}_{xy}},
\]

then validate the result on:

- flat-floor motion;
- threshold crossing;
- ramp or uneven floor;
- pure rotation;
- static operation.

The scale must reduce false vertical motion without flattening genuine ramps.

### 3.7 Planar prior parameters

| Parameters | Measurement method |
|---|---|
| `use_planar_prior` | Compare enabled and disabled conditions on flat floor, ramps, and threshold crossings |
| `planar_z_sigma` | Set from acceptable vertical deviation of the platform on the target floor |
| `planar_roll_sigma` | Set from measured roll variation during normal motion |
| `planar_pitch_sigma` | Set from measured pitch variation during normal motion |
| `planar_reference_z` | Measure the local floor and base reference height; do not assume zero if the local frame is offset |
| `planar_reference_roll` | Estimate the stationary base attitude relative to the local frame |
| `planar_reference_pitch` | Estimate the stationary base attitude relative to the local frame |

The planar prior is a finite-covariance soft constraint, not a hard
assumption. Determine its sigmas from the platform's physically admissible
motion envelope and floor roughness. A small sigma can improve flat-floor
stability but can also suppress valid body motion on ramps or obstacles.

The acceptance test must include a no-prior versus prior comparison of:

- vertical drift;
- roll and pitch variation;
- horizontal ATE/RPE;
- ramp height change;
- threshold-crossing continuity;
- factor residual distribution.

### 3.8 Stationary detection and static handling

| Parameters | Measurement method |
|---|---|
| `stationary_speed_threshold` | Measure wheel speed distribution while physically stationary and during the slowest intended motion |
| `stationary_yaw_rate_threshold` | Measure wheel yaw-rate distribution while stationary and during the slowest intended rotation |
| `static_gyro_threshold` | Measure gyro norm distribution during stationary intervals |
| `static_min_duration` | Measure transient duration at start and stop; choose longer than the longest transient |
| `stationary_lio_scale` | Sweep covariance inflation during static LIO jitter |
| `stationary_pose_sigma` | Set from static fused-pose repeatability and acceptable hold variation |
| `zero_velocity_sigma` | Inactive in the current `Pose3` backend; do not tune as an implemented zero-velocity factor |

Collect at least three stationary recordings at each relevant operating
condition. Mark the true stationary intervals from the wheel command,
encoder state, and independent motion reference.

Choose thresholds using two distributions:

- stationary noise distribution;
- minimum intentional motion distribution.

The stationary threshold should lie above the 99th percentile of stationary
noise but below the smallest motion that the system must preserve. Validate
with slow lateral motion and slow yaw motion, because these are the most
likely to be incorrectly classified as stationary.

`stationary_lio_scale` and `stationary_pose_sigma` are then tuned after the
detector is fixed. Otherwise, an incorrect detector can be hidden by
excessive covariance inflation.

The required behavior is:

```text
confirmed stop -> capture one soft anchor -> reduce LIO influence
motion resumes -> release anchor -> preserve continuous output
```

### 3.9 Health-monitor parameters

| Parameters | Measurement method |
|---|---|
| `lio_nis_degraded_threshold` | Quantile of normal NIS-like values, followed by validation on labeled degradation |
| `lio_nis_failed_threshold` | ROC or cost-based separation of failed and acceptable observations |
| `lio_consistency_degraded_threshold` | Quantile of normal wheel/IMU consistency scores |
| `lio_consistency_failed_threshold` | Separation threshold for labeled inconsistent observations |
| `wheel_consistency_position_sigma` | Robust scale of normal wheel-LIO relative position error in the planar axes |
| `wheel_consistency_yaw_sigma` | Robust scale of normal wheel-LIO relative yaw error |
| `imu_consistency_yaw_sigma` | Robust scale of normal LIO yaw increment minus IMU-integrated yaw increment |
| `max_pose_jump` | Maximum physically plausible translation per LIO interval plus uncertainty margin |
| `max_pose_jump_yaw` | Maximum physically plausible yaw change per LIO interval plus uncertainty margin |
| `mag_norm_relative_change_threshold` | Quantile of normal consecutive magnetic-field norm changes |
| `mag_field_norm_min`, `mag_field_norm_max` | Magnetic calibration envelope; currently compatibility fields unless consumed by a validation path |

Health calibration requires the health monitor to be active. The current
profile `D` disables `use_lio_health`; use an explicitly documented health
profile or a custom profile with `use_lio_health: true` during calibration.
Do not mix health-enabled and health-disabled data when estimating thresholds.

For NIS-like calibration:

1. collect normal operation sequences;
2. compute the same six-dimensional diagonal covariance-normalized statistic
   used online;
3. set the degraded threshold from the normal upper quantile;
4. collect controlled degradation and failure sequences;
5. select the failed threshold using detection rate, false isolation rate, and
   detection delay;
6. validate on held-out sequences.

For the consistency scales, use the same residual definitions as the online
monitor:

\[
e_{\mathrm{wheel}}
=
\operatorname{Log}\left(
\Delta T_{\mathrm{wheel}}^{-1}
\Delta T_{\mathrm{LIO}}
\right),
\]

with the planar position components and yaw component extracted separately,
and:

\[
e_{\mathrm{imu,yaw}}
=
\operatorname{wrap}\left(
\Delta\psi_{\mathrm{LIO}}-\Delta\psi_{\mathrm{IMU}}
\right).
\]

Estimate each sigma from normal-operation data using the robust MAD scale,
then verify the false-alarm rate under slow lateral motion, slow yaw motion,
and normal wheel slip. The consistency thresholds are dimensionless after
division by these sigmas. Therefore, changing any of the three sigma values
requires rechecking the `3.0` and `6.0` degraded/failed thresholds.

The current monitor maps health evidence to covariance scaling, factor
acceptance, and recovery alignment. Record at minimum:

```text
lio_state
lio_nis
lio_innovation_norm
lio_wheel_consistency
lio_imu_consistency
lio_mag_consistency
lio_covariance_scale
lio_covariance_source
factor decision
recovery alignment
```

Report normal false alarms per sequence and per 1,000 LIO updates, not only
the largest observed NIS.

For jump thresholds, let \(v_{\max}\) and \(\omega_{\max}\) be the tested
platform limits and \(\Delta t_{\max}\) the largest accepted LIO interval:

\[
d_{\max}=v_{\max}\Delta t_{\max}+d_{\mathrm{margin}},
\]

\[
\psi_{\max}=\omega_{\max}\Delta t_{\max}+\psi_{\mathrm{margin}}.
\]

The margin must include measurement repeatability, timestamp uncertainty, and
the LiDAR-to-base lever-arm effect.

### 3.10 Parameters currently inactive in the localization graph

The following fields are declared and passed through the node interface but
do not currently define active IMU or magnetic localization factors:

```text
gravity
use_motioncore_imu
use_magnetic_heading_factor
mag_calibration_valid
zero_velocity_sigma
imu_accel_sigma
imu_gyro_sigma
imu_bias_accel_sigma
imu_bias_gyro_sigma
mag_noise_sigma
mag_field_norm_min
mag_field_norm_max
```

The current backend keeps IMU and magnetic data outside the localization graph
and uses them only for health-related processing where implemented. Do not
report these values as calibrated factor parameters or claim that changing
them changes the current fused pose. They should be calibrated only after an
explicit IMU preintegration, velocity/bias state, zero-velocity factor, or
magnetic-heading factor is implemented and separately validated.

## 4. Experimental Data Collection

### 4.1 Required topics and transforms

Record the following for every calibration bag:

```text
/motioncore/wheel_odom
/motioncore/imu/data
/motioncore/imu/mag
/fast_lio/odom
/fast_lio/cloud_registered_body
/odom
/motioncore/diagnostics
/tf
/tf_static
```

Also record:

- active YAML file and launch arguments;
- Git commit or source-tree identifier;
- robot payload and battery state;
- LiDAR and MotionCore startup order;
- floor type and route description;
- external reference trajectory, if available;
- CPU, memory, processing rate, and message-drop statistics.

### 4.2 Motion matrix

At minimum, collect:

1. static hold;
2. straight forward and backward motion;
3. pure lateral motion;
4. pure yaw rotation;
5. combined planar motion;
6. repeated start-stop motion;
7. slow motion near stationary thresholds;
8. rapid acceleration and braking;
9. wheel-slip or uneven-floor motion;
10. long corridor or low-feature motion;
11. LiDAR occlusion or controlled point-cloud degradation;
12. LIO loss and recovery;
13. flat floor, ramp, and threshold crossing;
14. repeated closed trajectory when a reference is available.

Use at least three repeats per condition and reserve complete sequences for
validation. Do not split consecutive samples from the same bag across training
and test sets.

### 4.3 External reference

Use one of:

- motion capture;
- surveyed indoor reference;
- RTK GNSS where the environment permits;
- a separately validated reference trajectory.

If no metric reference is available, report only relative repeatability,
continuity, residuals, and failure/recovery behavior. Do not claim an absolute
accuracy improvement from RViz screenshots or a single trajectory overlay.

## 5. Calibration and Validation Procedure

### Step 0: Freeze the interface

Validate topics, frame IDs, TF ownership, timestamp monotonicity, QoS, and
covariance order. Freeze the launch composition before numerical tuning.

### Step 1: Calibrate extrinsics and timing

Resolve LiDAR-to-base extrinsics and clock/arrival statistics. Repeat the
straight, lateral, and in-place rotation checks after every mounting change.

### Step 2: Measure raw sensor repeatability

Estimate wheel and FAST-LIO relative-pose errors separately. Do not use fused
output as the reference for either sensor's own noise model.

### Step 3: Select factor sigmas

Set wheel and LIO noise floors from robust empirical scales, then validate the
normalized residuals. If normal residuals are systematically much larger than
one standard deviation, the covariance model is overconfident or the
measurement model is incomplete.

### Step 4: Select planar and static behavior

Tune planar sigmas and stationary parameters using flat, ramp, threshold, and
slow-motion data. Confirm that the system does not convert real motion into a
stationary anchor.

### Step 5: Select window and robust loss

Run the same bags over the window and robust-kernel sweep. Select the smallest
configuration that meets accuracy, continuity, and compute constraints.

### Step 6: Calibrate health gates

Enable the health monitor, label normal/degraded/failed intervals, and tune
NIS, consistency, jump, scale, and recovery parameters. Evaluate the complete
state sequence:

```text
HEALTHY -> DEGRADED -> FAILED -> RECOVERING -> HEALTHY
```

### Step 7: Hold-out validation

Use unseen routes or operating conditions. Do not change thresholds after
looking at the final test results. Report:

- ATE and RPE;
- static position and attitude jitter;
- normal health false-alarm rate;
- failure detection rate;
- detection delay;
- factor acceptance/rejection rate;
- recovery duration;
- recovery pose jump;
- output frequency and latency;
- CPU and memory;
- message rejection and queue overflow counts.

## 6. Recommended Initial Ranges

These are sweep ranges, not final measured values:

| Group | Initial sweep |
|---|---|
| `window_duration` | `2, 4, 6, 8, 12 s` |
| `lio_position_sigma` | `0.02, 0.05, 0.10, 0.20 m` |
| `lio_rotation_sigma` | `0.01, 0.03, 0.05, 0.10 rad` |
| `lio_z_sigma_scale` | `1, 5, 10, 20, 50` |
| `wheel_x_sigma`, `wheel_y_sigma` | empirical scale multiplied by `0.75, 1.0, 1.5, 2.0` |
| `wheel_yaw_sigma` | empirical scale multiplied by `0.75, 1.0, 1.5, 2.0` |
| `robust_kernel` | `gaussian`, `huber` |
| `robust_kernel_scale` | `0.5, 1.0, 1.5, 2.0, 3.0` |
| `lio_degraded_scale` | `5, 10, 20, 50` |
| `recovery_stable_count` | `3, 5, 8, 10` LIO updates |
| NIS degraded threshold | normal empirical `0.99, 0.995, 0.999` quantiles |
| NIS failed threshold | ROC/cost sweep above the degraded threshold |

Do not sweep all parameters simultaneously. Use the order defined in Section
5 and keep one parameter group fixed while measuring another.

## 7. Recording Template

Each accepted parameter set must have a record containing:

```text
date:
operator:
robot_configuration:
payload:
sensor_mounting:
external_reference:
bag_paths:
yaml_path:
launch_arguments:
source_revision:
calibration_stage:
parameter_changes:
raw_measurements:
selected_values:
validation_sequences:
ATE:
RPE:
static_jitter:
health_false_alarm_rate:
failure_detection_rate:
detection_delay:
recovery_time:
recovery_jump:
processing_rate:
cpu:
memory:
message_drop_count:
queue_overflow_count:
known_limitations:
```

The final YAML values should only be changed after the corresponding record
contains both the measurement evidence and the held-out validation result.
