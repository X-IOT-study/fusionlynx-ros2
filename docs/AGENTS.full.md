# AGENTS.md

Last updated: 2026-08-23

This file is the entry point for agents working in this workspace.

Read it before changing code, configuration, documentation, experiment records,
or research artifacts.

---

## 1. Project Goal

This repository is a thesis-oriented robotics research workspace for a
MotionCore mecanum robot.

The project is a complete research pipeline rather than a standalone ROS 2
driver:

```text
MotionCore Firmware
        ↓
motioncore-client / SDK
        ↓
ROS 2 hardware driver
        ↓
robot description / TF
        ↓
bringup
        ↓
wheel odometry + FAST-LIO
        ↓
state_fusion_backend
        ↓
fused /odom
        ↓
RTAB-Map
        ↓
evaluation / experiments
        ↓
thesis figures, tables, and paper assets
```

The primary LIO frontend is FAST-LIO.

ORB-SLAM3 is comparison-only and must not replace the production FAST-LIO
path.

The long-term objective is a reproducible thesis pipeline in which code,
configuration, experiments, logs, results, and paper artifacts remain
consistent.

---

## 2. Core Locations

```text
ROS workspace:
    /home/dream/ros2_ws

Research vault:
    /home/dream/Documents/work_xxxx/SLAM_Thesis_Vault

Firmware source of truth:
    /home/dream/tools/MotionCore-Firmware

SDK / host client source of truth:
    /home/dream/tools/motioncore-client

Control baseline / demo UI:
    /home/dream/tools/BodyMonitor
```

ROS distribution:

```text
ROS 2 Humble
```

The workspace vendors the MotionCore SDK under `src/`.

Do not create a second implementation of the MotionCore wire protocol inside
ROS packages unless explicitly required.

---

## 3. Repository Structure

### Hardware / protocol layer

```text
MotionCore Firmware
    → motor control, sensors, wire protocol

motioncore-client
    → host-side protocol / SDK authority

src/motioncore/
    → ROS 2 integration
```

### ROS 2 layer

```text
src/motioncore/
    → hardware driver, wheel kinematics, messages

src/FAST_LIO/
    → FAST-LIO frontend and UniLidar configuration

src/fusionlynx/
    → robot, SLAM, fusion, mapping, and bringup packages
```

Important packages:

* `motioncore_msgs`: custom ROS 2 messages.
* `motioncore_driver`: MotionCore lifecycle hardware driver.
* `wheel_kinematics`: wheel encoder to odometry conversion.
* `fusionlynx_description`: URDF/Xacro, robot model, sensor mounting
  extrinsics, static TF, and RViz model configuration.
* `fusionlynx_slam`: system-level SLAM composition and parameters.
* `state_fusion_backend`: GTSAM fixed-lag wheel/LIO fusion, health monitoring,
  diagnostics, and tests.
* `fusionlynx_mapping`: RTAB-Map integration, ICP loop closure, and
  map-localization launch controls.
* `fusionlynx_bringup`: integrated PC-side bringup.

### Research layer

The Research Vault contains:

```text
03_Experiment/
    reproducible experiment configurations and results

04_Log/
    issues, decisions, and progress logs

05_Data/
    bags, calibration data, evaluation data

06_Paper/
    chapter material, figures, tables, references

09_Assets/
    reusable visual and experimental assets
```

Do not treat the Vault as optional project documentation. It is part of the
research workflow.

---

## 4. System Architecture

The current localization/mapping architecture is:

```text
MotionCore wheel odometry ───────┐
                                 │
                                 ▼
                            state_fusion_backend
                                 ▲
                                 │
FAST-LIO ── /fast_lio/odom ──────┘
                                 │
                              /odom
                                 │
                                 ▼
                              RTAB-Map
                                 ▲
                                 │
                /fast_lio/cloud_registered_body
```

The responsibilities are intentionally separated:

```text
FAST-LIO
    → LIO frontend / local motion estimation

state_fusion_backend
    → authoritative local fused odometry

RTAB-Map
    → mapping / loop closure / map -> odom

fusionlynx_description
    → physical robot geometry and sensor extrinsics

fusionlynx_mapping
    → RTAB-Map integration, ICP loop closure, and map localization
```

Do not introduce another handwritten mapper into `fusionlynx_mapping`.

Do not introduce a second production localization path without an explicit
architecture decision.

---

## 5. TF Ownership Contract

Each dynamic TF edge must have exactly one authoritative publisher.

```text
map -> odom
    RTAB-Map

odom -> base_link
    state_fusion_backend

base_link -> sensor frames
    robot_state_publisher / static TF
```

These ownership rules are hard constraints.

In particular:

* `state_fusion_backend` owns `odom -> base_link`.
* RTAB-Map owns `map -> odom`.
* FAST-LIO must not publish a competing `odom -> base_link` in the integrated
  system.
* Do not add another dynamic publisher for an existing TF edge.
* Do not create disconnected or duplicate TF trees.

Before modifying TF, inspect the existing tree and identify the current
authority.

Useful commands:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

---

## 6. Sensor Extrinsic Contract

`fusionlynx_description` is the source of truth for physical sensor
placement.

Do not:

* guess sensor extrinsics;
* replace physical transforms with identity transforms;
* silently swap coordinate axes;
* duplicate sensor mounting parameters in unrelated packages;
* compensate for an incorrect URDF transform with an unrelated static TF.

When a downstream component requires an inverse or composed transform, derive
it mathematically from the authoritative extrinsic.

If the physical mounting changes:

1. update the owning URDF/Xacro/configuration;
2. verify the resulting TF;
3. update dependent fusion transforms;
4. test the affected sensor pipeline.

---

## 7. ROS 2 Topic Contracts

The following interfaces are stable unless an intentional architecture change
is being made:

```text
Motion / hardware:

/cmd_vel
/motioncore/encoder_raw
/motioncore/imu/data_raw
/motioncore/imu/mag_raw

Processed MotionCore data:

/motioncore/wheel_odom
/motioncore/imu/data
/motioncore/imu/mag
/motioncore/diagnostics

LiDAR / FAST-LIO:

/unilidar/cloud
/unilidar/imu
/fast_lio/odom
/fast_lio/cloud_registered_body

Fusion:

/odom
```

Do not silently rename, repurpose, or change frame semantics of an established
interface.

If an interface must change intentionally, update all affected publishers,
subscribers, launch files, YAML files, tests, documentation, and Vault records
in the same working session.

---

## 8. Motion Control Contract

The unified motion command interface is:

```text
/cmd_vel
```

The production control path is:

```text
joy
  ↓
game_controller_node
  ↓
teleop_twist_joy
  ↓
/cmd_vel
  ↓
motioncore_driver
  ↓
MotionCore SDK
  ↓
firmware
```

Do not bypass ROS 2 and send raw serial commands from teleoperation code.

Do not place local handwritten SDK/protocol logic into the ROS teleop path.

Current confirmed 8BitDo mapping:

```text
Left stick vertical      → linear.x
Left stick horizontal    → angular.z
Right stick horizontal   → linear.y

LB → deadman
RB → turbo
```

Expected behavior:

```text
LB held      → movement enabled
LB released  → immediate stop
Right stick  → mecanum lateral motion
```

Do not change joystick mappings without checking the actual controller
mapping.

If controller behavior changes, update both workspace documentation and the
Research Vault.

---

## 9. Lifecycle Safety Contract

`motioncore_driver` is a ROS 2 lifecycle node.

Expected lifecycle behavior:

```text
configure
    → prepare serial / SDK / publishers

activate
    → start live communication and publishing

deactivate
    → stop motion and release active resources
```

The lifecycle state is a safety boundary.

Do not weaken lifecycle behavior merely to simplify development or testing.

When debugging a robot that does not move, check lifecycle state before
changing motion-control code.

---

## 10. FAST-LIO Contract

FAST-LIO is the primary LIO frontend.

Treat:

```text
/fast_lio/odom
```

as a frontend observation rather than the final authoritative robot odometry.

At the fusion boundary, verify:

1. `header.frame_id`;
2. `child_frame_id`;
3. timestamp;
4. covariance;
5. sensor-to-base extrinsic;
6. pose direction / transform convention.

Do not silently reinterpret an unexpected frame contract.

The fusion backend should prefer consecutive relative LIO motion where the
factor graph requires relative measurements rather than repeatedly treating
absolute frontend poses as independent measurements.

FAST-LIO dynamic TF publication must remain disabled in the integrated system
when it would compete with `state_fusion_backend`.

---

## 11. State Fusion Backend

`state_fusion_backend` is the authoritative source of fused local odometry.

The current backend uses a GTSAM incremental fixed-lag smoother.

The intended localization chain is:

```text
wheel odometry
      +
FAST-LIO relative motion
      +
planar motion prior
      ↓
fixed-lag fusion
      ↓
/odom
```

MotionCore IMU and magnetometer data may currently be used for motion/LIO
health monitoring according to configuration.

Do not add a new localization factor merely because a sensor topic exists.

Before changing the factor graph, inspect the current configuration,
implementation, tests, and Research Vault design notes.

Preserve:

* fixed-lag behavior;
* timestamp handling;
* covariance handling;
* initialization;
* health state transitions;
* timeout handling;
* recovery behavior;
* single-source TF ownership.

Prefer small changes over redesigning the factor graph during unrelated bug
fixes.

---

## 12. LIO Health and Diagnostics

LIO health state changes should be logged, not every frame.

Use:

```text
INFO
    initialization
    recovery to HEALTHY

WARN
    DEGRADED
    RECOVERING

ERROR
    FAILED
    timeout
    unrecoverable tracking loss
```

Do not add per-frame `WARN` logging to high-rate processing loops.

High-rate diagnostic information belongs in:

```text
/motioncore/diagnostics
```

When debugging a health problem, distinguish:

```text
sensor data
    ↓
timestamp / QoS
    ↓
frame / extrinsic
    ↓
FAST-LIO
    ↓
fusion
    ↓
TF publication
```

Fix the failing boundary rather than adding downstream compensations.

---

## 13. RTAB-Map Contract

RTAB-Map is the mapping and loop-closure layer.

Its intended inputs are:

```text
/odom
/fast_lio/cloud_registered_body
```

RTAB-Map produces the map-side products and:

```text
map -> odom
```

Do not make RTAB-Map publish the authoritative `/odom`.

Use the cloud representation intended for RTAB-Map.

Do not transform an already world/global-registered cloud a second time as
though it were still in the body/sensor frame.

Preserve approximate synchronization where odometry and cloud timestamps are
produced by independent nodes.

Keep the RTAB-Map integration inside `fusionlynx_mapping`.

For an existing database, set RTAB-Map to localization mode explicitly:

```bash
ros2 launch fusionlynx_bringup pc.launch.py \
  mapping_database_path:=/absolute/path/fusionlynx_lidar_map.db \
  localization:=true \
  initial_pose:="x y z roll pitch yaw"
```

`database_path` selects the SQLite file; it does not select mapping versus
localization mode. The database must contain the scan data used by the current
LiDAR input contract. Exported PCD and TUM/CSV trajectories are analysis
outputs and cannot replace the RTAB-Map graph database for localization.

---

## 14. Code Modification Rules

1. Read the relevant package documentation before modifying behavior.
2. For architecture or data-contract changes, read the Research Vault first.
3. Preserve topic, frame, message, and lifecycle contracts.
4. Keep TF ownership single-source.
5. Do not add another `/odom` publisher.
6. Treat URDF/Xacro as the physical extrinsic source of truth.
7. Validate frame IDs at fusion boundaries.
8. Preserve timestamp and covariance semantics.
9. Prefer relative measurements for sequential LIO motion.
10. Use the existing health state machine for failure/recovery.
11. Keep changes local to the owning package.
12. Declare new ROS/package dependencies in `package.xml`.
13. Add focused tests for changed fusion behavior.
14. Reuse existing repository patterns before introducing new abstractions.
15. Avoid broad refactors while fixing a specific issue.
16. Search for all affected publishers, subscribers, TF broadcasters, launch
    files, YAML files, and tests before changing an interface.
17. After every change, stage the related files with `git add` and create a
    clear `git commit` so that each modification remains traceable.

---

## 15. Build and Test Workflow

Initialize the environment:

```bash
source /opt/ros/humble/setup.bash
source /home/dream/ros2_ws/install/setup.bash
```

For normal development, prefer targeted builds:

```bash
cd /home/dream/ros2_ws

colcon build --symlink-install --packages-up-to <package_name>

source install/setup.bash
```

Use a full workspace build when validating integration:

```bash
colcon build --symlink-install
source install/setup.bash
```

After changing launch files, YAML files, URDF/Xacro, or installed resources,
rebuild and source the overlay before testing.

Do not assume `ros2 launch` is executing the source-tree copy.

Focused tests:

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

For the fusion backend:

```bash
colcon test --packages-select state_fusion_backend
```

---

## 16. Integrated Bringup

The normal PC-side entry point is:

```bash
ros2 launch fusionlynx_bringup pc.launch.py
```

The integrated bringup composes the robot description, FAST-LIO,
state-fusion backend, RTAB-Map, and the intended RViz configuration.

Hardware drivers are deployment-specific and may need to be running
separately.

For isolated testing:

```bash
ros2 launch fusionlynx_slam slam.launch.py
ros2 launch fusionlynx_mapping mapping.launch.py
```

Do not modify integrated bringup to hide a problem belonging to an individual
subsystem.

---

## 17. Runtime Debugging

Useful checks:

```bash
ros2 topic info -v /odom
ros2 topic hz /odom

ros2 topic echo /fast_lio/odom --once

ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

When debugging raw odometry or motion axes:

* set RViz Fixed Frame to `odom`;
* test forward motion separately;
* test lateral motion separately;
* test rotation separately;
* verify TF before judging the trajectory.

Do not use a changing `map` frame to judge raw local odometry correctness.

For sensor/topic problems, check:

```text
topic existence
    ↓
QoS compatibility
    ↓
timestamp
    ↓
frame_id / child_frame_id
    ↓
TF availability
    ↓
data content
```

For hardware problems, check:

```text
lifecycle state
    ↓
serial device
    ↓
permissions
    ↓
baudrate
    ↓
device_id
    ↓
auto_report
    ↓
ROS topic output
```

---

## 18. Research Vault and Documentation Rules

The Research Vault is part of the project workflow.

Read the Vault before making architecture or experiment-contract changes.

Important references include:

```text
/home/dream/Documents/work_xxxx/SLAM_Thesis_Vault/README.md

00_Project/next_phase_plan.md

01_System/architecture.md
01_System/tf_tree.md

02_Development/README.md
02_Development/motioncore_driver/README.md
02_Development/motioncore_driver/design.md
```

When an interface or architecture changes, update the matching documentation
and log.

When hardware-affecting behavior changes, record the change in the Vault.

Experimental work should remain traceable:

```text
experiment
    → configuration
    → result
    → interpretation
```

Do not put temporary debugging observations or one-off parameter values into
`AGENTS.md`.

Use:

```text
AGENTS.md
    → stable rules and constraints

package README / design.md
    → package-specific implementation knowledge

Research Vault
    → architecture decisions, experiments, results, and thesis material

source / YAML / Xacro
    → actual implementation parameters
```

---

## 19. Research Papers and `.mineru_output`

The repository may contain a `.mineru_output/` directory containing Markdown
files extracted from research papers using MinerU.

These papers are reference material for:

* SLAM / LIO algorithms;
* factor-graph and state-estimation methods;
* sensor fusion;
* fixed-lag smoothing;
* loop closure;
* RTAB-Map and related mapping methods;
* experimental methodology;
* mathematical formulations and terminology.

When a research question is relevant to estimator or SLAM design, agents may
search `.mineru_output/` and consult the relevant papers before proposing an
algorithmic change.

Use the original paper's equations, definitions, assumptions, and experimental
setup where available. Do not rely on memory when the relevant paper is
available in `.mineru_output/`.

### Paper vs. Project Architecture

`.mineru_output/` is reference material, not the source of truth for the
current system.

A method described in a paper must not be treated as an existing project
feature merely because the paper is present in the repository.

Do not add, remove, or replace estimator factors solely because a paper uses a
different formulation.

For example, a paper containing IMU preintegration, magnetic heading factors,
or additional motion constraints does not by itself authorize adding those
factors to `state_fusion_backend`.

First determine whether the proposed method is:

1. part of the current architecture;
2. an explicitly requested feature;
3. a proposed experimental alternative; or
4. only theoretical background.

If a paper motivates an architecture change, document the proposed change
before modifying the implementation.

### Estimator Audit and Refactoring

When explicitly asked to audit or refactor `state_fusion_backend`, do not
assume that the existing implementation is architecturally correct.

First reconstruct and verify:

* state definition;
* factor graph structure;
* factor semantics;
* coordinate-frame conventions;
* sensor extrinsics;
* timestamp model;
* initialization;
* fixed-lag behavior;
* marginalization behavior;
* covariance handling;
* health-state interaction;
* output pose / twist / covariance;
* TF publication ownership.

Separate the following three concepts:

```text
intended architecture
    → what the project says the estimator should do

current implementation
    → what the source code currently does

observed behavior
    → what runtime logs, experiments, and tests demonstrate
```

If these disagree, identify the discrepancy before changing code.

Do not preserve an incorrect implementation merely because it already exists.

Do not introduce patches that make the implementation increasingly dependent on
special cases when the underlying estimator model is incorrect.

For estimator refactoring, prefer this sequence:

```text
audit
  ↓
mathematical model
  ↓
data / frame / timestamp contracts
  ↓
factor graph design
  ↓
implementation plan
  ↓
focused implementation
  ↓
unit tests
  ↓
runtime validation
```

Do not silently introduce algorithmic assumptions from papers, previous
experiments, or unrelated packages.

### Information Priority

For architectural decisions, use the following priority:

1. Explicit user instructions for the current task.
2. `AGENTS.md` and explicit architecture contracts.
3. Research Vault architecture and decision records.
4. Package README / design documents and configuration.
5. Relevant research papers in `.mineru_output/`.
6. Current source code, as evidence of the existing implementation.
7. Runtime behavior, logs, and experiments, as evidence of the current system.

When the current implementation conflicts with an explicit architecture
contract, do not assume the implementation is correct. Identify the
discrepancy first.

When runtime behavior conflicts with the documented architecture, treat it as
evidence of an implementation or integration problem until verified otherwise.

Never modify or overwrite `.mineru_output/` unless explicitly requested.
Treat extracted paper files as read-only reference material.

---

## 20. Files to Inspect First

For MotionCore driver changes:

```text
src/motioncore/motioncore_driver/README.md
src/motioncore/motioncore_driver/design.md
src/motioncore/motioncore_driver/launch/motioncore_driver.launch.py
src/motioncore/motioncore_driver/launch/motioncore_joy.launch.py
src/motioncore/motioncore_driver/config/motioncore_xbox.config.yaml
```

For system architecture changes:

```text
/home/dream/Documents/work_xxxx/SLAM_Thesis_Vault/README.md
/home/dream/Documents/work_xxxx/SLAM_Thesis_Vault/01_System/architecture.md
/home/dream/Documents/work_xxxx/SLAM_Thesis_Vault/01_System/tf_tree.md
```

For fusion changes, inspect:

```text
src/fusionlynx/state_fusion_backend/
```

including its configuration, source, launch files, and tests.

For mapping changes, inspect:

```text
src/fusionlynx/fusionlynx_mapping/
```

and the corresponding RTAB-Map configuration.

---

## 21. Reporting Rule

If a change affects any of the following:

```text
motion control
TF
topic names
frame semantics
lifecycle behavior
sensor extrinsics
fusion architecture
SLAM configuration
experiment configuration
evaluation methodology
paper assets
```

update the relevant documentation and Research Vault records in the same
working session.

Do not leave architectural or hardware-affecting changes undocumented.

---

## 22. Priority Order

When requirements conflict, prioritize:

```text
1. Safety and lifecycle correctness
2. TF and frame correctness
3. Sensor/topic contract correctness
4. Timestamp and synchronization correctness
5. State-estimation / fusion correctness
6. Mapping consistency
7. Reproducibility and experiment traceability
8. Runtime diagnostics and robustness
9. Code cleanliness / refactoring
```

Do not sacrifice an established interface, safety boundary, TF ownership rule,
or experiment reproducibility merely to simplify a local implementation.

---

## 23. Current Project Direction

The current development direction is:

```text
MotionCore driver
    ↓
robot_description / TF
    ↓
FAST-LIO
    ↓
wheel + LIO fixed-lag fusion
    ↓
authoritative /odom
    ↓
RTAB-Map
    ↓
evaluation
    ↓
thesis experiments and paper assets
```

Keep this architecture coherent.

Do not introduce parallel implementations of components that already have an
established owner.

When a new architecture is genuinely required, document the decision first,
then update the affected implementation, tests, and Research Vault records.
