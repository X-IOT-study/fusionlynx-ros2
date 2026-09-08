# AGENTS.md

Last updated: 2026-09-08

This file is the entry point for agents working in this workspace. Read it
before changing code, configuration, documentation, experiment records, or
research artifacts.

## 1. Project scope

This repository is the thesis-oriented ROS 2 integration workspace for the
MotionCore mecanum robot. The production pipeline is:

```text
MotionCore firmware -> SDK -> ROS 2 driver -> robot description / TF
-> wheel odometry + FAST-LIO -> state_fusion_backend -> fused /odom
-> RTAB-Map -> evaluation and thesis artifacts
```

FAST-LIO is the primary LIO frontend. ORB-SLAM3 is comparison-only and must
not replace the production FAST-LIO path.

## 2. Source and documentation boundaries

```text
src/motioncore/       MotionCore ROS 2 integration and protocol-facing code
src/FAST_LIO/         FAST-LIO frontend and UniLidar configuration
src/fusionlynx/       robot, SLAM, fusion, mapping, and bringup packages
docs/                 architecture, contracts, development, and experiments
```

Authoritative documents:

- System architecture: `docs/architecture.md`
- TF and physical extrinsics: `docs/tf.md`
- Fusion and estimator contracts: `docs/fusion.md`
- FAST-LIO and RTAB-Map: `docs/slam.md`
- Deployment, build, and runtime checks: `docs/deployment.md`
- Development and research workflow: `docs/development.md`
- Git staging and commit policy: `docs/development/git_workflow.md`
- Complete pre-split rule text: `docs/AGENTS.full.md`

Read the relevant document before changing behavior. Treat the complete
pre-split file as an archive and compatibility reference; topic documents are
the maintained locations for future edits.

## 3. Hard system contracts

- `state_fusion_backend` owns the dynamic `odom -> base_link` transform.
- RTAB-Map owns the dynamic `map -> odom` transform.
- Robot description / static publishers own `base_link -> sensor` transforms.
- FAST-LIO must not publish a competing `odom -> base_link` in the integrated
  system.
- There must be one authoritative publisher for each dynamic TF edge.
- `/cmd_vel` is the unified motion command interface.
- `/odom` is authoritative fused local odometry, not RTAB-Map output.
- Do not silently rename or repurpose established topics, frames, messages, or
  lifecycle behavior.
- Do not add a second production localization path or handwritten mapper
  without an explicit architecture decision.
- URDF/Xacro is the source of truth for physical sensor placement.

## 4. Modification rules

1. Inspect the owning package and its documentation before editing behavior.
2. For architecture, data-contract, experiment, or hardware changes, read the
   relevant Research Vault material first.
3. Keep changes local to the owning package and preserve public contracts.
4. Validate frame IDs, timestamps, covariances, QoS, and pose conventions at
   sensor and fusion boundaries.
5. Preserve lifecycle safety, fixed-lag behavior, health transitions,
   initialization, timeout handling, and recovery behavior.
6. Add focused tests for changed behavior and declare new dependencies.
7. Search affected publishers, subscribers, TF broadcasters, launch files,
   YAML files, tests, and documentation before changing an interface.
8. Do not use broad refactors to hide a local defect.
9. After every change, stage the related files with `git add` and create a
   clear `git commit` so that each modification remains traceable.
10. Never use `git add .` or `git add -A`; stage an explicit file or directory
    allowlist as described in `docs/development/git_workflow.md`.
11. Do not reset, discard, or overwrite existing user changes. Do not commit
    unrelated work.
12. Do not commit build output, logs, raw bags, databases, point clouds,
    `.mineru_output/`, or unreviewed generated artifacts.

## 5. Required reporting

If a change affects motion control, TF, topic names, frame semantics,
lifecycle behavior, sensor extrinsics, fusion architecture, SLAM
configuration, experiment configuration, evaluation methodology, or paper
assets, update the matching topic document and Research Vault record in the
same working session.

Keep stable rules here, package implementation knowledge in package
`README.md`/`design.md` files, architecture contracts in `docs/`, and
experiment decisions/results in the Research Vault.

## 6. Priority order

When requirements conflict, use this order:

1. Safety and lifecycle correctness.
2. TF and frame correctness.
3. Sensor/topic contract correctness.
4. Timestamp and synchronization correctness.
5. State-estimation and fusion correctness.
6. Mapping consistency.
7. Reproducibility and experiment traceability.
8. Diagnostics and robustness.
9. Code cleanliness and refactoring.

## 7. Working tree layout

The workspace is `/home/dream/ros2_ws` and uses ROS 2 Humble. The Research
Vault is `/home/dream/Documents/work_xxxx/SLAM_Thesis_Vault`. Firmware and the
host SDK remain authoritative in `/home/dream/tools/MotionCore-Firmware` and
`/home/dream/tools/motioncore-client`.

Package-specific rules under `src/` are discovered by path. Start with
`src/fusionlynx/AGENTS.md` for FusionLynx package changes, then consult the
package README and the topic document named above.
