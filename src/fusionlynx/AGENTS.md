# FusionLynx Package Rules

These rules apply to packages under `src/fusionlynx/`. Workspace-wide rules
remain in the repository root `AGENTS.md`.

## Package ownership

- `fusionlynx_description` owns URDF/Xacro, robot geometry, sensor mounting,
  static TF, and RViz model configuration.
- `fusionlynx_slam` owns system-level FAST-LIO composition and parameters.
- `state_fusion_backend` owns wheel/LIO fixed-lag fusion, health monitoring,
  diagnostics, tests, `/odom`, and `odom -> base_link`.
- `fusionlynx_mapping` owns RTAB-Map integration, ICP loop closure, map
  localization controls, and mapping launch/configuration.
- `fusionlynx_bringup` owns integrated PC-side composition.

Do not duplicate an owning package's parameters, transforms, protocol logic,
or dynamic TF publisher in another package.

## Local change rules

Read the target package README/design document and relevant `docs/` topic file
before editing. Preserve topic, frame, lifecycle, timestamp, covariance, and
sensor-extrinsic contracts. Add focused tests for changed fusion behavior and
update package dependencies when needed.

For fusion changes, inspect the factor graph, initialization, fixed-lag
behavior, health transitions, timeout/recovery paths, and output TF before
coding. For mapping changes, preserve RTAB-Map's `/odom` input and its
`map -> odom` ownership. For description changes, verify the resulting TF and
all dependent sensor transforms.

Build the smallest affected package first:

```bash
colcon build --symlink-install --packages-up-to <package_name>
colcon test --packages-select <package_name>
```

Record architecture, interface, hardware, and experiment changes in the
matching `docs/` document and Research Vault.
