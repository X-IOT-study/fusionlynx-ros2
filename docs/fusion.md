# State Fusion and Estimation

This document owns `state_fusion_backend` contracts and estimator audit rules.
The complete pre-split text remains in `docs/AGENTS.full.md`.

## Intended chain

```text
wheel odometry + FAST-LIO relative motion + planar motion prior
    -> GTSAM incremental fixed-lag smoother
    -> authoritative /odom
```

`/fast_lio/odom` is a frontend observation, not final robot odometry. Where
the factor graph requires sequential motion, prefer consecutive relative LIO
measurements instead of treating absolute frontend poses as independent
measurements.

Preserve fixed-lag behavior, timestamp handling, covariance handling,
initialization, health transitions, timeout handling, recovery behavior, and
single-source TF ownership. Do not add a localization factor merely because
a sensor topic exists.

## Health and diagnostics

Log state changes rather than every high-rate frame:

```text
INFO: initialization and recovery to HEALTHY
WARN: DEGRADED and RECOVERING
ERROR: FAILED, timeout, and unrecoverable tracking loss
```

These transitions require `use_lio_health: true` (Vault profiles E–G); the
default profile D does not run LIO health evaluation.

High-rate information belongs in `/motioncore/diagnostics`. Debug the failing
boundary in this order:

```text
sensor data -> timestamp / QoS -> frame / extrinsic -> FAST-LIO
-> fusion -> TF publication
```

## Estimator audit

When explicitly auditing or refactoring the backend, reconstruct and verify:

- state definition and factor graph;
- factor semantics and coordinate-frame conventions;
- sensor extrinsics and timestamp model;
- initialization and fixed-lag marginalization;
- covariance handling and health-state interaction;
- output pose, twist, covariance, and TF ownership.

Separate intended architecture, current implementation, and observed runtime
behavior. If they disagree, identify the discrepancy before changing code.
Do not import factors or assumptions from a paper merely because the paper is
present in `.mineru_output/`; papers are reference material, not project
source of truth.
