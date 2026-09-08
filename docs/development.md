# Development and Research Workflow

This document owns cross-cutting modification, documentation, and research
rules. Git staging details remain in `docs/development/git_workflow.md`.
The complete pre-split text remains in `docs/AGENTS.full.md`.

## Before changing code

Read the owning package README/design document. For architecture or
data-contract changes, read the Research Vault, especially:

```text
/home/dream/Documents/work_xxxx/SLAM_Thesis_Vault/README.md
.../01_System/architecture.md
.../01_System/tf_tree.md
.../02_Development/README.md
```

Inspect current configuration, source, launch files, tests, publishers,
subscribers, TF broadcasters, and affected interfaces. Reuse repository
patterns and keep the change local.

## Documentation placement

```text
AGENTS.md                    stable workspace rules and index
docs/architecture.md         system responsibilities and topic contracts
docs/tf.md                   TF ownership and physical extrinsics
docs/fusion.md               estimator and health contracts
docs/slam.md                 FAST-LIO and RTAB-Map integration
docs/deployment.md           build, lifecycle, bringup, runtime checks
docs/development.md          cross-cutting workflow and research rules
package README/design.md     package implementation knowledge
Research Vault                decisions, experiments, results, thesis assets
source/YAML/Xacro             actual implementation parameters
```

When an interface or architecture changes, update matching documentation and
Vault records in the same session. Experimental work must remain traceable:

```text
experiment -> configuration -> result -> interpretation
```

Do not put temporary debugging observations or one-off parameter values into
`AGENTS.md`.

## Research references

`.mineru_output/` is read-only reference material for SLAM, LIO, fusion,
loop-closure, and evaluation questions. Do not modify it. A method in a paper
is not an existing project feature. Add or remove estimator factors only when
required by the current architecture, explicit user request, or a documented
experimental alternative.

## Verification

Use the smallest behavior-scoped test first, then a targeted build or test.
For estimator changes, validate frames, timestamps, covariance, initialization,
health transitions, fixed-lag behavior, and TF publication ownership. Do not
finish an architecture or hardware-affecting change without executable
validation when the environment provides it.
