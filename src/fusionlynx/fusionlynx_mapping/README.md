# fusionlynx_mapping

`fusionlynx_mapping` is the FusionLynx mapping integration package. It no
longer contains a handwritten point-cloud accumulator or an ikd-Tree mapping
implementation. The formal mapping backend is RTAB-Map.

## Input contract

```text
/odom
/fast_lio/cloud_registered_body
odom -> base_link -> body
```

RTAB-Map consumes the fused `/odom` as its odometry prior and FAST-LIO's
deskewed body-frame scan for 3D mapping. It publishes the global map topics
and owns the `map -> odom` TF. It does not publish `/odom`; that topic remains
owned by `state_fusion_backend`.

## Launch

```bash
ros2 launch fusionlynx_mapping mapping.launch.py
```

The wrapper defaults to:

```text
approx_sync: true
approx_sync_max_interval: 0.10
topic_queue_size: 30
sync_queue_size: 30
publish_tf_map: true
publish_tf_odom: false
rtabmap_viz: false
```

The approximate synchronizer is required because `/odom` and
`/fast_lio/cloud_registered_body` are produced by independent nodes and do not
normally have identical message timestamps. The global-frame
`/fast_lio/cloud_registered` and accumulated `/fast_lio/local_map` are
deliberately not used as RTAB-Map input: applying `/odom` to those topics would
transform an already world-registered cloud a second time.

The wrapper intentionally leaves RTAB-Map's `odom_frame_id` empty. In the
Humble `rtabmap_launch` interface, setting that parameter makes RTAB-Map use
the TF tree instead of subscribing to the odometry topic. FusionLynx requires
RTAB-Map to consume the fused `/odom` message, so the parameter must remain
unset. RTAB-Map still publishes `map -> odom` through `publish_tf_map`.

## Responsibility boundary

- `state_fusion_backend`: publishes `/odom` and `odom -> base_link`;
- `fusionlynx_mapping`: launches and configures RTAB-Map;
- RTAB-Map: consumes `/odom` and raw cloud, publishes the map and
  `map -> odom`;
- `robot_state_publisher`: publishes the static robot and sensor transforms;
- FAST-LIO: publishes `/fast_lio/odom` as an LIO observation and
  `/fast_lio/cloud_registered_body` as a deskewed scan.

RTAB-Map does not publish `/odom`. If `/odom` is absent, first check
`state_fusion_backend_node` and `/fast_lio/odom`; the mapping backend cannot
replace the fusion backend or bootstrap its output topic.
