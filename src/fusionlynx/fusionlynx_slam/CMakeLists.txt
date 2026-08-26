cmake_minimum_required(VERSION 3.8)
project(fusionlynx_slam)

find_package(ament_cmake REQUIRED)

# Modification: 2026-08-21, WenSheng Xu.
# Reason: The top-level SLAM package is launch/configuration only. Runtime
# algorithms remain in their own ROS packages so they can be tested and
# replaced independently.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/fast_lio_base_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
