cmake_minimum_required(VERSION 3.8)
project(fusionlynx_loop_closure)

find_package(ament_cmake REQUIRED)

# Modification: 2026-08-21, WenSheng Xu.
# Reason: Reserve the loop-closure package boundary without introducing a
# placeholder algorithm or publishing an unverified optimized trajectory.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
