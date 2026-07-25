cmake_minimum_required(VERSION 3.5)
project(unitree_lidar_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# ==========================================================
# 自动检测 unilidar-sdk2 安装位置
# ==========================================================
set(UNILIDAR_SDK_DIR "/usr/local/unilidar-sdk2" CACHE PATH "Path to unilidar-sdk2 installation")

if(EXISTS "${UNILIDAR_SDK_DIR}/include/unitree_lidar_sdk.h")
  message(STATUS "✅ Found UniLidar SDK in: ${UNILIDAR_SDK_DIR}")
else()
  message(WARNING "⚠️ UniLidar SDK not found in ${UNILIDAR_SDK_DIR}. Please set UNILIDAR_SDK_DIR manually:")
  message(WARNING "   cmake -DUNILIDAR_SDK_DIR=/path/to/unilidar-sdk2 ..")
endif()

# ==========================================================
# 包含 SDK 头文件和库
# ==========================================================
include_directories(
  ${PCL_INCLUDE_DIRS}
  include
  ${UNILIDAR_SDK_DIR}/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${UNILIDAR_SDK_DIR}/lib
)

add_definitions(${PCL_DEFINITIONS})

# ==========================================================
# 节点构建
# ==========================================================
add_executable(unitree_lidar_ros2_node src/unitree_lidar_ros2_node.cpp)

target_link_libraries(unitree_lidar_ros2_node
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
  ${UNILIDAR_SDK_DIR}/lib/libunilidar_sdk2.a
)

ament_target_dependencies(
  unitree_lidar_ros2_node
  rclcpp std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
)

install(TARGETS
  unitree_lidar_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# install rviz file and launch file
install(FILES rviz/view.rviz launch/launch.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
  