#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  motioncore_record_bag.sh [profile] [output_root]

Profiles:
  motioncore_base   MotionCore base chain: raw/process/teleop/tf
  motioncore_wheel  MotionCore base chain focused on wheel kinematics
  motioncore_static IMU + mag static calibration set
  motioncore_lio    MotionCore base chain + UniLidar L2 + FAST-LIO
  full              Alias for motioncore_lio

Defaults:
  profile      motioncore_base
  output_root  $HOME/rosbags/motioncore
EOF
}

profile="${1:-motioncore_base}"
output_root="${2:-$HOME/rosbags/motioncore}"

case "$profile" in
  motioncore_base)
    topics=(
      /motioncore/encoder_raw
      /motioncore/imu/data_raw
      /motioncore/imu/mag_raw
      /motioncore/imu/data
      /motioncore/imu/mag
      /motioncore/wheel_odom
      /motioncore/diagnostics
      /cmd_vel
      /joy
      /tf
      /tf_static
    )
    ;;
  motioncore_wheel)
    topics=(
      /motioncore/encoder_raw
      /motioncore/wheel_odom
      /motioncore/diagnostics
      /cmd_vel
      /joy
      /tf
      /tf_static
    )
    ;;
  motioncore_static)
    topics=(
      /motioncore/imu/data_raw
      /motioncore/imu/mag_raw
      /motioncore/diagnostics
    )
    ;;
  motioncore_lio|full)
    topics=(
      /motioncore/encoder_raw
      /motioncore/imu/data_raw
      /motioncore/imu/mag_raw
      /motioncore/imu/data
      /motioncore/imu/mag
      /motioncore/wheel_odom
      /motioncore/diagnostics
      /cmd_vel
      /joy
      /tf
      /tf_static
      /unilidar/cloud
      /unilidar/imu
      /Odometry
      /path
    )
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown profile: $profile" >&2
    usage >&2
    exit 1
    ;;
esac

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not found. Source your ROS 2 environment first." >&2
  exit 1
fi

stamp="$(date +%F_%H%M%S)"
bag_dir="$output_root/${profile}-${stamp}"
while [[ -e "$bag_dir" ]]; do
  stamp="${stamp}_$RANDOM"
  bag_dir="$output_root/${profile}-${stamp}"
done
mkdir -p "$output_root"

write_manifest() {
  local status="$1"
  local manifest="$bag_dir/manifest.txt"
  if [[ ! -d "$bag_dir" ]]; then
    echo "Bag directory not found after recording: $bag_dir" >&2
    return 0
  fi

  cat > "$manifest" <<EOF
profile=$profile
created=$stamp
host=$(hostname)
cwd=$(pwd)
exit_status=$status
topics:
$(printf '  - %s\n' "${topics[@]}")

command:
$(printf 'ros2 bag record -o %q ' "$bag_dir"; printf '%q ' "${topics[@]}"; echo)
EOF
}

trap 'status=$?; write_manifest "$status"' EXIT

echo "Recording bag:"
echo "  profile: $profile"
echo "  output:  $bag_dir"
echo "  topics:"
printf '    %s\n' "${topics[@]}"
echo
echo "Manifest:"
echo "  $bag_dir/manifest.txt"
echo

ros2 bag record -o "$bag_dir" "${topics[@]}"
