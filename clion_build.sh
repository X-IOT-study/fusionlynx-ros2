#!/usr/bin/env bash

set -euo pipefail

export CCACHE_DISABLE=1

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="${script_dir}"
package_dir="${workspace_root}/src/livox_ros_driver2"

if [[ ! -d "${package_dir}" ]]; then
    echo "Package directory not found: ${package_dir}" >&2
    exit 1
fi

ros_arg="${1:-${VERSION_HUMBLE}}"
ros_version=""
ros_humble=""

case "${ros_arg}" in
    "${VERSION_ROS1}")
        ros_version="${VERSION_ROS1}"
        ;;
    "${VERSION_ROS2}")
        ros_version="${VERSION_ROS2}"
        ;;
    "${VERSION_HUMBLE}")
        ros_version="${VERSION_ROS2}"
        ros_humble="${VERSION_HUMBLE}"
        ;;
    *)
        echo "Usage: $0 [ROS1|ROS2|humble]" >&2
        exit 1
        ;;
esac

echo "Workspace Path: ${workspace_root}"
echo "Package Path: ${package_dir}"
echo "ROS version is: ${ros_version}${ros_humble:+ (${ros_humble})}"

backup_dir="$(mktemp -d)"
launch_dir="${package_dir}/launch"
restore_launch=0

cleanup() {
    if [[ -f "${backup_dir}/package.xml" ]]; then
        cp -f "${backup_dir}/package.xml" "${package_dir}/package.xml"
    fi

    if [[ "${restore_launch}" -eq 1 ]]; then
        rm -rf "${launch_dir}"
        mv "${backup_dir}/launch" "${launch_dir}"
    else
        rm -rf "${launch_dir}"
    fi

    rm -rf "${backup_dir}"
}
trap cleanup EXIT

cp -f "${package_dir}/package.xml" "${backup_dir}/package.xml"

if [[ -d "${launch_dir}" ]]; then
    mv "${launch_dir}" "${backup_dir}/launch"
    restore_launch=1
fi

if [[ -f "${workspace_root}/src/CMakeLists.txt" ]]; then
    rm -f "${workspace_root}/src/CMakeLists.txt"
fi

rm -rf "${workspace_root}/build" "${workspace_root}/install" "${workspace_root}/log"

if [[ "${ros_version}" == "${VERSION_ROS1}" ]]; then
    cp -f "${package_dir}/package_ROS1.xml" "${package_dir}/package.xml"
else
    cp -f "${package_dir}/package_ROS2.xml" "${package_dir}/package.xml"
    cp -rf "${package_dir}/launch_ROS2/" "${launch_dir}"
fi

cd "${workspace_root}"

if [[ "${ros_version}" == "${VERSION_ROS1}" ]]; then
    rm -rf devel/
    catkin_make -DROS_EDITION="${VERSION_ROS1}"
else
    build_args=(
        colcon build
        --cmake-args
        -DROS_EDITION="${VERSION_ROS2}"
    )
    if [[ -n "${ros_humble}" ]]; then
        build_args+=(-DHUMBLE_ROS="${ros_humble}")
    fi
    build_args+=(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja)
    "${build_args[@]}"
fi
