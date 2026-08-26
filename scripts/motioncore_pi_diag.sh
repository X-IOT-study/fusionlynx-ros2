#!/usr/bin/env bash
set -uo pipefail

OUT_DIR="${1:-$HOME/motioncore_pi_diag_$(date +%F_%H%M%S)}"
INTERVAL_SEC="${2:-5}"
mkdir -p "$OUT_DIR"

LOG_FILE="$OUT_DIR/monitor.log"
SNAPSHOT_FILE="$OUT_DIR/snapshot.txt"
KERNEL_FILE="$OUT_DIR/kernel.log"

have() { command -v "$1" >/dev/null 2>&1; }

ts() { date -Is; }

append() {
  printf '%s\n' "$*" | tee -a "$1" >/dev/null
}

snapshot_once() {
  {
    echo "=== $(ts) snapshot ==="
    echo "host=$(hostname)"
    echo "boot_id=$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || true)"
    echo "uptime=$(uptime -p 2>/dev/null || true)"
    echo "loadavg=$(cat /proc/loadavg 2>/dev/null || true)"
    echo
    echo "[mem]"
    free -h 2>/dev/null || true
    echo
    echo "[vcgencmd]"
    if have vcgencmd; then
      vcgencmd measure_temp || true
      vcgencmd get_throttled || true
      vcgencmd measure_volts core || true
      vcgencmd measure_clock arm || true
    else
      echo "vcgencmd: not found"
    fi
    echo
    echo "[usb]"
    lsusb 2>/dev/null || true
    echo
    echo "[kernel-current-tail]"
    dmesg -T 2>/dev/null | tail -n 80 || true
    echo
    echo "[kernel-current-journal]"
    if have journalctl; then
      journalctl -k -b --no-pager -n 80 2>/dev/null || true
    else
      echo "journalctl: not found"
    fi
    echo
    echo "[processes]"
    pgrep -af 'motioncore_driver_node|motioncore_bringup|imu_preprocess|wheel_kinematics|FAST_LIO|ros2 launch' 2>/dev/null || true
    echo
    echo "[ros2]"
    if have ros2; then
      ros2 node list 2>/dev/null || true
      ros2 topic list 2>/dev/null || true
      ros2 node info /motioncore_driver_node 2>/dev/null || true
    else
      echo "ros2: not found"
    fi
  } >> "$SNAPSHOT_FILE"
}

start_kernel_tap() {
  if have journalctl; then
    journalctl -kf --no-pager >> "$KERNEL_FILE" 2>/dev/null &
    echo $!
    return 0
  fi
  return 1
}

monitor_loop() {
  local i=0
  while true; do
    {
      echo "=== $(ts) sample #$i ==="
      echo "boot_id=$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || true)"
      echo "uptime=$(uptime -p 2>/dev/null || true)"
      echo "loadavg=$(cat /proc/loadavg 2>/dev/null || true)"
      if have vcgencmd; then
        vcgencmd measure_temp || true
        vcgencmd get_throttled || true
        vcgencmd measure_volts core || true
      fi
      pgrep -af 'motioncore_driver_node|motioncore_bringup|imu_preprocess|wheel_kinematics|FAST_LIO' 2>/dev/null || true
      if have ros2; then
        ros2 node list 2>/dev/null || true
      fi
      echo
    } >> "$LOG_FILE"
    i=$((i + 1))
    sleep "$INTERVAL_SEC"
  done
}

cleanup() {
  if [[ -n "${KERNEL_PID:-}" ]] && kill -0 "$KERNEL_PID" 2>/dev/null; then
    kill "$KERNEL_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

snapshot_once

KERNEL_PID=""
if KERNEL_PID="$(start_kernel_tap 2>/dev/null)"; then
  echo "kernel log tap started: pid=$KERNEL_PID" | tee -a "$LOG_FILE" >/dev/null
else
  echo "journalctl not available; skipping live kernel tap" | tee -a "$LOG_FILE" >/dev/null
fi

echo "writing logs to: $OUT_DIR"
echo "snapshot: $SNAPSHOT_FILE"
echo "monitor:  $LOG_FILE"
echo "kernel:   $KERNEL_FILE"
echo "interval: $INTERVAL_SEC sec"
echo
monitor_loop
