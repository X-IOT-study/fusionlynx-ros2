#!/usr/bin/env python3
"""Record a FusionLynx rosbag2 experiment and append analysis tables.

The recorder is intentionally an orchestration script. ROS 2 owns the bag
recording process; this script owns experiment metadata, signal-safe shutdown,
and post-recording table generation.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
ANALYZER = WORKSPACE_ROOT / "scripts" / "motioncore_analyze_bag.py"

RAW_TOPICS = [
    "/motioncore/encoder_raw",
    "/motioncore/imu/data_raw",
    "/motioncore/imu/mag_raw",
    "/motioncore/diagnostics",
]
WHEEL_TOPICS = [
    *RAW_TOPICS,
    "/motioncore/wheel_odom",
    "/cmd_vel",
    "/joy",
    "/tf",
    "/tf_static",
]
LIO_TOPICS = [
    "/unilidar/cloud",
    "/unilidar/imu",
    "/fast_lio/odom",
    "/fast_lio/cloud_registered_body",
    "/tf",
    "/tf_static",
]
FULL_TOPICS = [
    *WHEEL_TOPICS,
    "/motioncore/imu/data",
    "/motioncore/imu/mag",
    *LIO_TOPICS,
    "/odom",
]

PROFILE_CHAINS = {
    "A": "wheel",
    "B": "lio",
    "C": "full",
    "D": "full",
    "E": "full",
    "F": "full",
    "G": "full",
}

RUN_COLUMNS = [
    "run_id",
    "experiment_id",
    "fusion_profile",
    "chain",
    "scene",
    "repeat_index",
    "parameter_version",
    "date",
    "start_time",
    "end_time",
    "requested_duration_s",
    "recorded_duration_s",
    "operator",
    "robot_id",
    "location",
    "firmware_version",
    "sdk_version",
    "ros_distro",
    "rmw_implementation",
    "host_name",
    "pi_host",
    "pc_host",
    "time_sync_method",
    "config_file",
    "launch_command",
    "bag_path",
    "status",
    "notes",
]

TOPIC_COLUMNS = [
    "run_id",
    "bag_path",
    "topic",
    "msgtype",
    "frame_id",
    "child_frame_id",
    "header_coverage",
    "connections",
    "messages",
    "duration_s",
    "avg_hz",
    "min_gap_s",
    "max_gap_s",
    "mean_gap_s",
    "p95_gap_s",
    "std_gap_s",
    "bag_minus_header_mean_ms",
    "bag_minus_header_min_ms",
    "bag_minus_header_max_ms",
    "bag_minus_header_p95_ms",
    "header_backwards",
    "missing",
    "notes",
]

QUALITY_COLUMNS = [
    "run_id",
    "bag_path",
    "required_topics_present",
    "missing_topics",
    "tf_available",
    "odom_available",
    "wheel_odom_available",
    "fast_lio_odom_available",
    "cloud_available",
    "imu_available",
    "message_rate_ok",
    "timestamp_monotonic",
    "timestamp_offset_ok",
    "duration_ok",
    "qos_checked",
    "hardware_runtime_verified",
    "replay_verified",
    "quality_status",
    "quality_notes",
]


def _unique(items: Iterable[str]) -> list[str]:
    return list(dict.fromkeys(items))


def _sanitize(value: str, fallback: str = "unknown") -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "-", value.strip())
    cleaned = re.sub(r"-{2,}", "-", cleaned).strip("-._")
    return cleaned or fallback


def _local_now() -> dt.datetime:
    return dt.datetime.now().astimezone()


def _iso(value: dt.datetime) -> str:
    return value.isoformat(timespec="seconds")


def _chain_topics(chain: str) -> list[str]:
    if chain == "raw":
        return list(RAW_TOPICS)
    if chain == "wheel":
        return list(WHEEL_TOPICS)
    if chain == "lio":
        return list(LIO_TOPICS)
    return list(FULL_TOPICS)


def _selected_chain(profile: str, requested_chain: str | None) -> str:
    if requested_chain:
        return requested_chain
    return PROFILE_CHAINS.get(profile, "full")


def _required_topics(profile: str, chain: str) -> list[str]:
    if chain == "raw":
        return ["/motioncore/encoder_raw", "/motioncore/imu/data_raw"]
    if profile == "A" or chain == "wheel":
        return ["/motioncore/wheel_odom"]
    if profile == "B" or chain == "lio":
        return ["/fast_lio/odom", "/unilidar/cloud", "/unilidar/imu"]
    return [
        "/motioncore/wheel_odom",
        "/fast_lio/odom",
        "/fast_lio/cloud_registered_body",
        "/odom",
    ]


def _read_topics_file(path: Path | None) -> list[str]:
    if path is None:
        return []
    if not path.exists():
        raise SystemExit(f"topics file not found: {path}")
    topics = []
    for line in path.read_text(encoding="utf-8").splitlines():
        value = line.split("#", 1)[0].strip()
        if value:
            topics.append(value)
    return topics


def _run_command(command: list[str], *, timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=WORKSPACE_ROOT,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def _topic_snapshot() -> tuple[list[str], str]:
    try:
        result = _run_command(["ros2", "topic", "list", "-t"], timeout=10.0)
    except subprocess.TimeoutExpired as exc:
        return [], f"ros2 topic list timed out: {exc}"
    output = (result.stdout + result.stderr).strip()
    if result.returncode != 0:
        return [], output
    topics = []
    for line in result.stdout.splitlines():
        value = line.strip().split(" ", 1)[0]
        if value.startswith("/"):
            topics.append(value)
    return _unique(topics), output


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _append_csv(path: Path, columns: list[str], row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        if needs_header:
            writer.writeheader()
        writer.writerow({column: row.get(column, "") for column in columns})


def _format_command(command: list[str]) -> str:
    return " ".join(subprocess.list2cmdline([item]) for item in command)


def _run_analysis(
    bag_path: Path,
    topics: list[str],
    json_path: Path,
) -> tuple[dict[str, Any] | None, str]:
    if not ANALYZER.exists():
        return None, f"analyzer not found: {ANALYZER}"
    command = [
        sys.executable,
        str(ANALYZER),
        str(bag_path),
        "--topics",
        *topics,
        "--json-out",
        str(json_path),
    ]
    result = _run_command(command, timeout=300.0)
    output = (result.stdout + result.stderr).strip()
    if result.returncode != 0:
        return None, output or f"analysis exited with {result.returncode}"
    try:
        return json.loads(json_path.read_text(encoding="utf-8")), output
    except (OSError, json.JSONDecodeError) as exc:
        return None, f"analysis JSON could not be read: {exc}\n{output}"


def _topic_rows(
    run_id: str,
    bag_path: Path,
    analysis: dict[str, Any] | None,
    selected_topics: list[str],
) -> list[dict[str, Any]]:
    if analysis is None:
        return [
            {
                "run_id": run_id,
                "bag_path": str(bag_path),
                "topic": topic,
                "missing": "unknown",
                "notes": "analysis unavailable",
            }
            for topic in selected_topics
        ]

    present = {row["topic"]: row for row in analysis.get("topics", [])}
    missing = set(analysis.get("missing_topics", []))
    rows = []
    for topic in selected_topics:
        if topic in present:
            row = dict(present[topic])
            row.update({"run_id": run_id, "bag_path": str(bag_path), "missing": "false"})
        else:
            row = {
                "run_id": run_id,
                "bag_path": str(bag_path),
                "topic": topic,
                "missing": "true" if topic in missing else "unknown",
            }
        rows.append(row)
    return rows


def _quality_row(
    run_id: str,
    bag_path: Path,
    profile: str,
    chain: str,
    required_topics: list[str],
    analysis: dict[str, Any] | None,
    requested_duration_s: float,
    max_offset_ms: float,
    hardware_runtime_verified: bool,
    replay_verified: bool,
) -> dict[str, Any]:
    topic_rows = _topic_rows(run_id, bag_path, analysis, required_topics)
    present = {row["topic"]: row for row in topic_rows if row.get("missing") == "false"}
    missing = [topic for topic in required_topics if topic not in present]
    durations = [float(row.get("duration_s") or 0.0) for row in present.values()]
    recorded_duration = max(durations, default=0.0)

    def available(topic: str) -> str:
        return "true" if topic in present else "false"

    core_rate_ok = all(int(row.get("messages") or 0) > 0 for row in present.values())
    backwards_values = [row.get("header_backwards") for row in present.values()]
    timestamp_monotonic = (
        "unknown"
        if any(value in (None, "", "unknown") for value in backwards_values)
        else str(all(int(value) == 0 for value in backwards_values)).lower()
    )
    offset_values = []
    for row in present.values():
        for field in ("bag_minus_header_min_ms", "bag_minus_header_max_ms"):
            value = row.get(field)
            if value not in (None, ""):
                offset_values.append(abs(float(value)))
    timestamp_offset_ok = (
        "unknown"
        if not offset_values
        else str(max(offset_values) <= max_offset_ms).lower()
    )
    duration_ok = (
        "true"
        if requested_duration_s <= 0.0
        else str(recorded_duration >= requested_duration_s * 0.9).lower()
    )
    quality_notes = []
    if missing:
        quality_notes.append("missing required topics: " + ", ".join(missing))
    if timestamp_offset_ok == "false":
        quality_notes.append(f"timestamp offset exceeds {max_offset_ms:.1f} ms")
    if duration_ok == "false":
        quality_notes.append(
            f"recorded duration {recorded_duration:.3f}s is shorter than requested"
        )
    if analysis is None:
        quality_notes.append("bag analysis unavailable")

    checks = [not missing, core_rate_ok, timestamp_monotonic != "false",
              timestamp_offset_ok != "false", duration_ok != "false"]
    status = "PASS" if all(checks) else "FAIL"
    if analysis is None:
        status = "UNANALYZED"

    return {
        "run_id": run_id,
        "bag_path": str(bag_path),
        "required_topics_present": str(not missing).lower(),
        "missing_topics": ";".join(missing),
        "tf_available": available("/tf"),
        "odom_available": available("/odom"),
        "wheel_odom_available": available("/motioncore/wheel_odom"),
        "fast_lio_odom_available": available("/fast_lio/odom"),
        "cloud_available": str(
            "/fast_lio/cloud_registered_body" in present or "/unilidar/cloud" in present
        ).lower(),
        "imu_available": str(
            "/motioncore/imu/data" in present
            or "/motioncore/imu/data_raw" in present
            or "/unilidar/imu" in present
        ).lower(),
        "message_rate_ok": str(core_rate_ok).lower(),
        "timestamp_monotonic": timestamp_monotonic,
        "timestamp_offset_ok": timestamp_offset_ok,
        "duration_ok": duration_ok,
        "qos_checked": "ros2 bag recorder default; inspect metadata.yaml for profiles",
        "hardware_runtime_verified": str(hardware_runtime_verified).lower(),
        "replay_verified": str(replay_verified).lower(),
        "quality_status": status,
        "quality_notes": "; ".join(quality_notes),
        "_recorded_duration_s": recorded_duration,
        "_profile": profile,
        "_chain": chain,
    }


def _copy_template_tables(output_root: Path) -> None:
    """Create empty table files for a new output root."""
    for filename, columns in (
        ("experiment_runs.csv", RUN_COLUMNS),
        ("bag_topic_stats.csv", TOPIC_COLUMNS),
        ("bag_quality.csv", QUALITY_COLUMNS),
    ):
        path = output_root / "tables" / filename
        if not path.exists():
            _append_csv(path, columns, {})


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record a FusionLynx ROS 2 test bag and append CSV analysis tables."
    )
    parser.add_argument("--experiment-id", default="adhoc", help="Vault experiment ID, e.g. exp010.")
    parser.add_argument("--fusion-profile", "--profile", choices=list("ABCDEFG"), default="D")
    parser.add_argument("--chain", choices=["raw", "wheel", "lio", "full", "compare"])
    parser.add_argument(
        "--scene",
        choices=["lineturn", "mecanum_side", "spin", "slip", "lidar_drop", "imu_anomaly", "lift"],
        default="lineturn",
    )
    parser.add_argument("--repeat-index", type=int, default=1)
    parser.add_argument("--parameter-version", default="v1")
    parser.add_argument("--duration", type=float, default=0.0, help="Record duration in seconds; 0 waits for Ctrl-C.")
    parser.add_argument("--output-root", type=Path, default=Path.home() / "rosbags" / "fusionlynx")
    parser.add_argument("--topics-file", type=Path)
    parser.add_argument("--extra-topic", action="append", default=[])
    parser.add_argument("--operator", default="")
    parser.add_argument("--robot-id", default="motioncore-01")
    parser.add_argument("--location", default="")
    parser.add_argument("--firmware-version", default="")
    parser.add_argument("--sdk-version", default="")
    parser.add_argument("--pi-host", default="")
    parser.add_argument("--pc-host", default=socket.gethostname())
    parser.add_argument("--time-sync-method", default="host_receive_time")
    parser.add_argument("--config-file", default="")
    parser.add_argument("--launch-command", default="")
    parser.add_argument("--max-offset-ms", type=float, default=250.0)
    parser.add_argument("--storage", choices=["sqlite3", "mcap"], default="sqlite3")
    parser.add_argument("--compression-mode", choices=["none", "file", "message"], default="none")
    parser.add_argument("--compression-format", choices=["zstd", "fake_comp"], default="zstd")
    parser.add_argument("--strict-topic-check", action="store_true")
    parser.add_argument("--hardware-runtime-verified", action="store_true")
    parser.add_argument("--replay-verified", action="store_true")
    parser.add_argument("--no-analyze", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--notes", default="")
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.repeat_index < 1:
        raise SystemExit("--repeat-index must be >= 1")
    if args.duration < 0.0:
        raise SystemExit("--duration must be >= 0")
    if args.max_offset_ms < 0.0:
        raise SystemExit("--max-offset-ms must be >= 0")
    if args.compression_mode == "none" and args.compression_format != "zstd":
        raise SystemExit("--compression-format is only valid with compression enabled")


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    _validate_args(args)

    if shutil.which("ros2") is None and not args.dry_run:
        raise SystemExit("ros2 not found; source /opt/ros/humble/setup.bash first")

    chain = _selected_chain(args.fusion_profile, args.chain)
    base_topics = _chain_topics(chain)
    selected_topics = _unique(base_topics + _read_topics_file(args.topics_file) + args.extra_topic)
    required_topics = _required_topics(args.fusion_profile, chain)
    now = _local_now()
    date_label = now.strftime("%Y%m%d")
    run_id = f"{_sanitize(args.experiment_id)}-{args.fusion_profile}-{chain}-{date_label}-{args.repeat_index:02d}-{now.strftime('%H%M%S')}"
    bag_name = (
        f"{_sanitize(chain)}-{_sanitize(args.scene)}-"
        f"{_sanitize(args.parameter_version)}-{date_label}-{args.repeat_index:02d}.bag"
    )
    output_root = args.output_root.expanduser().resolve()
    bag_path = output_root / "bags" / bag_name
    manifest_path = output_root / "manifests" / f"{run_id}.json"
    analysis_path = bag_path / "topic_stats.json"
    command = ["ros2", "bag", "record", "-o", str(bag_path), "--storage", args.storage]
    if args.compression_mode != "none":
        command += ["--compression-mode", args.compression_mode, "--compression-format", args.compression_format]
    command += selected_topics

    if args.dry_run:
        print(f"run_id: {run_id}")
        print(f"bag_path: {bag_path}")
        print(f"fusion_profile: {args.fusion_profile}")
        print(f"chain: {chain}")
        print(f"required_topics: {', '.join(required_topics)}")
        print("topics:")
        for topic in selected_topics:
            print(f"  {topic}")
        print(f"command: {_format_command(command)}")
        print("tables:")
        print(f"  {output_root / 'tables' / 'experiment_runs.csv'}")
        print(f"  {output_root / 'tables' / 'bag_topic_stats.csv'}")
        print(f"  {output_root / 'tables' / 'bag_quality.csv'}")
        return 0

    if bag_path.exists():
        raise SystemExit(f"bag path already exists: {bag_path}")
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "bags").mkdir(parents=True, exist_ok=True)
    _copy_template_tables(output_root)

    available_topics, topic_snapshot = _topic_snapshot()
    missing_at_start = [topic for topic in selected_topics if topic not in available_topics]
    if missing_at_start:
        message = "topics not present at recording start: " + ", ".join(missing_at_start)
        if args.strict_topic_check:
            raise SystemExit(message)
        print(f"WARNING: {message}", file=sys.stderr)

    started = _local_now()
    manifest: dict[str, Any] = {
        "run_id": run_id,
        "experiment_id": args.experiment_id,
        "fusion_profile": args.fusion_profile,
        "chain": chain,
        "scene": args.scene,
        "repeat_index": args.repeat_index,
        "parameter_version": args.parameter_version,
        "bag_path": str(bag_path),
        "required_topics": required_topics,
        "selected_topics": selected_topics,
        "topics_present_at_start": available_topics,
        "topics_missing_at_start": missing_at_start,
        "topic_snapshot_output": topic_snapshot,
        "command": command,
        "command_string": _format_command(command),
        "host_name": socket.gethostname(),
        "platform": platform.platform(),
        "python": sys.version,
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
        "start_time": _iso(started),
        "status": "recording",
        "notes": args.notes,
    }
    _write_json(manifest_path, manifest)

    print(f"Recording run {run_id}")
    print(f"Bag: {bag_path}")
    print("Press Ctrl-C once to stop cleanly." if args.duration <= 0.0 else f"Duration: {args.duration:.1f}s")

    process = subprocess.Popen(
        command,
        cwd=WORKSPACE_ROOT,
        start_new_session=True,
        text=True,
    )
    interrupted = False

    def stop_recording(signum: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT if signum == signal.SIGINT else signal.SIGTERM)
            except ProcessLookupError:
                pass

    old_sigint = signal.signal(signal.SIGINT, stop_recording)
    old_sigterm = signal.signal(signal.SIGTERM, stop_recording)
    try:
        if args.duration > 0.0:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline and process.poll() is None:
                time.sleep(min(0.2, max(deadline - time.monotonic(), 0.0)))
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            try:
                return_code = process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                return_code = process.wait()
        else:
            return_code = process.wait()
    finally:
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)
    ended = _local_now()
    status = "recorded" if return_code == 0 else "record_failed"
    if interrupted and bag_path.exists():
        status = "recorded_interrupted"

    bag_info_output = ""
    if bag_path.exists():
        info_result = _run_command(["ros2", "bag", "info", str(bag_path)], timeout=30.0)
        bag_info_output = (info_result.stdout + info_result.stderr).strip()
        (bag_path / "bag_info.txt").write_text(bag_info_output + "\n", encoding="utf-8")

    analysis = None
    analysis_output = ""
    if status.startswith("recorded") and not args.no_analyze and bag_path.exists():
        analysis, analysis_output = _run_analysis(bag_path, selected_topics, analysis_path)
        if analysis is None:
            status = "recorded_unanalyzed"

    topic_rows = _topic_rows(run_id, bag_path, analysis, selected_topics)
    quality = _quality_row(
        run_id,
        bag_path,
        args.fusion_profile,
        chain,
        required_topics,
        analysis,
        args.duration,
        args.max_offset_ms,
        args.hardware_runtime_verified,
        args.replay_verified,
    )
    recorded_duration = quality.pop("_recorded_duration_s", 0.0)
    quality.pop("_profile", None)
    quality.pop("_chain", None)

    run_row = {
        "run_id": run_id,
        "experiment_id": args.experiment_id,
        "fusion_profile": args.fusion_profile,
        "chain": chain,
        "scene": args.scene,
        "repeat_index": args.repeat_index,
        "parameter_version": args.parameter_version,
        "date": date_label,
        "start_time": _iso(started),
        "end_time": _iso(ended),
        "requested_duration_s": args.duration,
        "recorded_duration_s": recorded_duration,
        "operator": args.operator,
        "robot_id": args.robot_id,
        "location": args.location,
        "firmware_version": args.firmware_version,
        "sdk_version": args.sdk_version,
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
        "host_name": socket.gethostname(),
        "pi_host": args.pi_host,
        "pc_host": args.pc_host,
        "time_sync_method": args.time_sync_method,
        "config_file": args.config_file,
        "launch_command": args.launch_command,
        "bag_path": str(bag_path),
        "status": status,
        "notes": args.notes,
    }
    _append_csv(output_root / "tables" / "experiment_runs.csv", RUN_COLUMNS, run_row)
    for row in topic_rows:
        _append_csv(output_root / "tables" / "bag_topic_stats.csv", TOPIC_COLUMNS, row)
    _append_csv(output_root / "tables" / "bag_quality.csv", QUALITY_COLUMNS, quality)

    manifest.update(
        {
            "end_time": _iso(ended),
            "status": status,
            "return_code": return_code,
            "interrupted": interrupted,
            "bag_info_output": bag_info_output,
            "analysis_output": analysis_output,
            "analysis_json": str(analysis_path) if analysis_path.exists() else "",
            "quality": quality,
            "tables": {
                "experiment_runs": str(output_root / "tables" / "experiment_runs.csv"),
                "bag_topic_stats": str(output_root / "tables" / "bag_topic_stats.csv"),
                "bag_quality": str(output_root / "tables" / "bag_quality.csv"),
            },
        }
    )
    _write_json(manifest_path, manifest)
    if bag_path.exists():
        _write_json(bag_path / "collection_manifest.json", manifest)

    print(f"status: {status}")
    print(f"quality: {quality['quality_status']}")
    print(f"tables: {output_root / 'tables'}")
    if analysis_output:
        print(analysis_output)
    return 0 if status.startswith("recorded") else return_code or 1


if __name__ == "__main__":
    raise SystemExit(main())
