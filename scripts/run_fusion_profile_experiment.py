#!/usr/bin/env python3
"""Run bringup, record a rosbag, and export a trajectory for one profile."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shlex
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
COLLECTOR = WORKSPACE_ROOT / "scripts" / "collect_fusion_test_bag.py"
EXPORTER = WORKSPACE_ROOT / "scripts" / "export_rtabmap_trajectory.py"
DEFAULT_BAG_ROOT = Path.home() / "rosbags" / "fusionlynx"
DEFAULT_DB_DIR = Path("/tmp")
DEFAULT_PATH_DIR = WORKSPACE_ROOT / "data" / "path"


def _now() -> dt.datetime:
    return dt.datetime.now().astimezone()


def _safe_timestamp() -> str:
    return _now().strftime("%Y%m%d_%H%M%S")


def _terminate_group(process: subprocess.Popen[str] | None, signum: int) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        pass


def _wait_for_topic(topic: str, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(timeout_s, 0.0)
    while time.monotonic() < deadline:
        remaining = max(deadline - time.monotonic(), 0.1)
        try:
            result = subprocess.run(
                ["ros2", "topic", "echo", "--no-daemon", topic, "--once"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=min(10.0, remaining),
                check=False,
            )
        except subprocess.TimeoutExpired:
            continue
        if result.returncode == 0:
            return True
        time.sleep(0.5)
    return False


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Launch FusionLynx, record a rosbag, and export RTAB-Map poses."
    )
    parser.add_argument(
        "--profile",
        choices=list("ABCDEFG"),
        default="E",
        help="Fusion profile to run (default: E)",
    )
    parser.add_argument(
        "--all-profiles",
        action="store_true",
        help="Run profiles A through G sequentially",
    )
    parser.add_argument("--experiment-id", default="fusion-profile")
    parser.add_argument("--scene", default="lineturn")
    parser.add_argument("--repeat-index", type=int, default=1)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--start-delay", type=float, default=5.0)
    parser.add_argument("--topic-timeout", type=float, default=60.0)
    parser.add_argument("--bag-output-root", type=Path, default=DEFAULT_BAG_ROOT)
    parser.add_argument("--database-dir", type=Path, default=DEFAULT_DB_DIR)
    parser.add_argument("--path-output-dir", type=Path, default=DEFAULT_PATH_DIR)
    parser.add_argument("--parameter-version", default="v1")
    parser.add_argument("--config-file", default="")
    parser.add_argument("--operator", default="")
    parser.add_argument("--robot-id", default="motioncore-01")
    parser.add_argument("--location", default="")
    parser.add_argument("--pi-host", default="")
    parser.add_argument("--time-sync-method", default="host_receive_time")
    parser.add_argument("--use-rviz", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--run-mapping", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--hardware-runtime-verified", action="store_true")
    parser.add_argument("--replay-verified", action="store_true")
    parser.add_argument("--stop-on-error", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def _launch_command(
    profile: str,
    database_path: Path,
    use_rviz: bool,
    run_mapping: bool,
) -> list[str]:
    return [
        "ros2",
        "launch",
        "fusionlynx_bringup",
        "pc.launch.py",
        f"fusion_profile:={profile}",
        f"run_mapping:={'true' if run_mapping else 'false'}",
        f"use_rviz:={'true' if use_rviz else 'false'}",
        f"mapping_database_path:={database_path}",
    ]


def _collector_command(
    args: argparse.Namespace,
    profile: str,
    launch_command: list[str],
    bag_output_root: Path,
) -> list[str]:
    command = [
        sys.executable,
        str(COLLECTOR),
        "--experiment-id",
        args.experiment_id,
        "--fusion-profile",
        profile,
        "--scene",
        args.scene,
        "--repeat-index",
        str(args.repeat_index),
        "--parameter-version",
        args.parameter_version,
        "--duration",
        str(args.duration),
        "--output-root",
        str(bag_output_root.expanduser().resolve()),
        "--launch-command",
        shlex.join(launch_command),
        "--time-sync-method",
        args.time_sync_method,
        "--robot-id",
        args.robot_id,
        "--pc-host",
        socket.gethostname(),
    ]
    for option, value in (
        ("--config-file", args.config_file),
        ("--operator", args.operator),
        ("--location", args.location),
        ("--pi-host", args.pi_host),
    ):
        if value:
            command.extend([option, value])
    if args.hardware_runtime_verified:
        command.append("--hardware-runtime-verified")
    if args.replay_verified:
        command.append("--replay-verified")
    return command


def _run_one(args: argparse.Namespace, profile: str) -> int:
    timestamp = _safe_timestamp()
    run_name = f"{args.experiment_id}_{profile}_{timestamp}"
    run_root = args.database_dir.expanduser().resolve() / "fusionlynx_experiments"
    run_dir = run_root / run_name
    database_path = run_dir / f"fusionlynx_{timestamp}_{profile}.db"
    path_prefix = run_name
    launch_command = _launch_command(
        profile,
        database_path,
        args.use_rviz,
        args.run_mapping,
    )
    collector_command = _collector_command(
        args,
        profile,
        launch_command,
        args.bag_output_root / run_name,
    )
    if args.dry_run:
        print(f"[{profile}] run_name: {run_name}")
        print(f"[{profile}] database: {database_path}")
        print(f"[{profile}] launch: {shlex.join(launch_command)}")
        print(f"[{profile}] collector: {shlex.join(collector_command)}")
        if args.run_mapping:
            export_command = [
                sys.executable,
                str(EXPORTER),
                str(database_path),
                "--output-dir",
                str(args.path_output_dir.expanduser().resolve()),
                "--prefix",
                path_prefix,
                "--force",
            ]
            print(
                f"[{profile}] export: {shlex.join(export_command)}"
            )
        return 0

    run_dir.mkdir(parents=True, exist_ok=True)
    launch_log = run_dir / "bringup.log"
    collector_log = run_dir / "collector.log"
    metadata_path = run_dir / "run.json"
    metadata: dict[str, Any] = {
        "profile": profile,
        "run_name": run_name,
        "database_path": str(database_path),
        "path_output_dir": str(args.path_output_dir.expanduser().resolve()),
        "launch_command": launch_command,
        "collector_command": collector_command,
        "start_time": _now().isoformat(timespec="seconds"),
        "host": socket.gethostname(),
        "status": "starting",
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    launch_stream = launch_log.open("w", encoding="utf-8")
    collector_stream = None
    launch_process: subprocess.Popen[str] | None = None
    collector_process: subprocess.Popen[str] | None = None
    return_code = 1
    try:
        environment = os.environ.copy()
        environment["ROS_LOG_DIR"] = str(run_dir / "ros_logs")
        launch_process = subprocess.Popen(
            launch_command,
            cwd=WORKSPACE_ROOT,
            env=environment,
            stdout=launch_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        metadata["launch_pid"] = launch_process.pid
        metadata["status"] = "bringup_running"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

        deadline = time.monotonic() + max(args.start_delay, 0.0)
        while time.monotonic() < deadline:
            if launch_process.poll() is not None:
                raise RuntimeError(
                    f"bringup exited during startup with code {launch_process.returncode}"
                )
            time.sleep(min(0.25, max(deadline - time.monotonic(), 0.0)))

        required_topics = ["/odom"]
        if profile in "CDEFG":
            required_topics += ["/motioncore/wheel_odom", "/fast_lio/odom"]
        for topic in required_topics:
            print(f"[{profile}] waiting for {topic}", flush=True)
            if not _wait_for_topic(topic, args.topic_timeout):
                raise RuntimeError(
                    f"topic not ready within {args.topic_timeout:.1f}s: {topic}"
                )

        collector_stream = collector_log.open("w", encoding="utf-8")
        collector_process = subprocess.Popen(
            collector_command,
            cwd=WORKSPACE_ROOT,
            env=environment,
            stdout=collector_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        metadata["collector_pid"] = collector_process.pid
        metadata["status"] = "recording"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        return_code = collector_process.wait()
        metadata["collector_return_code"] = return_code
        metadata["status"] = "recorded" if return_code == 0 else "record_failed"
    except KeyboardInterrupt:
        metadata["status"] = "interrupted"
        return_code = 130
    except (OSError, RuntimeError) as error:
        metadata["status"] = "start_failed"
        metadata["error"] = str(error)
        print(f"[{profile}] {error}", file=sys.stderr)
    finally:
        _terminate_group(collector_process, signal.SIGINT)
        if collector_process is not None and collector_process.poll() is None:
            try:
                collector_process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                _terminate_group(collector_process, signal.SIGTERM)
        _terminate_group(launch_process, signal.SIGINT)
        if launch_process is not None and launch_process.poll() is None:
            try:
                launch_process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                _terminate_group(launch_process, signal.SIGTERM)
        if collector_stream is not None:
            collector_stream.close()
        launch_stream.close()

    export_return_code = None
    if database_path.is_file() and args.run_mapping:
        export_command = [
            sys.executable,
            str(EXPORTER),
            str(database_path),
            "--output-dir",
            str(args.path_output_dir.expanduser().resolve()),
            "--prefix",
            path_prefix,
            "--force",
        ]
        export_result = subprocess.run(
            export_command,
            cwd=WORKSPACE_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        (run_dir / "trajectory_export.log").write_text(
            export_result.stdout + export_result.stderr,
            encoding="utf-8",
        )
        export_return_code = export_result.returncode
        metadata["export_command"] = export_command
        metadata["export_return_code"] = export_return_code
    else:
        metadata["export_skipped"] = (
            "database not created" if not database_path.is_file() else "mapping disabled"
        )

    metadata["end_time"] = _now().isoformat(timespec="seconds")
    metadata["status"] = (
        "complete"
        if return_code == 0 and (export_return_code in (None, 0))
        else metadata.get("status", "failed")
    )
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"[{profile}] status: {metadata['status']}")
    print(f"[{profile}] database: {database_path}")
    print(f"[{profile}] logs: {run_dir}")
    if export_return_code == 0:
        print(f"[{profile}] trajectory: {args.path_output_dir.expanduser().resolve() / (path_prefix + '_estimate.tum')}")
    return return_code or (export_return_code or 0)


def main() -> int:
    args = _build_parser().parse_args()
    if args.duration < 0.0:
        print("--duration must be >= 0", file=sys.stderr)
        return 2
    if args.start_delay < 0.0:
        print("--start-delay must be >= 0", file=sys.stderr)
        return 2
    if args.topic_timeout < 0.0:
        print("--topic-timeout must be >= 0", file=sys.stderr)
        return 2
    if not COLLECTOR.is_file() or not EXPORTER.is_file():
        print("required workspace scripts are missing", file=sys.stderr)
        return 2
    profiles = list("ABCDEFG") if args.all_profiles else [args.profile]
    result = 0
    for profile in profiles:
        code = _run_one(args, profile)
        if code != 0:
            result = code
            if args.stop_on_error:
                break
    return result


if __name__ == "__main__":
    raise SystemExit(main())
