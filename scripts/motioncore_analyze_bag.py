#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path
from statistics import mean, pstdev
from typing import Optional

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
ENCODER_MSG = WORKSPACE_ROOT / "src/motioncore/motioncore_msgs/msg/EncoderRaw.msg"


@dataclass
class TopicStats:
    topic: str
    msgtype: str
    frame_id: str = ""
    child_frame_id: str = ""
    header_coverage: float = 0.0
    connections: int = 0
    messages: int = 0
    duration_s: float = 0.0
    avg_hz: float = 0.0
    min_gap_s: float = 0.0
    max_gap_s: float = 0.0
    mean_gap_s: float = 0.0
    p95_gap_s: float = 0.0
    std_gap_s: float = 0.0
    bag_minus_header_mean_ms: Optional[float] = None
    bag_minus_header_min_ms: Optional[float] = None
    bag_minus_header_max_ms: Optional[float] = None
    bag_minus_header_p95_ms: Optional[float] = None
    header_backwards: Optional[int] = None
    notes: str = ""


def _ns_to_s(value: int) -> float:
    return value / 1e9


def _stamp_to_ns(stamp) -> Optional[int]:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nsec = getattr(stamp, "nanosec", None)
    if sec is None or nsec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nsec)


def _extract_header_stamp_ns(msg) -> Optional[int]:
    header = getattr(msg, "header", None)
    stamp = _stamp_to_ns(getattr(header, "stamp", None))
    if stamp is not None:
        return stamp

    transforms = getattr(msg, "transforms", None)
    if transforms:
        header = getattr(transforms[0], "header", None)
        return _stamp_to_ns(getattr(header, "stamp", None))

    return None


def _extract_frame_ids(msg) -> tuple[str, str]:
    frame_ids: list[str] = []
    child_frame_ids: list[str] = []

    header = getattr(msg, "header", None)
    frame_id = getattr(header, "frame_id", "")
    if frame_id:
        frame_ids.append(frame_id)

    child_frame_id = getattr(msg, "child_frame_id", "")
    if child_frame_id:
        child_frame_ids.append(child_frame_id)

    transforms = getattr(msg, "transforms", None)
    if transforms:
        for transform in transforms:
            transform_header = getattr(transform, "header", None)
            transform_frame_id = getattr(transform_header, "frame_id", "")
            if transform_frame_id:
                frame_ids.append(transform_frame_id)
            transform_child_frame_id = getattr(transform, "child_frame_id", "")
            if transform_child_frame_id:
                child_frame_ids.append(transform_child_frame_id)

    return ",".join(dict.fromkeys(frame_ids)), ",".join(dict.fromkeys(child_frame_ids))


def _percentile(sorted_values: list[float], q: float) -> float:
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = (len(sorted_values) - 1) * q
    lower = math.floor(pos)
    upper = math.ceil(pos)
    if lower == upper:
        return sorted_values[int(pos)]
    weight = pos - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def _make_reader(bag_path: Path) -> AnyReader:
    store = get_typestore(Stores.ROS2_HUMBLE)
    if ENCODER_MSG.exists():
        store.register(get_types_from_msg(ENCODER_MSG.read_text(), "motioncore_msgs/msg/EncoderRaw"))
    return AnyReader([bag_path], default_typestore=store)


def analyze_topic(reader: AnyReader, topic: str, connections) -> TopicStats:
    bag_times: list[int] = []
    header_times: list[int] = []
    offsets_ms: list[float] = []
    frame_ids: list[str] = []
    child_frame_ids: list[str] = []
    header_backwards = 0
    last_header: Optional[int] = None

    for _, timestamp, rawdata in reader.messages(connections):
        bag_times.append(timestamp)
        try:
            msg = reader.deserialize(rawdata, connections[0].msgtype)
        except Exception:
            continue
        header_stamp = _extract_header_stamp_ns(msg)
        frame_id, child_frame_id = _extract_frame_ids(msg)
        if frame_id:
            frame_ids.append(frame_id)
        if child_frame_id:
            child_frame_ids.append(child_frame_id)
        if header_stamp is None:
            continue
        header_times.append(header_stamp)
        offsets_ms.append(_ns_to_s(timestamp - header_stamp) * 1000.0)
        if last_header is not None and header_stamp < last_header:
            header_backwards += 1
        last_header = header_stamp

    stats = TopicStats(
        topic=topic,
        msgtype=connections[0].msgtype,
        frame_id=",".join(dict.fromkeys(frame_ids)),
        child_frame_id=",".join(dict.fromkeys(child_frame_ids)),
        header_coverage=len(header_times) / len(bag_times) if bag_times else 0.0,
        connections=len(connections),
        messages=len(bag_times),
    )
    if len(bag_times) >= 2:
        gaps = [_ns_to_s(b - a) for a, b in zip(bag_times, bag_times[1:])]
        gaps_sorted = sorted(gaps)
        stats.duration_s = _ns_to_s(bag_times[-1] - bag_times[0])
        stats.avg_hz = (len(bag_times) - 1) / stats.duration_s if stats.duration_s > 0 else 0.0
        stats.min_gap_s = min(gaps)
        stats.max_gap_s = max(gaps)
        stats.mean_gap_s = mean(gaps)
        stats.p95_gap_s = _percentile(gaps_sorted, 0.95)
        stats.std_gap_s = pstdev(gaps) if len(gaps) > 1 else 0.0
    elif len(bag_times) == 1:
        stats.duration_s = 0.0
        stats.avg_hz = 0.0

    if offsets_ms:
        stats.bag_minus_header_mean_ms = mean(offsets_ms)
        stats.bag_minus_header_min_ms = min(offsets_ms)
        stats.bag_minus_header_max_ms = max(offsets_ms)
        stats.bag_minus_header_p95_ms = _percentile(sorted(offsets_ms), 0.95)

    if header_times and len(header_times) == len(bag_times):
        if len(connections) == 1:
            stats.header_backwards = header_backwards
        else:
            stats.header_backwards = None
            stats.notes = "multi-publisher topic"
    elif header_times:
        stats.header_backwards = None
        stats.notes = "partial header coverage"

    return stats


def format_row(stats: TopicStats) -> str:
    offset_mean = "-" if stats.bag_minus_header_mean_ms is None else f"{stats.bag_minus_header_mean_ms:8.2f}"
    offset_min = "-" if stats.bag_minus_header_min_ms is None else f"{stats.bag_minus_header_min_ms:8.2f}"
    offset_max = "-" if stats.bag_minus_header_max_ms is None else f"{stats.bag_minus_header_max_ms:8.2f}"
    offset_p95 = "-" if stats.bag_minus_header_p95_ms is None else f"{stats.bag_minus_header_p95_ms:8.2f}"
    backwards = "-" if stats.header_backwards is None else f"{stats.header_backwards:8d}"
    note = stats.notes or "-"
    return (
        f"{stats.topic:28} "
        f"{stats.connections:3d} "
        f"{stats.messages:8d} "
        f"{stats.avg_hz:8.2f} "
        f"{stats.min_gap_s:8.4f} "
        f"{stats.max_gap_s:8.4f} "
        f"{stats.p95_gap_s:8.4f} "
        f"{offset_mean:>8} "
        f"{offset_min:>8} "
        f"{offset_max:>8} "
        f"{offset_p95:>8} "
        f"{backwards:>8} "
        f"{note}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze MotionCore rosbag2 recordings.")
    parser.add_argument("bag", type=Path, help="Path to rosbag2 directory")
    parser.add_argument(
        "--topics",
        nargs="*",
        default=[
            "/motioncore/encoder_raw",
            "/motioncore/imu/data_raw",
            "/motioncore/imu/mag_raw",
            "/motioncore/imu/data",
            "/motioncore/imu/mag",
            "/motioncore/wheel_odom",
            "/motioncore/diagnostics",
            "/cmd_vel",
            "/joy",
            "/tf",
            "/tf_static",
            "/unilidar/cloud",
            "/unilidar/imu",
            "/fast_lio/odom",
            "/fast_lio/cloud_registered_body",
            "/odom",
            "/Odometry",
            "/path",
        ],
        help="Topics to analyze. Default covers the current MotionCore + FAST-LIO chain.",
    )
    parser.add_argument("--json-out", type=Path, default=None, help="Optional JSON summary output path")
    args = parser.parse_args()

    if not args.bag.exists():
        raise SystemExit(f"Bag path not found: {args.bag}")

    results: list[TopicStats] = []
    with _make_reader(args.bag) as reader:
        connections_by_topic = defaultdict(list)
        for conn in reader.connections:
            connections_by_topic[conn.topic].append(conn)
        present_topics = [topic for topic in args.topics if topic in connections_by_topic]
        missing_topics = [topic for topic in args.topics if topic not in connections_by_topic]

        for topic in present_topics:
            results.append(analyze_topic(reader, topic, connections_by_topic[topic]))

    print(f"bag: {args.bag}")
    print("topic                        conns     msgs     avgHz   minGap   maxGap   p95Gap  offMean   offMin   offMax   offP95  backwd note")
    print("-" * 132)
    for stats in results:
        print(format_row(stats))

    if missing_topics:
        print()
        print("missing topics:")
        for topic in missing_topics:
            print(f"  - {topic}")

    summary = {
        "bag": str(args.bag),
        "topics": [asdict(item) for item in results],
        "missing_topics": missing_topics,
    }
    if args.json_out is not None:
        args.json_out.write_text(json.dumps(summary, indent=2, ensure_ascii=False))
        print()
        print(f"json written to: {args.json_out}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
