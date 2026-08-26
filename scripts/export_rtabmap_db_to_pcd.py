#!/usr/bin/env python3
"""Export RTAB-Map laser scans stored in SQLite to binary PCD.

The global cloud applies:

    T_global_scan = T_node * T_scan_local

where T_node is Node.pose and T_scan_local is stored in Data.scan_info.
"""

from __future__ import annotations

import argparse
import math
import sqlite3
import struct
import sys
import zlib
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence


TRANSFORM_FLOATS = 12
SCAN_INFO_FLOATS = 19
WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = WORKSPACE_ROOT / "data" / "PointClouds"

# RTAB-Map LaserScan::Format channel counts.
FORMAT_CHANNELS = {
    1: 2,  # kXY
    2: 3,  # kXYI
    3: 5,  # kXYNormal
    4: 6,  # kXYINormal
    5: 3,  # kXYZ
    6: 4,  # kXYZI
    7: 4,  # kXYZRGB
    8: 6,  # kXYZNormal
    9: 7,  # kXYZINormal
    10: 7,  # kXYZRGBNormal
    11: 5,  # kXYZIT
    12: 6,  # kXYZIRT
}

FORMAT_NAMES = {
    1: "kXY",
    2: "kXYI",
    3: "kXYNormal",
    4: "kXYINormal",
    5: "kXYZ",
    6: "kXYZI",
    7: "kXYZRGB",
    8: "kXYZNormal",
    9: "kXYZINormal",
    10: "kXYZRGBNormal",
    11: "kXYZIT",
    12: "kXYZIRT",
}


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float
    intensity: float = 0.0
    normal_x: float = 0.0
    normal_y: float = 0.0
    normal_z: float = 0.0


def _matrix_multiply(lhs: Sequence[float], rhs: Sequence[float]) -> tuple[float, ...]:
    """Multiply two row-major 3x4 rigid transforms."""
    result = [0.0] * TRANSFORM_FLOATS
    for row in range(3):
        for col in range(3):
            result[row * 4 + col] = sum(
                lhs[row * 4 + index] * rhs[index * 4 + col]
                for index in range(3)
            )
        result[row * 4 + 3] = lhs[row * 4 + 3] + sum(
            lhs[row * 4 + index] * rhs[index * 4 + 3]
            for index in range(3)
        )
    return tuple(result)


def _transform_point(transform: Sequence[float], point: Point) -> Point:
    return Point(
        transform[0] * point.x
        + transform[1] * point.y
        + transform[2] * point.z
        + transform[3],
        transform[4] * point.x
        + transform[5] * point.y
        + transform[6] * point.z
        + transform[7],
        transform[8] * point.x
        + transform[9] * point.y
        + transform[10] * point.z
        + transform[11],
        point.intensity,
        transform[0] * point.normal_x
        + transform[1] * point.normal_y
        + transform[2] * point.normal_z,
        transform[4] * point.normal_x
        + transform[5] * point.normal_y
        + transform[6] * point.normal_z,
        transform[8] * point.normal_x
        + transform[9] * point.normal_y
        + transform[10] * point.normal_z,
    )


def _is_finite(point: Point) -> bool:
    return all(
        math.isfinite(value)
        for value in (
            point.x,
            point.y,
            point.z,
            point.intensity,
            point.normal_x,
            point.normal_y,
            point.normal_z,
        )
    )


def _scan_info(scan_info: bytes) -> tuple[int, tuple[float, ...]]:
    if len(scan_info) != SCAN_INFO_FLOATS * 4:
        raise ValueError(
            f"unsupported scan_info length: {len(scan_info)} bytes; "
            f"expected {SCAN_INFO_FLOATS * 4}"
        )
    values = struct.unpack("<19f", scan_info)
    scan_format = int(round(values[0]))
    if scan_format not in FORMAT_CHANNELS:
        raise ValueError(f"unsupported RTAB-Map scan format: {scan_format}")
    return scan_format, tuple(values[7:19])


def _decode_scan(scan_blob: bytes, scan_info: bytes) -> tuple[list[Point], int]:
    scan_format, _ = _scan_info(scan_info)
    channels = FORMAT_CHANNELS[scan_format]
    try:
        raw = zlib.decompress(scan_blob)
    except zlib.error as error:
        raise ValueError(f"cannot decompress scan BLOB: {error}") from error

    if len(raw) % (channels * 4) != 0:
        raise ValueError(
            f"decompressed scan size {len(raw)} is invalid for "
            f"{FORMAT_NAMES[scan_format]} ({channels} channels)"
        )

    values = struct.unpack(f"<{len(raw) // 4}f", raw)
    points: list[Point] = []
    for offset in range(0, len(values), channels):
        fields = values[offset : offset + channels]
        is_2d = scan_format in (1, 2, 3, 4)
        point = Point(
            fields[0],
            fields[1],
            0.0 if is_2d else fields[2],
        )

        if scan_format in (2, 4):
            point = Point(point.x, point.y, point.z, fields[2])
        elif scan_format in (6, 9, 11, 12):
            point = Point(point.x, point.y, point.z, fields[3])

        if scan_format in (3, 4, 8, 9, 10):
            normal_offset = 3 if scan_format in (9, 10) else 2
            point = Point(
                point.x,
                point.y,
                point.z,
                point.intensity,
                fields[normal_offset],
                fields[normal_offset + 1],
                fields[normal_offset + 2],
            )

        if _is_finite(point):
            points.append(point)
    return points, scan_format


def _decode_transform(blob: bytes | None, label: str, node_id: int) -> tuple[float, ...]:
    if blob is None or len(blob) != TRANSFORM_FLOATS * 4:
        actual = 0 if blob is None else len(blob)
        raise ValueError(
            f"node {node_id} has invalid {label}: {actual} bytes, "
            f"expected {TRANSFORM_FLOATS * 4}"
        )
    return struct.unpack("<12f", blob)


def _write_pcd(path: Path, points: Iterable[Point]) -> int:
    point_list = list(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity normal_x normal_y normal_z\n"
        "SIZE 4 4 4 4 4 4 4\n"
        "TYPE F F F F F F F\n"
        "COUNT 1 1 1 1 1 1 1\n"
        f"WIDTH {len(point_list)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(point_list)}\n"
        "DATA binary\n"
    ).encode("ascii")

    payload = bytearray()
    for point in point_list:
        payload.extend(
            struct.pack(
                "<7f",
                point.x,
                point.y,
                point.z,
                point.intensity,
                point.normal_x,
                point.normal_y,
                point.normal_z,
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + payload)
    return len(point_list)


def _append_timestamp(path: Path, timestamp: str) -> Path:
    return path.with_name(f"{path.stem}_{timestamp}{path.suffix}")


def export_database(
    database: Path,
    global_output: Path,
    local_output: Path | None,
) -> tuple[int, int, dict[int, int]]:
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    global_points: list[Point] = []
    local_points: list[Point] = []
    format_counts: dict[int, int] = {}
    node_count = 0
    try:
        rows = connection.execute(
            """
            SELECT n.id, n.pose, d.scan, d.scan_info
            FROM Node AS n
            INNER JOIN Data AS d ON d.id = n.id
            ORDER BY n.id
            """
        )
        for node_id, pose_blob, scan_blob, scan_info_blob in rows:
            node_count += 1
            if not scan_blob or not scan_info_blob:
                continue

            points, scan_format = _decode_scan(scan_blob, scan_info_blob)
            format_counts[scan_format] = (
                format_counts.get(scan_format, 0) + len(points)
            )
            local_points.extend(points)

            _, scan_transform = _scan_info(scan_info_blob)
            node_transform = _decode_transform(pose_blob, "pose", node_id)
            global_transform = _matrix_multiply(node_transform, scan_transform)
            global_points.extend(
                _transform_point(global_transform, point) for point in points
            )
    finally:
        connection.close()

    global_count = _write_pcd(global_output, global_points)
    if local_output is not None:
        _write_pcd(local_output, local_points)
    return node_count, global_count, format_counts


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Export RTAB-Map Data.scan records to binary PCD."
    )
    parser.add_argument("database", type=Path, help="RTAB-Map .db file")
    parser.add_argument(
        "--output",
        type=Path,
        help="global PCD output (default: data/PointClouds/<database>_global_<timestamp>.pcd)",
    )
    parser.add_argument(
        "--local-output",
        type=Path,
        help="raw sensor-frame PCD output (default: data/PointClouds/<database>_scan_sensor_<timestamp>.pcd)",
    )
    parser.add_argument(
        "--no-local-output",
        action="store_true",
        help="do not write the raw sensor-frame PCD",
    )
    parser.add_argument(
        "--no-timestamp",
        action="store_true",
        help="do not append YYYYMMDD_HHMMSS to output filenames",
    )
    args = parser.parse_args()

    database = args.database.expanduser().resolve()
    if not database.is_file():
        parser.error(f"database does not exist: {database}")
    output = (
        args.output.expanduser()
        if args.output
        else DEFAULT_OUTPUT_DIR / f"{database.stem}_global.pcd"
    )
    if args.no_local_output:
        local_output = None
    elif args.local_output:
        local_output = args.local_output.expanduser()
    else:
        local_output = output.with_name(
            f"{database.stem}_scan_sensor{output.suffix}"
        )

    if not args.no_timestamp:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = _append_timestamp(output, timestamp)
        if local_output is not None:
            local_output = _append_timestamp(local_output, timestamp)

    try:
        node_count, point_count, format_counts = export_database(
            database, output, local_output
        )
    except (OSError, sqlite3.Error, ValueError, struct.error) as error:
        print(f"export failed: {error}", file=sys.stderr)
        return 1

    formats = ", ".join(
        f"{FORMAT_NAMES.get(scan_format, scan_format)}={count}"
        for scan_format, count in sorted(format_counts.items())
    )
    print(f"nodes visited: {node_count}")
    print(f"global points: {point_count}")
    print(f"formats: {formats or 'none'}")
    print(f"global PCD: {output}")
    if local_output is not None:
        print(f"local PCD: {local_output}")
    return 0 if point_count else 2


if __name__ == "__main__":
    raise SystemExit(main())
