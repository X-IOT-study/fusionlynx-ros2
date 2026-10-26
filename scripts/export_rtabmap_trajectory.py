#!/usr/bin/env python3
"""Export RTAB-Map Node poses from a SQLite database.

The RTAB-Map Node.pose and Node.ground_truth_pose fields are row-major
3x4 float32 rigid transforms. The output TUM format uses:

    timestamp x y z qx qy qz qw

The database is opened read-only. A ground-truth file is written only when at
least one non-zero, valid ground-truth pose is present.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sqlite3
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


TRANSFORM_BYTES = 12 * 4
WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = WORKSPACE_ROOT / "data" / "path"


@dataclass(frozen=True)
class PoseRecord:
    node_id: int
    map_id: int
    stamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    label: str


def _decode_transform(blob: bytes | None, field: str, node_id: int) -> tuple[float, ...]:
    if blob is None or len(blob) != TRANSFORM_BYTES:
        actual = 0 if blob is None else len(blob)
        raise ValueError(
            f"node {node_id} has invalid {field}: {actual} bytes; "
            f"expected {TRANSFORM_BYTES}"
        )
    values = struct.unpack("<12f", blob)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"node {node_id} has non-finite values in {field}")
    return values


def _quaternion_from_rotation(transform: Sequence[float]) -> tuple[float, ...]:
    """Convert a row-major 3x3 rotation matrix to normalized xyzw."""
    m00, m01, m02 = transform[0], transform[1], transform[2]
    m10, m11, m12 = transform[4], transform[5], transform[6]
    m20, m21, m22 = transform[8], transform[9], transform[10]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(max(1.0 + m00 - m11 - m22, 0.0)) * 2.0
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
        qw = (m21 - m12) / scale
    elif m11 > m22:
        scale = math.sqrt(max(1.0 + m11 - m00 - m22, 0.0)) * 2.0
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
        qw = (m02 - m20) / scale
    else:
        scale = math.sqrt(max(1.0 + m22 - m00 - m11, 0.0)) * 2.0
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
        qw = (m10 - m01) / scale

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12 or not math.isfinite(norm):
        raise ValueError("rotation matrix produced an invalid quaternion")
    return qx / norm, qy / norm, qz / norm, qw / norm


def _record_from_transform(
    node_id: int,
    map_id: int,
    stamp: float,
    transform: Sequence[float],
    label: str,
) -> PoseRecord:
    qx, qy, qz, qw = _quaternion_from_rotation(transform)
    return PoseRecord(
        node_id=node_id,
        map_id=map_id,
        stamp=float(stamp),
        x=float(transform[3]),
        y=float(transform[7]),
        z=float(transform[11]),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        label=label or "",
    )


def _read_records(
    database: Path,
) -> tuple[list[PoseRecord], list[PoseRecord], int, int, list[str]]:
    estimates: list[PoseRecord] = []
    ground_truth: list[PoseRecord] = []
    invalid_estimates = 0
    invalid_ground_truth = 0
    warnings: list[str] = []

    connection = sqlite3.connect(f"file:{database.as_posix()}?mode=ro", uri=True)
    try:
        rows = connection.execute(
            """
            SELECT id, map_id, stamp, pose, ground_truth_pose, label
            FROM Node
            WHERE pose IS NOT NULL
            ORDER BY stamp, id
            """
        )
        for node_id, map_id, stamp, pose_blob, ground_truth_blob, label in rows:
            try:
                pose = _decode_transform(pose_blob, "pose", int(node_id))
                if stamp is None or not math.isfinite(float(stamp)):
                    raise ValueError(f"node {node_id} has an invalid stamp")
                estimates.append(
                    _record_from_transform(
                        int(node_id),
                        int(map_id or 0),
                        float(stamp),
                        pose,
                        str(label or ""),
                    )
                )
            except (ValueError, struct.error) as error:
                invalid_estimates += 1
                warnings.append(str(error))

            if ground_truth_blob is None:
                continue
            try:
                values = _decode_transform(
                    ground_truth_blob, "ground_truth_pose", int(node_id)
                )
                if not any(abs(value) > 1e-12 for value in values):
                    invalid_ground_truth += 1
                    continue
                ground_truth.append(
                    _record_from_transform(
                        int(node_id),
                        int(map_id or 0),
                        float(stamp),
                        values,
                        str(label or ""),
                    )
                )
            except (ValueError, struct.error) as error:
                invalid_ground_truth += 1
                warnings.append(str(error))
    finally:
        connection.close()

    return estimates, ground_truth, invalid_estimates, invalid_ground_truth, warnings


def _path_length(records: Iterable[PoseRecord]) -> float:
    previous: PoseRecord | None = None
    length = 0.0
    for record in records:
        if previous is not None:
            length += math.dist(
                (previous.x, previous.y, previous.z),
                (record.x, record.y, record.z),
            )
        previous = record
    return length


def _write_tum(path: Path, records: Iterable[PoseRecord]) -> None:
    lines = []
    for record in records:
        lines.append(
            f"{record.stamp:.9f} {record.x:.9f} {record.y:.9f} {record.z:.9f} "
            f"{record.qx:.9f} {record.qy:.9f} {record.qz:.9f} {record.qw:.9f}"
        )
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def _write_csv(path: Path, records: Iterable[PoseRecord]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "timestamp",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
                "node_id",
                "map_id",
                "label",
            ]
        )
        for record in records:
            writer.writerow(
                [
                    f"{record.stamp:.9f}",
                    f"{record.x:.9f}",
                    f"{record.y:.9f}",
                    f"{record.z:.9f}",
                    f"{record.qx:.9f}",
                    f"{record.qy:.9f}",
                    f"{record.qz:.9f}",
                    f"{record.qw:.9f}",
                    record.node_id,
                    record.map_id,
                    record.label,
                ]
            )


def _write_report(path: Path, report: dict[str, object]) -> None:
    lines = [
        f"database: {report['database']}",
        f"estimate_poses: {report['estimate_poses']}",
        f"estimate_path_length_m: {float(report['estimate_path_length_m']):.6f}",
        f"ground_truth_poses: {report['ground_truth_poses']}",
        f"ground_truth_available: {report['ground_truth_available']}",
        f"invalid_estimates: {report['invalid_estimates']}",
        f"invalid_ground_truth: {report['invalid_ground_truth']}",
        f"estimate_tum: {report['estimate_tum']}",
        f"estimate_csv: {report['estimate_csv']}",
        f"ground_truth_tum: {report.get('ground_truth_tum', '')}",
        f"warnings: {len(report['warnings'])}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export RTAB-Map Node.pose records to TUM and CSV."
    )
    parser.add_argument("database", type=Path, help="RTAB-Map SQLite .db file")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Output directory (default: data/path)",
    )
    parser.add_argument(
        "--prefix",
        help="Output filename prefix (default: database filename without suffix)",
    )
    parser.add_argument("--no-csv", action="store_true", help="Do not write CSV")
    parser.add_argument("--no-report", action="store_true", help="Do not write reports")
    parser.add_argument("--force", action="store_true", help="Overwrite existing outputs")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    database = args.database.expanduser().resolve()
    if not database.is_file():
        print(f"database does not exist: {database}", file=sys.stderr)
        return 2

    output_dir = args.output_dir.expanduser().resolve()
    prefix = args.prefix or database.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    estimate_tum = output_dir / f"{prefix}_estimate.tum"
    estimate_csv = output_dir / f"{prefix}_estimate.csv"
    ground_truth_tum = output_dir / f"{prefix}_ground_truth.tum"
    report_json = output_dir / f"{prefix}_trajectory_report.json"
    report_text = output_dir / f"{prefix}_trajectory_report.txt"
    outputs = [estimate_tum]
    if not args.no_csv:
        outputs.append(estimate_csv)
    if not args.no_report:
        outputs.extend([report_json, report_text])

    existing = [path for path in outputs if path.exists()]
    if existing and not args.force:
        print(
            "output exists; use --force to overwrite: "
            + ", ".join(str(path) for path in existing),
            file=sys.stderr,
        )
        return 2

    try:
        (
            estimates,
            ground_truth,
            invalid_estimates,
            invalid_ground_truth,
            warnings,
        ) = _read_records(database)
    except (OSError, sqlite3.Error, ValueError) as error:
        print(f"export failed: {error}", file=sys.stderr)
        return 1

    if not estimates:
        print("database contains no valid Node.pose records", file=sys.stderr)
        return 1

    _write_tum(estimate_tum, estimates)
    if not args.no_csv:
        _write_csv(estimate_csv, estimates)

    ground_truth_output = ""
    if ground_truth:
        if ground_truth_tum.exists() and not args.force:
            print(f"output exists; use --force to overwrite: {ground_truth_tum}", file=sys.stderr)
            return 2
        _write_tum(ground_truth_tum, ground_truth)
        ground_truth_output = str(ground_truth_tum)

    report = {
        "database": str(database),
        "estimate_poses": len(estimates),
        "estimate_path_length_m": _path_length(estimates),
        "ground_truth_poses": len(ground_truth),
        "ground_truth_available": bool(ground_truth),
        "invalid_estimates": invalid_estimates,
        "invalid_ground_truth": invalid_ground_truth,
        "estimate_tum": str(estimate_tum),
        "estimate_csv": "" if args.no_csv else str(estimate_csv),
        "ground_truth_tum": ground_truth_output,
        "warnings": warnings,
    }
    if not args.no_report:
        report_json.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        _write_report(report_text, report)

    print(f"estimate poses: {len(estimates)}")
    print(f"estimate path length: {_path_length(estimates):.6f} m")
    print(f"estimate TUM: {estimate_tum}")
    if not args.no_csv:
        print(f"estimate CSV: {estimate_csv}")
    if ground_truth:
        print(f"ground truth TUM: {ground_truth_tum}")
    else:
        print("ground truth: unavailable (no non-zero valid ground_truth_pose records)")
    if not args.no_report:
        print(f"report: {report_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
