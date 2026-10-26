#!/usr/bin/env python3
"""Compare multiple TUM trajectories with the configured evo environment."""

from __future__ import annotations

import argparse
import os
import re
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EVO_BIN = Path("/home/dream/academic/research-tools/venvs/evo/bin/evo_traj")
DEFAULT_OUTPUT_DIR = WORKSPACE_ROOT / "data" / "path" / "plots"


def _sanitize(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    return value.strip("._-") or "trajectory"


def _parse_trajectory(value: str) -> tuple[str, Path]:
    if "=" in value:
        label, path_text = value.split("=", 1)
        label = label.strip() or Path(path_text).stem
    else:
        path_text = value
        label = Path(path_text).stem
    path = Path(path_text).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"trajectory does not exist: {path}")
    return label, path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot multiple TUM trajectories with evo."
    )
    parser.add_argument(
        "trajectories",
        nargs="+",
        metavar="LABEL=TRAJECTORY.tum",
        help="One or more trajectories; a bare path uses its filename as label.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for PNG and evo table (default: data/path/plots)",
    )
    parser.add_argument("--prefix", help="Output filename prefix")
    parser.add_argument(
        "--plot-mode",
        choices=["xy", "xz", "yx", "yz", "zx", "zy", "xyz"],
        default="xy",
    )
    parser.add_argument(
        "--evo-bin",
        type=Path,
        default=DEFAULT_EVO_BIN,
        help="evo_traj executable",
    )
    parser.add_argument(
        "--evo-home",
        type=Path,
        default=Path("/tmp/fusionlynx_evo_home"),
        help="Writable HOME used by evo for settings",
    )
    parser.add_argument(
        "--matplotlib-dir",
        type=Path,
        default=Path("/tmp/fusionlynx_evo_matplotlib"),
        help="Writable Matplotlib cache directory",
    )
    parser.add_argument(
        "--backend",
        choices=["Agg", "qtagg", "QtAgg", "TkAgg"],
        help="Matplotlib/evo backend (default: qtagg, or Agg with --no-show)",
    )
    parser.add_argument(
        "--show",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Open an interactive plot window (default; use --no-show for headless mode)",
    )
    parser.add_argument("--ref", help="Reference label or trajectory path for evo alignment/sync")
    parser.add_argument("--align", action="store_true", help="Align trajectories to --ref")
    parser.add_argument("--sync", action="store_true", help="Synchronize timestamps to --ref")
    parser.add_argument("--correct-scale", action="store_true", help="Correct scale; requires --ref")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def _resolve_reference(
    reference: str,
    trajectories: list[tuple[str, Path]],
    aliases: dict[str, Path],
) -> Path:
    if reference in aliases:
        return aliases[reference]
    candidate = Path(reference).expanduser().resolve()
    for _label, path in trajectories:
        if path == candidate:
            return path
    raise ValueError(
        f"reference {reference!r} is not a trajectory label or path"
    )


def _prepare_env(args: argparse.Namespace, backend: str) -> dict[str, str]:
    evo_home = args.evo_home.expanduser().resolve()
    matplotlib_dir = args.matplotlib_dir.expanduser().resolve()
    evo_home.mkdir(parents=True, exist_ok=True)
    matplotlib_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["HOME"] = str(evo_home)
    env["MPLCONFIGDIR"] = str(matplotlib_dir)
    env["MPLBACKEND"] = backend
    env["PYTHONNOUSERSITE"] = "1"
    return env


def main() -> int:
    args = _build_parser().parse_args()
    try:
        trajectories = [_parse_trajectory(value) for value in args.trajectories]
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2

    if not args.evo_bin.is_file():
        print(f"evo_traj not found: {args.evo_bin}", file=sys.stderr)
        return 2
    evo_config = args.evo_bin.with_name("evo_config")
    if not evo_config.is_file():
        print(f"evo_config not found next to evo_traj: {evo_config}", file=sys.stderr)
        return 2
    if (args.align or args.sync or args.correct_scale) and not args.ref:
        print("--align, --sync and --correct-scale require --ref", file=sys.stderr)
        return 2

    labels = {}
    for index, (label, _path) in enumerate(trajectories, start=1):
        labels[label] = labels.get(label, 0) + 1
        if labels[label] > 1:
            trajectories[index - 1] = (f"{label}_{labels[label]}", _path)
    aliases = {label: path for label, path in trajectories}
    reference = None
    if args.ref:
        try:
            reference = _resolve_reference(args.ref, trajectories, aliases)
        except ValueError as error:
            print(error, file=sys.stderr)
            return 2

    backend = args.backend or ("qtagg" if args.show else "Agg")
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix or "trajectory_compare"
    plot_base = output_dir / f"{prefix}_{args.plot_mode}.png"
    plot_path = output_dir / f"{prefix}_{args.plot_mode}_trajectories.png"
    table_path = output_dir / f"{prefix}_{args.plot_mode}.txt"

    with tempfile.TemporaryDirectory(prefix="evo_inputs_", dir=str(output_dir)) as temp_dir:
        temp_root = Path(temp_dir)
        input_paths: list[Path] = []
        for index, (label, source) in enumerate(trajectories, start=1):
            link = temp_root / f"{index:02d}_{_sanitize(label)}.tum"
            link.symlink_to(source)
            input_paths.append(link)

        command = [
            str(args.evo_bin),
            "tum",
            *[str(path) for path in input_paths],
            "--plot_mode",
            args.plot_mode,
            "--save_plot",
            str(plot_base),
            "--save_table",
            str(table_path),
            "--show_full_names",
            "--no_warnings",
        ]
        if args.show:
            command.append("--plot")
        if reference is not None:
            command.extend(["--ref", str(next(path for path, (_label, source) in zip(input_paths, trajectories) if source == reference))])
        if args.align:
            command.append("--align")
        if args.sync:
            command.append("--sync")
        if args.correct_scale:
            command.append("--correct_scale")

        env = _prepare_env(args, backend)
        config_result = subprocess.run(
            [str(evo_config), "set", "plot_backend", backend],
            env=env,
            cwd=WORKSPACE_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        if config_result.returncode != 0:
            print(
                "failed to configure evo plot backend:\n"
                + (config_result.stdout + config_result.stderr),
                file=sys.stderr,
            )
            return config_result.returncode or 1

        print("command:", shlex.join(command))
        print("backend:", backend)
        print("plot base:", plot_base)
        print("trajectory plot:", plot_path)
        print("table:", table_path)
        if args.dry_run:
            return 0

        result = subprocess.run(
            command,
            env=env,
            cwd=WORKSPACE_ROOT,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            return result.returncode
        if not plot_path.is_file():
            print(f"evo completed without creating plot: {plot_path}", file=sys.stderr)
            return 1
        for suffix in ("_xyz.png", "_rpy.png", "_speeds.png"):
            extra_plot = output_dir / f"{prefix}_{args.plot_mode}{suffix}"
            if extra_plot.is_file():
                print("plot:", extra_plot)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
