#!/usr/bin/env python3
"""Cross-repo helpers for hex_maze_interface development from ClusterController."""

from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def _find_hex_repo() -> Path:
    configured = os.environ.get("HEX_MAZE_INTERFACE_REPO")
    candidates: list[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser())

    repo_root = Path(__file__).resolve().parents[1]
    workspace_root = repo_root.parent.parent
    candidates.extend(
        [
            repo_root.parent / "hex_maze_interface_python",
            repo_root.parent / "python" / "hex_maze_interface_python",
            workspace_root / "hex_maze_interface_python",
            workspace_root / "python" / "hex_maze_interface_python",
        ]
    )

    for candidate in candidates:
        if (candidate / "pyproject.toml").exists() and (candidate / "hex_maze_interface").is_dir():
            return candidate.resolve()

    raise SystemExit(
        "Unable to locate hex_maze_interface_python. "
        "Expected a sibling checkout or set HEX_MAZE_INTERFACE_REPO."
    )


def _run(cmd: list[str], *, cwd: Path) -> int:
    completed = subprocess.run(cmd, cwd=str(cwd), check=False)
    return completed.returncode


def _run_or_exit(cmd: list[str], *, cwd: Path) -> None:
    raise SystemExit(_run(cmd, cwd=cwd))


def _python_repo_cmd(repo: Path, args: argparse.Namespace) -> None:
    command_map = {
        "python": [sys.executable],
        "help": [sys.executable, "-m", "hex_maze_interface", "--help"],
        "discover": [sys.executable, "-m", "hex_maze_interface", "discover-clusters", "--json"],
        "verify": [sys.executable, "-m", "hex_maze_interface", "verify-all-clusters", "--json"],
        "format": ["ruff", "format", "."],
        "lint": ["ruff", "check", "."],
        "test": [sys.executable, "-m", "pytest"],
        "build": [sys.executable, "-m", "build"],
    }
    if args.command == "check-dist":
        dist_paths = sorted(glob.glob(str(repo / "dist" / "*")))
        if not dist_paths:
            raise SystemExit("No files found in dist/. Run the build task first.")
        _run_or_exit([sys.executable, "-m", "twine", "check", *dist_paths], cwd=repo)
        return
    base = command_map[args.command]
    extra = list(args.extra_args)
    if args.command == "python":
        extra = extra or ["--version"]
    _run_or_exit(base + extra, cwd=repo)


def _check(repo: Path) -> None:
    for cmd in (["ruff", "check", "."], [sys.executable, "-m", "pytest"]):
        code = _run(cmd, cwd=repo)
        if code:
            raise SystemExit(code)


def _release_check(repo: Path) -> None:
    steps = (
        ["ruff", "check", "."],
        [sys.executable, "-m", "pytest"],
        [sys.executable, "-m", "build"],
    )
    for cmd in steps:
        code = _run(cmd, cwd=repo)
        if code:
            raise SystemExit(code)
    dist_paths = sorted(glob.glob(str(repo / "dist" / "*")))
    if not dist_paths:
        raise SystemExit("No files found in dist/ after build.")
    raise SystemExit(_run([sys.executable, "-m", "twine", "check", *dist_paths], cwd=repo))


def _hardware_script(repo: Path, script_name: str, args: argparse.Namespace) -> None:
    extra_args = list(args.extra_args)
    if extra_args[:1] == ["--"]:
        extra_args = extra_args[1:]
    _run_or_exit([sys.executable, script_name, *extra_args], cwd=repo)


def _add_hex_repo_to_syspath(repo: Path) -> None:
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))


def _repro_post_home(repo: Path, args: argparse.Namespace) -> None:
    _add_hex_repo_to_syspath(repo)
    from hex_maze_interface import HexMazeInterface, HomeParameters

    cluster = args.cluster
    trials = args.trials
    home_parameters = HomeParameters(
        travel_limit=args.travel_limit,
        max_velocity=args.max_velocity,
        run_current=args.run_current,
        stall_threshold=args.stall_threshold,
    )
    targets = tuple(args.targets)
    results: list[dict[str, object]] = []

    with HexMazeInterface(debug=args.debug) as hmi:
        for trial in range(trials):
            hmi.home_cluster(cluster, home_parameters)
            for _ in range(300):
                outcomes = hmi.read_home_outcomes_cluster(cluster)
                if all(outcome.name != "IN_PROGRESS" for outcome in outcomes):
                    break
                time.sleep(0.1)

            record = {
                "trial": trial,
                "after_home_mm": list(hmi.read_positions_cluster(cluster)),
                "write_targets_cluster_ok": hmi.write_targets_cluster(cluster, targets),
                "snapshots": [],
            }

            elapsed_checkpoints = (0.2, 1.0, 3.0, 8.0, 15.0)
            prior = 0.0
            for checkpoint in elapsed_checkpoints:
                time.sleep(checkpoint - prior)
                prior = checkpoint
                record["snapshots"].append(
                    {
                        "t_s": checkpoint,
                        "positions_mm": list(hmi.read_positions_cluster(cluster)),
                    }
                )
            results.append(record)

    print(json.dumps(results, indent=2, sort_keys=True))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    for name in (
        "repo-path",
        "python",
        "help",
        "discover",
        "verify",
        "format",
        "lint",
        "test",
        "check",
        "build",
        "check-dist",
        "release-check",
        "hardware-smoke",
        "hardware-gui-sequence",
        "hardware-regression",
        "hardware-edge",
        "hardware-acceptance",
    ):
        subparser = subparsers.add_parser(name)
        if name not in {"repo-path", "check", "release-check"}:
            subparser.add_argument("extra_args", nargs=argparse.REMAINDER)

    repro = subparsers.add_parser("repro-post-home")
    repro.add_argument("--cluster", type=int, default=10)
    repro.add_argument("--trials", type=int, default=5)
    repro.add_argument("--travel-limit", type=int, default=250)
    repro.add_argument("--max-velocity", type=int, default=20)
    repro.add_argument("--run-current", type=int, default=50)
    repro.add_argument("--stall-threshold", type=int, default=10)
    repro.add_argument(
        "--targets",
        type=int,
        nargs=7,
        default=(90, 100, 110, 120, 130, 140, 150),
    )
    repro.add_argument("--debug", action="store_true")

    return parser


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    repo = _find_hex_repo()

    if args.command == "repo-path":
        print(repo)
        return 0
    if args.command in {
        "python",
        "help",
        "discover",
        "verify",
        "format",
        "lint",
        "test",
        "build",
        "check-dist",
    }:
        _python_repo_cmd(repo, args)
        return 0
    if args.command == "check":
        _check(repo)
        return 0
    if args.command == "release-check":
        _release_check(repo)
        return 0
    if args.command == "hardware-smoke":
        _hardware_script(repo, "hardware_smoke_test.py", args)
        return 0
    if args.command == "hardware-gui-sequence":
        _hardware_script(repo, "hardware_gui_sequence_test.py", args)
        return 0
    if args.command == "hardware-regression":
        _hardware_script(repo, "hardware_regression_test.py", args)
        return 0
    if args.command == "hardware-edge":
        _hardware_script(repo, "hardware_edge_case_test.py", args)
        return 0
    if args.command == "hardware-acceptance":
        _hardware_script(repo, "hardware_preinstall_acceptance_test.py", args)
        return 0
    if args.command == "repro-post-home":
        _repro_post_home(repo, args)
        return 0

    parser.error(f"Unhandled command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
