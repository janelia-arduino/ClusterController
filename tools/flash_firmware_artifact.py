#!/usr/bin/env python3
"""Flash a prebuilt UF2 artifact, bootstrapping picotool if needed."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
PLATFORMIO_CORE_DIR = REPO_ROOT / ".platformio"
PICOTOOL_PACKAGE = "tool-picotool-rp2040-earlephilhower"


def _picotool_path() -> Path:
    binary_name = "picotool.exe" if os.name == "nt" else "picotool"
    return PLATFORMIO_CORE_DIR / "packages" / PICOTOOL_PACKAGE / binary_name


def _platformio_env() -> dict[str, str]:
    env = os.environ.copy()
    env["PLATFORMIO_CORE_DIR"] = str(PLATFORMIO_CORE_DIR)
    env["PLATFORMIO_SETTING_ENABLE_TELEMETRY"] = "no"
    return env


def _run(cmd: list[str]) -> None:
    completed = subprocess.run(cmd, cwd=REPO_ROOT, env=_platformio_env(), check=False)
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


def _ensure_picotool() -> Path:
    picotool = _picotool_path()
    if picotool.exists():
        return picotool

    _run(
        [
            "pio",
            "pkg",
            "install",
            "--global",
            "--tool",
            PICOTOOL_PACKAGE,
        ]
    )
    if not picotool.exists():
        raise SystemExit(f"expected picotool at {picotool}, but it was not installed")
    return picotool


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("uf2", help="Path to the UF2 artifact to flash.")
    parser.add_argument(
        "picotool_args",
        nargs=argparse.REMAINDER,
        help="Additional arguments passed directly to picotool.",
    )
    args = parser.parse_args()

    uf2_path = Path(args.uf2)
    if not uf2_path.is_absolute():
        uf2_path = REPO_ROOT / uf2_path
    uf2_path = uf2_path.resolve()
    if not uf2_path.exists():
        raise SystemExit(f"UF2 artifact does not exist: {uf2_path}")

    picotool = _ensure_picotool()
    extra_args = list(args.picotool_args)
    if extra_args[:1] == ["--"]:
        extra_args = extra_args[1:]

    _run(
        [
            str(picotool),
            "load",
            str(uf2_path),
            "-t",
            "uf2",
            "-v",
            "-x",
            "-f",
            *extra_args,
        ]
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
