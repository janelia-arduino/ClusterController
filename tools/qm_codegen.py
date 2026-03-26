#!/usr/bin/env python3
"""Run QM code generation for ClusterController.qm.

This wraps the pinned QM install and provides a single repo-local entrypoint:

  pixi run codegen

On Linux, QM 5.2.3 is still a GUI/Qt application even when used with `-c`, so
it needs either:

- a working X display (`DISPLAY`), or
- `xvfb-run` available on PATH.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
QM_WRAPPER = REPO_ROOT / ".tools" / "quantum-leaps" / "qm-5.2.3" / "bin" / "qm.sh"
MODEL_PATH = REPO_ROOT / "src" / "ClusterController.qm"


def main() -> int:
    if not QM_WRAPPER.exists():
        print("ERROR: QM is not installed for this repo.", file=sys.stderr)
        print("Run `pixi run qm-install` or `pixi run ql-install` first.", file=sys.stderr)
        return 1

    cmd = [str(QM_WRAPPER), str(MODEL_PATH), "-c", "--"]

    env = os.environ.copy()
    display = env.get("DISPLAY", "").strip()
    xvfb_run = shutil.which("xvfb-run")

    if display:
        print(f"Using existing DISPLAY={display}")
        print("Running:", " ".join(cmd))
        result = subprocess.run(cmd, check=False, env=env)
        if result.returncode == 0:
            return 0
        print("QM failed while trying to use the current DISPLAY.", file=sys.stderr)
        print("If this shell is headless or the X server is unreachable, rerun from a desktop session", file=sys.stderr)
        print("or install `xvfb-run`/`Xvfb` and retry `pixi run codegen`.", file=sys.stderr)
        return result.returncode

    if xvfb_run:
        wrapped = [xvfb_run, "-a"] + cmd
        print("Running:", " ".join(wrapped))
        return subprocess.run(wrapped, check=False, env=env).returncode

    print("ERROR: QM code generation requires an X display on this host.", file=sys.stderr)
    print("No DISPLAY is set, and `xvfb-run` was not found on PATH.", file=sys.stderr)
    print("", file=sys.stderr)
    print("Options:", file=sys.stderr)
    print("  1. Run `pixi run codegen` from a desktop session with DISPLAY set.", file=sys.stderr)
    print("  2. Install `xvfb-run`/`Xvfb` on the host and rerun `pixi run codegen`.", file=sys.stderr)
    print("  3. Launch `pixi run qm`, open the model, and generate code from QM directly.", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
