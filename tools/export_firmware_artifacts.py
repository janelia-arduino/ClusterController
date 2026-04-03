#!/usr/bin/env python3
"""Export the rewrite firmware UF2 into a repo-local directory."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_ROOT = REPO_ROOT / ".pio" / "build"
EXPORT_ROOT = REPO_ROOT / "artifacts" / "firmware"

ENVIRONMENT = "pico-rewrite"
VARIANT = "rewrite"

def export_rewrite() -> dict[str, str]:
    source_root = BUILD_ROOT / ENVIRONMENT
    source_uf2 = source_root / "firmware.uf2"

    if not source_uf2.exists():
        raise SystemExit(
            f"Missing {source_uf2}. Build the {ENVIRONMENT} environment first."
        )

    EXPORT_ROOT.mkdir(parents=True, exist_ok=True)
    exported_uf2 = EXPORT_ROOT / f"clustercontroller-{VARIANT}.uf2"
    shutil.copy2(source_uf2, exported_uf2)

    return {
        "variant": VARIANT,
        "environment": ENVIRONMENT,
        "uf2": str(exported_uf2.relative_to(REPO_ROOT)),
        "sha256": hashlib.sha256(exported_uf2.read_bytes()).hexdigest(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

    exports = [export_rewrite()]

    manifest_path = EXPORT_ROOT / "manifest.json"
    manifest_path.write_text(json.dumps(exports, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"exports": exports, "manifest": str(manifest_path)}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
