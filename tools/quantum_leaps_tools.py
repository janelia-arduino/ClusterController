#!/usr/bin/env python3
"""
quantum_leaps_tools.py

Install and run the Quantum Leaps host tools pinned to this repository:

- QM      5.2.3
- QTools  6.9.3  (includes QSPY)

The tools are installed locally into:
  <repo>/.tools/quantum-leaps/

Typical usage (via pixi tasks):

  pixi run ql-install
  pixi run qm
  pixi run qspy -c /dev/ttyACM0 -b 115200
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import textwrap
import urllib.error
import urllib.request
import zipfile
from pathlib import Path
from typing import Iterable, Optional

QM_VERSION = "5.2.3"
QTOOLS_VERSION = "6.9.3"


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _tools_base_dir() -> Path:
    override = os.environ.get("QL_TOOLS_DIR")
    if override:
        return Path(override).expanduser().resolve()
    return _repo_root() / ".tools" / "quantum-leaps"


def _cache_dir() -> Path:
    return _tools_base_dir() / "_cache"


def _sys_id() -> tuple[str, str]:
    return platform.system().lower(), platform.machine().lower()


def _print(msg: str) -> None:
    print(msg, flush=True)


def _eprint(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def _is_probably_html(path: Path) -> bool:
    try:
        head = path.read_bytes()[:512].lstrip()
    except Exception:
        return False
    return head.startswith(b"<!doctype html") or head.startswith(b"<html") or head.startswith(b"<HTML")


def _download(urls: Iterable[str], dest: Path, *, force: bool = False, validate_zip: bool = False) -> Path:
    dest.parent.mkdir(parents=True, exist_ok=True)

    if dest.exists() and not force:
        return dest

    tmp = dest.with_suffix(dest.suffix + ".tmp")
    if tmp.exists():
        tmp.unlink()

    last_err: Optional[BaseException] = None

    for url in urls:
        try:
            _print(f"Downloading: {url}")
            req = urllib.request.Request(
                url,
                headers={
                    "User-Agent": "ClusterController/quantum_leaps_tools.py (+https://github.com/janelia-arduino/ClusterController)",
                },
            )
            with urllib.request.urlopen(req) as response, open(tmp, "wb") as out:
                shutil.copyfileobj(response, out)

            if validate_zip and (not zipfile.is_zipfile(tmp) or _is_probably_html(tmp)):
                raise RuntimeError("downloaded file is not a valid zip archive")

            tmp.replace(dest)
            return dest
        except (urllib.error.HTTPError, urllib.error.URLError, RuntimeError, OSError) as exc:
            last_err = exc
            _eprint(f"  failed: {exc}")
            if tmp.exists():
                try:
                    tmp.unlink()
                except OSError:
                    pass

    raise SystemExit(f"ERROR: Unable to download {dest.name}. Last error: {last_err}")


def _extract_zip_move_top(
    archive: Path,
    dest_dir: Path,
    *,
    expected_top_dir_name: Optional[str] = None,
    force: bool = False,
) -> Path:
    if dest_dir.exists() and any(dest_dir.iterdir()) and not force:
        return dest_dir

    if dest_dir.exists() and force:
        shutil.rmtree(dest_dir)

    dest_dir.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="ql_zip_") as td:
        tmp_root = Path(td)
        with zipfile.ZipFile(archive, "r") as zf:
            zf.extractall(tmp_root)

        src_dir: Optional[Path] = None

        if expected_top_dir_name:
            for path in tmp_root.iterdir():
                if path.is_dir() and path.name.lower() == expected_top_dir_name.lower():
                    src_dir = path
                    break

        if src_dir is None:
            dirs = [path for path in tmp_root.iterdir() if path.is_dir()]
            files = [path for path in tmp_root.iterdir() if path.is_file()]
            if len(dirs) == 1 and not files:
                src_dir = dirs[0]

        if src_dir is None:
            dest_dir.mkdir(parents=True, exist_ok=True)
            for item in tmp_root.iterdir():
                dst = dest_dir / item.name
                if item.is_dir():
                    shutil.copytree(item, dst, dirs_exist_ok=True)
                else:
                    shutil.copy2(item, dst)
            return dest_dir

        shutil.move(str(src_dir), str(dest_dir))

    return dest_dir


def _qm_candidate_urls(sysname: str, machine: str) -> list[str]:
    tag = f"v{QM_VERSION}"
    gh_base = f"https://github.com/QuantumLeaps/qm/releases/download/{tag}/"
    urls: list[str] = []

    if sysname == "linux":
        urls.extend(
            [
                gh_base + f"qm_{QM_VERSION}-linux64.zip",
                gh_base + f"qm_{QM_VERSION}-linux.zip",
                f"https://sourceforge.net/projects/qpc/files/QM/{QM_VERSION}/qm_{QM_VERSION}-linux64.zip/download",
                f"https://sourceforge.net/projects/qpc/files/QM/{QM_VERSION}/qm_{QM_VERSION}-linux.zip/download",
            ]
        )
    elif sysname == "windows":
        urls.extend(
            [
                gh_base + f"qm_{QM_VERSION}-windows.zip",
                gh_base + f"qm_{QM_VERSION}-win64.zip",
                gh_base + f"qm_{QM_VERSION}-win32.zip",
                f"https://sourceforge.net/projects/qpc/files/QM/{QM_VERSION}/qm_{QM_VERSION}-windows.zip/download",
                f"https://sourceforge.net/projects/qpc/files/QM/{QM_VERSION}/qm_{QM_VERSION}-win64.zip/download",
                f"https://sourceforge.net/projects/qpc/files/QM/{QM_VERSION}/qm_{QM_VERSION}-win32.zip/download",
            ]
        )
    elif sysname == "darwin":
        urls.extend(
            [
                gh_base + f"qm_{QM_VERSION}-macx64.dmg",
                gh_base + f"qm_{QM_VERSION}-macos.dmg",
                gh_base + f"qm_{QM_VERSION}-macosx.dmg",
                f"https://sourceforge.net/projects/qpc/files/QM/{QM_VERSION}/qm_{QM_VERSION}-macx64.dmg/download",
                f"https://sourceforge.net/projects/qpc/files/QM/{QM_VERSION}/qm_{QM_VERSION}-macos.dmg/download",
            ]
        )
    else:
        raise SystemExit(f"Unsupported OS: {sysname} {machine}")

    return urls


def _qtools_candidate_urls(sysname: str) -> list[str]:
    tag = f"v{QTOOLS_VERSION}"
    gh_base = f"https://github.com/QuantumLeaps/qtools/releases/download/{tag}/"

    if sysname == "windows":
        fname = f"qtools-windows_{QTOOLS_VERSION}.zip"
    else:
        fname = f"qtools-posix_{QTOOLS_VERSION}.zip"

    return [
        gh_base + fname,
        f"https://sourceforge.net/projects/qtools/files/v{QTOOLS_VERSION}/{fname}/download",
    ]


def install_qm(*, force: bool = False) -> Path:
    sysname, machine = _sys_id()
    cache = _cache_dir()
    cache.mkdir(parents=True, exist_ok=True)
    install_dir = _tools_base_dir() / f"qm-{QM_VERSION}"

    if sysname == "darwin":
        dmg = cache / f"qm_{QM_VERSION}.dmg"
        _download(_qm_candidate_urls(sysname, machine), dmg, force=force, validate_zip=False)

        if install_dir.exists() and any(install_dir.iterdir()) and not force:
            return install_dir

        if install_dir.exists() and force:
            shutil.rmtree(install_dir)

        install_dir.mkdir(parents=True, exist_ok=True)
        mount_point = Path(tempfile.mkdtemp(prefix="qm_dmg_mount_"))
        try:
            subprocess.run(["xattr", "-c", str(dmg)], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(
                ["hdiutil", "attach", "-nobrowse", "-readonly", "-mountpoint", str(mount_point), str(dmg)],
                check=True,
            )
            apps = list(mount_point.glob("*.app")) or list(mount_point.rglob("*.app"))
            if not apps:
                raise SystemExit(f"ERROR: Could not find a QM *.app inside {mount_point}")
            app_src = apps[0]
            app_dst = install_dir / app_src.name
            if app_dst.exists():
                shutil.rmtree(app_dst)
            shutil.copytree(app_src, app_dst, dirs_exist_ok=True)
            subprocess.run(["xattr", "-cr", str(app_dst)], check=False)
            return install_dir
        finally:
            subprocess.run(["hdiutil", "detach", str(mount_point)], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            shutil.rmtree(mount_point, ignore_errors=True)

    zip_path = cache / f"qm_{QM_VERSION}_{sysname}.zip"
    _download(_qm_candidate_urls(sysname, machine), zip_path, force=force, validate_zip=True)
    _extract_zip_move_top(zip_path, install_dir, expected_top_dir_name="qm", force=force)

    if sysname == "linux":
        for exe in (install_dir / "bin" / "qm", install_dir / "bin" / "qm.sh"):
            if exe.exists():
                exe.chmod(exe.stat().st_mode | 0o111)

    return install_dir


def _is_file(path: Path) -> bool:
    try:
        return path.is_file()
    except OSError:
        return False


def _find_qspy_binary(qtools_dir: Path) -> Optional[Path]:
    sysname, _ = _sys_id()
    if sysname == "windows":
        candidates = [
            qtools_dir / "bin" / "qspy.exe",
            qtools_dir / "qspy" / "bin" / "qspy.exe",
            qtools_dir / "qspy.exe",
        ]
        name = "qspy.exe"
    else:
        candidates = [
            qtools_dir / "bin" / "qspy",
            qtools_dir / "qspy" / "posix" / "qspy",
            qtools_dir / "qspy" / "bin" / "qspy",
        ]
        name = "qspy"

    for candidate in candidates:
        if _is_file(candidate):
            return candidate

    matches = [path for path in qtools_dir.rglob(name) if _is_file(path)]
    return matches[0] if matches else None


def _ensure_executable(path: Path) -> None:
    try:
        path.chmod(path.stat().st_mode | 0o111)
    except OSError:
        pass


def _build_qspy_posix(qtools_dir: Path) -> None:
    posix_dir = qtools_dir / "qspy" / "posix"
    if not posix_dir.exists():
        _eprint(f"WARNING: expected directory not found: {posix_dir}")
        return

    _print("Building qspy from source (make) ...")
    try:
        subprocess.run(["make"], cwd=str(posix_dir), check=True)
    except FileNotFoundError:
        _eprint("ERROR: 'make' not found. Ensure the pixi environment includes build tools.")
        return
    except subprocess.CalledProcessError as exc:
        _eprint(f"ERROR: qspy build failed: {exc}")
        return

    qspy_bin = qtools_dir / "bin" / "qspy"
    if _is_file(qspy_bin):
        _ensure_executable(qspy_bin)
        return

    built_candidates = [
        posix_dir / "qspy",
        posix_dir / "bin" / "qspy",
        posix_dir / "../bin/qspy",
    ]
    for candidate in built_candidates:
        resolved = candidate.resolve()
        if _is_file(resolved):
            (qtools_dir / "bin").mkdir(parents=True, exist_ok=True)
            shutil.copy2(resolved, qspy_bin)
            _ensure_executable(qspy_bin)
            return

    matches = [path for path in posix_dir.rglob("qspy") if _is_file(path)]
    if matches:
        (qtools_dir / "bin").mkdir(parents=True, exist_ok=True)
        shutil.copy2(matches[0], qspy_bin)
        _ensure_executable(qspy_bin)


def install_qtools(*, force: bool = False, build_qspy: bool = True) -> Path:
    sysname, _ = _sys_id()
    cache = _cache_dir()
    cache.mkdir(parents=True, exist_ok=True)

    fname = f"qtools-windows_{QTOOLS_VERSION}.zip" if sysname == "windows" else f"qtools-posix_{QTOOLS_VERSION}.zip"
    zip_path = cache / fname
    install_dir = _tools_base_dir() / f"qtools-{QTOOLS_VERSION}"

    _download(_qtools_candidate_urls(sysname), zip_path, force=force, validate_zip=True)
    _extract_zip_move_top(zip_path, install_dir, expected_top_dir_name="qtools", force=force)

    if sysname != "windows" and build_qspy and _find_qspy_binary(install_dir) is None:
        _build_qspy_posix(install_dir)

    return install_dir


def run_qm(qm_args: list[str]) -> int:
    sysname, _ = _sys_id()
    qm_dir = install_qm(force=False)

    if sysname == "linux":
        entry = qm_dir / "bin" / "qm.sh"
        if not entry.exists():
            entry = qm_dir / "bin" / "qm"
        if not entry.exists():
            raise SystemExit(f"ERROR: QM executable not found under {qm_dir}/bin")
        _ensure_executable(entry)
        return subprocess.run([str(entry)] + qm_args, check=False).returncode

    if sysname == "windows":
        candidates = [
            qm_dir / "bin" / "qm.exe",
            qm_dir / "qm.exe",
            qm_dir / "bin" / "qm.bat",
            qm_dir / "qm.bat",
        ]
        entry = next((path for path in candidates if path.exists()), None)
        if entry is None:
            matches = list(qm_dir.rglob("qm.exe")) + list(qm_dir.rglob("qm.bat"))
            entry = matches[0] if matches else None
        if entry is None:
            raise SystemExit(f"ERROR: QM executable not found under {qm_dir}")
        return subprocess.run([str(entry)] + qm_args, check=False).returncode

    if sysname == "darwin":
        apps = list(qm_dir.glob("*.app")) or list(qm_dir.rglob("*.app"))
        if not apps:
            raise SystemExit(f"ERROR: QM app bundle (*.app) not found under {qm_dir}")
        cmd = ["open", "-a", str(apps[0])]
        if qm_args:
            cmd += ["--args"] + qm_args
        return subprocess.run(cmd, check=False).returncode

    raise SystemExit(f"Unsupported OS: {sysname}")


def _augment_runtime_env(env: dict[str, str], qtools_dir: Path) -> dict[str, str]:
    sysname, _ = _sys_id()
    bin_dir = qtools_dir / "bin"
    if not bin_dir.exists():
        return env

    old_path = env.get("PATH", "")
    env["PATH"] = str(bin_dir) + (os.pathsep + old_path if old_path else "")

    if sysname == "linux":
        ld = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = str(bin_dir) + (os.pathsep + ld if ld else "")
    elif sysname == "darwin":
        dyld = env.get("DYLD_LIBRARY_PATH", "")
        env["DYLD_LIBRARY_PATH"] = str(bin_dir) + (os.pathsep + dyld if dyld else "")
    return env


def run_qspy(qspy_args: list[str]) -> int:
    qtools_dir = install_qtools(force=False, build_qspy=True)
    qspy = _find_qspy_binary(qtools_dir)
    if qspy is None:
        raise SystemExit(
            "ERROR: qspy binary was not found after installing QTools.\n\n"
            "On Linux/macOS you may need build tools (make + a C compiler).\n"
            "Try:\n"
            "  pixi install\n"
            "  pixi run qtools-install\n"
        )

    if _sys_id()[0] != "windows":
        _ensure_executable(qspy)

    env = os.environ.copy()
    env.setdefault("QTOOLS", str(qtools_dir))
    env.setdefault("QTOOLS_HOME", str(qtools_dir))
    env = _augment_runtime_env(env, qtools_dir)

    cmd = [str(qspy)] + qspy_args
    _print("Running: " + " ".join(cmd))
    return subprocess.run(cmd, env=env, check=False).returncode


def install_all(*, force: bool = False) -> None:
    install_qm(force=force)
    install_qtools(force=force, build_qspy=True)
    _print(f"Installed QM {QM_VERSION} and QTools {QTOOLS_VERSION} into: {_tools_base_dir()}")
    _print("Consider adding '.tools/' (or '.tools/quantum-leaps/') to your .gitignore.")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="quantum_leaps_tools.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent(
            f"""\
            Install and run Quantum Leaps host tools pinned for this repository:

              QM      {QM_VERSION}
              QTools  {QTOOLS_VERSION} (includes QSPY)

            Tools are installed locally into:
              {str(_tools_base_dir())}
            """
        ),
    )

    sub = parser.add_subparsers(dest="command", required=True)

    p_install = sub.add_parser("install", help="Install both QM and QTools")
    p_install.add_argument("--force", action="store_true", help="Re-download and reinstall from scratch")

    p_qm = sub.add_parser("qm", help="Launch QM (installs first if needed)")
    p_qm.add_argument("--install-only", action="store_true", help="Only install QM (do not launch)")
    p_qm.add_argument("--force", action="store_true", help="Re-download and reinstall QM from scratch")

    p_qtools = sub.add_parser("qtools-install", help="Install QTools (and build qspy on POSIX if needed)")
    p_qtools.add_argument("--force", action="store_true", help="Re-download and reinstall QTools from scratch")

    p_qspy = sub.add_parser("qspy", help="Run QSPY (installs QTools first if needed)")
    p_qspy.add_argument("--force", action="store_true", help="Re-download and reinstall QTools from scratch")

    args, remainder = parser.parse_known_args(argv)

    if args.command == "install":
        if remainder:
            parser.error("unrecognized arguments: " + " ".join(remainder))
        install_all(force=args.force)
        return 0

    if args.command == "qm":
        install_qm(force=args.force)
        qm_args = remainder[1:] if remainder and remainder[0] == "--" else remainder
        if not args.install_only:
            return run_qm(qm_args)
        return 0

    if args.command == "qtools-install":
        if remainder:
            parser.error("unrecognized arguments: " + " ".join(remainder))
        install_qtools(force=args.force, build_qspy=True)
        return 0

    if args.command == "qspy":
        qspy_args = remainder[1:] if remainder and remainder[0] == "--" else remainder
        if args.force:
            install_qtools(force=True, build_qspy=True)
        return run_qspy(qspy_args)

    raise SystemExit("unreachable")


if __name__ == "__main__":
    raise SystemExit(main())
