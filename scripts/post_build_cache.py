"""
PlatformIO post-build script: caches firmware.bin in fw_cache/ named by git hash.

Naming: fw_<7-char-hash>[_dirty].bin
  - _dirty suffix appended when there are uncommitted changes in the working tree.
  - Skips caching if the exact filename already exists (same hash, same dirty state).
"""

import subprocess
import shutil
import os

Import("env")  # noqa: F821 — PlatformIO injects this


def _git(*args):
    try:
        return subprocess.check_output(["git"] + list(args),
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return ""


def after_build(source, target, env):
    short_hash = _git("rev-parse", "--short", "HEAD")
    if not short_hash:
        print("[FW Cache] WARNING: could not get git hash — skipping cache")
        return

    dirty = bool(_git("status", "--porcelain"))
    tag = f"fw_{short_hash}{'_dirty' if dirty else ''}"

    cache_dir = os.path.join(env["PROJECT_DIR"], "fw_cache")
    os.makedirs(cache_dir, exist_ok=True)

    dst = os.path.join(cache_dir, f"{tag}.bin")
    src = str(target[0])

    if os.path.exists(dst):
        print(f"[FW Cache] Already cached: {tag}.bin")
        return

    shutil.copy2(src, dst)
    print(f"[FW Cache] Cached: fw_cache/{tag}.bin")

    # Keep a 'latest.bin' symlink / copy for convenience
    latest = os.path.join(cache_dir, "latest.bin")
    shutil.copy2(src, latest)
    print(f"[FW Cache] Updated: fw_cache/latest.bin -> {tag}.bin")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build)
