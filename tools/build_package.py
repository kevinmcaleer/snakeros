#!/usr/bin/env python3
"""Generate ``package.json`` for ``mip`` and optionally cross-compile to .mpy.

``mip`` is MicroPython's package installer. With a ``package.json`` at the
repo root, installing SnakeROS on a board is one line::

    import mip
    mip.install("github:kevinmcaleer/snakeros")

Message packs are separate ``mip`` targets so a board only pays heap for the
packages it actually imports.
"""

import argparse
import json
import os
import subprocess
import sys

REPO = "github:kevinmcaleer/snakeros"

CORE = [
    "snakeros/__init__.py",
    "snakeros/errors.py",
    "snakeros/node.py",
    "snakeros/services.py",
    "snakeros/parameters.py",
    "snakeros/board.py",
    "snakeros/cdr/__init__.py",
    "snakeros/cdr/writer.py",
    "snakeros/cdr/reader.py",
    "snakeros/transport/__init__.py",
    "snakeros/transport/udp.py",
    "snakeros/transport/serial.py",
    "snakeros/transport/framing.py",
    "snakeros/xrce/__init__.py",
    "snakeros/xrce/const.py",
    "snakeros/xrce/entities.py",
    "snakeros/xrce/session.py",
    "snakeros/xrce/reliable.py",
    "snakeros/msg/__init__.py",
    "snakeros/msg/_base.py",
]

# Shipped by default: the ones almost every robot needs.
DEFAULT_PACKS = ["std_msgs", "geometry_msgs", "builtin_interfaces"]
OPTIONAL_PACKS = ["sensor_msgs", "nav_msgs", "std_srvs", "rcl_interfaces"]


def read_version():
    with open("snakeros/__init__.py") as f:
        for line in f:
            if line.startswith("__version__"):
                return line.split("=")[1].strip().strip('"').strip("'")
    return "0.0.0"


def build_package_json(version, ext=".py"):
    urls = []
    for path in CORE:
        src = path if ext == ".py" else path[:-3] + ext
        urls.append([src, "{}/{}".format(REPO, src)])
    for pack in DEFAULT_PACKS:
        p = "snakeros/msg/{}{}".format(pack, ext)
        urls.append([p, "{}/{}".format(REPO, p)])
    return {"urls": urls, "version": version}


def build_pack_json(pack, version, ext=".py"):
    p = "snakeros/msg/{}{}".format(pack, ext)
    return {"urls": [[p, "{}/{}".format(REPO, p)]], "version": version}


def compile_mpy(out_dir):
    """Cross-compile every module to .mpy.

    Bytecode is meaningfully smaller in RAM than source, which matters on a
    Pico W. The .mpy format has an ABI version that changes between
    MicroPython releases -- a .mpy built for one may be rejected by another --
    so releases are tagged with the version they were built for.
    """
    try:
        import mpy_cross
    except ImportError:
        print("mpy-cross not installed: pip install mpy-cross", file=sys.stderr)
        return 1
    count = 0
    for root, _dirs, files in os.walk("snakeros"):
        for fn in files:
            if not fn.endswith(".py"):
                continue
            src = os.path.join(root, fn)
            dst_dir = os.path.join(out_dir, root)
            os.makedirs(dst_dir, exist_ok=True)
            dst = os.path.join(dst_dir, fn[:-3] + ".mpy")
            proc = mpy_cross.run("-o", dst, src, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
            proc.wait()
            if proc.returncode != 0:
                print("FAILED {}: {}".format(
                    src, proc.stderr.read().decode()), file=sys.stderr)
                return 1
            count += 1
    print("compiled {} modules to {}".format(count, out_dir))

    py = sum(
        os.path.getsize(os.path.join(r, f))
        for r, _d, fs in os.walk("snakeros") for f in fs if f.endswith(".py")
    )
    mpy = sum(
        os.path.getsize(os.path.join(r, f))
        for r, _d, fs in os.walk(os.path.join(out_dir, "snakeros"))
        for f in fs if f.endswith(".mpy")
    )
    print(".py total  {:>8,} bytes".format(py))
    print(".mpy total {:>8,} bytes  ({:.0f}% of source)".format(mpy, 100.0 * mpy / py))
    return 0


def audit():
    """Every module on disk must appear in some manifest.

    ``board.py`` was added after this file was first written and was silently
    left out of ``package.json`` -- so ``mip.install`` produced a package that
    imported fine until the first ``from snakeros.board import ...``, which is
    what every example and half the docs do. This check exists so that cannot
    happen again.
    """
    listed = set(CORE) | {"snakeros/msg/%s.py" % p
                          for p in DEFAULT_PACKS + OPTIONAL_PACKS}
    on_disk = set()
    for root, _dirs, files in os.walk("snakeros"):
        for fn in files:
            if fn.endswith(".py"):
                on_disk.add(os.path.join(root, fn).replace(os.sep, "/"))
    missing = sorted(on_disk - listed)
    if missing:
        print("ERROR: modules missing from every manifest:", file=sys.stderr)
        for m in missing:
            print("  " + m, file=sys.stderr)
        return 1
    print("audit:        {} modules, all packaged".format(len(on_disk)))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mpy", action="store_true", help="also cross-compile")
    ap.add_argument("--out", default="build/mpy")
    args = ap.parse_args()

    if audit():
        return 1

    version = read_version()
    with open("package.json", "w") as f:
        json.dump(build_package_json(version), f, indent=2)
        f.write("\n")
    print("package.json  ({} core files + {} default packs)".format(
        len(CORE), len(DEFAULT_PACKS)))

    os.makedirs("packages", exist_ok=True)
    for pack in OPTIONAL_PACKS:
        path = os.path.join("packages", pack + ".json")
        with open(path, "w") as f:
            json.dump(build_pack_json(pack, version), f, indent=2)
            f.write("\n")
    print("packages/     ({} optional message packs)".format(len(OPTIONAL_PACKS)))

    if args.mpy:
        return compile_mpy(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
