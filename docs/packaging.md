---
title: Packaging
---

# Packaging

## Install with `mip`

```python
import mip
mip.install('github:kevinmcaleer/snakeros')
```

That pulls the core plus `std_msgs`, `geometry_msgs` and
`builtin_interfaces`. Other packs are separate so a board only pays heap for
what it uses:

```python
mip.install('github:kevinmcaleer/snakeros/packages/sensor_msgs.json')
mip.install('github:kevinmcaleer/snakeros/packages/nav_msgs.json')
mip.install('github:kevinmcaleer/snakeros/packages/std_srvs.json')
mip.install('github:kevinmcaleer/snakeros/packages/rcl_interfaces.json')
```

## With `mpremote` instead

```console
$ mpremote connect /dev/tty.usbmodem1101 mip install github:kevinmcaleer/snakeros
```

Or copy the tree directly:

```console
$ mpremote connect /dev/tty.usbmodem1101 fs cp -r snakeros :
```

## The `.mpy` build

Bytecode instead of source: **38% of the size**, and measurably less heap.
On a Pico W that can be the difference between fitting and not.

```console
$ make mpy
compiled 26 modules to build/mpy
.py total   128,340 bytes
.mpy total   48,787 bytes  (38% of source)
```

Copy `build/mpy/snakeros` to the board in place of `snakeros`.

### Are `.mpy` files board-dependent?

**No — but they are MicroPython-version-dependent.**

SnakeROS's `.mpy` files are **pure bytecode**: the native-architecture field in
the header is 0, so the same file runs on an RP2040, an RP2350, an ESP32 or an
STM32.

```console
$ xxd -l4 build/mpy/snakeros/node.mpy
4d 06 00 1f      # 'M', .mpy version 6, flags 0x00 -> arch 0 = portable
```

What **must** match is the **`.mpy` format version** (6 above). MicroPython
bumps it between releases, and a runtime expecting v5 rejects a v6 file. Check
with:

```python
import sys
sys.implementation._mpy & 0xff     # the .mpy version this firmware wants
```

Releases are tagged with the MicroPython version they were built for.

This portability holds **only because nothing in SnakeROS uses
`@micropython.native`, `@micropython.viper` or inline assembler.** Those
compile to real machine code and would make the containing module
architecture-specific, requiring a build per `-march`. If the CDR hot paths
ever take the viper escape hatch, that trade has to be made deliberately —
and those modules would need per-architecture artefacts.

## Building the manifests

```console
$ python3 tools/build_package.py          # package.json + packages/*.json
$ python3 tools/build_package.py --mpy    # and cross-compile
```

`mpy-cross` comes from `pip install mpy-cross`, or with a MicroPython build.

## Releasing

1. Bump `__version__` in `snakeros/__init__.py`.
2. `python3 tools/build_package.py --mpy`
3. `make test` — unit, CDR differential, integration.
4. Tag, and attach `build/mpy` as a release artefact **naming the MicroPython
   version it was built for**.
