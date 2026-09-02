---
title: Packaging
---

# Packaging

## Install with `mip`

```python
import mip
mip.install('github:kevinmcaleer/snakeros')
```

That is the whole minimum install: **23 files, ~121 KB** — the core plus
`std_msgs`, `geometry_msgs` and `builtin_interfaces`.

| | files | bytes |
|---|---|---|
| `snakeros/` (node, board, services, parameters, errors) | 6 | 36,006 |
| `snakeros/xrce/` (the protocol) | 5 | 40,284 |
| `snakeros/msg/` (base + 3 default packs) | 5 | 21,701 |
| `snakeros/transport/` (UDP, serial, framing) | 4 | 15,474 |
| `snakeros/cdr/` (encode/decode) | 3 | 7,739 |
| **total** | **23** | **121,204** |

Other packs are separate so a board only pays heap for what it uses:

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

```console
$ make mpy
compiled 26 modules to build/mpy
.py total   ~137,000 bytes
.mpy total    ~51,000 bytes  (~37% of source)
```

Copy `build/mpy/snakeros` to the board in place of `snakeros`.

### What it actually buys

Measured on **32-bit ARM MicroPython v1.29.0** with a 190 KB heap — i.e. a
realistic Pico W footprint, not a 64-bit desktop:

| | `.py` | `.mpy` | |
|---|---|---|---|
| Flash / filesystem | ~137 KB | **~51 KB** | ~37% |
| Import time | 78.7 ms | **18.6 ms** | 4.2× faster |
| **Peak heap dip during import** | 128,848 B | **96,848 B** | **32 KB less** |
| Steady heap after import | 71,344 B | 73,760 B | ~2 KB *more* |

**The peak matters more than the steady state.** Importing `.py` needs the
parser and compiler's working memory, and that transient spike is what raises
`MemoryError` on a constrained board — not the settled figure. Saving 32 KB of
peak on a board with 190 KB of heap is the difference between fitting and not.

Steady-state heap is **not** improved — it is marginally worse, because
loading a `.mpy` interns qstrs slightly differently. An earlier version of this
page claimed `.mpy` "measurably reduces heap"; that was measured on the 64-bit
Unix port and does not hold on a 32-bit target. Corrected here.

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
