# Running SnakeROS on bare-metal ARM Cortex-M

MicroPython's QEMU port runs **real ARM Thumb code on a bare-metal Cortex-M3**
(MPS2-AN385) with no operating system and modules frozen into firmware —
exactly how code is deployed to a real MCU. It is the closest you can get to a
Pico without silicon, and it verifies the instruction-set-dependent parts of
the stack.

**What it cannot do:** the QEMU port sets `MICROPY_PY_ASYNCIO=0`, has no
`socket`, and exposes no `machine.UART` to Python. So there is no transport of
any kind and no end-to-end test against an Agent. This verifies the
*computational core* — CDR, the message system, entity XML — not the network
path. For end-to-end under board-like memory, use the 32-bit ARM container
described in `docs/memory.md`.

## Build

```console
$ git clone --depth 1 --branch v1.29.0 https://github.com/micropython/micropython /tmp/mpqemu
$ cd /tmp/mpqemu/ports/qemu
$ make BOARD=MPS2_AN385 submodules
$ make BOARD=MPS2_AN385 FROZEN_MANIFEST=<path>/manifest.py -j4
```

`manifest.py` freezes `snakeros/{errors,cdr,xrce/const,xrce/entities,msg}` —
everything that does not need a transport.

## Run

```console
$ ( printf '\x01'; cat test_cortex_m.py; printf '\x04' ) | \
    qemu-system-arm -machine mps2-an385 -cpu cortex-m3 -nographic \
      -monitor null -semihosting -serial stdio \
      -kernel build-MPS2_AN385/firmware.elf
```

## Result

```
MicroPython v1.29.0 on 2026-09-01; mps2-an385 with Cortex-M3
heap total: 140032
CDR u8+f64 : 01000000000000000000000000000440 len 16
Vector3 fast fmt: <ddd
Twist bytes: 48 ... roundtrip: True
Imu bytes: 320  roundtrip: True
String roundtrip: hello from Cortex-M
objid(300,2): 12c2 -> (300, 2)
mangle: rt/chatter | std_msgs::msg::dds_::String_
free heap after: 113584
=== CORTEX-M OK ===
```

Firmware is 216 KB text / 144 KB bss, and 113 KB of the 140 KB heap remains
free with every frozen message pack loaded.
