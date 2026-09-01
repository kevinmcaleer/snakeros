"""Differential test: SnakeROS CDR against rclpy, in both directions.

Byte-equality is deliberately *not* the test. rclpy leaves uninitialised stack
garbage in CDR padding bytes and can emit a few bytes of trailing slack, so
two encoders that are both correct still differ byte for byte. What has to
hold is semantic:

1. SnakeROS decodes bytes produced by rclpy into the right values.
2. rclpy decodes bytes produced by SnakeROS into the right values
   (``tests/rclpy_check.py``, run inside a ROS 2 container).

This half is (1), plus a self-round-trip. Run it on the MicroPython Unix port.
"""

import json
import sys

sys.path.insert(0, ".")
sys.path.insert(0, "tests")

from corpus import CORPUS  # noqa: E402

FLOAT32_TOL = 1e-6


def _resolve(type_str):
    pkg, name = type_str.split("/")
    mod = __import__("snakeros.msg." + pkg, None, None, (pkg,))
    return getattr(mod, name)


def _field_type(cls, name):
    for n, t in cls._fields_:
        if n == name:
            return t
    raise KeyError(name)


def _build(cls, values):
    obj = cls()
    _fill(obj, values)
    return obj


def _fill(obj, values):
    for k, v in values.items():
        t = _field_type(type(obj), k)
        if isinstance(v, dict):
            _fill(getattr(obj, k), v)
        elif isinstance(v, list) and v and isinstance(v[0], dict):
            setattr(obj, k, [_build(t[1], item) for item in v])
        else:
            setattr(obj, k, v)


def _close(a, b, tol):
    """Compare, tolerating float32 rounding."""
    if isinstance(a, float) and isinstance(b, float):
        return abs(a - b) <= tol * max(1.0, abs(a), abs(b))
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            return False
        for x, y in zip(a, b):
            if not _close(x, y, tol):
                return False
        return True
    if hasattr(a, "_fields_") and hasattr(b, "_fields_"):
        for n, _t in a._fields_:
            if not _close(getattr(a, n), getattr(b, n), tol):
                return False
        return True
    return a == b


def main(path):
    truth = json.load(open(path))
    results = []
    failures = []

    for (type_str, values), expect in zip(CORPUS, truth):
        assert expect["type"] == type_str, "corpus and truth out of step"
        cls = _resolve(type_str)
        want_obj = _build(cls, values)
        rclpy_bytes = bytes(bytearray.fromhex(expect["body"]))

        # 1. decode what rclpy produced
        try:
            got_obj = cls.deserialize(rclpy_bytes)
            decoded_ok = _close(got_obj, want_obj, FLOAT32_TOL)
        except Exception as e:
            got_obj = None
            decoded_ok = False
            failures.append((type_str, "decode of rclpy bytes raised: " + str(e)))

        if got_obj is not None and not decoded_ok:
            failures.append((type_str, "decoded rclpy bytes != corpus\n"
                             "    want {}\n    got  {}".format(want_obj, got_obj)))

        # 2. self round-trip through our own encoder
        mine = want_obj.serialize()
        back = cls.deserialize(mine)
        if not _close(back, want_obj, FLOAT32_TOL):
            failures.append((type_str, "self round-trip changed the value"))

        # 3. our encoding must be a prefix-compatible length: never longer
        #    than rclpy's (which may carry trailing slack)
        if len(mine) > len(rclpy_bytes):
            failures.append((type_str, "encoding longer than rclpy: {} > {}".format(
                len(mine), len(rclpy_bytes))))

        results.append({"type": type_str, "body": mine.hex()})

    for t, why in failures:
        print("FAIL", t)
        print("   ", why)

    print("{}/{} cases passed".format(len(CORPUS) - len(failures), len(CORPUS)))

    with open("/tmp/snakeros.json", "w") as f:
        json.dump(results, f)
    print("wrote /tmp/snakeros.json for the rclpy-side check")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/rclpy.json"))
