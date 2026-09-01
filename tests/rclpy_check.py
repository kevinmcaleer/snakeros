#!/usr/bin/env python3
"""Deserialise SnakeROS-produced CDR with rclpy and check the values.

This is the direction that actually proves wire compatibility: bytes written
by the MicroPython encoder must be readable by real ROS 2. Run inside a ROS 2
container against /tmp/snakeros.json.
"""
import importlib
import json
import sys

from rclpy.serialization import deserialize_message

sys.path.insert(0, "/work/tests")
from corpus import CORPUS  # noqa: E402

TOL = 1e-6


def resolve(type_str):
    pkg, name = type_str.split("/")
    return getattr(importlib.import_module(pkg + ".msg"), name)


def check(obj, values, path=""):
    problems = []
    for k, v in values.items():
        got = getattr(obj, k)
        p = path + "." + k if path else k
        if isinstance(v, dict):
            problems += check(got, v, p)
        elif isinstance(v, list) and v and isinstance(v[0], dict):
            if len(got) != len(v):
                problems.append("{}: length {} != {}".format(p, len(got), len(v)))
            else:
                for i, item in enumerate(v):
                    problems += check(got[i], item, "{}[{}]".format(p, i))
        elif isinstance(v, list):
            if len(got) != len(v):
                problems.append("{}: length {} != {}".format(p, len(got), len(v)))
            else:
                for i, (a, b) in enumerate(zip(got, v)):
                    if isinstance(b, float):
                        if abs(a - b) > TOL * max(1.0, abs(b)):
                            problems.append("{}[{}]: {} != {}".format(p, i, a, b))
                    elif a != b:
                        problems.append("{}[{}]: {!r} != {!r}".format(p, i, a, b))
        elif isinstance(v, float):
            if abs(got - v) > TOL * max(1.0, abs(v)):
                problems.append("{}: {} != {}".format(p, got, v))
        else:
            if got != v:
                problems.append("{}: {!r} != {!r}".format(p, got, v))
    return problems


data = json.load(open(sys.argv[1] if len(sys.argv) > 1 else "/tmp/snakeros.json"))
fails = 0
for (type_str, values), rec in zip(CORPUS, data):
    assert rec["type"] == type_str
    cls = resolve(type_str)
    raw = bytes.fromhex("00010000") + bytes.fromhex(rec["body"])  # add CDR_LE encap
    try:
        msg = deserialize_message(raw, cls)
    except Exception as e:
        print("FAIL {}: rclpy could not deserialise: {}".format(type_str, e))
        fails += 1
        continue
    problems = check(msg, values)
    if problems:
        fails += 1
        print("FAIL", type_str)
        for pr in problems[:6]:
            print("   ", pr)

print("{}/{} cases readable by rclpy".format(len(CORPUS) - fails, len(CORPUS)))
sys.exit(1 if fails else 0)
