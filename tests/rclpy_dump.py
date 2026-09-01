#!/usr/bin/env python3
"""Serialise the shared corpus with rclpy and dump hex.

Runs inside a ROS 2 container; the output is the ground truth the SnakeROS
encoder is diffed against.
"""
import importlib
import json
import sys

from rclpy.serialization import serialize_message

sys.path.insert(0, "/work/tests")
from corpus import CORPUS  # noqa: E402


def build(type_str, values):
    pkg, name = type_str.split("/")
    mod = importlib.import_module(pkg + ".msg")
    cls = getattr(mod, name)
    obj = cls()
    _fill(obj, values)
    return obj


def _fill(obj, values):
    for k, v in values.items():
        cur = getattr(obj, k)
        if isinstance(v, dict):
            _fill(cur, v)
        elif isinstance(v, list) and v and isinstance(v[0], dict):
            elem_cls = type(cur).__args__[0] if hasattr(type(cur), "__args__") else None
            items = []
            for item in v:
                sub = _new_like(obj, k)
                _fill(sub, item)
                items.append(sub)
            setattr(obj, k, items)
        else:
            setattr(obj, k, v)


def _new_like(obj, field):
    # Resolve the element type of a sequence field from its slot type.
    t = obj.get_fields_and_field_types()[field]
    inner = t.split("<")[-1].rstrip(">")
    if inner.startswith("sequence"):
        inner = inner.split("<")[-1].rstrip(">")
    pkg, _, name = inner.partition("/")
    if "/" in inner:
        parts = inner.split("/")
        pkg, name = parts[0], parts[-1]
    mod = importlib.import_module(pkg + ".msg")
    return getattr(mod, name)()


out = []
for type_str, values in CORPUS:
    msg = build(type_str, values)
    raw = serialize_message(msg)
    out.append({"type": type_str, "encap": raw[:4].hex(), "body": raw[4:].hex()})

print(json.dumps(out))
