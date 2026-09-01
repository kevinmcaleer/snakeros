#!/usr/bin/env python3
"""Generate SnakeROS message modules from ROS 2 ``.msg`` / ``.srv`` files.

This is the escape hatch that stops SnakeROS from being limited to whatever
message packs happen to ship with it: point it at any ``.msg`` file -- a
custom robot interface included -- and it emits a small Python module the
board can import.

It deliberately runs on plain CPython with no ROS 2 installation required, so
the dev machine does not need a ROS environment just to add a message type.

Usage::

    python3 tools/snakeros_gen.py --search-path /opt/ros/jazzy/share \\
        --package std_msgs --out snakeros/msg
    python3 tools/snakeros_gen.py MyThing.msg --package my_robot --out ./gen
"""

import argparse
import os
import re
import sys

# ROS 2 primitive -> SnakeROS field code (see snakeros/msg/_base.py)
PRIMITIVES = {
    "bool": "b",
    "byte": "o",
    "char": "c",
    "float32": "f",
    "float64": "d",
    "int8": "1",
    "uint8": "o",
    "int16": "s",
    "uint16": "S",
    "int32": "i",
    "uint32": "I",
    "int64": "l",
    "uint64": "L",
    "string": "T",
    "wstring": "T",
}

ARRAY_RE = re.compile(r"^(.*?)\[(<=)?(\d*)\]$")
BOUNDED_STR_RE = re.compile(r"^(w?string)<=\d+$")


class Field:
    def __init__(self, name, spec, comment=""):
        self.name = name
        self.spec = spec
        self.comment = comment


class MsgDef:
    def __init__(self, package, kind, name):
        self.package = package
        self.kind = kind
        self.name = name
        self.fields = []
        self.constants = []
        self.deps = set()


def _strip_comment(line):
    out = []
    in_str = False
    quote = ""
    for ch in line:
        if in_str:
            out.append(ch)
            if ch == quote:
                in_str = False
        elif ch in "'\"":
            in_str = True
            quote = ch
            out.append(ch)
        elif ch == "#":
            break
        else:
            out.append(ch)
    return "".join(out).strip()


def parse_msg(text, package, name, kind="msg"):
    """Parse a ``.msg`` body into a :class:`MsgDef`."""
    d = MsgDef(package, kind, name)
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if not line:
            continue
        # constant:  TYPE NAME=value
        if "=" in line and not line.split("=")[0].strip().endswith("]"):
            head, _, value = line.partition("=")
            parts = head.split()
            if len(parts) == 2 and parts[0].split("[")[0] in PRIMITIVES:
                d.constants.append((parts[1].strip(), value.strip()))
                continue
        parts = line.split(None, 1)
        if len(parts) < 2:
            continue
        type_tok, rest = parts[0], parts[1]
        fname = rest.split()[0]
        # a field default (e.g. "int32 x 5") is parsed but not emitted; the
        # CDR default of 0/""/[] is what the wire needs.
        spec = _resolve_type(type_tok, package, d)
        d.fields.append(Field(fname, spec))
    return d


def _resolve_type(tok, package, d):
    m = ARRAY_RE.match(tok)
    if m:
        base, bounded, size = m.group(1), m.group(2), m.group(3)
        elem = _resolve_scalar(base, package, d)
        if size and not bounded:
            return ("a", elem, int(size))
        return ("q", elem)
    return _resolve_scalar(tok, package, d)


def _resolve_scalar(tok, package, d):
    bs = BOUNDED_STR_RE.match(tok)
    if bs:
        return "T"
    if tok in PRIMITIVES:
        return PRIMITIVES[tok]
    # a nested message: "Header", "std_msgs/Header" or "std_msgs/msg/Header"
    parts = tok.split("/")
    if len(parts) == 1:
        pkg, tname = package, parts[0]
    elif len(parts) == 2:
        pkg, tname = parts[0], parts[1]
    else:
        pkg, tname = parts[0], parts[2]
    d.deps.add((pkg, tname))
    return ("m", pkg, tname)


def _spec_literal(spec, package, imports):
    if isinstance(spec, str):
        return repr(spec)
    kind = spec[0]
    if kind == "a":
        return "('a', {}, {})".format(_spec_literal(spec[1], package, imports), spec[2])
    if kind == "q":
        return "('q', {})".format(_spec_literal(spec[1], package, imports))
    if kind == "m":
        pkg, tname = spec[1], spec[2]
        if pkg == package:
            return tname
        imports.add(pkg)
        return "_{}.{}".format(pkg, tname)
    raise ValueError(spec)


def _toposort(defs):
    """Order definitions so a nested type is emitted before its user."""
    by_name = {d.name: d for d in defs}
    out, seen, temp = [], set(), set()

    def visit(d):
        if d.name in seen:
            return
        if d.name in temp:
            return  # cycle: ROS 2 messages are acyclic, but do not hang
        temp.add(d.name)
        for pkg, tname in sorted(d.deps):
            if pkg == d.package and tname in by_name:
                visit(by_name[tname])
        temp.discard(d.name)
        seen.add(d.name)
        out.append(d)

    for d in sorted(defs, key=lambda x: x.name):
        visit(d)
    return out


def generate_module(package, defs):
    defs = _toposort(defs)
    imports = set()
    body = []
    for d in defs:
        lines = ["class {}(Msg):".format(d.name)]
        lines.append('    _package_ = "{}"'.format(d.package))
        lines.append('    _kind_ = "{}"'.format(d.kind))
        lines.append('    _name_ = "{}"'.format(d.name))
        if d.constants:
            for cname, cval in d.constants:
                lines.append("    {} = {}".format(cname, cval))
        if not d.fields:
            lines.append("    _fields_ = ()")
        else:
            lines.append("    _fields_ = (")
            for f in d.fields:
                lines.append(
                    '        ("{}", {}),'.format(
                        f.name, _spec_literal(f.spec, package, imports)
                    )
                )
            lines.append("    )")
        body.append("\n".join(lines))

    head = [
        '"""SnakeROS message definitions for ``{}``.'.format(package),
        "",
        "Generated by tools/snakeros_gen.py -- do not edit by hand.",
        '"""',
        "",
        "from ._base import Msg",
    ]
    for pkg in sorted(imports):
        head.append("from . import {} as _{}".format(pkg, pkg))
    head.append("")
    head.append("")
    return "\n".join(head) + "\n\n\n".join(body) + "\n"


def collect_package(search_path, package, kinds=("msg", "srv")):
    defs = []
    for kind in kinds:
        d = os.path.join(search_path, package, kind)
        if not os.path.isdir(d):
            continue
        for fn in sorted(os.listdir(d)):
            if kind == "msg" and fn.endswith(".msg"):
                text = open(os.path.join(d, fn)).read()
                defs.append(parse_msg(text, package, fn[:-4], "msg"))
            elif kind == "srv" and fn.endswith(".srv"):
                text = open(os.path.join(d, fn)).read()
                req, _, res = text.partition("\n---")
                base = fn[:-4]
                defs.append(parse_msg(req, package, base + "_Request", "srv"))
                defs.append(parse_msg(res, package, base + "_Response", "srv"))
    return defs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("files", nargs="*", help="individual .msg/.srv files")
    ap.add_argument("--search-path", help="a ROS 2 share/ directory")
    ap.add_argument("--package", required=True)
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--only", help="comma-separated list of type names to keep")
    args = ap.parse_args(argv)

    defs = []
    if args.search_path:
        defs.extend(collect_package(args.search_path, args.package))
    for path in args.files:
        text = open(path).read()
        base = os.path.basename(path)
        name, ext = os.path.splitext(base)
        if ext == ".srv":
            req, _, res = text.partition("\n---")
            defs.append(parse_msg(req, args.package, name + "_Request", "srv"))
            defs.append(parse_msg(res, args.package, name + "_Response", "srv"))
        else:
            defs.append(parse_msg(text, args.package, name, "msg"))

    if args.only:
        keep = set(args.only.split(","))
        # keep requested types plus anything they depend on within the package
        by_name = {d.name: d for d in defs}
        need, stack = set(), list(keep)
        while stack:
            n = stack.pop()
            if n in need or n not in by_name:
                continue
            need.add(n)
            for pkg, tname in by_name[n].deps:
                if pkg == args.package:
                    stack.append(tname)
        defs = [d for d in defs if d.name in need]

    if not defs:
        print("no definitions found", file=sys.stderr)
        return 1

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, args.package + ".py")
    with open(path, "w") as f:
        f.write(generate_module(args.package, defs))
    print("{}: {} types -> {}".format(args.package, len(defs), path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
