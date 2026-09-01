"""Unit tests that need no Agent and no Docker.

Runs on the MicroPython Unix port in well under a second, so it is the first
thing CI does and the fastest way to catch a regression locally.
"""

import sys

sys.path.insert(0, ".")

from snakeros.cdr import CDRWriter, CDRReader
from snakeros.errors import CDRTruncated, CDRError
from snakeros.xrce import entities as E
from snakeros.xrce import const as C
from snakeros.msg._base import Msg

_fails = []


def check(cond, what):
    if not cond:
        _fails.append(what)


def eq(got, want, what):
    if got != want:
        _fails.append("{}: got {!r}, want {!r}".format(what, got, want))


# -- CDR primitives ------------------------------------------------------

w = CDRWriter()
w.u8(1)
w.f64(2.5)
eq(len(w), 16, "u8 then f64 pads to 8-alignment")
eq(w.bytes()[1:8], b"\x00" * 7, "padding is zeroed")

w = CDRWriter()
w.string("hello")
eq(w.bytes(), b"\x06\x00\x00\x00hello\x00", "string length includes the null")

w = CDRWriter()
w.string("")
eq(w.bytes(), b"\x01\x00\x00\x00\x00", "empty string is length 1 plus null")

# round-trip every primitive
w = CDRWriter()
vals = [("u8", 200), ("i8", -100), ("u16", 65000), ("i16", -30000),
        ("u32", 4000000000), ("i32", -2000000000),
        ("u64", 18000000000000000000), ("i64", -9000000000000000000),
        ("f32", 1.5), ("f64", -2.718281828459045), ("boolean", True)]
for name, v in vals:
    getattr(w, name)(v)
r = CDRReader(w.bytes())
for name, v in vals:
    got = getattr(r, name)()
    if name == "f32":
        check(abs(got - v) < 1e-6, "f32 round-trip")
    else:
        eq(got, v, name + " round-trip")
eq(r.remaining(), 0, "reader consumed exactly the buffer")

# endianness flag is honoured
w = CDRWriter()
w.encapsulation()
eq(w.bytes(), b"\x00\x01\x00\x00", "CDR_LE encapsulation header")
r = CDRReader(b"\x00\x00\x00\x00\x00\x00\x00\x2a")
r.encapsulation()
check(not r.little_endian, "big-endian encapsulation detected")
eq(r.u32(), 42, "big-endian u32 decoded")

# -- bounds checking -----------------------------------------------------

try:
    CDRReader(b"\x05\x00\x00\x00ab").string()
    _fails.append("truncated string did not raise")
except CDRTruncated:
    pass

try:
    CDRReader(b"\xff\xff\xff\xff").string()
    _fails.append("implausible length did not raise")
except CDRError:
    pass

try:
    CDRReader(b"\x01").u32()
    _fails.append("short read did not raise")
except CDRTruncated:
    pass

# -- object ids ----------------------------------------------------------

for oid in (0, 1, 15, 16, 300, 0xFFF):
    for kind in (C.OBJK_PARTICIPANT, C.OBJK_DATAWRITER, C.OBJK_DATAREADER):
        raw = E.object_id(oid, kind)
        eq(len(raw), 2, "object id is 2 bytes")
        eq(E.parse_object_id(raw), (oid, kind), "object id round-trip {}".format(oid))

alloc = E.ObjectIdAllocator()
a = alloc.alloc(C.OBJK_TOPIC)
b = alloc.alloc(C.OBJK_TOPIC)
check(a != b, "allocator hands out distinct ids")
eq(E.parse_object_id(alloc.alloc(C.OBJK_PARTICIPANT))[0], 0,
   "ids are scoped per kind")

# -- name mangling -------------------------------------------------------

eq(E.mangle_topic("chatter"), "rt/chatter", "topic without slash")
eq(E.mangle_topic("/chatter"), "rt/chatter", "topic with slash")
eq(E.mangle_topic("/ns/thing"), "rt/ns/thing", "namespaced topic")
eq(E.mangle_service_request("/add"), "rq/addRequest", "service request name")
eq(E.mangle_service_reply("/add"), "rr/addReply", "service reply name")
eq(E.dds_type_name("std_msgs", "msg", "String"),
   "std_msgs::msg::dds_::String_", "dds type name")

# -- XML -----------------------------------------------------------------

x = E.topic_xml("rt/chatter", "std_msgs::msg::dds_::String_")
check("<name>rt/chatter</name>" in x, "topic xml carries the name")
check("dds_::String_" in x, "topic xml carries the type")
check(E.publisher_xml() == "", "publisher representation is empty")
check("BEST_EFFORT" in E.datawriter_xml("rt/x", "t", False),
      "best-effort writer xml")
check("RELIABLE" in E.datawriter_xml("rt/x", "t", True), "reliable writer xml")
check("&amp;" in E.topic_xml("rt/a&b", "t"), "xml escaping")

# -- message system ------------------------------------------------------

from snakeros.msg.geometry_msgs import Twist, Vector3  # noqa: E402
from snakeros.msg.std_msgs import String, Header  # noqa: E402
from snakeros.msg.sensor_msgs import Imu  # noqa: E402

eq(Vector3._fast(), "<ddd", "Vector3 compiles to a single struct format")
check(Twist._fast() is None, "nested message has no fast path")
check(String._fast() is None, "string message has no fast path")

t = Twist()
t.linear.x = 0.5
t.angular.z = -1.25
eq(len(t.serialize()), 48, "Twist is 48 bytes")
eq(Twist.deserialize(t.serialize()), t, "Twist round-trip")

s = String(data="hi")
eq(String.deserialize(s.serialize()).data, "hi", "String round-trip")
eq(String.type_name(), "std_msgs::msg::dds_::String_", "String type name")
eq(String.ros_name(), "std_msgs/msg/String", "String ros name")

imu = Imu()
eq(len(imu.orientation_covariance), 9, "covariance array default length")
imu.orientation_covariance = [float(i) for i in range(9)]
eq(Imu.deserialize(imu.serialize()).orientation_covariance,
   [float(i) for i in range(9)], "covariance round-trip")

# constants come through from the .msg
from snakeros.msg.sensor_msgs import BatteryState  # noqa: E402
eq(BatteryState.POWER_SUPPLY_STATUS_CHARGING, 1, "message constants generated")

# unknown field is rejected
try:
    String(nope=1)
    _fails.append("unknown field accepted")
except TypeError:
    pass

# -- report --------------------------------------------------------------

if _fails:
    for f in _fails:
        print("FAIL:", f)
    print("{} failure(s)".format(len(_fails)))
    sys.exit(1)
print("unit tests: all passed")
