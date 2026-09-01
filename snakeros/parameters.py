"""ROS 2 parameters.

Lets a board's tuning values -- wheel radius, PID gains, publish rates -- be
read and changed at runtime with ``ros2 param``, instead of by editing source
and reflashing. Live PID tuning without a reflash is the demo that sells this.

ROS 2 implements parameters as services on the node, so this builds on
:mod:`snakeros.services`.

Memory cost
-----------
A declared parameter costs roughly 120-200 bytes (name, value, descriptor and
dict overhead). The five parameter services together cost about the same as
five ordinary services. On a Pico W that is worth counting before declaring
thirty of them.
"""

from .errors import ParameterError
from .msg.rcl_interfaces import (
    DescribeParameters,
    GetParameterTypes,
    GetParameters,
    ListParameters,
    ParameterDescriptor,
    ParameterValue,
    SetParameters,
    SetParametersAtomically,
    SetParametersResult,
)

# rcl_interfaces/msg/ParameterType
NOT_SET = 0
BOOL = 1
INTEGER = 2
DOUBLE = 3
STRING = 4
BYTE_ARRAY = 5
BOOL_ARRAY = 6
INTEGER_ARRAY = 7
DOUBLE_ARRAY = 8
STRING_ARRAY = 9

_TYPE_NAMES = {
    NOT_SET: "not set", BOOL: "bool", INTEGER: "integer", DOUBLE: "double",
    STRING: "string", BYTE_ARRAY: "byte array", BOOL_ARRAY: "bool array",
    INTEGER_ARRAY: "integer array", DOUBLE_ARRAY: "double array",
    STRING_ARRAY: "string array",
}


def infer_type(value):
    if isinstance(value, bool):
        return BOOL
    if isinstance(value, int):
        return INTEGER
    if isinstance(value, float):
        return DOUBLE
    if isinstance(value, str):
        return STRING
    if isinstance(value, (bytes, bytearray)):
        return BYTE_ARRAY
    if isinstance(value, (list, tuple)):
        if not value:
            return STRING_ARRAY
        head = value[0]
        if isinstance(head, bool):
            return BOOL_ARRAY
        if isinstance(head, int):
            return INTEGER_ARRAY
        if isinstance(head, float):
            return DOUBLE_ARRAY
        return STRING_ARRAY
    raise ParameterError("cannot infer a ROS parameter type for {!r}".format(value))


def to_value(value, ptype=None):
    """Python value -> ``rcl_interfaces/ParameterValue``."""
    if ptype is None:
        ptype = infer_type(value)
    pv = ParameterValue()
    pv.type = ptype
    if ptype == BOOL:
        pv.bool_value = bool(value)
    elif ptype == INTEGER:
        pv.integer_value = int(value)
    elif ptype == DOUBLE:
        pv.double_value = float(value)
    elif ptype == STRING:
        pv.string_value = str(value)
    elif ptype == BYTE_ARRAY:
        pv.byte_array_value = list(value)
    elif ptype == BOOL_ARRAY:
        pv.bool_array_value = [bool(v) for v in value]
    elif ptype == INTEGER_ARRAY:
        pv.integer_array_value = [int(v) for v in value]
    elif ptype == DOUBLE_ARRAY:
        pv.double_array_value = [float(v) for v in value]
    elif ptype == STRING_ARRAY:
        pv.string_array_value = [str(v) for v in value]
    return pv


def from_value(pv):
    """``rcl_interfaces/ParameterValue`` -> Python value."""
    t = pv.type
    if t == BOOL:
        return pv.bool_value
    if t == INTEGER:
        return pv.integer_value
    if t == DOUBLE:
        return pv.double_value
    if t == STRING:
        return pv.string_value
    if t == BYTE_ARRAY:
        return bytes(pv.byte_array_value)
    if t == BOOL_ARRAY:
        return list(pv.bool_array_value)
    if t == INTEGER_ARRAY:
        return list(pv.integer_array_value)
    if t == DOUBLE_ARRAY:
        return list(pv.double_array_value)
    if t == STRING_ARRAY:
        return list(pv.string_array_value)
    return None


class _Param:
    __slots__ = ("name", "value", "type", "description", "min", "max",
                 "read_only", "callback")

    def __init__(self, name, value, description="", minimum=None, maximum=None,
                 read_only=False, callback=None):
        self.name = name
        self.value = value
        self.type = infer_type(value)
        self.description = description
        self.min = minimum
        self.max = maximum
        self.read_only = read_only
        self.callback = callback

    def validate(self, new):
        """Return ``None`` if acceptable, else a human-readable reason."""
        if self.read_only:
            return "parameter '{}' is read-only".format(self.name)
        try:
            nt = infer_type(new)
        except ParameterError as e:
            return str(e)
        # allow int -> double promotion, which is what ros2 param set sends
        if nt != self.type:
            if self.type == DOUBLE and nt == INTEGER:
                new = float(new)
                nt = DOUBLE
            else:
                return "parameter '{}' is {}, not {}".format(
                    self.name, _TYPE_NAMES[self.type], _TYPE_NAMES[nt]
                )
        if self.min is not None and new < self.min:
            return "{} is below the minimum {}".format(new, self.min)
        if self.max is not None and new > self.max:
            return "{} is above the maximum {}".format(new, self.max)
        return None


class ParameterServer:
    """Serves the standard ROS 2 parameter services for a node."""

    def __init__(self, node):
        self.node = node
        self._params = {}
        self._order = []
        base = "/" + node.name.strip("/")
        node.create_service(GetParameters, base + "/get_parameters", self._get)
        node.create_service(SetParameters, base + "/set_parameters", self._set)
        node.create_service(ListParameters, base + "/list_parameters", self._list)
        node.create_service(
            DescribeParameters, base + "/describe_parameters", self._describe
        )
        node.create_service(
            GetParameterTypes, base + "/get_parameter_types", self._types
        )
        # ros2's AsyncParameterClient waits for *all six* parameter services
        # before it will talk to a node. Omit this one and `ros2 param list`
        # times out with "waiting for parameter services" even though every
        # other service works and answers a direct `ros2 service call`.
        node.create_service(
            SetParametersAtomically,
            base + "/set_parameters_atomically",
            self._set_atomically,
        )

    # -- declaration -------------------------------------------------------

    def declare(self, name, default, description="", minimum=None,
                maximum=None, read_only=False, callback=None):
        if name in self._params:
            raise ParameterError("parameter '{}' already declared".format(name))
        p = _Param(name, default, description, minimum, maximum,
                   read_only, callback)
        self._params[name] = p
        self._order.append(name)
        return p

    def get(self, name, default=None):
        p = self._params.get(name)
        return default if p is None else p.value

    def set(self, name, value):
        """Set locally. Returns ``(ok, reason)``."""
        p = self._params.get(name)
        if p is None:
            return False, "parameter '{}' not declared".format(name)
        reason = p.validate(value)
        if reason:
            return False, reason
        if p.type == DOUBLE and isinstance(value, int):
            value = float(value)
        old = p.value
        p.value = value
        if p.callback is not None:
            try:
                if p.callback(value) is False:
                    p.value = old
                    return False, "rejected by the parameter callback"
            except Exception as e:
                p.value = old
                return False, "parameter callback raised: {}".format(e)
        return True, ""

    def names(self):
        return list(self._order)

    # -- service handlers --------------------------------------------------

    def _get(self, req):
        res = GetParameters.Response()
        vals = []
        for n in req.names:
            p = self._params.get(n)
            vals.append(to_value(p.value, p.type) if p else ParameterValue())
        res.values = vals
        return res

    def _set(self, req):
        res = SetParameters.Response()
        results = []
        for param in req.parameters:
            ok, reason = self.set(param.name, from_value(param.value))
            r = SetParametersResult()
            r.successful = ok
            r.reason = reason
            results.append(r)
        res.results = results
        return res

    def _set_atomically(self, req):
        """Apply every parameter or none of them."""
        res = SetParametersAtomically.Response()
        snapshot = {n: self._params[n].value for n in self._params}
        for param in req.parameters:
            ok, reason = self.set(param.name, from_value(param.value))
            if not ok:
                for n, v in snapshot.items():
                    self._params[n].value = v
                res.result.successful = False
                res.result.reason = reason
                return res
        res.result.successful = True
        res.result.reason = ""
        return res

    def _list(self, req):
        res = ListParameters.Response()
        res.result.names = list(self._order)
        res.result.prefixes = []
        return res

    def _describe(self, req):
        res = DescribeParameters.Response()
        out = []
        for n in req.names:
            d = ParameterDescriptor()
            d.name = n
            p = self._params.get(n)
            if p is not None:
                d.type = p.type
                d.description = p.description
                d.read_only = p.read_only
            out.append(d)
        res.descriptors = out
        return res

    def _types(self, req):
        res = GetParameterTypes.Response()
        res.types = [
            self._params[n].type if n in self._params else NOT_SET
            for n in req.names
        ]
        return res
