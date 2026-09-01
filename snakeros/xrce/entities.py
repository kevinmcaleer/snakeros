"""XRCE entity identity, ROS 2 name mangling, and entity XML.

This module is where the project's central trick lives: XRCE describes its
entities with **XML strings built at runtime**, so supporting a new ROS 2
message type never requires regenerating C or reflashing a board. Everything
here is string manipulation.

The XML templates and the name-mangling rules are matched to what
``rmw_microxrcedds`` itself emits, so the Agent sees exactly what it would see
from a C micro-ROS client.
"""

from .const import (
    OBJK_PARTICIPANT,
    OBJK_TOPIC,
    OBJK_PUBLISHER,
    OBJK_SUBSCRIBER,
    OBJK_DATAWRITER,
    OBJK_DATAREADER,
    OBJK_REQUESTER,
    OBJK_REPLIER,
)

# ROS 2 -> DDS name prefixes. See design.ros2.org/articles/topic_and_service_names
TOPIC_PREFIX = "rt"
REQUEST_PREFIX = "rq"
REPLY_PREFIX = "rr"
REQUEST_SUFFIX = "Request"
REPLY_SUFFIX = "Reply"


# -- object ids ----------------------------------------------------------


def object_id(id_, kind):
    """Pack a 12-bit id and 4-bit kind into the 2-byte XRCE ObjectId.

    Layout per ``uxr_object_id_to_raw``: high 8 bits of the id in byte 0, the
    low 4 bits shifted up in byte 1, kind in the low nibble.
    """
    return bytes(((id_ >> 4) & 0xFF, ((id_ & 0x0F) << 4) | (kind & 0x0F)))


def parse_object_id(raw):
    """Inverse of :func:`object_id`; returns ``(id, kind)``."""
    return ((raw[0] << 4) | (raw[1] >> 4), raw[1] & 0x0F)


class ObjectIdAllocator:
    """Hands out unique ids per entity kind.

    XRCE scopes ids by kind, so a participant and a topic may share a numeric
    id without colliding. Ids are 12-bit.
    """

    def __init__(self):
        self._next = {}

    def alloc(self, kind):
        n = self._next.get(kind, 0)
        if n > 0x0FFF:
            raise ValueError("exhausted object ids for kind 0x{:02x}".format(kind))
        self._next[kind] = n + 1
        return object_id(n, kind)


# -- ROS 2 <-> DDS naming ------------------------------------------------


def _leading_slash(name):
    return name if name.startswith("/") else "/" + name


def mangle_topic(topic):
    """``chatter`` or ``/chatter`` -> ``rt/chatter``."""
    return TOPIC_PREFIX + _leading_slash(topic)


def mangle_service_request(service):
    """``/add`` -> ``rq/addRequest``."""
    return REQUEST_PREFIX + _leading_slash(service) + REQUEST_SUFFIX


def mangle_service_reply(service):
    """``/add`` -> ``rr/addReply``."""
    return REPLY_PREFIX + _leading_slash(service) + REPLY_SUFFIX


def dds_type_name(package, kind, name):
    """Build the DDS type name, e.g. ``std_msgs::msg::dds_::String_``.

    ``kind`` is ``msg`` or ``srv``. Matches ``generate_type_name`` in
    rmw_microxrcedds: ``<namespace>::dds_::<Name>_``.
    """
    return "{}::{}::dds_::{}_".format(package, kind, name)


# -- XML representations -------------------------------------------------


def _xml_escape(s):
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def participant_xml(name):
    return (
        "<dds>"
        "<participant>"
        "<rtps>"
        "<name>" + _xml_escape(name) + "</name>"
        "</rtps>"
        "</participant>"
        "</dds>"
    )


def topic_xml(dds_topic_name, type_name):
    return (
        "<dds>"
        "<topic>"
        "<name>" + _xml_escape(dds_topic_name) + "</name>"
        "<dataType>" + _xml_escape(type_name) + "</dataType>"
        "</topic>"
        "</dds>"
    )


def publisher_xml():
    # rmw_microxrcedds sends an empty representation for publishers and
    # subscribers -- they carry no configuration of their own.
    return ""


def subscriber_xml():
    return ""


def _endpoint_xml(tag, dds_topic_name, type_name, reliable, history_depth):
    kind = "RELIABLE" if reliable else "BEST_EFFORT"
    hist = (
        "<historyQos><kind>KEEP_ALL</kind></historyQos>"
        if history_depth is None
        else "<historyQos><kind>KEEP_LAST</kind><depth>"
        + str(history_depth)
        + "</depth></historyQos>"
    )
    return (
        "<dds>"
        "<" + tag + ">"
        "<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>"
        "<qos><reliability><kind>" + kind + "</kind></reliability></qos>"
        "<topic>"
        "<kind>NO_KEY</kind>"
        "<name>" + _xml_escape(dds_topic_name) + "</name>"
        "<dataType>" + _xml_escape(type_name) + "</dataType>"
        + hist +
        "</topic>"
        "</" + tag + ">"
        "</dds>"
    )


def datawriter_xml(dds_topic_name, type_name, reliable=False, history_depth=None):
    return _endpoint_xml("data_writer", dds_topic_name, type_name, reliable, history_depth)


def datareader_xml(dds_topic_name, type_name, reliable=False, history_depth=None):
    return _endpoint_xml("data_reader", dds_topic_name, type_name, reliable, history_depth)


__all__ = [
    "object_id",
    "parse_object_id",
    "ObjectIdAllocator",
    "mangle_topic",
    "mangle_service_request",
    "mangle_service_reply",
    "dds_type_name",
    "participant_xml",
    "topic_xml",
    "publisher_xml",
    "subscriber_xml",
    "datawriter_xml",
    "datareader_xml",
    "OBJK_PARTICIPANT",
    "OBJK_TOPIC",
    "OBJK_PUBLISHER",
    "OBJK_SUBSCRIBER",
    "OBJK_DATAWRITER",
    "OBJK_DATAREADER",
    "OBJK_REQUESTER",
    "OBJK_REPLIER",
]
