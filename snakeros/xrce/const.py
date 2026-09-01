"""DDS-XRCE protocol constants.

Values verified against eProsima's Micro-XRCE-DDS-Client C sources rather than
against prose summaries -- several of these are easy to get subtly wrong and
fail in ways that look like something else entirely.
"""

# -- session -------------------------------------------------------------

XRCE_COOKIE = b"XRCE"
XRCE_VERSION = b"\x01\x00"
VENDOR_ID_EPROSIMA = b"\x01\x0f"

# Sessions numbered below this carry the 4-byte client key in *every* message
# header; at or above it, the header is 4 bytes. See xrce_header.c.
SESSION_ID_WITHOUT_CLIENT_KEY = 0x80

CLIENT_KEY_SIZE = 4
MIN_HEADER_SIZE = 4
MAX_HEADER_SIZE = MIN_HEADER_SIZE + CLIENT_KEY_SIZE
SUBHEADER_SIZE = 4
CREATE_CLIENT_PAYLOAD_SIZE = 16

# -- streams -------------------------------------------------------------
# 0x00 none; 0x01-0x7F best-effort; 0x80-0xFF reliable.
STREAM_NONE = 0x00
STREAM_BEST_EFFORT = 0x01
STREAM_RELIABLE = 0x80

# -- submessage ids ------------------------------------------------------

SUBMESSAGE_CREATE_CLIENT = 0
SUBMESSAGE_CREATE = 1
SUBMESSAGE_GET_INFO = 2
SUBMESSAGE_DELETE = 3
SUBMESSAGE_STATUS_AGENT = 4
SUBMESSAGE_STATUS = 5
SUBMESSAGE_INFO = 6
SUBMESSAGE_WRITE_DATA = 7
SUBMESSAGE_READ_DATA = 8
SUBMESSAGE_DATA = 9
SUBMESSAGE_ACKNACK = 10
SUBMESSAGE_HEARTBEAT = 11
SUBMESSAGE_RESET = 12
SUBMESSAGE_FRAGMENT = 13
SUBMESSAGE_TIMESTAMP = 14
SUBMESSAGE_TIMESTAMP_REPLY = 15

# -- submessage flags ----------------------------------------------------

FLAG_ENDIANNESS = 0x01  # set => little-endian payload
FLAG_LAST_FRAGMENT = 0x02
FLAG_REUSE = 0x02       # on CREATE
FLAG_REPLACE = 0x04     # on CREATE

FORMAT_DATA = 0x00
FORMAT_SAMPLE = 0x02
FORMAT_DATA_SEQ = 0x08
FORMAT_SAMPLE_SEQ = 0x0A
FORMAT_PACKED_SAMPLES = 0x0E

# -- object kinds --------------------------------------------------------

OBJK_INVALID = 0x00
OBJK_PARTICIPANT = 0x01
OBJK_TOPIC = 0x02
OBJK_PUBLISHER = 0x03
OBJK_SUBSCRIBER = 0x04
OBJK_DATAWRITER = 0x05
OBJK_DATAREADER = 0x06
OBJK_REQUESTER = 0x07
OBJK_REPLIER = 0x08
OBJK_TYPE = 0x0A
OBJK_QOSPROFILE = 0x0B
OBJK_APPLICATION = 0x0C
OBJK_AGENT = 0x0D
OBJK_CLIENT = 0x0E
OBJK_OTHER = 0x0F

# The client object itself, used as the target of DELETE on logout.
OBJECTID_CLIENT = b"\xff\xfe"
OBJECTID_SESSION = b"\xff\xff"

# -- representation formats ---------------------------------------------

REPRESENTATION_BY_REFERENCE = 0x01
REPRESENTATION_AS_XML_STRING = 0x02
REPRESENTATION_IN_BINARY = 0x03

# -- status codes --------------------------------------------------------

STATUS_OK = 0x00
STATUS_OK_MATCHED = 0x01
STATUS_ERR_DDS_ERROR = 0x80
STATUS_ERR_MISMATCH = 0x81
STATUS_ERR_ALREADY_EXISTS = 0x82
STATUS_ERR_DENIED = 0x83
STATUS_ERR_UNKNOWN_REFERENCE = 0x84
STATUS_ERR_INVALID_DATA = 0x85
STATUS_ERR_INCOMPATIBLE = 0x86
STATUS_ERR_RESOURCES = 0x87

STATUS_NAMES = {
    0x00: "OK",
    0x01: "OK_MATCHED",
    0x80: "ERR_DDS_ERROR",
    0x81: "ERR_MISMATCH",
    0x82: "ERR_ALREADY_EXISTS",
    0x83: "ERR_DENIED",
    0x84: "ERR_UNKNOWN_REFERENCE",
    0x85: "ERR_INVALID_DATA",
    0x86: "ERR_INCOMPATIBLE",
    0x87: "ERR_RESOURCES",
}


def status_name(code):
    return STATUS_NAMES.get(code, "UNKNOWN(0x{:02x})".format(code))
