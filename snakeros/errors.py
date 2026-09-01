"""Exception hierarchy for SnakeROS.

Errors are layered so a failure says *which* layer broke: a transport error is
a different problem from a session error, which is different again from the
Agent rejecting an entity.
"""


class SnakeROSError(Exception):
    """Base for every SnakeROS error."""


class TransportError(SnakeROSError):
    """The bytes did not get where they were going."""


class NotConnectedError(TransportError):
    """Transport used before open() or after close()."""


class SessionError(SnakeROSError):
    """XRCE session-level failure."""


class HandshakeError(SessionError):
    """The Agent did not accept CREATE_CLIENT."""


class SessionTimeout(SessionError):
    """The Agent stopped answering."""


class EntityError(SnakeROSError):
    """The Agent rejected an entity creation or deletion."""

    def __init__(self, message, status=None):
        # MicroPython does not support ``BaseClass.__init__(self, ...)`` on a
        # subclass of a built-in exception -- it raises AttributeError, which
        # replaces a clear "the Agent rejected this entity" message with a
        # baffling one at exactly the moment you need the real error. Use
        # super(), which works on both.
        super().__init__(message)
        self.status = status


class CDRError(SnakeROSError):
    """Serialisation or deserialisation failure."""


class CDRTruncated(CDRError):
    """Ran off the end of the buffer while decoding."""


class MessageDefinitionError(SnakeROSError):
    """A message class is malformed."""


class ServiceError(SnakeROSError):
    """Service call failure."""


class ServiceTimeout(ServiceError):
    """No reply within the timeout."""


class ParameterError(SnakeROSError):
    """Parameter declaration, type or range failure."""
