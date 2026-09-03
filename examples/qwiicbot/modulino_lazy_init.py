"""A lazy drop-in replacement for ``modulino/__init__.py``.

Arduino's ``arduino-modulino-mpy`` package eagerly imports **all seventeen**
drivers when you touch it -- roughly 131 KB of source -- even if you only use
five. On an ESP32 with SnakeROS already loaded, that import is what fails:

    File "/lib/modulino/__init__.py", line 11, in <module>
    MemoryError: memory allocation failed, allocating 584 bytes

This replacement exposes exactly the same names, but resolves each on first
use (MicroPython supports module ``__getattr__``, PEP 562). Importing
``ModulinoMotors`` then loads ``motors.py`` and the base class, and nothing
else.

To use it::

    mpremote fs cp examples/qwiicbot/modulino_lazy_init.py :lib/modulino/__init__.py

Nothing else about the library changes, and the unused drivers stay on flash
in case you want them later. If flash is also tight, they can simply be
deleted -- with this __init__ nothing references them until you ask.

Derived from arduino-modulino-mpy (MPL-2.0, Arduino / Sebastian Romero).
"""

__version__ = "1.0.0"
__author__ = "Sebastian Romero"
__license__ = "MPL 2.0"
__maintainer__ = "Arduino"

# exported name -> the submodule that defines it
_LAZY = {
    "map_value": "helpers",
    "map_value_int": "helpers",
    "constrain": "helpers",
    "Modulino": "modulino",
    "ModulinoHub": "hub",
    "ModulinoHubPort": "hub",
    "DeviceManager": "device_manager",
    "ModulinoPixels": "pixels",
    "ModulinoColor": "pixels",
    "ModulinoThermo": "thermo",
    "ModulinoBuzzer": "buzzer",
    "ModulinoButtons": "buttons",
    "ModulinoKnob": "knob",
    "ModulinoMovement": "movement",
    "ModulinoDistance": "distance",
    "ModulinoJoystick": "joystick",
    "ModulinoLatchRelay": "latch_relay",
    "ModulinoVibro": "vibro",
    "PowerLevel": "vibro",
    "ModulinoLEDMatrix": "led_matrix",
    "MPJAnimation": "led_matrix",
    "FPSAnimation": "led_matrix",
    "Animation": "led_matrix",
    "ModulinoLight": "light",
    "ModulinoMotors": "motors",
    "DecayMode": "motors",
}

__all__ = sorted(_LAZY)


def __getattr__(name):
    where = _LAZY.get(name)
    if where is None:
        raise AttributeError("module 'modulino' has no attribute '%s'" % name)
    module = __import__("modulino." + where, None, None, (where,))
    value = getattr(module, name)
    globals()[name] = value  # cache: this costs once
    return value
