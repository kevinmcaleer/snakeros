"""Board-specific I2C setup for the QwiicBot.

Some boards gate power to their Qwiic / STEMMA QT connector behind a GPIO, and
some put I2C on pins MicroPython's defaults do not match. Neither is a fault,
and neither is obvious: the modules simply show no power and every I2C call
fails.

The Adafruit ESP32 Feather V2 is the notable case. Its STEMMA QT port has its
**own regulator, enabled by GPIO 2** (`NEOPIXEL_I2C_POWER`). CircuitPython and
Arduino raise that pin automatically as part of board init; **MicroPython does
not**, so with generic firmware the connector is simply unpowered. The
downstream symptom -- dead modules, I2C errors -- looks like a hardware fault
and is a single line of code.

    from board_setup import setup_i2c
    i2c = setup_i2c("feather_esp32_v2")

Then pass the bus to the drivers, since the board's pins are not MicroPython's
defaults either.
"""

try:
    from machine import Pin, SoftI2C
except ImportError:  # the Unix port has no Pin; the tables below still read
    Pin = SoftI2C = None

try:
    from machine import I2C
except ImportError:
    I2C = None

# name -> (i2c power pin or None, sda, scl, active-high?)
BOARDS = {
    # STEMMA QT is on its own regulator behind GPIO 2. SCL is GPIO 20, which
    # MicroPython's hardware I2C does not accept on this chip -- hence SoftI2C.
    "feather_esp32_v2": (2, 22, 20, True),
    # Most generic ESP32 boards: no power gate, conventional pins.
    "generic_esp32": (None, 21, 22, True),
    # Pico / Pico 2 W with a Qwiic breakout on I2C0.
    "pico": (None, 4, 5, True),
}


def enable_i2c_power(pin=2, active_high=True, settle_ms=50):
    """Raise the I2C/STEMMA power-enable pin and let the rail settle.

    Returns the Pin so callers can drop power later for deep sleep, which is
    what the gate exists for.
    """
    import time

    if Pin is None:
        raise RuntimeError("no machine.Pin -- not running on a board")

    p = Pin(pin, Pin.OUT)
    p.value(1 if active_high else 0)
    time.sleep_ms(settle_ms)      # the regulator needs a moment
    return p


def setup_i2c(board="generic_esp32", freq=100000, soft=None):
    """Power the Qwiic rail if needed and return a usable I2C bus.

    ``soft=True`` forces SoftI2C, which is required where the board's SCL is a
    pin the hardware I2C peripheral will not take.
    """
    if board not in BOARDS:
        raise ValueError("unknown board %r -- known: %s"
                         % (board, ", ".join(sorted(BOARDS))))
    power, sda, scl, active_high = BOARDS[board]

    if power is not None:
        enable_i2c_power(power, active_high)
        print("[board] I2C power enabled on GPIO %d" % power)

    if soft is None:
        # Prefer hardware I2C, fall back to SoftI2C when the pins are not
        # acceptable to the peripheral.
        soft = board == "feather_esp32_v2"

    if soft or I2C is None:
        bus = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=freq)
        kind = "SoftI2C"
    else:
        try:
            bus = I2C(0, scl=Pin(scl), sda=Pin(sda), freq=freq)
            kind = "I2C(0)"
        except (ValueError, OSError):
            bus = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=freq)
            kind = "SoftI2C (hardware I2C refused the pins)"

    print("[board] %s on sda=%d scl=%d" % (kind, sda, scl))
    return bus


def scan(bus, verbose=True):
    """Scan the bus and name any Modulinos found.

    An empty scan on a board with a power gate almost always means the gate is
    off, not that the modules are broken.
    """
    found = bus.scan()
    if verbose:
        if not found:
            print("[board] no I2C devices found.")
            print("        If this board gates Qwiic power, that gate is the")
            print("        first thing to check -- unpowered modules scan as")
            print("        an empty bus, exactly like an unplugged cable.")
        else:
            for a in found:
                print("[board] found 0x%02X (%d)" % (a, a))
    return found
