"""Bring up WiFi before anything else. Copy this to the device as boot.py.

MicroPython runs boot.py before main.py and before any of your imports, which
is exactly what a memory-tight board needs: the WiFi radio wants a large
contiguous allocation, and it will fail with "Wifi Out of Memory" if the heap
is already full of SnakeROS and its message packs.

On an ESP32 with ~178 KB of heap this is not optional -- importing first and
connecting afterwards reliably fails.
"""

SSID = "your-wifi"
PASSWORD = "your-password"

import gc
import network
import time

wlan = network.WLAN(network.STA_IF)
gc.collect()
wlan.active(True)
if not wlan.isconnected():
    wlan.connect(SSID, PASSWORD)
    t0 = time.time()
    while not wlan.isconnected() and time.time() - t0 < 20:
        time.sleep(0.25)

if wlan.isconnected():
    print("WiFi up:", wlan.ifconfig()[0])
else:
    print("WiFi FAILED -- check SSID/password")

gc.collect()

# Collect early rather than growing the heap. This is the single most useful
# line on an ESP32: MicroPython's GC heap grows by claiming blocks from the
# ESP-IDF heap -- the same heap lwIP takes its network buffers from -- and it
# never gives them back. Setting a threshold here, before any heavy import,
# keeps the Python heap small and leaves lwIP room to breathe.
gc.threshold(gc.mem_alloc() + gc.mem_free() // 4)

print("free heap after WiFi:", gc.mem_free())
try:
    import esp32
    # (total, free, largest_free, min_free) -- 'largest_free' is what lwIP
    # needs, and a send fails with ENOMEM when it gets small regardless of
    # what gc.mem_free() says.
    print("idf heap:", esp32.idf_heap_info(esp32.HEAP_DATA)[-1])
except ImportError:
    pass
