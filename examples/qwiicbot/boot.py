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
print("free heap after WiFi:", gc.mem_free())
