"""Hardware bring-up check for SnakeROS.

Run this on a board *first*, before anything else. It works through the stack
one layer at a time and stops at the first thing that is actually wrong,
rather than leaving you to guess from a silent failure.

    mpremote connect /dev/tty.usbmodem1101 run tools/bringup.py

Or, on the board:

    import bringup; bringup.main(ssid="my-wifi", password="secret",
                                 agent="192.168.1.10")
"""

import gc
import sys
import time


def _ok(msg):
    print("  PASS  " + msg)


def _bad(msg):
    print("  FAIL  " + msg)


def main(ssid=None, password=None, agent="192.168.1.10", port=8888):
    fails = 0
    print("=" * 58)
    print("SnakeROS bring-up")
    print("=" * 58)

    # 1. platform
    print("\n-- platform --")
    print("  ", sys.implementation)
    gc.collect()
    free = gc.mem_free()
    total = free + gc.mem_alloc()
    print("   heap: {:,} free of {:,}".format(free, total))
    if total < 100000:
        print("   NOTE: under 100 KB of heap. Import only the message packs")
        print("         you need, and consider installing the .mpy build.")

    # 2. shadowing check -- before anything else, because a stale copy makes
    #    every later symptom lie about its cause
    print("\n-- install --")
    try:
        import os as _os
        import sys as _sys

        found = []
        for entry in _sys.path:
            d = (entry or ".") + "/snakeros"
            try:
                _os.stat(d)
                found.append(d)
            except OSError:
                pass
        if len(found) > 1:
            _bad("snakeros found in {} places: {}".format(len(found), found))
            print("        The first on sys.path wins, and MicroPython searches")
            print("        the working directory BEFORE /lib. Delete the stale")
            print("        copy -- a partial one silently overrides a good one.")
            fails += 1
        elif found:
            _ok("single install at {}".format(found[0]))
        else:
            _bad("snakeros not found on sys.path: {}".format(_sys.path))
            return 1
    except Exception as e:
        print("   (could not check for shadowing: {})".format(e))

    # 10. import
    print("\n-- import --")
    try:
        import snakeros
        gc.collect()
        _ok("snakeros {} imported, {:,} bytes free".format(
            snakeros.__version__, gc.mem_free()))
    except Exception as e:
        _bad("import failed: {}".format(e))
        return 1

    # 3. message packs
    print("\n-- message packs --")
    for pack in ("std_msgs", "geometry_msgs", "sensor_msgs"):
        before = gc.mem_free()
        try:
            __import__("snakeros.msg." + pack, None, None, (pack,))
            gc.collect()
            _ok("{:<16} {:>7,} bytes".format(pack, before - gc.mem_free()))
        except Exception as e:
            _bad("{}: {}".format(pack, e))
            fails += 1

    # 4. serialisation without any network
    print("\n-- serialisation (no network needed) --")
    try:
        from snakeros.msg.geometry_msgs import Twist

        t = Twist()
        t.linear.x = 0.5
        blob = t.serialize()
        back = Twist.deserialize(blob)
        if len(blob) == 48 and abs(back.linear.x - 0.5) < 1e-9:
            _ok("Twist round-trips ({} bytes)".format(len(blob)))
        else:
            _bad("Twist round-trip wrong: {} bytes".format(len(blob)))
            fails += 1
    except Exception as e:
        _bad("serialisation failed: {}".format(e))
        fails += 1

    # 5. wifi
    if ssid:
        print("\n-- wifi --")
        try:
            from snakeros.board import connect_wifi

            ip = connect_wifi(ssid, password, verbose=False)
            _ok("associated, address {}".format(ip))
        except Exception as e:
            _bad("wifi: {}".format(e))
            return 1
    else:
        print("\n-- wifi --\n   skipped (no ssid given)")

    # 6. the Agent
    print("\n-- agent at {}:{} --".format(agent, port))
    try:
        from snakeros import Node

        t0 = time.time()
        node = Node("snakeros_bringup", agent=agent, port=port)
        _ok("session established in {:.2f} s".format(time.time() - t0))
    except Exception as e:
        _bad("could not reach the Agent: {}".format(e))
        print("\n   Check: is it running?  ros2 run micro_ros_agent "
              "micro_ros_agent udp4 --port {}".format(port))
        print("   Check: is {} reachable from the board?".format(agent))
        return 1

    # 7. publish
    print("\n-- publish --")
    try:
        from snakeros.msg.std_msgs import String

        pub = node.create_publisher(String, "bringup")
        for i in range(20):
            pub.publish(String(data="bringup %d" % i))
            node.spin_once(10)
            time.sleep(0.05)
        _ok("published 20 samples to /bringup")
        print("        verify with:  ros2 topic echo /bringup")
    except Exception as e:
        _bad("publish failed: {}".format(e))
        fails += 1

    # 8. rate
    print("\n-- publish rate --")
    try:
        from snakeros.msg.geometry_msgs import Twist

        p2 = node.create_publisher(Twist, "bringup_rate")
        msg = Twist()
        gc.collect()
        t0 = time.time()
        n = 200
        for _ in range(n):
            p2.publish(msg)
        dt = time.time() - t0
        print("   {:.0f} Twist/s ({:.0f} us each)".format(n / dt, dt / n * 1e6))
    except Exception as e:
        _bad("rate test: {}".format(e))

    # 9. memory after a soak
    print("\n-- memory --")
    gc.collect()
    before = gc.mem_free()
    try:
        from snakeros.msg.sensor_msgs import Imu

        p3 = node.create_publisher(Imu, "bringup_imu")
        imu = Imu()
        for _ in range(300):
            p3.publish(imu)
            node.spin_once(2)
        gc.collect()
        after = gc.mem_free()
        drift = before - after
        print("   free before {:,}, after 300 publishes {:,}".format(before, after))
        if drift > 2000:
            _bad("heap dropped {:,} bytes -- possible leak or fragmentation".format(drift))
            fails += 1
        else:
            _ok("heap stable (drift {:,} bytes)".format(drift))
    except Exception as e:
        _bad("soak: {}".format(e))
        fails += 1

    node.destroy()
    print("\n" + "=" * 58)
    print("bring-up: {}".format("ALL PASSED" if not fails else
                                "{} FAILURE(S)".format(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    main()
