---
title: SnakeROS
---

# SnakeROS

**A pure-Python ROS 2 client for MicroPython.**

Publish and subscribe to real ROS 2 topics from a Pico 2 W, a Pico W or an
ESP32. No C toolchain. No colcon. No custom firmware. No reflashing to add a
message type.

```python
from snakeros import Node
from snakeros.msg.std_msgs import String

node = Node('pico_node', agent='192.168.1.10')
pub = node.create_publisher(String, 'chatter')
pub.publish(String(data='hello from a Pico'))
```

```console
$ ros2 topic echo /chatter
data: hello from a Pico
```

> **Not affiliated with micro-ROS.** SnakeROS is an independent project, not
> endorsed by [micro-ROS](https://micro.ros.org/), eProsima or Open Robotics.
> It is *compatible with* the micro-ROS Agent, which is a different claim.

## Ten minutes from nothing to a published message

### 1. Run the Agent

The Agent is stock and unmodified — SnakeROS ships no host-side software.
Docker is the shortest path, and this is where most people get stuck first:

```console
$ docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v4
```

If `--net=host` is unavailable (macOS, Windows), publish the port instead:

```console
$ docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:jazzy udp4 --port 8888 -v4
```

Leave it running. `-v4` prints entity creation, which is what you want the
first time.

### 2. Install SnakeROS on the board

```python
import mip
mip.install('github:kevinmcaleer/snakeros')
```

Optional message packs are separate, so you only pay heap for what you use:

```python
mip.install('github:kevinmcaleer/snakeros/packages/sensor_msgs.json')
```

See [Packaging](packaging.md) for the smaller `.mpy` build.

### 3. Connect and publish

```python
from snakeros.board import connect_wifi
from snakeros import Node
from snakeros.msg.std_msgs import String

connect_wifi('my-wifi', 'secret')
node = Node('pico_node', agent='192.168.1.10')   # your PC's LAN address
pub = node.create_publisher(String, 'chatter')

while True:
    pub.publish(String(data='hello'))
    node.spin_once(10)
```

### 4. See it

```console
$ ros2 topic list
/chatter
$ ros2 topic echo /chatter
data: hello
```

If any of that failed, [Troubleshooting](troubleshooting.md) is written from
the actual bugs hit building this, and `tools/bringup.py` will tell you which
layer is broken.

## Where next

| Page | |
|---|---|
| [Architecture](architecture.md) | How it works, and the idea that makes it possible |
| [API reference](api.md) | Every public class and method |
| [Messages](messages.md) | What ships, and generating your own from `.msg` |
| [Transports](transports.md) | UDP vs serial, and the REPL conflict |
| [Memory and performance](memory.md) | What things cost, with real numbers |
| [Troubleshooting](troubleshooting.md) | When it does not work |
| [Limitations](limitations.md) | What SnakeROS deliberately does not do |
| [Hardware bring-up](bringup.md) | Getting onto a real board |
| [Packaging](packaging.md) | `mip`, `.mpy`, releases |

## Licence

Apache 2.0.
