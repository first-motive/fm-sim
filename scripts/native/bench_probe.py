"""Measure one running sim from inside its ROS graph, for scripts/native/bench.sh.

Started beside the launch, it waits for the sim clock and every camera's compressed
topic, then measures for a fixed window and prints one JSON line:

  ready_s     wall seconds from the probe's start to the first frame on every camera
  rtf         sim seconds per wall second over the window, from /clock
  camera_hz   frames per wall second per /cam_<role>/image_raw/compressed topic

Usage: python3 bench_probe.py <camera count> <window seconds> <timeout seconds>
It takes its input from argv alone, so the container runs it from stdin
(``docker exec -i ... python3 - <args> < bench_probe.py``) and the native runtime
runs it as a file.
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage

CAMERA_PREFIX = '/cam_'
CAMERA_SUFFIX = '/image_raw/compressed'
DISCOVERY_PERIOD_S = 0.5


def sim_seconds(msg):
    return msg.clock.sec + msg.clock.nanosec * 1e-9


class Probe(Node):

    def __init__(self):
        super().__init__('fm_sim_bench_probe')
        # The first and the latest (wall, sim) /clock sample: all the RTF needs.
        self.clock_first = None
        self.clock_last = None
        self.frames = {}
        self.create_subscription(Clock, '/clock', self.on_clock, 10)

    def on_clock(self, msg):
        sample = (time.monotonic(), sim_seconds(msg))
        self.clock_first = self.clock_first or sample
        self.clock_last = sample

    def reset_window(self):
        self.clock_first = self.clock_last = None
        for frames in self.frames.values():
            frames.clear()

    def discover_cameras(self):
        for name, _ in self.get_topic_names_and_types():
            is_camera = name.startswith(CAMERA_PREFIX) and name.endswith(CAMERA_SUFFIX)
            if is_camera and name not in self.frames:
                self.frames[name] = []
                self.create_subscription(
                    CompressedImage, name,
                    lambda _msg, topic=name: self.frames[topic].append(time.monotonic()),
                    qos_profile_sensor_data,
                )

    def ready(self, cameras):
        return (
            self.clock_last is not None
            and len(self.frames) >= cameras
            and all(self.frames.values())
        )


def spin_until(node, done, deadline):
    last_discovery = 0.0
    while not done():
        now = time.monotonic()
        if now > deadline:
            return False
        if now - last_discovery > DISCOVERY_PERIOD_S:
            node.discover_cameras()
            last_discovery = now
        rclpy.spin_once(node, timeout_sec=0.05)
    return True


def main():
    cameras, window_s, timeout_s = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    start = time.monotonic()
    rclpy.init()
    node = Probe()
    if not spin_until(node, lambda: node.ready(cameras), start + timeout_s):
        print(json.dumps({'error': f'not ready in {timeout_s:.0f} s', 'cameras': sorted(node.frames)}))
        sys.exit(1)
    ready_s = time.monotonic() - start
    # A second sim on this domain would merge into the graph and inflate every rate.
    clocks = len(node.get_publishers_info_by_topic('/clock'))
    if clocks != 1:
        print(json.dumps({'error': f'{clocks} /clock publishers on this domain; expected one sim'}))
        sys.exit(1)

    window_start = time.monotonic()
    node.reset_window()
    spin_until(node, lambda: time.monotonic() - window_start >= window_s, float('inf'))
    window_end = time.monotonic()

    if node.clock_first is None or node.clock_first == node.clock_last:
        print(json.dumps({'error': 'no /clock in the window'}))
        sys.exit(1)
    (wall0, sim0), (wall1, sim1) = node.clock_first, node.clock_last
    print(json.dumps({
        'ready_s': round(ready_s, 1),
        'rtf': round((sim1 - sim0) / (wall1 - wall0), 3),
        'camera_hz': {
            topic: round(len(frames) / (window_end - window_start), 2)
            for topic, frames in sorted(node.frames.items())
        },
    }))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
