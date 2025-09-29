#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
import time
from collections import deque

# 모니터링할 토픽
TOPICS = [
    ('/scan', LaserScan),
    # ('/odom', Odometry),
]


class SlidingWindowHzMonitor(Node):
    def __init__(self):
        super().__init__('sliding_window_hz_monitor')
        self.topic_data = {}

        for name, msg_type in TOPICS:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            sub = self.create_subscription(msg_type, name, lambda msg, n=name: self.callback(n), qos)
            # deque: 각 메시지 도착 시간 저장
            self.topic_data[name] = deque()

        self.timer = self.create_timer(1.0, self.log_hz)

    def callback(self, topic_name):
        now = time.time()
        self.topic_data[topic_name].append(now)

    def log_hz(self):
        now = time.time()
        for name, times in self.topic_data.items():
            # 1초 이전 메시지 제거
            while times and now - times[0] > 1.0:
                times.popleft()
            elapsed = now - (times[0] if times else now)
            hz = len(times) / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(f"Current {name} Hz: {hz:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = SlidingWindowHzMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
