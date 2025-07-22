#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class SensorManagerTogglePublisher(Node):
    def __init__(self):
        super().__init__('sensor_manager_toggle_test')
        self.publisher_ = self.create_publisher(Bool, 'cmd_sensor_manager', 10)

    def publish_toggle_messages(self, count=10000, interval_sec=1.0):
        value = True
        for i in range(count):
            msg = Bool()
            msg.data = value
            self.publisher_.publish(msg)
            self.get_logger().info(f'[{i+1}/{count}] Published: {value}')
            value = not value
            time.sleep(interval_sec)
        self.get_logger().info('✅ Finished publishing toggle messages.')

def main(args=None):
    rclpy.init(args=args)
    node = SensorManagerTogglePublisher()
    try:
        node.publish_toggle_messages()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
