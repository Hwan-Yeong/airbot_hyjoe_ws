#!/usr/bin/env python3

import os
import csv
import argparse
import numpy as np
from rclpy.node import Node
import rclpy
from datetime import datetime
from robot_custom_msgs.msg import TofData

def time_to_float(t):
    return t.sec + t.nanosec * 1e-9

class TofDiffLogger(Node):
    def __init__(self):
        super().__init__('tof_diff_logger')

        self.tof_diff_publisher = self.create_publisher(
            TofData,
            'tof_diff_data',
            10
        )

        self.tof_diff_data = TofData()

        self.sub = self.create_subscription(TofData, '/tof_data', self.callback, 10)

    def callback(self, msg: TofData):

        self.tof_diff_data.timestamp = msg.timestamp

        for i in range(13, 16):
            left_diff = msg.bot_left[i - 3] - msg.bot_left[i]
            self.tof_diff_data.bot_left[i] = round(left_diff, 3) if left_diff >= 0 else 0.0

            right_diff = msg.bot_right[i - 3] - msg.bot_right[i]
            self.tof_diff_data.bot_right[i] = round(right_diff, 3) if right_diff >= 0 else 0.0

        self.tof_diff_publisher.publish(self.tof_diff_data)


def main():

    rclpy.init()
    node = TofDiffLogger()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
