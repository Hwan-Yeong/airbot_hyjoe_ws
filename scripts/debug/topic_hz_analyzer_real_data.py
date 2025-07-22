#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import csv
import matplotlib.pyplot as plt


def imu_data_changed(prev_msg: Imu, curr_msg: Imu, threshold: float = 1e-3) -> bool:
    if prev_msg is None:
        return True

    def changed(a, b):
        return abs(a - b) > threshold

    return any([
        changed(prev_msg.angular_velocity.x, curr_msg.angular_velocity.x),
        changed(prev_msg.angular_velocity.y, curr_msg.angular_velocity.y),
        changed(prev_msg.angular_velocity.z, curr_msg.angular_velocity.z),
        changed(prev_msg.linear_acceleration.x, curr_msg.linear_acceleration.x),
        changed(prev_msg.linear_acceleration.y, curr_msg.linear_acceleration.y),
        changed(prev_msg.linear_acceleration.z, curr_msg.linear_acceleration.z),
    ])


class ImuChangeAnalyzer(Node):
    def __init__(self):
        super().__init__('imu_change_analyzer')

        self.subscription = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )

        self.prev_msg = None
        self.timestamps = []
        self.start_time = time.time()
        self.duration = 20.0
        self.csv_filename = '/home/hyjoe/airbot_hyjoe_ws/csv/imu/imu_change_timestamps.csv'

        self.get_logger().info('Listening for changed IMU data over 20 seconds...')

    def imu_callback(self, msg: Imu):
        now = time.time()
        elapsed = now - self.start_time

        if elapsed <= self.duration:
            if imu_data_changed(self.prev_msg, msg):
                stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.timestamps.append(stamp_sec)
                self.prev_msg = msg
        else:
            self.analyze()
            rclpy.shutdown()

    def analyze(self):
        self.get_logger().info('--- Changed IMU Timestamps ---')
        for t in self.timestamps:
            self.get_logger().info(f'{t:.9f} sec')

        if len(self.timestamps) < 2:
            self.get_logger().warn('Not enough changed samples for analysis.')
            return

        intervals = [t2 - t1 for t1, t2 in zip(self.timestamps[:-1], self.timestamps[1:])]
        avg_interval = sum(intervals) / len(intervals)
        avg_hz = 1.0 / avg_interval
        min_interval = min(intervals)
        max_interval = max(intervals)

        # CSV 저장
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Index', 'Changed Timestamp (sec)', 'Interval (sec)', 'Freq (Hz)'])
            for i in range(len(intervals)):
                writer.writerow([i + 1, self.timestamps[i + 1], intervals[i], 1.0 / intervals[i]])
            writer.writerow(['Summary', '', f'Avg: {avg_interval:.6f}', f'Avg: {avg_hz:.2f}'])

        self.get_logger().info('\n--- IMU Data Change Analysis ---')
        self.get_logger().info(f'Total changed samples: {len(self.timestamps)}')
        self.get_logger().info(f'Average interval: {avg_interval:.6f} sec')
        self.get_logger().info(f'Average frequency: {avg_hz:.2f} Hz')
        self.get_logger().info(f'Min interval: {min_interval:.6f} sec')
        self.get_logger().info(f'Max interval: {max_interval:.6f} sec')
        self.get_logger().info(f'Saved CSV: {self.csv_filename}')

        self.plot_intervals(intervals)

    def plot_intervals(self, intervals):
        times = list(range(1, len(intervals) + 1))
        plt.figure(figsize=(10, 4))
        plt.plot(times, intervals, marker='o', linestyle='-', color='red')
        plt.xlabel('Change Index')
        plt.ylabel('Interval (sec)')
        plt.title('Interval Between Changed IMU Data')
        plt.grid(True)
        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = ImuChangeAnalyzer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
