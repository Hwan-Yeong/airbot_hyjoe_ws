#!/usr/bin/env python3

import os
import csv
import argparse
from rclpy.node import Node
import rclpy
from datetime import datetime
from robot_custom_msgs.msg import TofData

class TofDataLogger(Node):
    def __init__(self, filename: str, left_indices, right_indices):
        super().__init__('tof_data_logger')
        self.filename = filename
        self.left_indices = left_indices
        self.right_indices = right_indices

        self.output_dir = "/home/hyjoe/dev_ws/test/tof"
        os.makedirs(self.output_dir, exist_ok=True)
        self.file_path = os.path.join(self.output_dir, f"{self.filename}.csv")
        self.sub = self.create_subscription(TofData, '/tof_data', self.callback, 10)
        self.logged_data = []

        self.write_header()

    def write_header(self):
        with open(self.file_path, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            header = ['timestamp'] \
                     + [f'Left[{i}]' for i in self.left_indices] \
                     + [f'Right[{i}]' for i in self.right_indices]
            writer.writerow(header)
            writer.writerow(['MIN'] + [''] * (len(self.left_indices) + len(self.right_indices)))
            writer.writerow(['MAX'] + [''] * (len(self.left_indices) + len(self.right_indices)))
            writer.writerow(['Diff_MIN_MAX'] + [''] * (len(self.left_indices) + len(self.right_indices)))

    def callback(self, msg: TofData):
        timestamp = datetime.fromtimestamp(msg.timestamp.sec + msg.timestamp.nanosec * 1e-9)
        time_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')

        left = [round(msg.bot_left[i], 3) for i in self.left_indices]
        right = [round(msg.bot_right[i], 3) for i in self.right_indices]

        row = [time_str] + left + right
        self.logged_data.append(row)

        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow(row)

        if len(self.logged_data) >= 1:
            left_cols = list(map(list, zip(*[r[1:1+len(self.left_indices)] for r in self.logged_data])))
            right_cols = list(map(list, zip(*[r[1+len(self.left_indices):] for r in self.logged_data])))

            min_vals = [round(min(col), 3) for col in left_cols + right_cols]
            max_vals = [round(max(col), 3) for col in left_cols + right_cols]
            diff_vals = [round(abs(max_v - min_v), 3) for min_v, max_v in zip(min_vals, max_vals)]

            min_row = ['MIN'] + min_vals
            max_row = ['MAX'] + max_vals
            diff_row = ['Diff_MIN_MAX'] + diff_vals

            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f, delimiter='\t')
                header = ['timestamp'] \
                         + [f'Left[{i}]' for i in self.left_indices] \
                         + [f'Right[{i}]' for i in self.right_indices]
                writer.writerow(header)
                writer.writerow(min_row)
                writer.writerow(max_row)
                writer.writerow(diff_row)
                writer.writerows(self.logged_data)


def convert_row_to_indices(row_str):
    """예: '3,4' → [8, 9, 10, 11, 12, 13, 14, 15]"""
    if not row_str:
        return list(range(16))
    indices = []
    for row in map(int, row_str.split(',')):
        if 1 <= row <= 4:
            start = (row - 1) * 4
            indices.extend(range(start, start + 4))
    return indices


def main():
    parser = argparse.ArgumentParser(description="ToF Data Logger (행 기반 인덱스 선택 가능)")
    parser.add_argument('filename', help="CSV 파일명 (확장자 제외)")
    parser.add_argument('--left_row', default='', help="왼쪽 행 선택 예: 1,2 또는 3,4")
    parser.add_argument('--right_row', default='', help="오른쪽 행 선택 예: 1,4")

    args = parser.parse_args()
    left_indices = convert_row_to_indices(args.left_row)
    right_indices = convert_row_to_indices(args.right_row)

    rclpy.init()
    node = TofDataLogger(args.filename, left_indices, right_indices)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
