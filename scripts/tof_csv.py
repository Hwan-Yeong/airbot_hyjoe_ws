#!/usr/bin/env python3

import os
import csv
import argparse
from rclpy.node import Node
import rclpy
from datetime import datetime
from robot_custom_msgs.msg import TofData

def time_to_float(t):
    return t.sec + t.nanosec * 1e-9

class TofDataLogger(Node):
    def __init__(self, filename: str, left_indices, right_indices, duration: int = None):
        super().__init__('tof_data_logger')
        self.filename = filename
        self.left_indices = left_indices
        self.right_indices = right_indices
        self.duration = duration

        self.output_dir = "/home/hyjoe/dev_ws/test/tof"
        os.makedirs(self.output_dir, exist_ok=True)
        self.file_path = os.path.join(self.output_dir, f"{self.filename}.csv")
        self.sub = self.create_subscription(TofData, '/tof_data', self.callback, 10)
        self.logged_data = []
        self.start_stamp = None  # 센서 timestamp 기준 시작 시간

        self.write_header()

    def write_header(self):
        with open(self.file_path, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            header = ['timestamp'] \
                     + [f'Left[{i}]' for i in self.left_indices] \
                     + [f'Right[{i}]' for i in self.right_indices]
            writer.writerow(header)
            # writer.writerow(['MIN'] + [''] * (len(self.left_indices) + len(self.right_indices)))
            # writer.writerow(['MAX'] + [''] * (len(self.left_indices) + len(self.right_indices)))
            # writer.writerow(['Diff_MIN_MAX'] + [''] * (len(self.left_indices) + len(self.right_indices)))

    def callback(self, msg: TofData):
        if self.start_stamp is None:
            self.start_stamp = msg.timestamp

        timestamp = datetime.fromtimestamp(msg.timestamp.sec + msg.timestamp.nanosec * 1e-9)
        time_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')

        left = [round(msg.bot_left[i], 3) for i in self.left_indices]
        right = [round(msg.bot_right[i], 3) for i in self.right_indices]
        row = [time_str] + left + right
        self.logged_data.append(row)

        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow(row)

        if self.logged_data:
            left_cols = list(map(list, zip(*[r[1:1+len(self.left_indices)] for r in self.logged_data])))
            right_cols = list(map(list, zip(*[r[1+len(self.left_indices):] for r in self.logged_data])))

            # min_vals = [round(min(col), 3) for col in left_cols + right_cols]
            # max_vals = [round(max(col), 3) for col in left_cols + right_cols]
            # diff_vals = [round(abs(max_v - min_v), 3) for min_v, max_v in zip(min_vals, max_vals)]

            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f, delimiter='\t')
                header = ['timestamp'] \
                         + [f'Left[{i}]' for i in self.left_indices] \
                         + [f'Right[{i}]' for i in self.right_indices]
                writer.writerow(header)
                # writer.writerow(['MIN'] + min_vals)
                # writer.writerow(['MAX'] + max_vals)
                # writer.writerow(['Diff_MIN_MAX'] + diff_vals)
                writer.writerows(self.logged_data)

        # 종료 조건 체크
        if self.duration is not None:
            elapsed_sec = time_to_float(msg.timestamp) - time_to_float(self.start_stamp)
            if elapsed_sec >= self.duration:
                self.get_logger().info(f"[Sensor Time] {elapsed_sec:.2f}s passed. Stopping logger.")
                self.destroy_node()  # 명시적으로 노드 제거
                rclpy.shutdown()


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
    parser.add_argument('--duration', type=int, default=None,
                        help="데이터 로깅 시간 (초). 설정하지 않으면 Ctrl+C로 수동 종료")
    parser.add_argument('--left_row', default='', help="왼쪽 행 선택 예: 1,2 또는 3,4")
    parser.add_argument('--right_row', default='', help="오른쪽 행 선택 예: 1,4")

    args = parser.parse_args()
    left_indices = convert_row_to_indices(args.left_row)
    right_indices = convert_row_to_indices(args.right_row)

    rclpy.init()
    node = TofDataLogger(args.filename, left_indices, right_indices, args.duration)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
