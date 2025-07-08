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

class TofDataLogger(Node):
    def __init__(self, filename: str, left_indices, right_indices, duration: int = None):
        super().__init__('tof_data_logger')
        self.filename = filename
        self.left_indices = left_indices
        self.right_indices = right_indices
        self.duration = duration

        self.output_dir = "/home/hyjoe/airbot_hyjoe_ws/csv"
        os.makedirs(self.output_dir, exist_ok=True)
        self.file_path = os.path.join(self.output_dir, f"{self.filename}.csv")
        self.sub = self.create_subscription(TofData, '/tof_data', self.callback, 10)
        self.logged_data = []
        self.start_stamp = None

        self.write_header_only()

    def write_header_only(self):
        with open(self.file_path, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            header = ['timestamp'] \
                    + [f'Left[{i}]' for i in self.left_indices] \
                    + [f'Right[{i}]' for i in self.right_indices]
            writer.writerow(header)

    def write_statistics_and_data(self):
        if not self.logged_data:
            return

        numeric_data = np.array([r[1:] for r in self.logged_data], dtype=np.float64)  # timestamp 제외

        min_vals, max_vals, avg_vals, std_vals = [], [], [], []

        for col in numeric_data.T:
            nonzero = col[col != 0]
            if len(nonzero) == 0:
                min_vals.append('')
                max_vals.append('')
                avg_vals.append('')
                std_vals.append('')
            else:
                min_vals.append(round(np.min(nonzero), 3))
                max_vals.append(round(np.max(nonzero), 3))
                avg_vals.append(round(np.mean(nonzero), 3))
                std_vals.append(round(np.std(nonzero), 3))

        def safe_round(x):
            return round(x, 3) if isinstance(x, (int, float)) else ''

        with open(self.file_path, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            header = ['timestamp'] \
                    + [f'Left[{i}]' for i in self.left_indices] \
                    + [f'Right[{i}]' for i in self.right_indices]
            writer.writerow(header)
            writer.writerow(['min'] + list(map(safe_round, min_vals)))
            writer.writerow(['max'] + list(map(safe_round, max_vals)))
            writer.writerow(['avg'] + list(map(safe_round, avg_vals)))
            writer.writerow(['std'] + list(map(safe_round, std_vals)))
            writer.writerows(self.logged_data)

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

        if self.duration is not None:
            elapsed_sec = time_to_float(msg.timestamp) - time_to_float(self.start_stamp)
            if elapsed_sec >= self.duration:
                self.get_logger().info(f"[Sensor Time] {elapsed_sec:.2f}s passed. Stopping logger.")
                self.write_statistics_and_data()
                self.destroy_node()
                rclpy.shutdown()

def convert_row_to_indices(row_str):
    row_map = {
        1: [0, 1, 2],
        3: [3, 4, 5],
        4: [6, 7],
        6: [8, 9],
        7: [10, 11, 12],
        8: [13, 14, 15]
    }

    if not row_str:
        all_indices = []
        for indices in row_map.values():
            all_indices.extend(indices)
        return all_indices

    indices = []
    for row in map(int, row_str.split(',')):
        if row in row_map:
            indices.extend(row_map[row])
    return indices

def main():
    parser = argparse.ArgumentParser(description="ToF Data Logger (행 기반 인덱스 선택 가능)")
    parser.add_argument('filename', help="CSV 파일명 (확장자 제외)")
    parser.add_argument('--duration', type=int, default=None,
                        help="데이터 로깅 시간 (초). 설정하지 않으면 Ctrl+C로 수동 종료")
    parser.add_argument('--left_row', default='', help="왼쪽 행 선택 예: 1,3 또는 4,6")
    parser.add_argument('--right_row', default='', help="오른쪽 행 선택 예: 7,8")

    args = parser.parse_args()
    left_indices = convert_row_to_indices(args.left_row)
    right_indices = convert_row_to_indices(args.right_row)

    rclpy.init()
    node = TofDataLogger(args.filename, left_indices, right_indices, args.duration)

    # Ctrl+C (SIGINT)로 종료될 때도 통계 기록
    def on_shutdown():
        node.write_statistics_and_data()
        node.get_logger().info("통계 데이터를 포함하여 CSV 저장 완료.")
        node.destroy_node()

    rclpy.get_default_context().on_shutdown(on_shutdown)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
