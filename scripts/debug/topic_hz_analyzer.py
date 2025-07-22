import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from collections import deque
import csv
import os

class ImuPitchMonitor(Node):
    def __init__(self):
        super().__init__('imu_pitch_monitor')

        self.subscription = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )

        # pitch 추출용
        self.prev_pitch = None
        self.pitch_window = deque(maxlen=10)
        self.pitch_event_timestamps = []

        # 토픽 수신 간격 측정용
        self.prev_topic_time = None
        self.topic_time_diffs = []

        # CSV 저장을 위한 경로
        self.csv_path = '/home/hyjoe/airbot_hyjoe_ws/csv/imu/imu_pitch_stats.csv'

        self.get_logger().info("IMU Pitch Monitor Initialized")

    def imu_callback(self, msg: Imu):
        # pitch 추출
        pitch = self.get_pitch_from_quaternion(msg.orientation.x,
                                               msg.orientation.y,
                                               msg.orientation.z,
                                               msg.orientation.w)

        # 충돌 감지 로직
        self.pitch_window.append(pitch)
        if len(self.pitch_window) == self.pitch_window.maxlen:
            recent = self.pitch_window[-1]
            oldest = self.pitch_window[0]
            diff = abs(recent - oldest)
            if diff < 1.0:
                self.pitch_event_timestamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

        # pitch 기반 주기 계산
        if len(self.pitch_event_timestamps) >= 2:
            last = self.pitch_event_timestamps[-1]
            prev = self.pitch_event_timestamps[-2]
            actual_pitch_period = last - prev
        else:
            actual_pitch_period = None

        # 토픽 간 시간 간격 측정
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_topic_time is not None:
            topic_period = current_time - self.prev_topic_time
        else:
            topic_period = None
        self.prev_topic_time = current_time

        # CSV 저장
        self.save_to_csv(current_time, actual_pitch_period, topic_period)

    def get_pitch_from_quaternion(self, x, y, z, w):
        # Roll, pitch, yaw 계산
        import math
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        return math.degrees(pitch)

    def save_to_csv(self, timestamp, actual_pitch_period, topic_period):
        file_exists = os.path.isfile(self.csv_path)
        with open(self.csv_path, mode='a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if not file_exists:
                writer.writerow(['Timestamp', 'Actual_Pitch_Period', 'Topic_Timestamp_Period'])
            writer.writerow([
                f"{timestamp:.9f}",
                f"{actual_pitch_period:.6f}" if actual_pitch_period is not None else '',
                f"{topic_period:.6f}" if topic_period is not None else ''
            ])

def main(args=None):
    rclpy.init(args=args)
    node = ImuPitchMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
