#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_custom_msgs.msg import BatteryStatus, StationData

class DebugPublisher(Node):
    def __init__(self):
        super().__init__('debug_publisher')
        # BatteryStatus와 StationData 토픽에 대한 퍼블리셔 생성 (QoS는 10으로 설정)
        self.battery_pub = self.create_publisher(BatteryStatus, 'battery_status', 10)
        self.station_pub = self.create_publisher(StationData, 'station_data', 10)

        # 10ms (0.01초) 간격으로 타이머 콜백 설정
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('DebugPublisher 노드가 시작되었습니다. 10ms 주기로 메시지를 발행합니다.')

    def timer_callback(self):
        battery_msg = BatteryStatus()
        station_msg = StationData()

        # BatteryStatus 메시지 예시 데이터 설정
        battery_msg.cell_voltage1 = 3737
        battery_msg.cell_voltage2 = 3737
        battery_msg.cell_voltage3 = 3737
        battery_msg.cell_voltage4 = 3737
        battery_msg.cell_voltage5 = 3737
        battery_msg.total_capacity = 9600
        battery_msg.remaining_capacity = 4300
        battery_msg.battery_manufacturer = 2
        battery_msg.battery_percent = 44
        battery_msg.battery_voltage = 186.8
        battery_msg.battery_current = 0.0
        battery_msg.battery_temperature1 = 37
        battery_msg.battery_temperature2 = 37
        battery_msg.design_capacity = 9600
        battery_msg.number_of_cycles = 17
        battery_msg.charge_status = 0
        battery_msg.charging_mode = 0

        # StationData 메시지 예시 데이터 설정
        station_msg.sig_short = 0
        station_msg.sig_long = 0
        station_msg.receiver_status = 0
        station_msg.docking_status = 48 # charger: 16, charging: 20, charger&charging: 48

        # 메시지 발행
        self.battery_pub.publish(battery_msg)
        self.station_pub.publish(station_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DebugPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
