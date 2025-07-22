#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ErrorTrigger(Node):
    def __init__(self):
        super().__init__('error_trigger_node')
        self.publisher_ = self.create_publisher(Bool, '/error/e_code/left_motor_overheat', 10)

    def run(self):
        # 5초 대기
        self.get_logger().info("Waiting 5 seconds...")
        self.timer_ = self.create_timer(5.0, self.publish_error)

    def publish_error(self):
        # 최초 한 번만 실행
        self.destroy_timer(self.timer_)

        # True 발행
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info("Published True for left_motor_overheat error")

        # 10ms 후 False 발행하는 타이머 생성
        self.timer_10ms = self.create_timer(0.01, self.publish_clear)

    def publish_clear(self):
        self.destroy_timer(self.timer_10ms)

        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)
        self.get_logger().info("Published False for left_motor_overheat error")
        # 모두 종료
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ErrorTrigger()
    node.run()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
