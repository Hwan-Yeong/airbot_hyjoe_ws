#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading

class SimpleFixedTeleop(Node):
    def __init__(self):
        super().__init__('simple_fixed_teleop')

        self.pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        # self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.current_twist = Twist()
        self.publish_enabled = False

        # 10 Hz 타이머 (항상 존재)
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info('Simple Fixed Teleop Started')
        self.get_logger().info('\n' \
        ' ----- [Key Map] ----- \n' \
        '|  q       w       r  |\n' \
        '                       \n' \
        '|  a    s(stop)    d  |\n' \
        '                       \n' \
        '|  z       x       c  |\n' \
        ' ---------------------')

        # 키 입력 스레드 시작
        self.key_thread = threading.Thread(target=self.key_loop, daemon=True)
        self.key_thread.start()

    def timer_cb(self):
        if self.publish_enabled:
            self.pub.publish(self.current_twist)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def key_loop(self):
        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                self.current_twist = Twist()
                self.current_twist.linear.x = 0.5
                self.publish_enabled = True
                self.get_logger().info('Linear x =  0.5 , Angular z =  0.0 : [Forward]')

            elif key == 'x':
                self.current_twist = Twist()
                self.current_twist.linear.x = -0.5
                self.publish_enabled = True
                self.get_logger().info('Linear x = -0.5 , Angular z =  0.0 : [Backward]')

            elif key == 'a':
                self.current_twist = Twist()
                self.current_twist.angular.z = 2.0
                self.publish_enabled = True
                self.get_logger().info('Linear x =  0.0 , Angular z =  2.0 : [Turn Left]')

            elif key == 'd':
                self.current_twist = Twist()
                self.current_twist.angular.z = -2.0
                self.publish_enabled = True
                self.get_logger().info('Linear x =  0.0 , Angular z = -2.0 : [Turn Right]')

            elif key == 'q':
                self.current_twist = Twist()
                self.current_twist.angular.z = 1.0
                self.current_twist.linear.x = 0.7
                self.publish_enabled = True
                self.get_logger().info('Linear x =  1.0 , Angular z =  0.7 : [Forward Left]')

            elif key == 'e':
                self.current_twist = Twist()
                self.current_twist.angular.z = -1.0
                self.current_twist.linear.x = 0.7
                self.publish_enabled = True
                self.get_logger().info('Linear x =  0.7 , Angular z = -1.0 : [Forward Right]')

            elif key == 'z':
                self.current_twist = Twist()
                self.current_twist.angular.z = -1.0
                self.current_twist.linear.x = -0.7
                self.publish_enabled = True
                self.get_logger().info('Linear x = -0.7 , Angular z = -1.0 : [Backward Left]')

            elif key == 'c':
                self.current_twist = Twist()
                self.current_twist.angular.z = 1.0
                self.current_twist.linear.x = -0.7
                self.publish_enabled = True
                self.get_logger().info('Linear x = -0.7 , Angular z =  1.0 : [Backward Right]')

            elif key == 's':
                self.publish_enabled = False
                self.pub.publish(Twist())
                self.get_logger().info('Linear x =  0.0 , Angular z =  0.0 : [Stop]')

            elif key == '\x03':  # Ctrl+C
                rclpy.shutdown()
                break

def main():
    rclpy.init()
    node = SimpleFixedTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
