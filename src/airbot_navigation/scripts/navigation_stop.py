#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from robot_navigator import BasicNavigator

class JoySubscriber(Node):
    def __init__(self):
        super().__init__('joy_subscriber')
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.basic_navigator = BasicNavigator()

    def joy_callback(self, msg):
        # Check if the 'X' button on the joystick is pressed
        if len(msg.buttons) > 2 and msg.buttons[2]: # 'X' button on Xbox joystick
            self.basic_navigator.cancelTask()

def main(args=None):
    rclpy.init(args=args)
    joy_subscriber = JoySubscriber()
    rclpy.spin(joy_subscriber)
    joy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
