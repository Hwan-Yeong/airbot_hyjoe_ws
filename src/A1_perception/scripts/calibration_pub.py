import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('multi_array_publisher')
        self.topic = '/perception/calibration/update'
        self.publisher = self.create_publisher(Float32MultiArray, self.topic, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초 주기
        
    
    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [0.442513, 0.437505, 0.438318, 0.481102, 0.484576, 0.482612]
        self.get_logger().info(f'Publishing to {self.topic}: {msg.data}')
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiArrayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()