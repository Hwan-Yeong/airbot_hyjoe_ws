import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class MapTFPublisher(Node):
    def __init__(self):
        super().__init__('map_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1  # 0.1초마다 TF 발행
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # TransformStamped 메시지 생성
        t = TransformStamped()

        # 헤더 설정
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'  # 기준 프레임
        t.child_frame_id = 'base_link'  # 자식 프레임

        # 위치 설정 (x=0, y=0, z=0)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 방향 설정 (쿼터니언, RPY = 0, 0, 0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # TF 발행
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Publishing TF: map -> base_link')

def main(args=None):
    rclpy.init(args=args)
    map_tf_publisher = MapTFPublisher()
    rclpy.spin(map_tf_publisher)
    map_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()