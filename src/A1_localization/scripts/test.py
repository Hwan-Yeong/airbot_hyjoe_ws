import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty

'''
아래 명령어로 acml 마지막 데이터로 로컬라이징 진행 가능

ros2 topic pub /localization/amcl/publish std_msgs/msg/Empty "{}" -r 0.1

명령어 사용 후 ctrl+c를 이용해 취소해야함. 
취소하지 않으면 10초에 한번씩 로컬라이제이션 진행함

'''

class AmclDataHandler(Node):
    def __init__(self):
        super().__init__('amcl_data_handler')
        
        # amcl_pose 토픽 구독
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10)
        
        # Empty 메시지 구독
        self.empty_subscription = self.create_subscription(
            Empty,
            '/localization/amcl/publish',
            self.empty_callback,
            10)
        
        # amcl 데이터 퍼블리셔
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/localization/request/pose',
            10)
        
        self.latest_amcl_pose = None
        
    def amcl_callback(self, msg):
        self.latest_amcl_pose = msg
    
    def empty_callback(self, msg):
        if self.latest_amcl_pose:
            self.publisher.publish(self.latest_amcl_pose)
            self.get_logger().info('저장된 AMCL 데이터를 퍼블리시함')
        else:
            self.get_logger().warn('저장된 AMCL 데이터가 없음')


def main(args=None):
    rclpy.init(args=args)
    node = AmclDataHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()