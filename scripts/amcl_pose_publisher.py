import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class AMCLPosePublisher(Node):
    def __init__(self):
        super().__init__('amcl_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 10)
        timer_period = 1.0  # 1초마다 퍼블리싱
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = PoseWithCovarianceStamped()
        
        # 헤더 설정
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # 위치 설정 (x=1, y=0, z=0)
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # 방향 설정 (RPY = 0, 0, 0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0  # w는 1로 설정 (쿼터니언)
        
        # 공분산 행렬 설정 (6x6 단위 행렬)
        msg.pose.covariance = np.eye(6).flatten().tolist()
        
        # 퍼블리싱
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    amcl_pose_publisher = AMCLPosePublisher()
    rclpy.spin(amcl_pose_publisher)
    amcl_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
