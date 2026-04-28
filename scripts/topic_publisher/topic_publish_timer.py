import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from robot_custom_msgs.msg import BlockAreaList, BlockArea, Position
import random, string

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

class KeepoutPublisher(Node):
    def __init__(self):
        super().__init__('keepout_publisher')
        self.keepout_publisher_ = self.create_publisher(BlockAreaList,'block_areas',10)
        timer_period = 0.01 # 10ms
        self.timer = self.create_timer(timer_period, self.publish_timer)
        self.x_ = 0.01
        self.y_ = 0.01
        self.theta_ = 0.01

    def publish_timer(self):
        msg = BlockAreaList()

        block_area = BlockArea()
        # block_area.id = "f0c9673f-368d-477f-804e-f1e6c55836e0"

        char = string.ascii_letters + string.digits
        block_area.id = ''.join(random.choice(char) for _ in range(10))

        image_position = Position()
        block_area.image_path = [image_position]

        robot_position_list = []
        for i in range(4):
            robot_position = Position()
            self.x_ += 0.01
            self.y_ += 0.01
            self.theta_ += 0.01
            robot_position.x = self.x_
            robot_position.y = self.y_
            robot_position.theta = self.theta_
            robot_position_list.append(robot_position)
            
        block_area.robot_path = robot_position_list

        msg.block_area_list = [block_area]

        self.keepout_publisher_.publish(msg)
        # self.get_logger().info('Publish: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    # amcl_pose_publisher = AMCLPosePublisher()
    keepout_publisher = KeepoutPublisher()
    # rclpy.spin(amcl_pose_publisher)
    # amcl_pose_publisher.destroy_node()
    rclpy.spin(keepout_publisher)
    keepout_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
