import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from robot_custom_msgs.msg import RobotState, BottomIrData, CameraData, CameraDataArray

class CliffDetectionErrorSimulation(Node):
    def __init__(self):
        super().__init__('cliff_detection_error_simulation')
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        self.robot_state_publisher = self.create_publisher(
            RobotState,
            'state_datas',
            10
        )
        self.bottom_ir_publisher = self.create_publisher(
            BottomIrData,
            'bottom_ir_data',
            10
        )
        self.camera_publisher = self.create_publisher(
            CameraDataArray,
            'camera_data',
            10
        )
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.robot_state = RobotState()
        self.bottom_ir_data = BottomIrData()
        self.odom_data = Odometry()
        self.camera_data_array = CameraDataArray()
        self.odom_data.header.stamp = self.get_clock().now().to_msg()
        self.odom_data.header.frame_id = 'odom'
        self.odom_data.child_frame_id = 'base_link'
        self.odom_data.pose.pose.position.x = 0.0
        self.odom_data.pose.pose.position.y = 0.0
        self.odom_data.pose.pose.position.z = 0.0
        self.odom_data.pose.pose.orientation.x = 0.0
        self.odom_data.pose.pose.orientation.y = 0.0
        self.odom_data.pose.pose.orientation.z = 0.0
        self.odom_data.pose.pose.orientation.w = 1.0
        self.odom_data.twist.twist.linear.x = 0.0
        self.odom_data.twist.twist.linear.y = 0.0
        self.odom_data.twist.twist.linear.z = 0.0
        self.odom_data.twist.twist.angular.x = 0.0
        self.odom_data.twist.twist.angular.y = 0.0
        self.odom_data.twist.twist.angular.z = 0.0
        self.pre_odom_data = Odometry()

        self.start_time = self.get_clock().now()
        self.ir_monified = False

    def pubRobotState(self):
        self.robot_state.state = 1
        self.robot_state.status = 0
        self.robot_state_publisher.publish(self.robot_state)

    def pubCameraData(self):
        self.camera_data_array.timestamp = self.get_clock().now().to_msg()
        self.camera_data_array.num = 1
        self.camera_data_array.robot_x = 0.0
        self.camera_data_array.robot_y = 0.0
        self.camera_data_array.robot_angle = 0.0

        camera_data = CameraData()
        camera_data.id = 12
        camera_data.score = 50
        camera_data.x = 0.0
        camera_data.y = 0.0
        camera_data.theta = np.deg2rad(30)
        camera_data.width = 1.0
        camera_data.height = 0.8
        camera_data.distance = 1.0

        self.camera_data_array.data_array = [camera_data]
        self.camera_publisher.publish(self.camera_data_array)


    def pubBottomIrData(self):
        self.bottom_ir_data.timestamp = self.get_clock().now().to_msg()

        if not self.ir_monified:
            self.bottom_ir_data.ff = True
            self.bottom_ir_data.br = True
            self.bottom_ir_data.adc_ff = 1900
            self.bottom_ir_data.adc_br = 1900
        else:
            self.bottom_ir_data.ff = False
            self.bottom_ir_data.br = False
            self.bottom_ir_data.adc_ff = 4070
            self.bottom_ir_data.adc_br = 4070

        # self.bottom_ir_data.ff = True
        self.bottom_ir_data.fl = False
        self.bottom_ir_data.bb = False
        self.bottom_ir_data.bl = False
        self.bottom_ir_data.fr = False
        self.bottom_ir_data.adc_fl = 4070
        self.bottom_ir_data.adc_bb = 4070
        self.bottom_ir_data.adc_bl = 4070
        self.bottom_ir_data.adc_fr = 4070
        self.bottom_ir_data.robot_x = 0.0
        self.bottom_ir_data.robot_y = 0.0
        self.bottom_ir_data.robot_angle = 0.0

        self.bottom_ir_publisher.publish(self.bottom_ir_data)

    def pubOdomData(self):
        self.odom_data.header.stamp = self.get_clock().now().to_msg()
        self.odom_data.pose.pose.position.x = self.pre_odom_data.pose.pose.position.x + 0.001

        self.odom_publisher.publish(self.odom_data)
        self.pre_odom_data = self.odom_data

    def timer_callback(self):
        currrent_time = self.get_clock().now()
        elapsed_time = (currrent_time - self.start_time).nanoseconds / 1e9

        if elapsed_time >= 10.0 and not self.ir_monified:
            self.ir_monified = True
            self.get_logger().info("IR Status Changed [True => False]")

        self.pubRobotState()
        self.pubBottomIrData()
        self.pubOdomData()
        self.pubCameraData()

def main(args=None):
    rclpy.init(args=args)
    node = CliffDetectionErrorSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
