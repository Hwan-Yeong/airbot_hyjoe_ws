import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Odometry
from robot_custom_msgs.msg import RobotState, BottomIrData, CameraData, CameraDataArray, TofData
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

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
        self.imu_publisher = self.create_publisher(
            Imu,
            'imu_data',
            10
        )
        self.tof_publisher = self.create_publisher(
            TofData,
            'tof_data',
            10
        )
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.robot_state = RobotState()
        self.bottom_ir_data = BottomIrData()
        self.odom_data = Odometry()
        self.camera_data_array = CameraDataArray()
        self.imu_data = Imu()
        self.tof_data = TofData()
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
        self.camera_data_array.num = 2
        self.camera_data_array.robot_x = 0.0
        self.camera_data_array.robot_y = 0.0
        self.camera_data_array.robot_angle = 0.0

        camera_data_1 = CameraData()
        camera_data_1.id = 12
        camera_data_1.score = 50
        camera_data_1.x = 0.0
        camera_data_1.y = 0.0
        camera_data_1.theta = np.deg2rad(30)
        camera_data_1.width = 1.0
        camera_data_1.height = 0.8
        camera_data_1.distance = 1.0

        camera_data_2 = CameraData()
        camera_data_2.id = 5
        camera_data_2.score = 50
        camera_data_2.x = 0.0
        camera_data_2.y = 0.0
        camera_data_2.theta = -np.deg2rad(30)
        camera_data_2.width = 1.0
        camera_data_2.height = 0.8
        camera_data_2.distance = 1.0

        self.camera_data_array.data_array = [camera_data_1, camera_data_2]
        # self.camera_data_array.data_array = []
        self.camera_publisher.publish(self.camera_data_array)

    def deg2rad(deg):
        return deg * math.pi / 180.0

    def pubImuData(self):
        self.imu_data.header.stamp = self.get_clock().now().to_msg()
        self.imu_data.header.frame_id = 'imu_link'
        self.imu_data.orientation.x = 0.05234
        self.imu_data.orientation.y = 0.00000
        self.imu_data.orientation.z = 0.00000
        self.imu_data.orientation.w = 0.99863
        # self.imu_data.orientation.x = 0.00
        # self.imu_data.orientation.y = 0.00
        # self.imu_data.orientation.z = 0.00
        # self.imu_data.orientation.w = 0.00

        self.imu_publisher.publish(self.imu_data)

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

    def pubToFData(self):
        self.tof_data.timestamp = self.get_clock().now().to_msg()
        self.tof_data.top = 0.05
        self.tof_publisher.publish(self.tof_data)

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
        self.pubImuData()
        self.pubToFData()

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
