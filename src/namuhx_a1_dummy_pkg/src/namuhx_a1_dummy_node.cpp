#include "namuhx_a1_dummy_node.hpp"

NamuhxA1DummyNode::NamuhxA1DummyNode() : Node("namuhx_a1_dummy_node")
{
    // parameters
    this->declare_parameter("uart_loop_rate_ms", 10);
    this->get_parameter("uart_loop_rate_ms", uart_loop_rate_ms_);
    this->declare_parameter("lidar_loop_rate_ms", 10);
    this->get_parameter("lidar_loop_rate_ms", lidar_loop_rate_ms_);

    uart_loop_cnt_ = 0;
    lidar_loop_cnt_ = 0;
    imu_prev_time_ = std::chrono::steady_clock::now();

    // uart communication
    odom_status_pub_         = this->create_publisher<std_msgs::msg::UInt8>("/odom_status", 10);
    bottom_status_pub_       = this->create_publisher<std_msgs::msg::UInt8>("/bottom_status", 10);
    fw_version_pub_          = this->create_publisher<std_msgs::msg::String>("/fw_version", 10);
    imu_data_pub_            = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);
    odom_pub_                = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    battery_status_pub_      = this->create_publisher<robot_custom_msgs::msg::BatteryStatus>("/battery_status", 10);
    motor_status_pub_        = this->create_publisher<robot_custom_msgs::msg::MotorStatus>("/motor_status", 10);
    tof_data_pub_            = this->create_publisher<robot_custom_msgs::msg::TofData>("/tof_data", 10);
    jig_imu_calibration_pub_ = this->create_publisher<robot_custom_msgs::msg::ImuCalibration>("/jig_response_imu_calibration", 1);
    station_data_pub_        = this->create_publisher<robot_custom_msgs::msg::StationData>("/station_data", 10);
    bottom_ir_data_pub_      = this->create_publisher<robot_custom_msgs::msg::BottomIrData>("/bottom_ir_data", 10);
    version_request_sub_     = this->create_subscription<std_msgs::msg::UInt8>(
        "/req_version", 1, std::bind(&NamuhxA1DummyNode::req_fw_version_callback, this, std::placeholders::_1));

    // lidar
    scan_state_ = this->create_publisher<std_msgs::msg::Bool>("/scan_state", 10);
    scan_pub_   = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    processor_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&NamuhxA1DummyNode::processor_callback, this)
    );
}

NamuhxA1DummyNode::~NamuhxA1DummyNode()
{
}

void NamuhxA1DummyNode::processor_callback()
{
    uart_loop_cnt_++;
    lidar_loop_cnt_++;

    if (uart_loop_cnt_ >= uart_loop_rate_ms_) {
        uart_communication_processor();
        uart_loop_cnt_ = 0;
    }

    if (lidar_loop_cnt_ >= lidar_loop_rate_ms_) {
        lidar_processor();
        lidar_loop_cnt_ = 0;
    }

    if (uart_loop_cnt_ >= 100000) uart_loop_cnt_ = 0;
    if (lidar_loop_cnt_ >= 100000) lidar_loop_cnt_ = 0;
}

void NamuhxA1DummyNode::uart_communication_processor()
{
    pub_odom_status();
    pub_bottom_status();
    pub_fw_version();
    pub_imu_data();
    pub_odom();
    pub_battery_status();
    pub_motor_status();
    pub_tof_data();
    pub_jig_imu_calibration();
    pub_station_data();
    pub_bottom_ir_data();
}

void NamuhxA1DummyNode::lidar_processor()
{
    pub_scan_state();
    pub_scan();
    // pub_front_scan();
    // pub_rear_scan();
    // pub_scan_pointcloud();
}

void NamuhxA1DummyNode::req_fw_version_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    if (msg->data & 0x01) {
        std_msgs::msg::String fw_msg;
        fw_msg.data = "2.01";
        fw_version_pub_->publish(fw_msg);
        RCLCPP_INFO(this->get_logger(), "fw version : %s", fw_msg.data.c_str());
    }
}

void NamuhxA1DummyNode::pub_odom_status()
{
    std_msgs::msg::UInt8 odom_status_msg;
    odom_status_msg.data = 0x02 | 0x20; // imu ready & odom ready
    odom_status_pub_->publish(odom_status_msg);
}

void NamuhxA1DummyNode::pub_bottom_status()
{
    std_msgs::msg::UInt8 bottom_status_msg;
    bottom_status_msg.data = 0x00; // 바닥 IR 미감지
    bottom_status_pub_->publish(bottom_status_msg);
}

void NamuhxA1DummyNode::pub_fw_version()
{
    //
}

#define RAD_0_01 (0.01 * (M_PI / 180.0))
#define ACCEL_SCALE (9.81 / 1000.0)
void NamuhxA1DummyNode::pub_imu_data()
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    //TODO  추후 가변 imu 수정 point
    double yaw   = -0.0 * RAD_0_01;
    double pitch = 0.0 * RAD_0_01;
    double roll  = 0.0 * RAD_0_01;

    auto imu_cur_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = imu_cur_time - imu_prev_time_;
    double dt = elapsed_seconds.count();

    if(dt > 0.005 && dt < 0.015)
    {
        double roll_dot = (roll - imu_prev_roll_) / dt;
        double pitch_dot = (pitch - imu_prev_pitch_) / dt;
        double yaw_dot = (yaw - imu_prev_yaw_) / dt;

        std::vector<double> angular_velocity = computeAngularVelocity(roll, pitch, roll_dot, pitch_dot, yaw_dot);
        if(angular_velocity.size() >= 3)
        {
            imu_msg.angular_velocity.x = angular_velocity[0];
            imu_msg.angular_velocity.y = angular_velocity[1];
            imu_msg.angular_velocity.z = angular_velocity[2];
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[setImuMsg] angular_velocity size is %zu",angular_velocity.size());
        }
    }

    imu_prev_roll_ = roll;
    imu_prev_pitch_ = pitch;
    imu_prev_yaw_ = yaw;
    imu_prev_time_ = imu_cur_time;

    tf2::Quaternion imu_q;
    imu_q.setRPY(roll, pitch, yaw);
    imu_msg.orientation = tf2::toMsg(imu_q);
    imu_orientation_ = imu_msg.orientation;

    //TODO  추후 가변 imu 수정 point
    int16_t linear_acceleration_x = 0.0;
    int16_t linear_acceleration_y = 0.0;
    int16_t linear_acceleration_z = 9.81;
    imu_msg.linear_acceleration.x = linear_acceleration_x * ACCEL_SCALE;
    imu_msg.linear_acceleration.y = linear_acceleration_y * ACCEL_SCALE;
    imu_msg.linear_acceleration.z = linear_acceleration_z * ACCEL_SCALE;

    imu_data_pub_->publish(imu_msg);
}

void NamuhxA1DummyNode::pub_odom()
{
    rclcpp::Time cur_time = this->get_clock()->now();;

    // odometry
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = cur_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    //TODO 추후 가변 odom 수정 point
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = imu_orientation_;

    odom_pub_->publish(odom_msg);

    // tf
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = cur_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
}

void NamuhxA1DummyNode::pub_battery_status()
{
    robot_custom_msgs::msg::BatteryStatus battery_status_msg;
    battery_status_msg.battery_voltage = 20.04;
    battery_status_msg.battery_current = 2000;

    battery_status_msg.battery_percent = 64;
    battery_status_msg.cell_voltage1 = 1924;
    battery_status_msg.cell_voltage2 = 1924;
    battery_status_msg.cell_voltage3 = 1924;
    battery_status_msg.cell_voltage4 = 1924;
    battery_status_msg.cell_voltage5 = 1924;
    battery_status_msg.total_capacity = 3000;
    battery_status_msg.remaining_capacity = 1000;
    battery_status_msg.battery_manufacturer = 0x01;
    battery_status_msg.battery_temperature1 = 35;
    battery_status_msg.battery_temperature2 = 35;
    battery_status_msg.design_capacity = 1000;
    battery_status_msg.number_of_cycles = 10;

    /**
     * 0x00: 충전안함
     * 0x01: 충전중
     */
    battery_status_msg.charge_status = 0x00;
    /**
     * 0x00: MCU에서 충전 제어 3.8A 고속충전 - default
     * 0x01: MCU에서 충전 제어 1.0A 저속충전
     * 0x02: 고속 충전 시작 (3.8A)
     * 0x03: 저속 충전 시작 (1.0A)
     * 0x0F: 충전 중기
     */
    battery_status_msg.charging_mode = 0x0F;
    /**
     * 0: unknown
     * 1: 배터리 off (shipping mode)
     * 2: 배터리 on
     */
    battery_status_msg.shipping_mode = 2;
    battery_status_msg.precharge_state = 0;
    battery_status_msg.charge_mode = 0;
    battery_status_msg.fet_state = 0;
    battery_status_msg.battery_version = 1; // Neonix only

    battery_status_pub_->publish(battery_status_msg);
}

void NamuhxA1DummyNode::pub_motor_status()
{

}

void NamuhxA1DummyNode::pub_tof_data()
{
    
}

void NamuhxA1DummyNode::pub_jig_imu_calibration()
{
    
}

void NamuhxA1DummyNode::pub_station_data()
{
    
}

void NamuhxA1DummyNode::pub_bottom_ir_data()
{
    
}

void NamuhxA1DummyNode::pub_scan_state()
{

}

void NamuhxA1DummyNode::pub_scan()
{
    
}

std::vector<double> NamuhxA1DummyNode::computeAngularVelocity(double roll, double pitch, double roll_dot, double pitch_dot, double yaw_dot)
{
    std::vector<double> angular_velocity(3);
    angular_velocity[0] = roll_dot + yaw_dot * std::sin(pitch);
    angular_velocity[1] = pitch_dot * std::cos(roll) - yaw_dot * std::sin(roll) * std::cos(pitch);
    angular_velocity[2] = pitch_dot * std::sin(roll) + yaw_dot * std::cos(roll) * std::cos(pitch);
    return angular_velocity;
}
