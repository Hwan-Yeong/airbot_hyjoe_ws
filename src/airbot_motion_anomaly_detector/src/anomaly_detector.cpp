#include "airbot_motion_anomaly_detector/anomaly_detector.hpp"


AnomalyDetector::AnomalyDetector() : Node("anomaly_detector") {
    // Load parameters
    declare_parameter("pitch_duration_threshold", 0.5);
    declare_parameter("pitch_slope_duration_threshold", 0.5);
    declare_parameter("acceleration_threshold_low", -0.3);
    declare_parameter("acceleration_threshold_high", -0.6);
    declare_parameter("pitch_threshold_collision", -0.03);
    declare_parameter("pitch_threshold_min", -0.122173);
    declare_parameter("pitch_threshold_max", 0.122173);
    declare_parameter("motor_current_threshold_front", 6500.0);
    declare_parameter("motor_current_threshold_back", 4000.0);
    declare_parameter("motor_current_exceed_limit", 50);

    // Assign parameters to variables
    pitch_duration_threshold_ = get_parameter("pitch_duration_threshold").as_double();
    pitch_slope_duration_threshold_ = get_parameter("pitch_slope_duration_threshold").as_double();
    acceleration_threshold_low_ = get_parameter("acceleration_threshold_low").as_double();
    acceleration_threshold_high_ = get_parameter("acceleration_threshold_high").as_double();
    pitch_threshold_collision_ = get_parameter("pitch_threshold_collision").as_double();
    pitch_threshold_min = get_parameter("pitch_threshold_min").as_double();
    pitch_threshold_max = get_parameter("pitch_threshold_max").as_double();
    motor_current_threshold_front_ = get_parameter("motor_current_threshold_front").as_double();
    motor_current_threshold_back_ = get_parameter("motor_current_threshold_back").as_double();
    motor_current_exceed_limit_ = get_parameter("motor_current_exceed_limit").as_int();

    // Initialize variables
    pitch_start_time_ = 0.0;
    pitch_slope_start_time_ = 0.0;
    motor_current_exceed_count_ = 0;
    cmd_vel_x_ = 0.0;

    // ROS 2 Publishers
    collision_pub_ = create_publisher<robot_custom_msgs::msg::AbnormalEventData>("collision_detected", 10);
    slope_pub_ = create_publisher<robot_custom_msgs::msg::AbnormalEventData>("slope_detected", 10);

    // ROS 2 Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu_data", 10, std::bind(&AnomalyDetector::imuCallback, this, std::placeholders::_1));
    motor_status_sub_ = create_subscription<robot_custom_msgs::msg::MotorStatus>("/motor_status", 10, std::bind(&AnomalyDetector::motorStatusCallback, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&AnomalyDetector::cmdVelCallback, this, std::placeholders::_1));
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, std::bind(&AnomalyDetector::amclPoseCallback, this, std::placeholders::_1));

}

void AnomalyDetector::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double roll, pitch, yaw;
    getRPYFromQuaternion(msg->orientation, roll, pitch, yaw);
    double ax = msg->linear_acceleration.x;
    double current_time = this->now().seconds();

    // Determine acceleration threshold based on speed
    double linear_accel_threshold = (cmd_vel_x_ > 0 && cmd_vel_x_ < 0.2) ? acceleration_threshold_low_ : acceleration_threshold_high_;

    // Collision detection
    detectCollision(ax, pitch, current_time, linear_accel_threshold);

    // Slope detection
    detectSlope(pitch, current_time);
}

void AnomalyDetector::motorStatusCallback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg) {
    double motor_current_left = abs(msg->left_motor_current);
    double motor_current_right = abs(msg->right_motor_current);

    if (cmd_vel_x_ > 0) {
        if (motor_current_left > motor_current_threshold_front_) {
            motor_current_exceed_count_++;
        } else {
            motor_current_exceed_count_ = 0;
        }
    } else if (cmd_vel_x_ < 0) {
        if (motor_current_left > motor_current_threshold_back_) {
            motor_current_exceed_count_++;
        } else {
            motor_current_exceed_count_ = 0;
        }
    }

    // if (motor_current_exceed_count_ >= motor_current_exceed_limit_) {
    //     RCLCPP_WARN(this->get_logger(), "[AnomalyDetector] Collision Detected: High Motor Current (%.1f)!", motor_current_left);
    //     publishCollision(true);
    //     motor_current_exceed_count_ = 0;
    // }
}

void AnomalyDetector::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_x_ = msg->linear.x;
}

void AnomalyDetector::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        const auto& pose = msg->pose.pose;
        robot_pose_x = pose.position.x;
        robot_pose_y = pose.position.y;
        robot_pose_angle = quaternion_to_euler(pose.orientation);
}

double AnomalyDetector:: quaternion_to_euler(const geometry_msgs::msg::Quaternion& quat)
{
    tf2::Quaternion q;
    tf2::fromMsg(quat, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw; // Return yaw as theta
}

void AnomalyDetector::detectCollision(double ax, double pitch, double current_time, double accel_threshold) {
    constexpr double epsilon = 1e-6;  // Small threshold for floating-point comparison
    if (cmd_vel_x_ > 0.0){
        if (ax < accel_threshold && std::abs(pitch_start_time_) < epsilon) {
            pitch_start_time_ = current_time;
        }

        if (std::abs(pitch_start_time_) >= epsilon) {
            if (pitch < pitch_threshold_collision_) {
                double pitch_duration = current_time - pitch_start_time_;
                if (pitch_duration >= pitch_duration_threshold_) {
                    RCLCPP_WARN(this->get_logger(), "Collision detected! Pitch stayed negative for %.3f seconds.", pitch_duration);
                    
                    publishCollision(true);
                    pitch_start_time_ = 0.0;  // Reset after detecting collision
                }
            } else {
                pitch_start_time_ = 0.0;  // Reset if pitch goes back to normal
            }
        }
    }
}

void AnomalyDetector::detectSlope(double pitch, double current_time ){    //, double left_rpm) {
    constexpr double epsilon = 1e-6;  // Small threshold for floating-point comparison

    if (pitch < pitch_threshold_min || pitch > pitch_threshold_max) {
        if (std::abs(pitch_slope_start_time_) < epsilon) {
            pitch_slope_start_time_ = current_time;
        }

        double pitch_slope_duration = current_time - pitch_slope_start_time_;
        if (pitch_slope_duration >= pitch_slope_duration_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Slope detected! Pitch stayed out of range for %.3f seconds.", pitch_slope_duration);
            // publishSlope(true);
            pitch_slope_start_time_ = 0.0;  // Reset timer after detection
        }
    } else {
        pitch_slope_start_time_ = 0.0;  // Reset if pitch returns to normal
    }

    // // 🚨 Additional Condition: If left RPM is negative and pitch exceeds max threshold → Publish Collision
    // if (left_rpm < 0 && pitch > pitch_threshold_max) {
    //     RCLCPP_WARN(this->get_logger(), "[AnomalyDetector] 🚨 Collision detected! Left RPM: %.2f, Pitch: %.2f", left_rpm, pitch);
    //     publishCollision(true);
    // }
}



void AnomalyDetector::publishCollision(bool detected) {

    robot_custom_msgs::msg::AbnormalEventData collision_msg;
    collision_msg.timestamp = this->get_clock()->now();
    collision_msg.event_trigger = true;
    collision_msg.robot_x = robot_pose_x;
    collision_msg.robot_y = robot_pose_y;
    collision_msg.robot_angle = robot_pose_angle;
    collision_pub_->publish(collision_msg);
}

void AnomalyDetector::publishSlope(bool detected) {
    robot_custom_msgs::msg::AbnormalEventData slope_msg;
    slope_msg.timestamp = this->get_clock()->now();
    slope_msg.event_trigger = true;
    slope_msg.robot_x = robot_pose_x;
    slope_msg.robot_y = robot_pose_y;
    slope_msg.robot_angle = robot_pose_angle;
    slope_pub_->publish(slope_msg);
}

void AnomalyDetector::getRPYFromQuaternion(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw) {
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AnomalyDetector>());
    rclcpp::shutdown();
    return 0;
}
