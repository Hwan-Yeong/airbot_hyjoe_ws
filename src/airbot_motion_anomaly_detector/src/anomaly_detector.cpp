#include "airbot_motion_anomaly_detector/anomaly_detector.hpp"


AnomalyDetector::AnomalyDetector() : Node("anomaly_detector") {
    // Load parameters
    declare_parameter("output.collision.time_duration_ms", 500);
    declare_parameter("output.collision.imu_pitch_th_deg", -4.0);
    declare_parameter("output.collision.imu_pitch_diff_th_deg", 1.0);
    declare_parameter("output.collision.imu_pitch_diff_window_size", 10);
    declare_parameter("output.slope.time_duration", 2.0);
    declare_parameter("output.slope.imu_pitch_th", 0.10472);

    // Assign parameters to variables
    pitch_collision_duration_threshold_ = get_parameter("output.collision.time_duration_ms").as_int();
    pitch_slope_duration_threshold_ = get_parameter("output.slope.time_duration").as_double();
    pitch_threshold_collision_ = get_parameter("output.collision.imu_pitch_th_deg").as_double();
    pitch_threshold_ = get_parameter("output.slope.imu_pitch_th").as_double();
    pitch_diff_threshold_ = get_parameter("output.collision.imu_pitch_diff_th_deg").as_double();
    pitch_diff_window_size_ = get_parameter("output.collision.imu_pitch_diff_window_size").as_int();

    // Initialize variables
    cmd_vel_x_ = 0.0;
    is_climb_detected_ = false;
    is_collision_detected_ = false;
    collision_cnt_ = 0;
    pitch_slope_start_time_ = 0.0;

    // ROS 2 Publishers
    collision_pub_ = create_publisher<robot_custom_msgs::msg::AbnormalEventData>("collision_detected", 10);

    // ROS 2 Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu_data", 10, std::bind(&AnomalyDetector::imuCallback, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&AnomalyDetector::cmdVelCallback, this, std::placeholders::_1));
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, std::bind(&AnomalyDetector::amclPoseCallback, this, std::placeholders::_1));
    perception_climb_sub_ = create_subscription<std_msgs::msg::Bool>("/perception/climb", 10, std::bind(&AnomalyDetector::perceptionClimbCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "node initialized");

}

void AnomalyDetector::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double roll, pitch, yaw;
    getRPYFromQuaternion(msg->orientation, roll, pitch, yaw);
    double current_time = this->now().seconds();

    // Collision detection
    detectCollision(pitch);

    // Slope detection
    detectSlope(pitch, roll, current_time);
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

void AnomalyDetector::perceptionClimbCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        is_climb_detected_ = true;
    } else {
        is_climb_detected_ = false;
    }
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

void AnomalyDetector::detectCollision(double pitch)
{
    double pitch_deg = pitch*180/M_PI;

    if (cmd_vel_x_ > 0.0 && is_climb_detected_ && !is_collision_detected_) {
        if (pitch_deg < pitch_threshold_collision_) {

            pitch_history_.push_back(pitch_deg);

            if (static_cast<int>(pitch_history_.size()) > pitch_diff_window_size_) {
                pitch_history_.pop_front();
            }

            if (static_cast<int>(pitch_history_.size()) == pitch_diff_window_size_) {
                double pitch_diff = std::abs(pitch_history_.back() - pitch_history_.front());

                if (pitch_diff < pitch_diff_threshold_) {
                    collision_cnt_++;
                    // if (collision_cnt_ % 10 == 0) {
                    //     RCLCPP_INFO(this->get_logger(),
                    //         "Suspect Collision! Pitch %.2f deg, diff: %.2f deg, cnt: %d ms",
                    //         pitch_deg, pitch_diff, collision_cnt_ * 10
                    //     );
                    // }

                    if (collision_cnt_ >= pitch_collision_duration_threshold_ / 10) {
                        publishCollision(true);
                        is_collision_detected_ = true;
                        RCLCPP_INFO(this->get_logger(),
                            "Collision Detected! Pitch %.2f deg, duration: %d ms",
                            pitch_deg, pitch_collision_duration_threshold_
                        );
                    }
                }
            } else {
                collision_cnt_ = 0;
            }

        } else {
            collision_cnt_ = 0;
            pitch_history_.clear();
        }
    } else {
        collision_cnt_ = 0;
        is_collision_detected_ = false;
        pitch_history_.clear();
    }
}

void AnomalyDetector::detectSlope(double pitch, double roll, double current_time ){    //, double left_rpm) {
    constexpr double epsilon = 1e-6;  // Small threshold for floating-point comparison
    if (cmd_vel_x_ > 0.0) {
        if (abs(pitch) > pitch_threshold_ || abs(roll) > pitch_threshold_) {
            if (std::abs(pitch_slope_start_time_) < epsilon) {
                pitch_slope_start_time_ = current_time;
            }

            double pitch_slope_duration = current_time - pitch_slope_start_time_;
            if (pitch_slope_duration >= pitch_slope_duration_threshold_) {
                RCLCPP_WARN(this->get_logger(), "Slope detected! Pitch stayed out of range for %.3f seconds.", pitch_slope_duration);
                // publishSlope(true);
                // [25.06.12] hyjoe: collision으로 퍼블리싱하도록 추가 (maneuver 동작 고려)
                publishCollision(true);
                pitch_slope_start_time_ = 0.0;  // Reset timer after detection
            }
        } else {
            pitch_slope_start_time_ = 0.0;  // Reset if pitch returns to normal
        }
    } else {
        pitch_slope_start_time_ = 0.0;
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
    collision_msg.event_trigger = detected;
    collision_msg.robot_x = robot_pose_x;
    collision_msg.robot_y = robot_pose_y;
    collision_msg.robot_angle = robot_pose_angle;
    collision_pub_->publish(collision_msg);
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
