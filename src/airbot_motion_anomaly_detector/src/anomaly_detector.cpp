#include "airbot_motion_anomaly_detector/anomaly_detector.hpp"


AnomalyDetector::AnomalyDetector() : Node("anomaly_detector") {
    // Load parameters
    declare_parameter("output.collision.front.time_duration_ms", 700);
    declare_parameter("output.collision.front.imu_pitch_th_deg", -5.0);
    declare_parameter("output.collision.front.imu_pitch_diff_th_deg", 1.0);
    declare_parameter("output.collision.front.imu_pitch_diff_window_size", 10);

    declare_parameter("output.collision.rear.time_duration_ms", 700);
    declare_parameter("output.collision.rear.imu_pitch_th_deg", 5.0);
    declare_parameter("output.collision.rear.imu_pitch_diff_th_deg", 1.0);
    declare_parameter("output.collision.rear.imu_pitch_diff_window_size", 10);

    declare_parameter("output.slope.time_duration", 2.0);
    declare_parameter("output.slope.imu_pitch_th", 0.10472);

    // Assign parameters to variables
    front_collision_pitch_duration_threshold_ = get_parameter("output.collision.front.time_duration_ms").as_int();
    front_collision_pitch_threshold_ = get_parameter("output.collision.front.imu_pitch_th_deg").as_double()*M_PI/180.0;
    front_collision_pitch_diff_threshold_ = get_parameter("output.collision.front.imu_pitch_diff_th_deg").as_double()*M_PI/180.0;
    front_collision_pitch_diff_window_size_ = get_parameter("output.collision.front.imu_pitch_diff_window_size").as_int();

    rear_collision_pitch_duration_threshold_ = get_parameter("output.collision.rear.time_duration_ms").as_int();
    rear_collision_pitch_threshold_ = get_parameter("output.collision.rear.imu_pitch_th_deg").as_double()*M_PI/180.0;
    rear_collision_pitch_diff_threshold_ = get_parameter("output.collision.rear.imu_pitch_diff_th_deg").as_double()*M_PI/180.0;
    rear_collision_pitch_diff_window_size_ = get_parameter("output.collision.rear.imu_pitch_diff_window_size").as_int();

    slope_pitch_duration_threshold_ = get_parameter("output.slope.time_duration").as_double();
    slope_pitch_threshold_ = get_parameter("output.slope.imu_pitch_th").as_double();

    // Initialize variables
    cmd_vel_x_ = 0.0;
    is_climb_detected_ = false;
    is_front_collision_detected_ = false;
    front_collision_cnt_ = 0;
    slope_pitch_start_time_ = 0.0;

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
    if (cmd_vel_x_ > 0.0 && is_climb_detected_ && !is_front_collision_detected_) {
        if (pitch < front_collision_pitch_threshold_) {

            front_pitch_history_.push_back(pitch);

            if (static_cast<int>(front_pitch_history_.size()) > front_collision_pitch_diff_window_size_) {
                front_pitch_history_.pop_front();
            }

            if (static_cast<int>(front_pitch_history_.size()) == front_collision_pitch_diff_window_size_) {
                double pitch_diff = std::abs(front_pitch_history_.back() - front_pitch_history_.front());

                if (pitch_diff < front_collision_pitch_diff_threshold_) {
                    front_collision_cnt_++;
                    // if (front_collision_cnt_ % 10 == 0) {
                    //     RCLCPP_INFO(this->get_logger(),
                    //         "Suspect Collision! Pitch %.2f deg, diff: %.2f deg, dirr_th: %.2f deg, cnt: %d ms",
                    //         pitch*180/M_PI, pitch_diff*180/M_PI, front_collision_pitch_diff_threshold_*180/M_PI, front_collision_cnt_ * 10
                    //     );
                    // }

                    if (front_collision_cnt_ >= front_collision_pitch_duration_threshold_ / 10) {
                        publishCollision(COLLISION_STATE::FRONT_COLLISION);
                        is_front_collision_detected_ = true;
                        RCLCPP_INFO(this->get_logger(),
                            "Collision Detected! Pitch %.2f deg, duration: %d ms",
                            pitch*180/M_PI, front_collision_pitch_duration_threshold_
                        );
                    }
                }
            } else {
                front_collision_cnt_ = 0;
            }

        } else {
            front_collision_cnt_ = 0;
            front_pitch_history_.clear();
        }
    } else if (cmd_vel_x_ < 0.0 && !is_rear_collision_detected_) {
        if (pitch > rear_collision_pitch_threshold_) {
            rear_pitch_history_.push_back(pitch);

            if (static_cast<int>(rear_pitch_history_.size()) > rear_collision_pitch_diff_window_size_) {
                rear_pitch_history_.pop_front();
            }

            if (static_cast<int>(rear_pitch_history_.size()) == rear_collision_pitch_diff_window_size_) {
                double pitch_diff = std::abs(rear_pitch_history_.back() - rear_pitch_history_.front());

                if (pitch_diff < rear_collision_pitch_diff_threshold_) {
                    rear_collision_cnt_++;
                    // if (rear_collision_cnt_ % 10 == 0) {
                    //     RCLCPP_INFO(this->get_logger(),
                    //         "Suspect Collision! Pitch %.2f deg, diff: %.2f deg, dirr_th: %.2f deg, cnt: %d ms",
                    //         pitch*180/M_PI, pitch_diff*180/M_PI, rear_collision_pitch_diff_threshold_*180/M_PI, rear_collision_cnt_ * 10
                    //     );
                    // }

                    if (rear_collision_cnt_ >= rear_collision_pitch_duration_threshold_ / 10) {
                        publishCollision(COLLISION_STATE::REAR_COLLISION);
                        is_rear_collision_detected_ = true;
                        RCLCPP_INFO(this->get_logger(),
                            "Collision Detected! Pitch %.2f deg, duration: %d ms",
                            pitch*180/M_PI, rear_collision_pitch_duration_threshold_
                        );
                    }
                }
            } else {
                rear_collision_cnt_ = 0;
            }

        } else {
            rear_collision_cnt_ = 0;
            rear_pitch_history_.clear();
        }
    } else {
        front_collision_cnt_ = 0;
        rear_collision_cnt_ = 0;
        is_front_collision_detected_ = false;
        is_rear_collision_detected_ = false;
        front_pitch_history_.clear();
        rear_pitch_history_.clear();
    }
}

void AnomalyDetector::detectSlope(double pitch, double roll, double current_time ){    //, double left_rpm) {
    constexpr double epsilon = 1e-6;  // Small threshold for floating-point comparison
    if (cmd_vel_x_ > 0.0) {
        if (abs(pitch) > slope_pitch_threshold_ || abs(roll) > slope_pitch_threshold_) {
            if (std::abs(slope_pitch_start_time_) < epsilon) {
                slope_pitch_start_time_ = current_time;
            }

            double pitch_slope_duration = current_time - slope_pitch_start_time_;
            if (pitch_slope_duration >= slope_pitch_duration_threshold_) {
                RCLCPP_WARN(this->get_logger(), "Slope detected! Pitch stayed out of range for %.3f seconds.", pitch_slope_duration);
                // publishSlope(FRONT_COLLISION);
                // [25.06.12] hyjoe: collision으로 퍼블리싱하도록 추가 (maneuver 동작 고려)
                publishCollision(COLLISION_STATE::FRONT_COLLISION);
                slope_pitch_start_time_ = 0.0;  // Reset timer after detection
            }
        } else {
            slope_pitch_start_time_ = 0.0;  // Reset if pitch returns to normal
        }
    } else {
        slope_pitch_start_time_ = 0.0;
    }

    // // 🚨 Additional Condition: If left RPM is negative and pitch exceeds max threshold → Publish Collision
    // if (left_rpm < 0 && pitch > slope_pitch_threshold_max) {
    //     RCLCPP_WARN(this->get_logger(), "[AnomalyDetector] 🚨 Collision detected! Left RPM: %.2f, Pitch: %.2f", left_rpm, pitch);
    //     publishCollision(FRONT_COLLISION);
    // }
}

void AnomalyDetector::publishCollision(COLLISION_STATE col_state) {
    RCLCPP_INFO(this->get_logger(), "Publish Collision Msg: %s", enumToString(col_state).c_str());
    robot_custom_msgs::msg::AbnormalEventData collision_msg;
    collision_msg.timestamp = this->get_clock()->now();
    collision_msg.event_trigger = static_cast<int16_t>(col_state);
    collision_msg.robot_x = robot_pose_x;
    collision_msg.robot_y = robot_pose_y;
    collision_msg.robot_angle = robot_pose_angle;
    if (col_state == COLLISION_STATE::FRONT_COLLISION || col_state == COLLISION_STATE::REAR_COLLISION) {
        collision_pub_->publish(collision_msg);
    } else {
        RCLCPP_INFO(this->get_logger(), "Unknown collision state: %d", static_cast<int>(col_state));
    }
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
