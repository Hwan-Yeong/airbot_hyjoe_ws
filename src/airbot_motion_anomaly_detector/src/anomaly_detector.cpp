#include "airbot_motion_anomaly_detector/anomaly_detector.hpp"


AnomalyDetector::AnomalyDetector() : Node("anomaly_detector") {
    // Load parameters
    declare_parameter("output.collision.time_duration", 1.3);
    declare_parameter("output.collision.imu_pitch_acc_th_min", -0.3);
    declare_parameter("output.collision.imu_pitch_acc_th_max", -0.6);
    declare_parameter("output.collision.imu_pitch_th", -0.1);
    declare_parameter("output.collision.motor_current_front_th", 6500.0);
    declare_parameter("output.collision.motor_current_back_th", 4000.0);
    declare_parameter("output.collision.motor_current_exceed_limit", 50);
    declare_parameter("output.slope.time_duration", 2.0);
    declare_parameter("output.slope.imu_pitch_th", 0.10472);

    // Assign parameters to variables
    pitch_collision_duration_threshold_ = get_parameter("output.collision.time_duration").as_double();
    pitch_slope_duration_threshold_ = get_parameter("output.slope.time_duration").as_double();
    acceleration_threshold_low_ = get_parameter("output.collision.imu_pitch_acc_th_min").as_double();
    acceleration_threshold_high_ = get_parameter("output.collision.imu_pitch_acc_th_max").as_double();
    pitch_threshold_collision_ = get_parameter("output.collision.imu_pitch_th").as_double();
    pitch_threshold_ = get_parameter("output.slope.imu_pitch_th").as_double();
    motor_current_threshold_front_ = get_parameter("output.collision.motor_current_front_th").as_double();
    motor_current_threshold_back_ = get_parameter("output.collision.motor_current_back_th").as_double();
    motor_current_exceed_limit_ = get_parameter("output.collision.motor_current_exceed_limit").as_int();

    // Initialize variables
    pitch_start_time_ = 0.0;
    pitch_slope_start_time_ = 0.0;
    motor_current_exceed_count_ = 0;
    cmd_vel_x_ = 0.0;
    ax_ = 0.0;
    ay_ = 0.0;

    // ROS 2 Publishers
    collision_pub_ = create_publisher<robot_custom_msgs::msg::AbnormalEventData>("collision_detected", 10);
    slope_pub_ = create_publisher<robot_custom_msgs::msg::AbnormalEventData>("slope_detected", 10);

    // ROS 2 Subscribers
    bottom_ir_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>("/bottom_ir_data", 10, std::bind(&AnomalyDetector::bottomIrCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu_data", 10, std::bind(&AnomalyDetector::imuCallback, this, std::placeholders::_1));
    motor_status_sub_ = create_subscription<robot_custom_msgs::msg::MotorStatus>("/motor_status", 10, std::bind(&AnomalyDetector::motorStatusCallback, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&AnomalyDetector::cmdVelCallback, this, std::placeholders::_1));
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, std::bind(&AnomalyDetector::amclPoseCallback, this, std::placeholders::_1));
    climb_state_sub_ = create_subscription<std_msgs::msg::Bool>("/perception/climb", 10, std::bind(&AnomalyDetector::climbStateCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "node initialized");

    collision_detection_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&AnomalyDetector::collisionMonitor, this));
}

void AnomalyDetector::collisionMonitor()
{
    // Determine acceleration threshold based on speed
    double linear_accel_threshold = (cmd_vel_x_ > 0 && cmd_vel_x_ < 0.2) ? acceleration_threshold_low_ : acceleration_threshold_high_;
    double current_time = this->now().seconds();

    // Collision detection
    detectCollision(ax_, ay_, pitch_, bottom_ir_, current_time, linear_accel_threshold);

    // Slope detection
    detectSlope(pitch_, roll_, current_time);
}

void AnomalyDetector::bottomIrCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(),
    //     "[Before] IR [ff: %d, fl: %d, fr: %d, bb: %d, bl: %d, br: %d]",
    //     msg->adc_ff, msg->adc_fl, msg->adc_fr, msg->adc_bb, msg->adc_bl, msg->adc_br
    // );
    bottom_ir_ = updateMovingAverageFilter(*msg);
    // bottom_ir_ = *msg;
    // RCLCPP_INFO(this->get_logger(),
    //     "[After] IR [ff: %d, fl: %d, fr: %d, bb: %d, bl: %d, br: %d]",
    //     bottom_ir_.adc_ff, bottom_ir_.adc_fl, bottom_ir_.adc_fr, bottom_ir_.adc_bb, bottom_ir_.adc_bl, bottom_ir_.adc_br
    // );

}

void AnomalyDetector::climbStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bClimb_ = msg->data;
}

robot_custom_msgs::msg::BottomIrData AnomalyDetector::updateMovingAverageFilter(const robot_custom_msgs::msg::BottomIrData input)
{
    robot_custom_msgs::msg::BottomIrData ret;

    auto update = [this](std::deque<int16_t>& history, int16_t new_value) -> int16_t {
        history.push_back(new_value);
        if (history.size() > 3) { // window size
            history.pop_front();
        }

        int sum = 0;
        for (int16_t val : history) {
            sum += val;
        }

        return static_cast<int16_t>(sum / history.size());
    };

    ret.adc_ff = update(history_ff_, input.adc_ff);
    ret.adc_fl = update(history_fl_, input.adc_fl);
    ret.adc_fr = update(history_fr_, input.adc_fr);
    ret.adc_bb = update(history_bb_, input.adc_bb);
    ret.adc_bl = update(history_bl_, input.adc_bl);
    ret.adc_br = update(history_br_, input.adc_br);

    return ret;
}

void AnomalyDetector::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double roll, pitch, yaw;
    getRPYFromQuaternion(msg->orientation, roll, pitch, yaw);
    ax_ = msg->linear_acceleration.x;
    ay_ = msg->linear_acceleration.y;
    roll_ = roll;
    pitch_ = pitch;
    yaw_= yaw;
}

void AnomalyDetector::motorStatusCallback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg) {
    double motor_current_left = abs(msg->left_motor_current);
    // double motor_current_right = abs(msg->right_motor_current);

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
    robot_pose_x_ = pose.position.x;
    robot_pose_y_ = pose.position.y;
    robot_pose_angle_ = quaternion_to_euler(pose.orientation);
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

void AnomalyDetector::detectCollision(double ax, double ay, double pitch, robot_custom_msgs::msg::BottomIrData ir_data, double current_time, double accel_threshold)
{
    constexpr double epsilon = 1e-6;  // Small threshold for floating-point comparison

    constexpr int drop_ir_th = 1500;

    int detected_ir_cnt = (ir_data.adc_ff < drop_ir_th) + (ir_data.adc_fl < drop_ir_th) + (ir_data.adc_fr < drop_ir_th)
                        + (ir_data.adc_bb < drop_ir_th) + (ir_data.adc_bl < drop_ir_th) + (ir_data.adc_br < drop_ir_th);

    if (cmd_vel_x_ > 0.0 || !bClimb_){
        // if (!bClimb_) {
            if ((abs(ax) >= 1.5 || abs(ay) >= 1.5)) {
                // RCLCPP_INFO(this->get_logger(),
                //     "[Suspected Collision Detection] IMU [ax: %.2f m/s2, ay: %.2f m/s2, Pitch: %.2f deg], IR [ff: %d, fl: %d, fr: %d, bb: %d, bl: %d, br: %d]",
                //     ax, ay, pitch, ir_data.adc_ff, ir_data.adc_fl, ir_data.adc_fr, ir_data.adc_bb, ir_data.adc_bl, ir_data.adc_br
                // );
                if ((abs(pitch) > 0.07 || detected_ir_cnt >= 1)) {
                    publishCollision(true);
                    RCLCPP_INFO(this->get_logger(),
                        "[Collision Detection] IMU [ax: %.2f m/s2, ay: %.2f m/s2, Pitch: %.2f deg], IR [ff: %d, fl: %d, fr: %d, bb: %d, bl: %d, br: %d]",
                        ax, ay, pitch, ir_data.adc_ff, ir_data.adc_fl, ir_data.adc_fr, ir_data.adc_bb, ir_data.adc_bl, ir_data.adc_br
                    );
                }
            }
        // } else {
        //     RCLCPP_INFO(this->get_logger(),
        //         "Climb Detected from A1_perception, Pitch(rad): %.3f",
        //         pitch
        //     );
        // }
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
    collision_msg.robot_x = robot_pose_x_;
    collision_msg.robot_y = robot_pose_y_;
    collision_msg.robot_angle = robot_pose_angle_;
    collision_pub_->publish(collision_msg);
}

void AnomalyDetector::publishSlope(bool detected) {
    robot_custom_msgs::msg::AbnormalEventData slope_msg;
    slope_msg.timestamp = this->get_clock()->now();
    slope_msg.event_trigger = detected;
    slope_msg.robot_x = robot_pose_x_;
    slope_msg.robot_y = robot_pose_y_;
    slope_msg.robot_angle = robot_pose_angle_;
    slope_pub_->publish(slope_msg);
    RCLCPP_INFO(this->get_logger(), "Publish slope detected!");
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
