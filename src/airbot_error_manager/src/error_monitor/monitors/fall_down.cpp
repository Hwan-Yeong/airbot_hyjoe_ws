#include "error_monitor/monitors/fall_down.hpp"

void FallDownErrorMonitor::loadParams(const std::string& ns) {
    if (!node_ptr_) return;
    node_ptr_->declare_parameter<int>(ns + ".occure.drop_ir_adc_th", 900);
    node_ptr_->declare_parameter<int>(ns + ".occure.drop_ir_cnt_min", 4);
    node_ptr_->declare_parameter<double>(ns + ".occure.imu_roll_th_deg", 60.0);
    node_ptr_->declare_parameter<double>(ns + ".occure.imu_pitch_th_deg", 60.0);
    node_ptr_->declare_parameter<int>(ns + ".monitoring_rate_ms", 1000);

    node_ptr_->get_parameter(ns + ".occure.drop_ir_adc_th", params.drop_ir_adc_th);
    node_ptr_->get_parameter(ns + ".occure.drop_ir_cnt_min", params.drop_ir_cnt_min);
    node_ptr_->get_parameter(ns + ".occure.imu_roll_th_deg", params.imu_roll_th);
    node_ptr_->get_parameter(ns + ".occure.imu_pitch_th_deg", params.imu_pitch_th);
    node_ptr_->get_parameter(ns + ".monitoring_rate_ms", params.monitoring_rate_ms);
}

void FallDownErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] drop_ir_adc: %d, ir_cnt_min: %d, imu_roll_th: %.1f, imu_pitch_th: %.1f, rate: %d",
        paramNamespace().c_str(),
        params.drop_ir_adc_th,
        params.drop_ir_cnt_min,
        params.imu_roll_th,
        params.imu_pitch_th,
        params.monitoring_rate_ms
    );
}

void FallDownErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
        "error/s_code/fall_down", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void FallDownErrorMonitor::timerCallback()
{
    std::pair<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu> input;
    {
        auto ir = blackboard_->getIrData();
        auto imu = blackboard_->getImuData();
        
        if (checkSensorState(paramNamespace(), 10, {ir.last_update_time, imu.last_update_time})
            != SensorState::NORMAL) {
            return;
        }

        if (!ir.is_updated || !imu.is_updated) return;
        input = std::make_pair(ir.data, imu.data);
    }

    bool bottomdata_range = false;
    bool imu_range = false;
    int count = 0;
    static bool prev_status=false;
    static rclcpp::Clock clock(RCL_STEADY_TIME);

    // 밑 센서값 조정
    // 값이 방향에 따라서 일정하게 변하지 않기 때문에
    // 방향을 나누어서 값 변화에 대해서 전도 현상값의 범위를 조정해야 함
    // front - front_L - back_L - back - back_R - front_R
    if (input.first.adc_ff < params.drop_ir_adc_th) {
        count++;
    }
    if (input.first.adc_fl < params.drop_ir_adc_th) {
        count++;
    }
    if (input.first.adc_fr < params.drop_ir_adc_th) {
        count++;
    }
    if (input.first.adc_bb < params.drop_ir_adc_th) {
        count++;
    }
    if (input.first.adc_bl < params.drop_ir_adc_th) {
        count++;
    }
    if (input.first.adc_br < params.drop_ir_adc_th) {
        count++;
    }

    if (count >= params.drop_ir_cnt_min) { // 4
        bottomdata_range = true;
    } else {
        bottomdata_range = false;
    }

    // imu 센서값 조정
    // 선형 가속도 값을 roll, pitch 각도값으로 변환
    double deg_pitch, deg_roll;  // 각도 값
    double roll, pitch, yaw;
    get_rpy_from_quaternion(input.second.orientation, roll, pitch, yaw);

    deg_pitch = pitch * 180.0 / M_PI;
    deg_roll = roll * 180.0 / M_PI;

    if(is_first_boot) {
        baseline_pitch_deg = deg_pitch;
        baseline_roll_deg = deg_roll;
        baseline_time = clock.now().seconds();
        is_first_boot = false;
    }

    // roll, pitch 10도 이상의 변화가 생길때마다 로깅 (처음 저장한 시점으로부터의 pitch, roll, elapsed time 출력)
    if (abs(deg_roll - baseline_roll_deg) >= 10.0 || abs(deg_pitch - baseline_pitch_deg) >= 10.0) {
        double duration = clock.now().seconds() - baseline_time;
        RCLCPP_INFO(node_ptr_->get_logger(),
            "[FallDownErrorMonitor] Pitch/Roll Changed more than 10 degress! "
            "Previous [pitch : %.3f deg, roll : %.3f deg], "
            "Current [pitch : %.3f deg, roll : %.3f deg], "
            "angle_change_elapsed_time: %.2f sec",
            baseline_pitch_deg,
            baseline_roll_deg,
            deg_pitch,
            deg_roll,
            duration
        );
        baseline_pitch_deg = deg_pitch;
        baseline_roll_deg = deg_roll;
        baseline_time = clock.now().seconds();
    }

    if (abs(deg_pitch) >= params.imu_pitch_th || abs(deg_roll) >= params.imu_roll_th) { // 60 deg, 60 deg
        imu_range = true;
    }

    if (imu_range && bottomdata_range) { // 전도가 일어남, 데이터 값에 따른 결정
        if(!prev_status){
            // [250407] hyjoe : 전도 에러 발생시 낙하IR상태, roll, pitch 정보 1번만 로깅
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[FallDownErrorMonitor] Occured (adc_ff : %d)  (adc_fl : %d) (adc_fr :%d) "
                "(adc_bb : %d) (adc_bl : %d) (adc_br : %d)  (pitch : %.3f deg) (roll : %.3f deg)",
                input.first.adc_ff,
                input.first.adc_fr,
                input.first.adc_fr,
                input.first.adc_bb,
                input.first.adc_bl,
                input.first.adc_br,
                deg_pitch,
                deg_roll
            );
            prev_status=true;
        }
        std_msgs::msg::Bool msg;
        msg.data = true;
        error_pub_->publish(msg);
        return;
    } else { // 전도가 일어나지 않음
        if(prev_status){
            // [250407] hyjoe : 전도 에러 발생 한적이 있었던 경우, 해제시 1번만 낙하IR상태, roll, pitch 정보 1번만 로깅
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[FallDownErrorMonitor] Released (adc_ff : %d)  (adc_fl : %d) (adc_fr :%d) "
                "(adc_bb : %d) (adc_bl : %d) (adc_br : %d)  (pitch : %.3f deg) (roll : %.3f deg)",
                input.first.adc_ff,
                input.first.adc_fr,
                input.first.adc_fr,
                input.first.adc_bb,
                input.first.adc_bl,
                input.first.adc_br,
                deg_pitch,
                deg_roll
            );
            prev_status = false;
            baseline_pitch_deg = deg_pitch;
            baseline_roll_deg = deg_roll;
            baseline_time = clock.now().seconds();
        }
        std_msgs::msg::Bool msg;
        msg.data = false;
        error_pub_->publish(msg);
        return;
    }
}

void FallDownErrorMonitor::get_rpy_from_quaternion(const geometry_msgs::msg::Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}
