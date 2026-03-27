#include "error_monitor/monitors/fall_down.hpp"

void FallDownErrorMonitor::loadParams(const YAML::Node& config) {
    if (!node_ptr_) return;
    
    // Default values
    params.monitoring_rate_ms = 1000;
    params.drop_ir_adc_th = 900;
    params.drop_ir_cnt_min = 4;
    params.imu_roll_th = 60.0;
    params.imu_pitch_th = 60.0;

    if (config["monitoring_rate_ms"]) {
        params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
    }
    if (config["occure"]) {
        if (config["occure"]["drop_ir_adc_th"]) {
            params.drop_ir_adc_th = config["occure"]["drop_ir_adc_th"].as<int>();
        }
        if (config["occure"]["drop_ir_cnt_min"]) {
            params.drop_ir_cnt_min = config["occure"]["drop_ir_cnt_min"].as<int>();
        }
        if (config["occure"]["imu_roll_th_deg"]) {
            params.imu_roll_th = config["occure"]["imu_roll_th_deg"].as<double>();
        }
        if (config["occure"]["imu_pitch_th_deg"]) {
            params.imu_pitch_th = config["occure"]["imu_pitch_th_deg"].as<double>();
        }
    }
}

void FallDownErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "\n[%s] rate: %d\n"
        "drop_ir_adc: %d, ir_cnt_min: %d, imu_roll_th: %.1f, imu_pitch_th: %.1f",
        paramNamespace().c_str(),
        params.monitoring_rate_ms,
        params.drop_ir_adc_th,
        params.drop_ir_cnt_min,
        params.imu_roll_th,
        params.imu_pitch_th);
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
    auto ir = blackboard_->getIrData();
    auto imu = blackboard_->getImuData();
    
    if (checkSensorState(paramNamespace(), 10, {ir.last_update_time, imu.last_update_time})
        != SensorState::NORMAL) {
        return;
    }

    if (!ir.is_updated || !imu.is_updated) return;

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    bool is_imu_out_of_range = false;

    // 밑 센서값 조정
    // 값이 방향에 따라서 일정하게 변하지 않기 때문에
    // 방향을 나누어서 값 변화에 대해서 전도 현상값의 범위를 조정해야 함
    // front - front_L - back_L - back - back_R - front_R
    int count = (ir.data.adc_ff < params.drop_ir_adc_th) +
                (ir.data.adc_fl < params.drop_ir_adc_th) +
                (ir.data.adc_fr < params.drop_ir_adc_th) +
                (ir.data.adc_bb < params.drop_ir_adc_th) +
                (ir.data.adc_bl < params.drop_ir_adc_th) +
                (ir.data.adc_br < params.drop_ir_adc_th);

    bool is_ir_low_adc = (count >= params.drop_ir_cnt_min); //4

    // imu 센서값 조정
    // 선형 가속도 값을 roll, pitch 각도값으로 변환
    double deg_pitch, deg_roll;  // 각도 값
    double roll, pitch, yaw;
    get_rpy_from_quaternion(imu.data.orientation, roll, pitch, yaw);

    deg_pitch = pitch * 180.0 / M_PI;
    deg_roll = roll * 180.0 / M_PI;

    if(is_first_boot) {
        baseline_pitch_deg = deg_pitch;
        baseline_roll_deg = deg_roll;
        baseline_time = clock.now().seconds();
        is_first_boot = false;
    }

    // roll, pitch 10도 이상의 변화가 생길때마다 로깅
    // (처음 저장한 시점으로부터의 pitch, roll, elapsed time 출력)
    if (abs(deg_roll - baseline_roll_deg) >= 10.0 ||
        abs(deg_pitch - baseline_pitch_deg) >= 10.0) {
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

    if (abs(deg_pitch) >= params.imu_pitch_th ||
        abs(deg_roll) >= params.imu_roll_th) { // 60 deg, 60 deg
        is_imu_out_of_range = true;
    }

    if (is_imu_out_of_range && is_ir_low_adc) { // 전도가 일어남, 데이터 값에 따른 결정
        if(!prev_status){
            // [250407] hyjoe : 전도 에러 발생시 낙하IR상태, roll, pitch 정보 1번만 로깅
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[FallDownErrorMonitor] Occured (adc_ff : %d) (adc_fl : %d) "
                "(adc_fr :%d) (adc_bb : %d) (adc_bl : %d) (adc_br : %d) "
                "(pitch : %.3f deg) (roll : %.3f deg)",
                ir.data.adc_ff,
                ir.data.adc_fr,
                ir.data.adc_fr,
                ir.data.adc_bb,
                ir.data.adc_bl,
                ir.data.adc_br,
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
                "[FallDownErrorMonitor] Released (adc_ff : %d) (adc_fl : %d) "
                "(adc_fr :%d) (adc_bb : %d) (adc_bl : %d) (adc_br : %d) "
                "(pitch : %.3f deg) (roll : %.3f deg)",
                ir.data.adc_ff,
                ir.data.adc_fr,
                ir.data.adc_fr,
                ir.data.adc_bb,
                ir.data.adc_bl,
                ir.data.adc_br,
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
