#include "error_monitor/monitors/tof.hpp"

void TofErrorMonitor::loadParams(const std::string& ns) {
    if (!node_ptr_) return;
    node_ptr_->declare_parameter<double>(ns + ".occure.one_d_min_dist_m", 0.01);
    node_ptr_->declare_parameter<double>(ns + ".occure.one_d_max_dist_m", 0.04);
    node_ptr_->declare_parameter<double>(ns + ".occure.duration_sec", 60.0);
    node_ptr_->declare_parameter<int>(ns + ".monitoring_rate_ms", 100);

    node_ptr_->get_parameter(ns + ".occure.one_d_min_dist_m", params.one_d_min_dist_m);
    node_ptr_->get_parameter(ns + ".occure.one_d_max_dist_m", params.one_d_max_dist_m);
    node_ptr_->get_parameter(ns + ".occure.duration_sec", params.duration_sec);
    node_ptr_->get_parameter(ns + ".monitoring_rate_ms", params.monitoring_rate_ms);
}

void TofErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] duration_sec: %.1f, min_dist: %.2f, max_dist: %.2f, rate: %d",
        paramNamespace().c_str(),
        params.duration_sec,
        params.one_d_min_dist_m,
        params.one_d_max_dist_m,
        params.monitoring_rate_ms
    );
}

void TofErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
        "error/s_code/tof", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void TofErrorMonitor::timerCallback() {
    robot_custom_msgs::msg::TofData input;
    {
        auto tof = blackboard_->getTofData();
        
        if (checkSensorState(paramNamespace(), 10, {tof.last_update_time}) != SensorState::NORMAL) {
            return;
        }

        if (!tof.is_updated) return;
        input = tof.data;
    }

    static bool is_first_detect = false;
    static rclcpp::Clock clock(RCL_STEADY_TIME);
    static double check_oned_startTime;
    static bool isError = false;
    static double next_check_sec = 1;

    double tof_data = input.top;

    if (tof_data >= params.one_d_min_dist_m && tof_data <= params.one_d_max_dist_m) {
        if (!is_first_detect) {
            check_oned_startTime = clock.now().seconds();
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[1D_TofErrorMonitor] check start dist=%.2f",
                tof_data
            );
            is_first_detect = true;
            next_check_sec = 1;
        }

        double time_diff = clock.now().seconds() - check_oned_startTime;
        if ((time_diff >= next_check_sec) && !isError) { // 10초 단위로 로깅
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[1D_TofErrorMonitor] Error Checking for %.2fsec",
                time_diff
            );
            next_check_sec += 1;
        }

        // 에러 상태가 60초 이상 유지된 경우
        if (time_diff >= params.duration_sec) {
            if (!isError) {
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[1D_TofErrorMonitor] Error occurred"
                );
            }
            next_check_sec = 1;
            isError = true;
        }
    } else {
        is_first_detect = false;
        if (isError) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[1D_TofErrorMonitor] Error resolved"
            );
        }
        isError = false;
    }

    std_msgs::msg::Bool msg;
    msg.data = isError;
    error_pub_->publish(msg);
}
