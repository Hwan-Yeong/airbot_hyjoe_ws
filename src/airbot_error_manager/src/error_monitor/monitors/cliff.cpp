#include "error_monitor/monitors/cliff.hpp"

void CliffErrorMonitor::loadParams(const YAML::Node& config) {
    if (!node_ptr_) return;

    // Default values matched with yaml for fallback
    params.accum_dist_th = 0.3;
    params.duration_sec = 3.0;
    params.monitoring_rate_ms = 10;

    if (config["occure"]) {
        if (config["occure"]["accum_dist_th_m"]) params.accum_dist_th = config["occure"]["accum_dist_th_m"].as<double>();
        else if (config["occure"]["accum_dist_th"]) params.accum_dist_th = config["occure"]["accum_dist_th"].as<double>(); // fallback
        if (config["occure"]["duration_sec"]) params.duration_sec = config["occure"]["duration_sec"].as<double>();
    }
    if (config["monitoring_rate_ms"]) params.monitoring_rate_ms = config["monitoring_rate_ms"].as<int>();
}

void CliffErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] duration_sec: %.1f, accum_dist_th_m: %.1f, rate: %d",
        paramNamespace().c_str(),
        params.duration_sec,
        params.accum_dist_th,
        params.monitoring_rate_ms
    );
}

void CliffErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(
        "error/s_code/cliff", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void CliffErrorMonitor::timerCallback()
{
    static rclcpp::Clock clock(RCL_STEADY_TIME);

    double curDist, curPositionX, curPositionY, timeDiff;
    bool cliff[6]={}, errorState = false;

    auto ir = blackboard_->getIrData();
    auto odom = blackboard_->getOdomData();
    auto rState = blackboard_->getRobotStateData();

    if (checkSensorState(paramNamespace(), 10, {ir.last_update_time, odom.last_update_time, rState.last_update_time})
        != SensorState::NORMAL) {
        return;
    }

    if (!ir.is_updated || !odom.is_updated || !rState.is_updated) return;

    if (rState.data.state == 0 || rState.data.state == 7 || rState.data.state == 9) {
        for (int i=0; i<6; i++) {
            isFirstCheckArray[i] = true;
        }
        std_msgs::msg::Bool msg;
        msg.data = false;
        error_pub_->publish(msg);
        return;
    }

    cliff[0]= ir.data.ff;    cliff[1]= ir.data.fl;    cliff[2]= ir.data.bl;
    cliff[3]= ir.data.bb;    cliff[4]= ir.data.br;    cliff[5]= ir.data.fr;

    for (int i=0; i<6; i++) {
        if (cliff[i] == false) {
            isFirstCheckArray[i] = true;
            if (preErrorState[i] == true) { // 낙하 에러 해제시 로깅
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[CliffErrorMonitor] Cliff IR #[%d] : %s, "
                    "IR Detection Error Released",
                    i+1,
                    cliff[i] ? "true" : "false"
                );
            }
            preErrorState[i] = false;
            continue;
        } else {
            if (isFirstCheckArray[i]) {
                startErrorCheckTimeArray[i]=clock.now().seconds();
                prePositionXArray[i] = odom.data.pose.pose.position.x;
                prePositionYArray[i] = odom.data.pose.pose.position.y;
                accumDist[i] = 0.0;
                isFirstCheckArray[i] = false;
                // 낙하 에러 체크 시작 시 최초 한번 로깅
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[CliffErrorMonitor] Initial check => Cliff IR #[%d] : %s,"
                    "pre_position (X, Y): (%.3f, %.3f)",
                    i+1,
                    cliff[i] ? "true" : "false",
                    prePositionXArray[i],
                    prePositionYArray[i]
                );
            }

            timeDiff = clock.now().seconds() - startErrorCheckTimeArray[i];

            curPositionX = odom.data.pose.pose.position.x;
            curPositionY = odom.data.pose.pose.position.y;

            double dx = curPositionX - prePositionXArray[i];
            double dy = curPositionY - prePositionYArray[i];

            curDist = std::sqrt(dx*dx + dy*dy);
            accumDist[i] += curDist;

            prePositionXArray[i] = curPositionX;
            prePositionYArray[i] = curPositionY;

            if (accumDist[i] >= params.accum_dist_th) {
                if (preErrorState[i] == false) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[CliffErrorMonitor] Cliff IR #[%d] : %s, "
                        "IR Detection Error Occured,"
                        "timediff: %.3f sec," 
                        "Accumulated Distance: %.3f m",
                        i+1,
                        cliff[i] ? "true" : "false",
                        timeDiff,
                        accumDist[i]
                    );
                }
                errorState |= true;
                preErrorState[i] = true;
            }
        }
    }

    std_msgs::msg::Bool msg;
    msg.data = errorState;
    error_pub_->publish(msg);
}
