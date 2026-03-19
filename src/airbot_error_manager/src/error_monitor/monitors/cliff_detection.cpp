#include "error_monitor/monitors/cliff_detection.hpp"

void CliffDetectionErrorMonitor::loadParams(const std::string& ns) {
    if (!node_ptr_) return;
    node_ptr_->declare_parameter<double>(ns + ".occure.accum_dist_th", 0.3);
    node_ptr_->declare_parameter<double>(ns + ".occure.duration_sec", 3.0);
    node_ptr_->declare_parameter<int>(ns + ".monitoring_rate_ms", 100);

    node_ptr_->get_parameter(ns + ".occure.accum_dist_th", params.accum_dist_th);
    node_ptr_->get_parameter(ns + ".occure.duration_sec", params.duration_sec);
    node_ptr_->get_parameter(ns + ".monitoring_rate_ms", params.monitoring_rate_ms);
}

void CliffDetectionErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] duration_sec: %.1f, accum_dist_th_m: %.1f, rate: %d",
        paramNamespace().c_str(),
        params.duration_sec,
        params.accum_dist_th,
        params.monitoring_rate_ms
    );
}

void CliffDetectionErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>("error/s_code/cliff_detection", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void CliffDetectionErrorMonitor::timerCallback()
{
    std::tuple<robot_custom_msgs::msg::BottomIrData, nav_msgs::msg::Odometry, robot_custom_msgs::msg::RobotState> input;
    {
        auto ir = blackboard_->getIrData();
        auto odom = blackboard_->getOdomData();
        
        if (checkSensorState(paramNamespace(), 10, {ir.last_update_time, odom.last_update_time})
            != SensorState::NORMAL) {
            return;
        }

        auto state = blackboard_->getRobotStateData();
        if (!ir.is_updated || !odom.is_updated || !state.is_updated) return;
        input = std::make_tuple(ir.data, odom.data, state.data);
    }

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    static double startErrorCheckTimeArray[6]={}, prePositionXArray[6]={}, prePositionYArray[6]={}, accumDist[6]={};
    static bool isFirstCheckArray[6] = {true, true, true, true, true, true};
    static bool preErrorState[6] = {}; 
    double curDist, curPositionX, curPositionY, timeDiff;
    bool cliff[6]={}, errorState = false;

    auto bottomIrData = std::get<0>(input);
    auto odom = std::get<1>(input);
    auto robotState = std::get<2>(input);

    if (robotState.state == 0 || robotState.state == 7 || robotState.state == 9) {
        for (int i=0; i<6; i++) {
            isFirstCheckArray[i] = true;
        }
        std_msgs::msg::Bool msg;
        msg.data = false;
        error_pub_->publish(msg);
        return;
    }

    cliff[0]= bottomIrData.ff;    cliff[1]= bottomIrData.fl;    cliff[2]= bottomIrData.bl;
    cliff[3]= bottomIrData.bb;    cliff[4]= bottomIrData.br;    cliff[5]= bottomIrData.fr;

    for (int i=0; i<6; i++) {
        if (cliff[i] == false) {
            isFirstCheckArray[i] = true;
            if (preErrorState[i] == true) { // 낙하 에러 해제시 로깅
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[CliffDetectionErrorMonitor] Cliff IR #[%d] : %s, IR Detection Error Released",
                    i+1,
                    cliff[i] ? "true" : "false"
                );
            }
            preErrorState[i] = false;
            continue;
        } else {
            if (isFirstCheckArray[i]) {
                startErrorCheckTimeArray[i]=clock.now().seconds();
                prePositionXArray[i] = odom.pose.pose.position.x;
                prePositionYArray[i] = odom.pose.pose.position.y;
                accumDist[i] = 0.0;
                isFirstCheckArray[i] = false;
                // 낙하 에러 체크 시작 시 최초 한번 로깅
                RCLCPP_INFO(node_ptr_->get_logger(),
                    "[CliffDetectionErrorMonitor] Initial check => Cliff IR #[%d] : %s,"
                    "pre_position (X, Y): (%.3f, %.3f)",
                    i+1,
                    cliff[i] ? "true" : "false",
                    prePositionXArray[i],
                    prePositionYArray[i]
                );
            }

            timeDiff = clock.now().seconds() - startErrorCheckTimeArray[i];

            curPositionX = odom.pose.pose.position.x;
            curPositionY = odom.pose.pose.position.y;

            double dx = curPositionX - prePositionXArray[i];
            double dy = curPositionY - prePositionYArray[i];

            curDist = std::sqrt(dx*dx + dy*dy);
            accumDist[i] += curDist;

            prePositionXArray[i] = curPositionX;
            prePositionYArray[i] = curPositionY;

            if (accumDist[i] >= params.accum_dist_th) {
                if (preErrorState[i] == false) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[CliffDetectionErrorMonitor] Cliff IR #[%d] : %s, timediff: %.3f sec," 
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
