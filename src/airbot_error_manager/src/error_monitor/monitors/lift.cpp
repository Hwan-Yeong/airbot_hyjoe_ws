#include "error_monitor/monitors/lift.hpp"

void LiftErrorMonitor::loadParams(const std::string& ns) {
    if (!node_ptr_) return;
    node_ptr_->declare_parameter<int>(ns + ".occure.drop_ir_adc_th", 900);
    node_ptr_->declare_parameter<int>(ns + ".occure.drop_ir_cnt_min", 4);
    node_ptr_->declare_parameter<double>(ns + ".occure.imu_z_acc_low_th", 9.2);
    node_ptr_->declare_parameter<double>(ns + ".occure.imu_z_acc_hight_th", 10.5);
    node_ptr_->declare_parameter<int>(ns + ".monitoring_rate_ms", 10);

    node_ptr_->get_parameter(ns + ".occure.drop_ir_adc_th", params.drop_ir_adc_th);
    node_ptr_->get_parameter(ns + ".occure.drop_ir_cnt_min", params.drop_ir_cnt_min);
    node_ptr_->get_parameter(ns + ".occure.imu_z_acc_low_th", params.imu_z_acc_low_th);
    node_ptr_->get_parameter(ns + ".occure.imu_z_acc_hight_th", params.imu_z_acc_hight_th);
    node_ptr_->get_parameter(ns + ".monitoring_rate_ms", params.monitoring_rate_ms);
}

void LiftErrorMonitor::printParams() const {
    if (!node_ptr_) return;
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] drop_ir_adc: %d, ir_cnt_min: %d, imu_z_acc_low: %.1f, imu_z_acc_high: %.1f, rate: %d",
        paramNamespace().c_str(),
        params.drop_ir_adc_th,
        params.drop_ir_cnt_min,
        params.imu_z_acc_low_th,
        params.imu_z_acc_hight_th,
        params.monitoring_rate_ms
    );
}

void LiftErrorMonitor::startMonitor(std::shared_ptr<RobotStateBlackboard> blackboard) {
    blackboard_ = blackboard;
    error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Bool>("error/s_code/lift", 10);
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(params.monitoring_rate_ms),
        [this](){ timerCallback(); }
    );
}

void LiftErrorMonitor::timerCallback()
{
    std::tuple<robot_custom_msgs::msg::BottomIrData, sensor_msgs::msg::Imu, robot_custom_msgs::msg::StationData> input;
    {
        auto ir = blackboard_->getIrData();
        auto imu = blackboard_->getImuData();
        auto st = blackboard_->getStationData();
        
        if (checkSensorState(paramNamespace(), 10, {ir.last_update_time, imu.last_update_time, st.last_update_time})
            != SensorState::NORMAL) {
            return;
        }

        if (!ir.is_updated || !imu.is_updated || !st.is_updated) return;
        input = std::make_tuple(ir.data, imu.data, st.data);
    }

    auto ir_data = std::get<0>(input);
    auto imu_data = std::get<1>(input);
    auto station = std::get<2>(input);

    static rclcpp::Clock clock(RCL_STEADY_TIME);
    static int count = 0;
    static bool irLiftFlag = false;
    static bool imuLiftFlag = false;

    static bool liftErrorCandidate = false;
    static double prevTime = 0.0;

    count = (ir_data.adc_ff < params.drop_ir_adc_th) + (ir_data.adc_fl < params.drop_ir_adc_th) +
            (ir_data.adc_fr < params.drop_ir_adc_th) + (ir_data.adc_bb < params.drop_ir_adc_th) +
            (ir_data.adc_bl < params.drop_ir_adc_th) + (ir_data.adc_br < params.drop_ir_adc_th);
    
    bool isChargerConnect = station.docking_status & 0x10; // 충전 단자 인식 시 LiftFlag 해제

    if (count == 0 || isChargerConnect) { // 모든 IR 센서가 false일 경우 에러 해제. 또는 충전 단자 인식 시 에러 해제
        if (errorState) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] LiftError Released! isChargerConnect = %d",
                isChargerConnect
            );
        }
        if (errorCount > 0)
        {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] Error Clear. errorCount = %d, count = %d, isChargerConnect = %d",
                errorCount,
                count,
                isChargerConnect
            );
        }
        errorCount = 0;
        errorState = false;
        irLiftFlag = false;
    } else if (count >= params.drop_ir_cnt_min) { // ir 센서 true개수 4개 이상이면 ir 들림 의심
        if (!irLiftFlag) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] Over 4 IR sensors Lift Detected! "
                "(adc_ff : %d)  (adc_fl : %d) (adc_fr :%d) "
                "(adc_bb : %d) (adc_bl : %d) (adc_br : %d)",
                ir_data.adc_ff,
                ir_data.adc_fr,
                ir_data.adc_fr,
                ir_data.adc_bb,
                ir_data.adc_bl,
                ir_data.adc_br
            );
        }
        irLiftFlag = true;
    } else {
        irLiftFlag = false;
    }

    double acc_z = imu_data.linear_acceleration.z;

    if (irLiftFlag && (acc_z <= params.imu_z_acc_low_th || acc_z >= params.imu_z_acc_hight_th)) { 
        if (!imuLiftFlag) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] Imu z axis acceleration Lift Detected! (acc_z: %.3f m/s^2)",
                acc_z
            );
        }
        imuLiftFlag = true;
    } else {
        imuLiftFlag = false;
    }

    if (irLiftFlag && imuLiftFlag) {
        errorCount++;
    } else if (errorCount > 0) { 
        errorCount--;
    }

    if (errorCount >= 10) {
        if (!liftErrorCandidate) {
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] (Error Suspected!) IR & IMU both Lift Detected! (error count: %d)",
                static_cast<int>(errorCount)
            );
            prevTime = clock.now().seconds();
        }
        liftErrorCandidate = true;
    }

    double timeDiff = clock.now().seconds() - prevTime;
    if (liftErrorCandidate) {
        if (!irLiftFlag) {
            liftErrorCandidate = false;
            RCLCPP_INFO(node_ptr_->get_logger(),
                "[LiftErrorMonitor] IR & IMU both Lift Detected BUT release in 2 sec "
                "(error count: %d, IR list duration: %.3f sec)",
                static_cast<int>(errorCount),
                timeDiff
            );
        } else {
            if (timeDiff >= 2) {
                if (!errorState) {
                    RCLCPP_INFO(node_ptr_->get_logger(),
                        "[LiftErrorMonitor] IR & IMU both Lift Detected! "
                        "(error count: %d, IR lift duration: %.3f sec)",
                        static_cast<int>(errorCount),
                        timeDiff
                    );
                }
                errorState = true;
            }
        }
    }

    std_msgs::msg::Bool msg;
    msg.data = errorState;
    error_pub_->publish(msg);
}
