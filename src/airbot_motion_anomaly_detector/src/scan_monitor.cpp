#include "scan_monitor.hpp"

ScanMonitorNode::ScanMonitorNode()
: Node("scan_monitor_node")
{
    double expected_interval_ms = 100.0;
    double tolerance = 0.10;
    lower_bound_ms_ = expected_interval_ms * (1.0 - tolerance);
    upper_bound_ms_ = expected_interval_ms * (1.0 + tolerance);
    timeout_ms_ = expected_interval_ms * 3.0;  // 예: 3번 이상 못 받으면 경고
    scan_front_state_ = false;
    scan_back_state_ = false;
    prev_scan_time_ = {};
    last_scan_time_ = {};
    #if USE_LIDAR_STATE_CHECK > 0
    lidar_state = 0;
    bLidarCmd = true; //hjkim : airbot_lidar에서 respwan 적용하면서 default를 ON으로 변경하여, TRUE로 초기화

    cmd_lidar_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"cmd_lidar", 10, std::bind(&ScanMonitorNode::lidar_cmd_callback, this, std::placeholders::_1));
    #else
    monitor_enabled_ = false;
    scanOk = false;
    
    scan_monitor_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_monitor_cmd", 10,
        std::bind(&ScanMonitorNode::cmdCallback, this, std::placeholders::_1));
    #endif
    scan_front_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_state_front", 10,
        std::bind(&ScanMonitorNode::scanFrontStateCallback, this, std::placeholders::_1)
    );

    scan_back_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_state_back", 10,
        std::bind(&ScanMonitorNode::scanBackStateCallback, this, std::placeholders::_1)
    );

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        std::bind(&ScanMonitorNode::scanCallback, this, std::placeholders::_1)
    );

    lidar_state_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/lidar_state", 10);
    scanHz_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/scanHz_state", 10);

    monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&ScanMonitorNode::checkScanHealth, this)
    );

    RCLCPP_INFO(this->get_logger(), "Scan Monitor Node initialized. Expecting 10Hz (±10%%)");
}

#if USE_LIDAR_STATE_CHECK > 0
void ScanMonitorNode::lidar_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data){
        bLidarCmd = true;
        RCLCPP_INFO(this->get_logger(), "[lidar_cmd_callback] Lidar ON");
    }
    else{
        bLidarCmd = false;
        RCLCPP_INFO(this->get_logger(), "[lidar_cmd_callback] Lidar OFF");
    }
    isScanHzOk = false;
    hz_check_count_ = 0;
    last_scan_time_ = {};
}
#else
void ScanMonitorNode::cmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    last_scan_time_ = {};
    monitor_enabled_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "SCAN Monitor %s", monitor_enabled_ ? "ENABLED" : "DISABLED");
}
#endif

void ScanMonitorNode::scanFrontStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool new_scan_state = msg->data;
    if(scan_front_state_ != new_scan_state){
        RCLCPP_INFO(this->get_logger(), "[scanFrontStateCallback] Front State %s", new_scan_state ? "ON" : "OFF");
    }
    scan_front_state_ = msg->data;
    
}

void ScanMonitorNode::scanBackStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool new_scan_state = msg->data;
    if(scan_back_state_ != new_scan_state){
        RCLCPP_INFO(this->get_logger(), "[scanBackStateCallback] Back State %s", new_scan_state ? "ON" : "OFF");
    }
    scan_back_state_ = msg->data;
}

void ScanMonitorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if(!msg){
       RCLCPP_INFO(this->get_logger(), "[scanCallback] ERROR : received empty scan message.");
       return; 
    }
    
    if(msg->ranges.empty()){
       RCLCPP_INFO(this->get_logger(), "[scanCallback] ERROR : received empty scan data.");
       return; 
    }

    prev_scan_time_ = last_scan_time_;
    last_scan_time_ = std::chrono::steady_clock::now();
}

#if USE_LIDAR_STATE_CHECK > 0
void ScanMonitorNode::checkScanHealth()
{
    checkScanState();
    checkScanHz();
}

void ScanMonitorNode::checkScanState()
{
    /* lidar_state
    0: OFF
    1: ON
    2: STARTING
    3: STOPING
    */
    if(scan_front_state_ && scan_back_state_){
        if(lidar_state != 1){
            RCLCPP_INFO(this->get_logger(), "[checkScanState] Lidar ON, prevState[%d](0:OFF, 1:ON, 2:STARTING, 3:STOPING) lidarCmd[%s] Front[%s] Back[%s]",
            lidar_state, bLidarCmd ? "ON" : "OFF", scan_front_state_ ? "ON" : "OFF", scan_back_state_ ? "ON" : "OFF");
        }
        lidar_state = 1;
    }else if(!scan_front_state_ && !scan_back_state_){
        if(lidar_state != 0){
            RCLCPP_INFO(this->get_logger(), "[checkScanState] Lidar OFF, prevState[%d](0:OFF, 1:ON, 2:STARTING, 3:STOPING) lidarCmd[%s] Front[%s] Back[%s]",
            lidar_state, bLidarCmd ? "ON" : "OFF", scan_front_state_ ? "ON" : "OFF", scan_back_state_ ? "ON" : "OFF");
        }
        lidar_state = 0;
    }else if(bLidarCmd){
        if(lidar_state != 2){
            RCLCPP_INFO(this->get_logger(), "[checkScanState] Lidar STARTING, prevState[%d](0:OFF, 1:ON, 2:STARTING, 3:STOPING) lidarCmd[%s] Front[%s] Back[%s]",
            lidar_state, bLidarCmd ? "ON" : "OFF", scan_front_state_ ? "ON" : "OFF", scan_back_state_ ? "ON" : "OFF");
        }
        lidar_state = 2;
    }else{
        if(lidar_state != 3){
            RCLCPP_INFO(this->get_logger(), "[checkScanState] Lidar STOPPING, prevState[%d](0:OFF, 1:ON, 2:STARTING, 3:STOPING) lidarCmd[%s] Front[%s] Back[%s]",
            lidar_state, bLidarCmd ? "ON" : "OFF", scan_front_state_ ? "ON" : "OFF", scan_back_state_ ? "ON" : "OFF");
        }
        lidar_state = 3;
    }
    publishLidarState(lidar_state);
}
void ScanMonitorNode::checkScanHz()
{
    if(!scan_front_state_ || !scan_back_state_){
        hz_check_count_ = 0;
        isScanHzOk = false;
        if(bLidarCmd){
            RCLCPP_INFO(this->get_logger(), "[checkScanHz] scan state is front[%s], back[%s]", 
            scan_front_state_ ? "ON" : "OFF", scan_back_state_ ? "ON" : "OFF" );
        }
        publishScanHzState(isScanHzOk);
        return;
    }

    if (prev_scan_time_.time_since_epoch().count() == 0 || last_scan_time_.time_since_epoch().count() == 0) {
        RCLCPP_INFO(this->get_logger(), "[checkScanHz] wait for two times scan received.");
        publishScanHzState(isScanHzOk);
        return;
    }

    double interval_ms = std::chrono::duration<double, std::milli>(last_scan_time_ - prev_scan_time_).count();
    
    constexpr double kMinDt = 1e-3; // 1 ms
    if (interval_ms < kMinDt) {
        if(hz_check_count_ > 0){
            hz_check_count_--;
        }else{
            isScanHzOk = false;
        }
        RCLCPP_INFO(this->get_logger(),"[checkScanHz] scan interval too small  interval(%.2f)ms, count[%u], isScanHzOk[%s]"
                    ,interval_ms,hz_check_count_,isScanHzOk? "TRUE" : "FALSE");
        publishScanHzState(isScanHzOk);
        return;
    }

    double hz = 1000.0 / interval_ms;
    if (interval_ms < lower_bound_ms_ || interval_ms > upper_bound_ms_) {
        if(hz_check_count_ > 0){
            hz_check_count_--;
        }else{
            isScanHzOk = false;
        }
        RCLCPP_INFO(this->get_logger(),"[checkScanHz] scan hz not in expected range interval(%.2f)ms hz(%.2f) count[%u] isScanHzOk[%s]"
                    ,interval_ms, hz,hz_check_count_,isScanHzOk? "TRUE" : "FALSE");
    }else{
        if(hz_check_count_ >= 3){
            isScanHzOk = true;
        }else{
            hz_check_count_++;
            RCLCPP_INFO(this->get_logger(),"[checkScanHz] scan is good... interval(%.2f)ms hz(%.2f) count[%u] isScanHzOk[%s]"
                        ,interval_ms, hz,hz_check_count_,isScanHzOk? "TRUE" : "FALSE"); 
        }
    }
    publishScanHzState(isScanHzOk);
}

void ScanMonitorNode::publishLidarState(uint8_t state)
{
    static uint8_t prev_msg_state = 0;
    std_msgs::msg::UInt8 lidar_state_msg;
    lidar_state_msg.data = state;
    lidar_state_pub_->publish(lidar_state_msg);

    if(prev_msg_state != state){
        RCLCPP_INFO(this->get_logger(), "[publishLidarState] state (0:OFF, 1:ON, 2:STARTING, 3:STOPING): %u", state);
    }
    prev_msg_state = state;
}

void ScanMonitorNode::publishScanHzState(bool state)
{
    static bool prev_msg_state = false;
    std_msgs::msg::Bool scanHz_state_msg;
    scanHz_state_msg.data = state;
    scanHz_state_pub_->publish(scanHz_state_msg);
    if(prev_msg_state != state){
        RCLCPP_INFO(this->get_logger(), "[publishScanHzState] state: %s", state ? "OK" : "NOT OK");
    }
    prev_msg_state = state;
}
#else
void ScanMonitorNode::checkScanHealth()
{
    if (!monitor_enabled_)
        return;
    
    auto now = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(now - last_scan_time_).count();

    if(!scan_front_state_ || !scan_back_state_){
        if(scanOk){
           RCLCPP_INFO(this->get_logger(), "[checkScanHealth] Need to Check LIDAR State...Front[%s] Back[%s]", scan_front_state_ ? "ON" : "OFF", scan_back_state_ ? "ON" : "OFF");
        }
        scanOk = false;
        last_scan_time_ = {};
        return;
    }

    if (prev_scan_time_.time_since_epoch().count() == 0) {
        RCLCPP_INFO(this->get_logger(), "[checkScanHealth] prev_scan_time doesn`t set. wait for two times scan received.");
        return;
    }

    if (elapsed_ms > timeout_ms_) {
        scanOk = false;
        //last_scan_time_ = {};
        RCLCPP_ERROR(this->get_logger(), "[checkScanHealth] No scan received for %.2f ms", elapsed_ms);
        return;
    }

    double interval_ms = std::chrono::duration<double, std::milli>(last_scan_time_ - prev_scan_time_).count();
    double hz = 1000.0 / interval_ms;
    if (interval_ms < lower_bound_ms_ || interval_ms > upper_bound_ms_) {
        scanOk = false;
        RCLCPP_INFO(this->get_logger(),"[checkScanHealth] scan hz not in expected range : %.2f ms (%.2f Hz)",interval_ms, hz);
    }else{
        if(!scanOk){
            RCLCPP_INFO(this->get_logger(),"[checkScanHealth] scan is good... hz : %.2f ms (%.2f Hz)",interval_ms, hz);
            scanOk = true;
        }
    }
}
#endif

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMonitorNode>());
    rclcpp::shutdown();
    return 0;
}