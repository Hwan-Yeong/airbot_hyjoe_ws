#include "scan_monitor.hpp"

ScanMonitorNode::ScanMonitorNode()
: Node("scan_monitor_node")
{
    double expected_interval_ms = 100.0;
    double tolerance = 0.10;
    lower_bound_ms_ = expected_interval_ms * (1.0 - tolerance);
    upper_bound_ms_ = expected_interval_ms * (1.0 + tolerance);
    timeout_ms_ = expected_interval_ms * 3.0;  // 예: 3번 이상 못 받으면 경고
    monitor_enabled_ = false;
    scan_front_state_ = false;
    scan_back_state_ = false;
    prev_scan_time_ = {};
    last_scan_time_ = {};
    scanOk = false;

    scan_front_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_state_front", 10,
        std::bind(&ScanMonitorNode::scanFrontStateCallback, this, std::placeholders::_1)
    );

    scan_back_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_state_back", 10,
        std::bind(&ScanMonitorNode::scanBackStateCallback, this, std::placeholders::_1)
    );

    scan_monitor_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/scan_monitor_cmd", 10,
        std::bind(&ScanMonitorNode::cmdCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        std::bind(&ScanMonitorNode::scanCallback, this, std::placeholders::_1)
    );

    monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&ScanMonitorNode::checkScanHealth, this)
    );

    RCLCPP_INFO(this->get_logger(), "Scan Monitor Node initialized. Expecting 10Hz (±10%%)");
}

void ScanMonitorNode::cmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    scanOk = false;
    scan_received_ = false;
    last_scan_time_ = {};
    monitor_enabled_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "SCAN Monitor %s", monitor_enabled_ ? "ENABLED" : "DISABLED");
}

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
    scan_received_ = true;
}

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

    if(!scan_received_){
        if (prev_scan_time_.time_since_epoch().count() == 0) {
            RCLCPP_INFO(this->get_logger(), "[checkScanHealth] wait scan received...");
        }else{
            RCLCPP_INFO(this->get_logger(), "[checkScanHealth] LIdar is On. but can`t received scan.");
        }
        return;
    }

    if (prev_scan_time_.time_since_epoch().count() == 0) {
        RCLCPP_INFO(this->get_logger(), "[checkScanHealth] prev_scan_time doesn`t set. wait for two times scan received.");
        return;
    }

    if (elapsed_ms > timeout_ms_) {
        scanOk = false;
        last_scan_time_ = {};
        RCLCPP_ERROR(this->get_logger(), "[checkScanHealth] No scan received for %.2f ms! Sensor or driver may be down.", elapsed_ms);
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMonitorNode>());
    rclcpp::shutdown();
    return 0;
}