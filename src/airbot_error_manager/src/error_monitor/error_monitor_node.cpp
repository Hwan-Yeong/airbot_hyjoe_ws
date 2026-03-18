#include "error_monitor/error_monitor_node.hpp"

ErrorMonitorNode::ErrorMonitorNode()
    : Node("airbot_error_monitor")
{
    blackboard_ = std::make_shared<RobotStateBlackboard>();

    rclcpp::QoS qos_state_profile = rclcpp::QoS(5).reliable().durability_volatile();
    rclcpp::QoS qos_profile_ai = rclcpp::QoS(5).reliable().transient_local();
    
    // Check parameters (Optional initVariables / setParams are left empty or removed)

    // Subscriber
    bottom_ir_data_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "bottom_ir_data", 10, std::bind(&ErrorMonitorNode::bottomIrDataCallback, this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10, std::bind(&ErrorMonitorNode::imuCallback, this, std::placeholders::_1)
    );
    battery_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BatteryStatus>(
        "/battery_status", 10, std::bind(&ErrorMonitorNode::batteryCallback, this, std::placeholders::_1)
    );
    station_data_sub_ = this->create_subscription<robot_custom_msgs::msg::StationData>(
        "/station_data", 10, std::bind(&ErrorMonitorNode::stationDataCallback, this, std::placeholders::_1)
    );
    robot_state_sub_ = this->create_subscription<robot_custom_msgs::msg::RobotState>(
        "/state_datas", qos_state_profile, std::bind(&ErrorMonitorNode::robotStateCallback, this, std::placeholders::_1)
    );
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ErrorMonitorNode::odomCallback, this, std::placeholders::_1)
    );
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "/tof_data", 10, std::bind(&ErrorMonitorNode::tofCallback, this, std::placeholders::_1)
    );
    ai_version_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/ai_version", qos_profile_ai, std::bind(&ErrorMonitorNode::aiVerCallback, this, std::placeholders::_1)
    );
    ai_temperature_sub_ = this->create_subscription<robot_custom_msgs::msg::AiTemperature>(
        "/aitemperature_data", 10, std::bind(&ErrorMonitorNode::aiTemperatureCallback, this, std::placeholders::_1)
    );
    ap_temperature_sub_ = this->create_subscription<robot_custom_msgs::msg::ApTemperature>(
        "/ap_temperature_data", 10, std::bind(&ErrorMonitorNode::apTemperatureCallback, this, std::placeholders::_1)
    );

    // Timer
    memory_monitor_timer_ = this->create_wall_timer(std::chrono::seconds(600), std::bind(&ErrorMonitorNode::checkMemoryUsage, this));
    sensor_delay_check_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ErrorMonitorNode::checkSensorDelays, this));
    
    RCLCPP_INFO(this->get_logger(), "node initialized");
}

ErrorMonitorNode::~ErrorMonitorNode()
{
    RCLCPP_INFO(this->get_logger(), "node terminated");
}

void ErrorMonitorNode::init()
{
    RCLCPP_INFO(this->get_logger(), "=================== ERROR MONITOR PARAMETER ===================");
    addMonitor<LowBatteryErrorMonitor>(std::make_shared<LowBatteryErrorMonitor>());
    addMonitor<FallDownErrorMonitor>(std::make_shared<FallDownErrorMonitor>());
    addMonitor<BoardOverheatErrorMonitor>(std::make_shared<BoardOverheatErrorMonitor>());
    addMonitor<BatteryDischargingErrorMonitor>(std::make_shared<BatteryDischargingErrorMonitor>());
    addMonitor<ChargingErrorMonitor>(std::make_shared<ChargingErrorMonitor>());
    addMonitor<LiftErrorMonitor>(std::make_shared<LiftErrorMonitor>());
    addMonitor<CliffDetectionErrorMonitor>(std::make_shared<CliffDetectionErrorMonitor>());
    addMonitor<TofErrorMonitor>(std::make_shared<TofErrorMonitor>());
    addMonitor<AICommunicationErrorMonitor>(std::make_shared<AICommunicationErrorMonitor>());
    RCLCPP_INFO(this->get_logger(), "===============================================================");
}

void ErrorMonitorNode::initVariables()
{
}

void ErrorMonitorNode::setParams()
{
}

void ErrorMonitorNode::batteryCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg)
{
    blackboard_->setBatteryData(*msg);
}

void ErrorMonitorNode::bottomIrDataCallback(const robot_custom_msgs::msg::BottomIrData::SharedPtr msg)
{
    blackboard_->setIrData(*msg);
}

void ErrorMonitorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    blackboard_->setImuData(*msg);
}

void ErrorMonitorNode::stationDataCallback(const robot_custom_msgs::msg::StationData::SharedPtr msg)
{
    blackboard_->setStationData(*msg);
}

void ErrorMonitorNode::robotStateCallback(const robot_custom_msgs::msg::RobotState::SharedPtr msg)
{
    blackboard_->setRobotStateData(*msg);
}

void ErrorMonitorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    blackboard_->setOdomData(*msg);
}

void ErrorMonitorNode::tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    blackboard_->setTofData(*msg);
}

void ErrorMonitorNode::aiVerCallback(const std_msgs::msg::String::SharedPtr msg)
{
    blackboard_->setAiVersionData(*msg);
}

void ErrorMonitorNode::aiTemperatureCallback(const robot_custom_msgs::msg::AiTemperature::SharedPtr msg)
{
    blackboard_->setAiTemperatureData(*msg);
}

void ErrorMonitorNode::apTemperatureCallback(const robot_custom_msgs::msg::ApTemperature::SharedPtr msg)
{
    blackboard_->setApTemperatureData(*msg);
}

void ErrorMonitorNode::checkMemoryUsage() {
  long mem_total = 0;
  long mem_available = 0;
  long mem_buffers = 0;
  long mem_cached = 0;

  std::ifstream meminfo("/proc/meminfo");
  if (!meminfo.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open /proc/meminfo to check memory usage.");
    return;
  }

  std::stringstream full_meminfo_content;
  std::string line;
  while (std::getline(meminfo, line))
  {
    full_meminfo_content << line << "\n"; //meminfo stream 저장.
    std::istringstream iss(line);
    std::string key;
    long value;
    std::string unit;

    if (iss >> key >> value >> unit)
    {
      if (key == "MemTotal:"){
        mem_total = value;
      } else if (key == "MemAvailable:"){
        mem_available = value;
      } else if (key == "Buffers:"){
        mem_buffers = value;
      } else if (key == "Cached:"){
        mem_cached = value;
      }
    }
  }

  if (mem_total == 0) {
    RCLCPP_ERROR(this->get_logger(), "Could not parse MemTotal from /proc/meminfo.");
    return;
  }

  long used = mem_total - mem_available;
  double used_percent = (double)used / (double)mem_total * 100.0;
  RCLCPP_INFO(this->get_logger(), "Memory MemTotal: %ld kB", mem_total);
  RCLCPP_INFO(this->get_logger(), "Memory MemAvailable: %ld kB", mem_available);
  RCLCPP_INFO(this->get_logger(), "Memory Buffers: %ld kB", mem_buffers);
  RCLCPP_INFO(this->get_logger(), "Memory Cached: %ld kB", mem_cached);
  RCLCPP_INFO(this->get_logger(), "Memory Usage: %.2f%% (%ld / %ld kB)-------------", used_percent, used, mem_total);
  if (used_percent > 90.0)
  {
    RCLCPP_WARN(this->get_logger(),"Memory usage exceeded 90%%. -->USED MEMORY[%.2f%%] OOM may occur!", used_percent);
  }
}

void ErrorMonitorNode::checkSensorDelays() {
    auto now = std::chrono::steady_clock::now();
    
    auto check_delay = [this, now](const auto& data, const std::string& name, int limit_ms) {
        // if (data.last_update_time.time_since_epoch().count() == 0) {
        //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        //                          "[%s] Data NOT received yet!", name.c_str());
        //     return;
        // }
        auto delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - data.last_update_time).count();
        if (delay_ms > limit_ms) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "[%s] Update delayed! (Delay: %ld ms, Limit: %d ms)", name.c_str(), delay_ms, limit_ms);
        }
    };

    check_delay(blackboard_->getIrData(), "bottom_ir", 30);
    check_delay(blackboard_->getImuData(), "imu", 30);
    check_delay(blackboard_->getBatteryData(), "battery", 30);
    check_delay(blackboard_->getStationData(), "station", 30);
    check_delay(blackboard_->getOdomData(), "odom", 30);
    check_delay(blackboard_->getTofData(), "tof", 30);
    
    check_delay(blackboard_->getAiVersionData(), "ai_version", 120);
    check_delay(blackboard_->getAiTemperatureData(), "aitemperature", 120);
    
    check_delay(blackboard_->getApTemperatureData(), "ap_temperature", 300);
}