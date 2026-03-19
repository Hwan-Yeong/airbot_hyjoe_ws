#include "error_monitor/error_monitor_node.hpp"

ErrorMonitorNode::ErrorMonitorNode()
    : Node("airbot_error_monitor")
{
    blackboard_ = std::make_shared<RobotStateBlackboard>();

    rclcpp::QoS qos_state_profile = rclcpp::QoS(5).reliable().durability_volatile();
    rclcpp::QoS qos_profile_ai = rclcpp::QoS(5).reliable().transient_local();
    
    // Subscriber
    bottom_ir_data_sub_ = this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
        "bottom_ir_data", rclcpp::SensorDataQoS(),
        [this](robot_custom_msgs::msg::BottomIrData::SharedPtr msg) {
            blackboard_->setIrData(*msg);
        }
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", rclcpp::SensorDataQoS(), 
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            blackboard_->setImuData(*msg);
        }
    );
    battery_status_sub_ = this->create_subscription<robot_custom_msgs::msg::BatteryStatus>(
        "/battery_status", rclcpp::SensorDataQoS(), 
        [this](robot_custom_msgs::msg::BatteryStatus::SharedPtr msg) {
            blackboard_->setBatteryData(*msg);
        }
    );
    station_data_sub_ = this->create_subscription<robot_custom_msgs::msg::StationData>(
        "/station_data", rclcpp::SensorDataQoS(), 
        [this](robot_custom_msgs::msg::StationData::SharedPtr msg) {
            blackboard_->setStationData(*msg);
        }
    );
    robot_state_sub_ = this->create_subscription<robot_custom_msgs::msg::RobotState>(
        "/state_datas", qos_state_profile, 
        [this](robot_custom_msgs::msg::RobotState::SharedPtr msg) {
            blackboard_->setRobotStateData(*msg);
        }
    );
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(), 
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            blackboard_->setOdomData(*msg);
        }
    );
    tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
        "/tof_data", rclcpp::SensorDataQoS(), 
        [this](robot_custom_msgs::msg::TofData::SharedPtr msg) {
            blackboard_->setTofData(*msg);
        }
    );
    ai_version_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/ai_version", qos_profile_ai, 
        [this](std_msgs::msg::String::SharedPtr msg) {
            blackboard_->setAiVersionData(*msg);
        }
    );
    ai_temperature_sub_ = this->create_subscription<robot_custom_msgs::msg::AiTemperature>(
        "/aitemperature_data", rclcpp::SensorDataQoS(), 
        [this](robot_custom_msgs::msg::AiTemperature::SharedPtr msg) {
            blackboard_->setAiTemperatureData(*msg);
        }
    );
    ap_temperature_sub_ = this->create_subscription<robot_custom_msgs::msg::ApTemperature>(
        "/ap_temperature_data", rclcpp::SensorDataQoS(), 
        [this](robot_custom_msgs::msg::ApTemperature::SharedPtr msg) {
            blackboard_->setApTemperatureData(*msg);
        }
    );

    // Timer
    memory_monitor_timer_ = this->create_wall_timer(std::chrono::seconds(600), std::bind(&ErrorMonitorNode::checkMemoryUsage, this));
    
    RCLCPP_INFO(this->get_logger(),
        "node initialized"
    );
}

ErrorMonitorNode::~ErrorMonitorNode()
{
    RCLCPP_INFO(this->get_logger(),
        "node terminated"
    );
}

void ErrorMonitorNode::init()
{
    RCLCPP_INFO(this->get_logger(),
        "=================== ERROR MONITOR PARAMETER ==================="
    );
    addMonitor<LowBatteryErrorMonitor>(std::make_shared<LowBatteryErrorMonitor>());
    addMonitor<FallDownErrorMonitor>(std::make_shared<FallDownErrorMonitor>());
    addMonitor<BoardOverheatErrorMonitor>(std::make_shared<BoardOverheatErrorMonitor>());
    addMonitor<BatteryDischargingErrorMonitor>(std::make_shared<BatteryDischargingErrorMonitor>());
    addMonitor<ChargingErrorMonitor>(std::make_shared<ChargingErrorMonitor>());
    addMonitor<LiftErrorMonitor>(std::make_shared<LiftErrorMonitor>());
    addMonitor<CliffDetectionErrorMonitor>(std::make_shared<CliffDetectionErrorMonitor>());
    addMonitor<TofErrorMonitor>(std::make_shared<TofErrorMonitor>());
    addMonitor<AICommunicationErrorMonitor>(std::make_shared<AICommunicationErrorMonitor>());
    RCLCPP_INFO(this->get_logger(),
        "==============================================================="
    );
}

void ErrorMonitorNode::checkMemoryUsage() {
  long mem_total = 0;
  long mem_available = 0;
  long mem_buffers = 0;
  long mem_cached = 0;

  std::ifstream meminfo("/proc/meminfo");
  if (!meminfo.is_open()) {
    RCLCPP_ERROR(this->get_logger(),
        "Could not open /proc/meminfo to check memory usage."
    );
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
    RCLCPP_ERROR(this->get_logger(),
        "Could not parse MemTotal from /proc/meminfo."
    );
    return;
  }

  long used = mem_total - mem_available;
  double used_percent = (double)used / (double)mem_total * 100.0;
  RCLCPP_INFO(this->get_logger(),
    "Memory MemTotal: %ld kB",
    mem_total
  );
  RCLCPP_INFO(this->get_logger(),
    "Memory MemAvailable: %ld kB",
    mem_available
  );
  RCLCPP_INFO(this->get_logger(),
    "Memory Buffers: %ld kB",
    mem_buffers
  );
  RCLCPP_INFO(this->get_logger(),
    "Memory Cached: %ld kB",
    mem_cached
  );
  RCLCPP_INFO(this->get_logger(),
    "Memory Usage: %.2f%% (%ld / %ld kB)-------------",
    used_percent,
    used,
    mem_total
  );
  if (used_percent > 90.0)
  {
    RCLCPP_WARN(this->get_logger(),
        "Memory usage exceeded 90%%. -->USED MEMORY[%.2f%%] OOM may occur!",
        used_percent
    );
  }
}