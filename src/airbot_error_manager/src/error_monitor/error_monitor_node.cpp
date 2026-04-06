#include "error_monitor/error_monitor_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

ErrorMonitorNode::ErrorMonitorNode() : Node("airbot_error_monitor") {
  blackboard_ = std::make_shared<RobotStateBlackboard>();

  rclcpp::QoS qos_state_profile =
      rclcpp::QoS(5).reliable().durability_volatile();
  rclcpp::QoS qos_profile_ai = rclcpp::QoS(5).reliable().transient_local();

  // Subscriber
  bottom_ir_data_sub_ =
      this->create_subscription<robot_custom_msgs::msg::BottomIrData>(
          "bottom_ir_data", rclcpp::SensorDataQoS(),
          [this](robot_custom_msgs::msg::BottomIrData::SharedPtr msg) {
            blackboard_->setIrData(*msg);
          });
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        blackboard_->setImuData(*msg);
      });
  battery_status_sub_ =
      this->create_subscription<robot_custom_msgs::msg::BatteryStatus>(
          "/battery_status", rclcpp::SensorDataQoS(),
          [this](robot_custom_msgs::msg::BatteryStatus::SharedPtr msg) {
            blackboard_->setBatteryData(*msg);
          });
  station_data_sub_ =
      this->create_subscription<robot_custom_msgs::msg::StationData>(
          "/station_data", rclcpp::SensorDataQoS(),
          [this](robot_custom_msgs::msg::StationData::SharedPtr msg) {
            blackboard_->setStationData(*msg);
          });
  robot_state_sub_ =
      this->create_subscription<robot_custom_msgs::msg::RobotState>(
          "/state_datas", qos_state_profile,
          [this](robot_custom_msgs::msg::RobotState::SharedPtr msg) {
            blackboard_->setRobotStateData(*msg);
          });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        blackboard_->setOdomData(*msg);
      });
  tof_sub_ = this->create_subscription<robot_custom_msgs::msg::TofData>(
      "/tof_data", rclcpp::SensorDataQoS(),
      [this](robot_custom_msgs::msg::TofData::SharedPtr msg) {
        blackboard_->setTofData(*msg);
      });
  ai_version_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/ai_version", qos_profile_ai,
      [this](std_msgs::msg::String::SharedPtr msg) {
        blackboard_->setAiVersionData(*msg);
      });
  ai_temperature_sub_ =
      this->create_subscription<robot_custom_msgs::msg::AiTemperature>(
          "/aitemperature_data", rclcpp::SensorDataQoS(),
          [this](robot_custom_msgs::msg::AiTemperature::SharedPtr msg) {
            blackboard_->setAiTemperatureData(*msg);
          });
  ap_temperature_sub_ =
      this->create_subscription<robot_custom_msgs::msg::ApTemperature>(
          "/ap_temperature_data", rclcpp::SensorDataQoS(),
          [this](robot_custom_msgs::msg::ApTemperature::SharedPtr msg) {
            blackboard_->setApTemperatureData(*msg);
          });

  // Timer
  memory_monitor_timer_ = this->create_wall_timer(
      std::chrono::seconds(600), // 10 minute
      std::bind(&ErrorMonitorNode::checkMemoryUsage, this));

  RCLCPP_INFO(this->get_logger(), "node initialized");
}

ErrorMonitorNode::~ErrorMonitorNode() {
  RCLCPP_INFO(this->get_logger(), "node terminated");
}

void ErrorMonitorNode::init() {
  YAML::Node config;
  try {
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("airbot_error_manager");
    std::string full_path =
        package_share_directory + "/config/error_manager_params.yaml";
    config =
        YAML::LoadFile(full_path)["airbot_error_monitor"]["ros__parameters"];
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to load error_manager_params.yaml: %s", e.what());
  }

  addMonitor(std::make_shared<LowBatteryErrorMonitor>(), config);
  addMonitor(std::make_shared<FallDownErrorMonitor>(), config);
  addMonitor(std::make_shared<BoardOverheatErrorMonitor>(), config);
  addMonitor(std::make_shared<BatteryDischargingErrorMonitor>(), config);
  addMonitor(std::make_shared<ChargingErrorMonitor>(), config);
  addMonitor(std::make_shared<LiftErrorMonitor>(), config);
  addMonitor(std::make_shared<CliffErrorMonitor>(), config);
  addMonitor(std::make_shared<OneDTofErrorMonitor>(), config);
  addMonitor(std::make_shared<AICommunicationErrorMonitor>(), config);
  RCLCPP_INFO(this->get_logger(), "ERROR MONITORs INITIALIZED");
}

void ErrorMonitorNode::addMonitor(std::shared_ptr<ErrorMonitorBase> monitor,
                const YAML::Node& config) {
  monitor->setNode(this);
  std::string ns = monitor->paramNamespace();

  // 1. Thread-safe lock & 중복 등록 방지
  std::lock_guard<std::mutex> lock(monitors_mutex_);
  
  for (const auto& m : monitors_) {
    if (m->paramNamespace() == ns) {
      RCLCPP_WARN(this->get_logger(),
                  "Monitor '%s' is already registered. Ignoring add request.",
                  ns.c_str());
      return;
    }
  }

  // 2. 파라미터 로드
  if (config && config[ns]) {
    monitor->loadParams(config[ns]);
  } else {
    RCLCPP_WARN(this->get_logger(),
                "No YAML config found for monitor: %s, using defaults",
                ns.c_str());
    YAML::Node empty_node;
    monitor->loadParams(empty_node);
  }
  monitor->printParams();
  
  // 3. 모니터 시작 및 컨테이너에 추가
  monitor->startMonitor(blackboard_);
  monitors_.push_back(monitor);
}

/**
 * @note target_namespace : error_manager_params.yaml 의 namespace와 일치해야 함
 *                          각 monitor의 순수 가상함수 paramNamespace에 기재.
 */
bool ErrorMonitorNode::removeMonitor(const std::string& target_namespace) {
  std::lock_guard<std::mutex> lock(monitors_mutex_);
  
  bool removed = false;
  
  // std::remove_if의 부작용(이동된 요소를 순회하는 문제)을 방지하기 위해 일반 순회 방식 사용.
  for (auto it = monitors_.begin(); it != monitors_.end(); ) {
    if ((*it)->paramNamespace() == target_namespace) {
      // 리스트에서 지우기 전에 먼저 stopMonitor 호출 (Segfault 방지)
      (*it)->stopMonitor();
      RCLCPP_INFO(this->get_logger(), 
                  "Successfully stopped and removed monitor: '%s'", 
                  (*it)->paramNamespace().c_str());
      it = monitors_.erase(it);
      removed = true;
    } else {
      ++it;
    }
  }

  if (!removed) {
    RCLCPP_WARN(this->get_logger(), 
                "Failed to remove monitor. Namespace '%s' not found.", 
                target_namespace.c_str());
    return false;
  }
  
  return true;
}

void ErrorMonitorNode::checkMemoryUsage() {
  long mem_total = 0;
  long mem_available = 0;
  long mem_buffers = 0;
  long mem_cached = 0;

  std::ifstream meminfo("/proc/meminfo");
  if (!meminfo.is_open()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not open /proc/meminfo to check memory usage.");
    return;
  }

  std::stringstream full_meminfo_content;
  std::string line;
  while (std::getline(meminfo, line)) {
    full_meminfo_content << line << "\n";  //meminfo stream 저장.
    std::istringstream iss(line);
    std::string key;
    long value;
    std::string unit;

    if (iss >> key >> value >> unit) {
      if (key == "MemTotal:") {
        mem_total = value;
      } else if (key == "MemAvailable:") {
        mem_available = value;
      } else if (key == "Buffers:") {
        mem_buffers = value;
      } else if (key == "Cached:") {
        mem_cached = value;
      }
    }
  }

  if (mem_total == 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not parse MemTotal from /proc/meminfo.");
    return;
  }

  long used = mem_total - mem_available;
  double used_percent = (double)used / (double)mem_total * 100.0;
  RCLCPP_INFO(this->get_logger(), "Memory MemTotal: %ld kB", mem_total);
  RCLCPP_INFO(this->get_logger(), "Memory MemAvailable: %ld kB", mem_available);
  RCLCPP_INFO(this->get_logger(), "Memory Buffers: %ld kB", mem_buffers);
  RCLCPP_INFO(this->get_logger(), "Memory Cached: %ld kB", mem_cached);
  RCLCPP_INFO(this->get_logger(),
              "Memory Usage: %.2f%% (%ld / %ld kB)-------------", used_percent,
              used, mem_total);
  if (used_percent > 90.0) {
    RCLCPP_WARN(
        this->get_logger(),
        "Memory usage exceeded 90%%. -->USED MEMORY[%.2f%%] OOM may occur!",
        used_percent);
  }
}