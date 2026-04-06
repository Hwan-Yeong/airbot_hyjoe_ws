#pragma once

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>
#include <vector>
#include <mutex>

#include "error_monitor/monitors/ai_communication.hpp"
#include "error_monitor/monitors/battery_discharging.hpp"
#include "error_monitor/monitors/board_overheat.hpp"
#include "error_monitor/monitors/charging.hpp"
#include "error_monitor/monitors/cliff.hpp"
#include "error_monitor/monitors/fall_down.hpp"
#include "error_monitor/monitors/lift.hpp"
#include "error_monitor/monitors/low_battery.hpp"
#include "error_monitor/monitors/one_d_tof.hpp"
#include "error_monitor/robot_state_blackboard.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class ErrorMonitorNode
 * @brief 모든 에러 모니터링 활동을 관리하는 중앙 ROS 2 노드입니다.
 * 
 * 이 노드는 에러 모니터링 시스템의 주 진입점이자 중앙 데이터 중개자 역할을 합니다. 
 * 주요 목적은 ROS 2 네트워크로부터 필요한 모든 센서 및 상태 토픽을 구독(subscribe)하고, 
 * 공유 데이터 구조인 `RobotStateBlackboard`를 지속적으로 업데이트하는 것입니다.
 * 
 * 사용법:
 * 이 노드는 내부에 개별적인 에러 평가 로직을 직접 포함하지 않고 매니저 역할만 수행합니다. 
 * 규칙별로 특화된 개별 모니터들(`ErrorMonitorBase` 파생 클래스)을 인스턴스화하고, 
 * 이들에게 `RobotStateBlackboard`의 참조를 넘겨주어 독립적으로 작동하게 합니다. 
 * 이를 통해 네트워크 I/O 작업과 실제 에러 필터링 수학 로직을 분리하여 모듈성을 높입니다.
 */
class ErrorMonitorNode : public rclcpp::Node {
 public:
  ErrorMonitorNode();
  ~ErrorMonitorNode();

  /**
   * @brief 모든 기본 에러 모니터들을 등록하고 시작하여 노드를 초기화합니다.
   */
  void init();

  /**
   * @brief 개별 에러 모니터를 노드의 실행 파이프라인에 등록합니다.
   * 
   * 동적 다형성(Dynamic polymorphism)을 활용하여 모니터를 ROS 2 생태계에 연결합니다. 
   * 자동으로 노드 포인터를 모니터에 주입하고, YAML 설정 파라미터를 파싱하도록 지시하며, 
   * 해당 파라미터를 콘솔에 출력하게 한 뒤 모니터의 내부 콜백 루프를 시작시킵니다.
   * 
   * @param monitor `ErrorMonitorBase`를 상속받은 임의의 모니터 클래스의 shared ptr
   * @param config 모니터들이 공통으로 사용할 수 있는 전체가 파싱된 YAML Node
   */
  void addMonitor(std::shared_ptr<ErrorMonitorBase> monitor,
                  const YAML::Node& config);

  /**
   * @brief 특정 namespace의 모니터를 리스트에서 제거하고 stopMonitor를 호출합니다.
   * 
   * @param target_namespace 제거할 모니터의 네임스페이스
   * @return true 제거 성공
   * @return false 제거 실패
   */
  bool removeMonitor(const std::string& target_namespace);

 private:
  /**
   * @brief 프로세스의 내부 리소스 및 메모리 사용량을 주기적으로 로깅합니다.
   */
  void checkMemoryUsage();

  std::shared_ptr<RobotStateBlackboard> blackboard_;

  rclcpp::Subscription<robot_custom_msgs::msg::BottomIrData>::SharedPtr
      bottom_ir_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::BatteryStatus>::SharedPtr
      battery_status_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr
      station_data_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::RobotState>::SharedPtr
      robot_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ai_version_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::AiTemperature>::SharedPtr
      ai_temperature_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::ApTemperature>::SharedPtr
      ap_temperature_sub_;

  std::vector<std::shared_ptr<ErrorMonitorBase>> monitors_;
  std::mutex monitors_mutex_;

  rclcpp::TimerBase::SharedPtr memory_monitor_timer_;
};
