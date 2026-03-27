#pragma once

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <initializer_list>
#include <memory>
#include <string>
#include "error_monitor/robot_state_blackboard.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

/**
 * @enum DataState
 * @brief 데이터의 실시간 통신 상태(freshness)를 나타냅니다.
 */
enum class DataState { NORMAL, DELAYED, TIMEOUT };

/**
 * @class ErrorMonitorBase
 * @brief 모든 개별 에러 모니터들의 기본 형태를 정의하는 Abstract base class 입니다.
 * 
 * 목적:
 * 이 클래스는 모든 특화된 에러 모니터링 컴포넌트(예: LowBattery, FallDown, ...)가 
 * 표준화된 실행 주기를 따르도록 엄격하고 통일된 인터페이스를 제공합니다.
 * 
 * 사용법:
 * 새로운 모니터를 추가하려면 반드시 이 클래스를 상속받고 순수 가상 함수들 
 * (`paramNamespace`, `loadParams`, `printParams`, `startMonitor`)
 * 을 구현해야 합니다.
 * 이를 통해 메인 `ErrorMonitorNode`는 개별 모니터의 내부 로직을 몰라도 
 * 다형성(polymorphism)을 활용하여 여러 모니터들을 일괄적으로 관리할 수 있습니다.
 * 
 * 또한 자식 클래스에서 중복하여 사용되는 센서들의 타임아웃 검사를 위해
 * `checkSensorState()`와 같은 내장 유틸리티 함수도 제공합니다. (사용 여부 선택사항)
 */
class ErrorMonitorBase {
 public:
  virtual ~ErrorMonitorBase() = default;

  /**
   * @brief 모니터에 적용될 지정된 ROS 파라미터 네임스페이스를 선언합니다. (필수 재정의)
   * 
   * 이 순수 가상 함수는 개발자가 자신의 모니터에 연결된 파라미터 블록 이름을 
   * 반드시 정의하도록 강제합니다.
   * 
   * @note `config/error_manager_params.yaml`
   *       에 정의된 namespace key와 정확히 일치해야 합니다.
   * @return const std::string 파라미터 네임스페이스 문자열 (예: "fall_down_error").
   */
  virtual const std::string paramNamespace() const = 0;

  /**
   * @brief 파일(YAML)로부터 알고리즘 구동에 필요한 파라미터들을 가져옵니다.
   * @param config 해당 모니터를 위한 파라미터를 담고 있는 YAML::Node 객체.
   */
  virtual void loadParams(const YAML::Node& config) = 0;

  /**
   * @brief 올바르게 초기화되었는지 확인하기 위해 로드된 파라미터들을 출력합니다.
   */
  virtual void printParams() const = 0;

  /**
   * @brief 모니터의 내부 Monitor 루프를 시작하고 blackboard에 바인딩합니다.
   * @param blackboard 실시간 센서 데이터가 모여있는 중앙 저장소의 shared pointer.
   */
  virtual void startMonitor(
      std::shared_ptr<RobotStateBlackboard> blackboard) = 0;

  /**
   * @brief 부모 ROS 노드의 원시 포인터(raw pointer)를 모니터에 주입합니다.
   * 
   * 모니터와 노드 간의 순환 참조(Cyclic reference)로 인한 메모리 누수를 방지하기 위해
   * `shared_ptr` 대신 원시 포인터(`rclcpp::Node*`)를 사용합니다.
   * 
   * @param node `ErrorMonitorNode`의 원시 포인터.
   */
  void setNode(rclcpp::Node* node) { node_ptr_ = node; }

 protected:
  rclcpp::Node* node_ptr_ = nullptr;
  std::shared_ptr<RobotStateBlackboard> blackboard_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  DataState current_sensor_state_ = DataState::NORMAL;

  /**
   * @brief 센서 데이터 스트림이 정상인지, 지연(Delayed)되었는지,
   *        혹은 끊겼는지(Timeout) 평가하는 중앙 헬퍼 함수입니다.
   * 
   * 임의의 개수의 센서 업데이트 타임스탬프를 검사하여
   * 현재 시스템 시간 대비 최대 지연 시간을 산출하고, 그 상태를 3단계 상태로 분류합니다.
   * 상태 전환이 발생할 경우 일회성 콘솔 로깅도 자동으로 처리합니다.
   * 
   * 조건값(Thresholds):
   * - DELAYED (지연): 최대 지연 시간이 `input_data_period_ms * 10`을 초과할 때 발생.
   * - TIMEOUT (시간초과): 최대 지연 시간이 `input_data_period_ms * 100`을 초과할 때 발생.
   * 
   * @param monitor_name 이 검사를 실행시키는 부모 모니터의 이름 (로깅 컨텍스트 용도).
   * @param input_data_period_ms 입력값들의 이상적인 업데이트 주기 (밀리초 단위).
   * @param stamps 입력값들의 `steady_clock::time_point` 객체들이 담긴 초기화 리스트.
   * @return DataState NORMAL, DELAYED, 또는 TIMEOUT을 반환합니다.
   */
  DataState checkSensorState(
      const std::string& monitor_name, int input_data_period_ms,
      std::initializer_list<std::chrono::steady_clock::time_point> stamps) {
    if (stamps.size() == 0)
      return DataState::NORMAL;

    auto now = std::chrono::steady_clock::now();
    long max_delay_ms = 0;

    for (const auto& stamp : stamps) {
      if (stamp.time_since_epoch().count() == 0) {
        max_delay_ms = std::max(max_delay_ms, 999999L);
        continue;
      }
      auto delay_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - stamp)
              .count();
      max_delay_ms = std::max(max_delay_ms, static_cast<long>(delay_ms));
    }

    int delay_limit = input_data_period_ms * 10;
    int timeout_limit = input_data_period_ms * 100;

    DataState new_state = DataState::NORMAL;
    if (max_delay_ms > timeout_limit) {
      new_state = DataState::TIMEOUT;
    } else if (max_delay_ms > delay_limit) {
      new_state = DataState::DELAYED;
    }

    if (current_sensor_state_ != new_state) {
      if (new_state == DataState::TIMEOUT) {
        RCLCPP_ERROR(node_ptr_->get_logger(),
                     "[%s] Sensor Communication Timeout Error (Critical)! "
                     "Delay: %ld ms. Suspending monitor...",
                     monitor_name.c_str(), max_delay_ms);
      } else if (new_state == DataState::DELAYED) {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "[%s] Sensor Delayed (Holding state). Delay: %ld ms. "
                    "Suspending monitor...",
                    monitor_name.c_str(), max_delay_ms);
      } else if (new_state == DataState::NORMAL) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "[%s] Sensor Recovered! Resuming monitor.",
                    monitor_name.c_str());
      }
      current_sensor_state_ = new_state;
    }

    return current_sensor_state_;
  }
};
