#ifndef AUTOMAPPING_HPP_
#define AUTOMAPPING_HPP_

#include "state_manager/states/state_base.hpp"

namespace airbot_state {


class AutoMapping : public stateBase {
public:

  AutoMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  // odom and odom reset
  ROBOT_STATUS processMappingReady();
  void setReadyMapping(READY_MAPPING set);
  void setExploreStatus(EXPLORE_STATUS set);
  bool runAutoMapping();
  void reset_subExploreStatus();
  void explore_status_callback(const std_msgs::msg::UInt8::SharedPtr);
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr explore_status_sub;

  double mapping_start_time;  

  READY_MAPPING ready_mapping;
  EXPLORE_STATUS explore_status;
  bool bUdateStatus = false;
  bool bNaviNodeRecovery = false;
  uint8_t keepout_enable_cnt = 0;

private:
  int max_retry_attempts_ = 3;       // 최대 재시도 횟수
  int current_retry_ = 0;            // 현재 재시도 횟수
  rclcpp::Time retry_start_time_;    // 재시도 대기 시작 시간
  void restartLaunchProcess(const std::shared_ptr<StateUtils> &state_utils);

};

} // namespace airbot_state

#endif // AUTOMAPPING_HPP_
