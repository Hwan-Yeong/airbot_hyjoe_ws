#ifndef MANUAL_CONTROL_HPP_
#define MANUAL_CONTROL_HPP_

#include "state_manager/states/state_base.hpp"
#include "follow_msgs/msg/ui_client.hpp"

namespace airbot_state {

class ManualControl : public stateBase {
public:

  ManualControl(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  // odom and odom reset
  ROBOT_STATUS processManualControlReady();
  void setReadyManualControl(READY_MANUAL_CONTROL set);
  void handleFollowFinish();
  int8_t localizationChecker();
  void publishStreamingCmd(const bool &cmd);

  READY_MANUAL_CONTROL ready_working;
  READY_MANUAL_CONTROL pre_ready_working;

  bool bNeedsInitPoseLocalization = false;
  bool move_started = false;
  bool pause_flag = false;

  uint8_t retry_localization;
  uint8_t local_mode_retry_cnt = 0;
  
  std::chrono::time_point<std::chrono::steady_clock> localization_start_time;
  rclcpp::Publisher<follow_msgs::msg::UiClient>::SharedPtr streaming_cmd_pub;
};

} // namespace airbot_state

#endif // AUTOMAPPING_HPP_
