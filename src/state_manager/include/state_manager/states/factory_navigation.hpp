#ifndef FACTORY_NAVIGATION_HPP_
#define FACTORY_NAVIGATION_HPP_

#define USE_JSLLOC 1

#include "state_manager/states/state_base.hpp"
#include "robot_custom_msgs/msg/position.hpp"


namespace airbot_state {

class FactoryNavigation : public stateBase {
  
public:
  FactoryNavigation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  rclcpp::Time nav_node_start_time;
  
  MOVING_DATA movingData;
  NAVI_STATE movingState;
  READY_NAVIGATION readyNavi;
  double node_start_time;
  uint8_t retry_localization;
  rclcpp::TimerBase::SharedPtr stabilization_timer_;
  bool paramset_test = false;
  bool bNaviNodeRecovery = false;

  std::chrono::time_point<std::chrono::steady_clock> localization_start_time;
  uint8_t local_mode_retry_cnt = 0;

  void setReadyNavigation(READY_NAVIGATION set);
  void processMoveTarget();
  ROBOT_STATUS processNavigationReady();

  int8_t localizationChecker();

};

} // namespace airbot_state

#endif // FACTORY_NAVIGATION_HPP_
