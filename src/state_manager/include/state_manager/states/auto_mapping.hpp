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
  bool resetOdomChecker();
  void runAutoMapping();
  void reset_subExploreFinish();
  void explore_finish_callback(const std_msgs::msg::Empty::SharedPtr msgs);
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr explore_finish_sub;

  void map_saver();

  double mapping_start_time;  
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;

  READY_MAPPING ready_mapping;
};

} // namespace airbot_state

#endif // AUTOMAPPING_HPP_
