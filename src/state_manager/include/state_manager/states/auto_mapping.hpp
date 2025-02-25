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

  void runAutoMapping();
  void reset_subExploreFinish();
  void explore_finish_callback(const std_msgs::msg::Empty::SharedPtr msgs);
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr explore_finish_sub;

  void map_saver();
  void exitMappingNode();
  void exitNavigationNode();

  double mapping_start_time;
  bool bSavedMap;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;

};

} // namespace airbot_state

#endif // AUTOMAPPING_HPP_
