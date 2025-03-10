#ifndef ONSTATION_HPP_
#define ONSTATION_HPP_

#include "state_manager/states/state_base.hpp"

namespace airbot_state {

class OnStation : public stateBase {
public:

  OnStation(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;
  void stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;
  int docking_status;
};

} // namespace airbot_state

#endif // ONSTATION_HPP_
