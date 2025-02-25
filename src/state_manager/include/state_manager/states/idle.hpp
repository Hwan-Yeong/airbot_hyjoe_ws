#ifndef IDLE_HPP_
#define IDLE_HPP_

#include "state_manager/states/state_base.hpp"

namespace airbot_state {

class Idle : public stateBase {
public:

  Idle(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;
  void stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;

  u_int8_t docking_status;
  bool first_booting;
};

} // namespace airbot_state

#endif // IDLE_HPP_
