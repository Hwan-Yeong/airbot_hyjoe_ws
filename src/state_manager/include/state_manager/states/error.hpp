#ifndef ERROR_HPP_
#define ERROR_HPP_

#include "state_manager/states/state_base.hpp"

namespace airbot_state {

class Error : public stateBase {
public:

  Error(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  bool exitAllNode();
  void sendStopCMD();

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  bool error_onstation = false;
};

} // namespace airbot_state

#endif // ERROR_HPP_
