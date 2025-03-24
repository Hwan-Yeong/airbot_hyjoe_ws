#ifndef UNDOCKING_HPP_
#define UNDOCKING_HPP_

#include "state_manager/states/state_base.hpp"

namespace airbot_state {

class UnDocking : public stateBase {
public:
  UnDocking(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);
  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;
  void processLinearMoving();
  void enableLinearTargetMoving();
  // undocking function
  void unDockingMove(const float dist);
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;
  rclcpp::TimerBase::SharedPtr linear_target_timer_;

  bool dock_pose_estimate = false;

  pose base_odom;
  pose odom;
  bool end_linear_target;

protected:

};

} // namespace airbot_state

#endif // UNDOCKING_HPP_
