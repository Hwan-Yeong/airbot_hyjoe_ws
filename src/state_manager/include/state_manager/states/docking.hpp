#ifndef DOCKING_HPP_
#define DOCKING_HPP_

#include "state_manager/states/state_base.hpp"

namespace airbot_state {


class Docking : public stateBase {
public:

  Docking(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;
  void publishDockingError();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_error_pub_;

  bool bUndockStart = false;
  bool pause_docking = false;
  double runTime = 0.0;
  std::chrono::time_point<std::chrono::steady_clock> start_time;
  bool bRedockingReady = false;
  bool bSetBaseOdom = false;
  pose baseOdom;
  pose tempOdom;

};

} // namespace airbot_state

#endif // DOCKING_HPP_
