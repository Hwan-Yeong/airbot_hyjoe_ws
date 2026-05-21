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

  void enableTryDocking();
  void disableTryDocking();
  bool tryDockingChecker();

  bool first_booting;
  bool bChkNode = false;
  bool bChkRedocking = false;
  bool bSetBaseOdom = false;
  uint8_t try_docking_cnt = 0;
  pose baseOdom;
  pose tempOdom;
  std::chrono::time_point<std::chrono::steady_clock> try_docking_starttime;
};

} // namespace airbot_state

#endif // IDLE_HPP_
