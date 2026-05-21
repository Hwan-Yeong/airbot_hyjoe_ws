#ifndef MANUALMAPPING_HPP_
#define MANUALMAPPING_HPP_

#include "state_manager/states/state_base.hpp"

namespace airbot_state {


class ManualMapping : public stateBase {
public:

  ManualMapping(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  // manualmapping
  void setReadyMapping(READY_MAPPING set);
  ROBOT_STATUS processMappingReady();
  
  double mapping_start_time;

  READY_MAPPING ready_mapping;
  bool bMapSave = false;
  uint8_t keepout_enable_cnt = 0;
};

} // namespace airbot_state

#endif // MANUALMAPPING_HPP_
