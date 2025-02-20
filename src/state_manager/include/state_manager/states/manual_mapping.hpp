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
  void runManualMapping();
  double mapping_start_time;

  void map_saver();
  void exitMappingNode();
  void exitNavigationNode();
  bool bSavedMap;

  ////process check
  bool startProcess(const std::string &command, const std::string &pidFilePath);
  bool stopProcess(const std::string &pidFilePath);

  void stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg);

  rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;

  u_int8_t docking_status;
};

} // namespace airbot_state

#endif // MANUALMAPPING_HPP_
