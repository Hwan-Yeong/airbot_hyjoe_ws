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
  void startDocking();
  void stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg);

  std_msgs::msg::UInt8 dock_cmd_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr dock_pub;
  rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;

  bool bUndockStart = false;
  bool bDockingError = false;
  uint8_t docking_status;
};

} // namespace airbot_state

#endif // DOCKING_HPP_
