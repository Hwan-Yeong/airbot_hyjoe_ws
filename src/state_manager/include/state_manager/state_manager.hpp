#ifndef STATE_MANAGER_HPP_
#define STATE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/u_int8.hpp"

#include "robot_custom_msgs/msg/error_list.hpp"
#include "robot_custom_msgs/msg/error_list_array.hpp"

#include "state_manager/states/state_base.hpp"
#include "state_manager/states/idle.hpp"
#include "state_manager/states/navigation.hpp"
#include "state_manager/states/docking.hpp"
#include "state_manager/states/undocking.hpp"
#include "state_manager/states/on_station.hpp"
#include "state_manager/states/auto_mapping.hpp"
#include "state_manager/states/manual_mapping.hpp"
#include "state_manager/states/return_charger.hpp"
#include "state_manager/states/factory_navigation.hpp"
#include "state_manager/states/error.hpp"
#include "state_manager/utils/state_utils.hpp"

namespace airbot_state {

class StateManager : public rclcpp::Node {
public:
  StateManager();
  ~StateManager() override;
  
  private:
  void setState(ROBOT_STATE state_id, ROBOT_STATUS status_id, const state_cmd &cmd_ids);

  void checkTransition( const state_cmd &cmd_ids);

  void setCurrentStateID( const ROBOT_STATE data);
  ROBOT_STATE getCurrentStateID();

  void handleSoCCMD(const std_msgs::msg::UInt8::SharedPtr msg);
  void handleRobotCMD(const std_msgs::msg::UInt8::SharedPtr msg);
  void handleError(const robot_custom_msgs::msg::ErrorListArray::SharedPtr msg);

  void runCurrentState();

  std::shared_ptr<StateUtils> state_utils;
  std::shared_ptr<Idle> idle;
  std::shared_ptr<AutoMapping> auto_mapping;
  std::shared_ptr<ManualMapping> manual_mapping;
  std::shared_ptr<Navigation> navigation;
  std::shared_ptr<ReturnCharger> return_charger;
  std::shared_ptr<Docking> docking;
  std::shared_ptr<UnDocking> undocking;
  std::shared_ptr<OnStation> on_station;
  std::shared_ptr<FactoryNavigation> factory_navigation;
  std::shared_ptr<Error> error;
  std::map<ROBOT_STATE, std::shared_ptr<stateBase>> states_;
  std::shared_ptr<stateBase> current_state_;
  
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr robot_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr soc_cmd_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::ErrorListArray>::SharedPtr error_list_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  
  std::vector<robot_custom_msgs::msg::ErrorList> error_list; 
  
  ROBOT_STATE current_state;
  state_cmd pre_cmds;
  
  bool error_occured = false;
};

}
#endif // STATE_MANAGER_HPP_
