#ifndef STATE_MANAGER_HPP_
#define STATE_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include "robot_custom_msgs/msg/navi_state.hpp"

#include "state_manager/states/state_base.hpp"
#include "state_manager/states/idle.hpp"
#include "state_manager/states/navigation.hpp"
#include "state_manager/states/docking.hpp"
#include "state_manager/states/undocking.hpp"
#include "state_manager/states/on_station.hpp"
#include "state_manager/states/auto_mapping.hpp"
#include "state_manager/states/manual_mapping.hpp"
#include "state_manager/states/return_charger.hpp"
#include "state_manager/utils/state_utils.hpp"

namespace airbot_state {

class StateManager : public rclcpp::Node {
public:
  StateManager();
  ~StateManager() override;

private:
  std::shared_ptr<stateBase> current_state_;
  std::map<ROBOT_STATE, std::shared_ptr<stateBase>> states_;
  std::shared_ptr<Idle> idle;
  std::shared_ptr<OnStation> on_station;
  std::shared_ptr<Navigation> navigation;
  std::shared_ptr<Docking> docking;
  std::shared_ptr<UnDocking> undocking;
  std::shared_ptr<AutoMapping> auto_mapping;
  std::shared_ptr<ManualMapping> manual_mapping;
  std::shared_ptr<ReturnCharger> return_charger;
  std::shared_ptr<Navigation> factory_navigation;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr robot_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr soc_cmd_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::Position>::SharedPtr req_target_sub_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr node_status_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::NaviState>::SharedPtr navi_state_pub_;

  void docking_callback(const std_msgs::msg::UInt8::SharedPtr msg);


  rclcpp::TimerBase::SharedPtr timer_;
  ROBOT_STATE current_state;
  state_cmd pre_cmds;
  

  std::shared_ptr<StateUtils> state_utils;

  void target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg);
  bool bsetMovetarget;
  pose target_pose;

  void setState(ROBOT_STATE state_id, 
                ROBOT_STATUS status_id,
                const state_cmd &cmd_ids); // STRUCT CMDS
  
  void checkTransition( const state_cmd &cmd_ids);

  void setCurrentStateID( const ROBOT_STATE data);
  ROBOT_STATE getCurrentStateID();

  void handleSoCCMD(const std_msgs::msg::UInt8::SharedPtr msg);
  void handleRobotCMD(const std_msgs::msg::UInt8::SharedPtr msg);

  void handleSharedData(const std_msgs::msg::String::SharedPtr msg);

  void runCurrentState();
};

}
#endif // STATE_MANAGER_HPP_
