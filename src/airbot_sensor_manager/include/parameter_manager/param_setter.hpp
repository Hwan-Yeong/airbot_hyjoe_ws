#ifndef AIRBOT_SENSOR_MANAGER_PARAMETER_MANAGER_PARAM_SETTER_HPP_
#define AIRBOT_SENSOR_MANAGER_PARAMETER_MANAGER_PARAM_SETTER_HPP_

#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

class ParamSetterNode : public rclcpp::Node {
 public:
  ParamSetterNode();
  ~ParamSetterNode();

 private:
  void SocCmdCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void SubCellIdxLeftCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void SubCellIdxRightCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      sub_cell_idx_left_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      sub_cell_idx_right_subscription_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
};

#endif  // AIRBOT_SENSOR_MANAGER_PARAMETER_MANAGER_PARAM_SETTER_HPP_
