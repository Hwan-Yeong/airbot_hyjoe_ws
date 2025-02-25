#ifndef STATEUTILS_HPP_
#define STATEUTILS_HPP_

#include <memory>
#include <functional>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/empty.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"
#include "robot_custom_msgs/msg/position.hpp"

#include "state_defines.hpp"
#include "navi_defines.hpp"

// #include "state_manager/states/state_base.hpp"

namespace airbot_state {



class StateUtils{
public:
  StateUtils(std::shared_ptr<rclcpp::Node> node);

  void enableOdomcallback();
  void disableOdomcallback();

  void enableLocalizationcallback();
  void disableLocalizationcallback();

  void enableSensorcallback();
  void disableSensorcallback();

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odom_status_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void localizationComplete_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void stationPoseCallack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg);
  

  double quaternion_to_euler(const geometry_msgs::msg::Quaternion &quat);
  void startMonitorOdomReset();
  void monitor_resetOdom();
  void stopMonitorOdom();

  void reset_timerResetOdom();
  bool isValidateResetOdom(const pose &odom);
  bool isStartOdomReset();
  bool getOdomResetDone();

  bool isStartLocalization();
  bool getLocalizationComplete();
  void publishLidarOff();
  void publishLidarOn();
  void publishMultiTofOff();
  void publishMultiTofOn();
  void publishAllSensorOff();
  void publishAllSensorOn();
  void publishClearCostMap();
  void publishLocalizeInitPose();
  void publishLocalizePose();

  void setAllRobotStateIDs(ROBOT_STATE data_state, ROBOT_STATUS data_status, state_cmd data_cmd);

  void setStateID(const ROBOT_STATE &id);
  ROBOT_STATE getStateID();

  void setStatusID(const ROBOT_STATUS &id);
  ROBOT_STATUS getStatusID();

  void setRobotCMDID(const state_cmd &datas);
  state_cmd getRobotCMDID();

  void setMovingStateID(const NAVI_STATE &id);
  NAVI_STATE getMovingStateID();

  void setMovingFailID(const NAVI_FAIL_REASON &id);
  NAVI_FAIL_REASON getMovingFailID();

  void setNodeStatusID(const NODE_STATUS &id);
  NODE_STATUS getNodeStatusID();

  ROBOT_STATE getPreStateID();

  pose getRobotPose();
  
  void setStartOnStation(bool set);
  bool isStartOnStation();

  void setOdomResetError(bool set);
  bool isOdomREsetError();
  bool isLidarSensorOK();
  bool isMultiToFSensorOK();
  double getLocalizationStartTime();

  void setOnstationStatus(const bool &data);
  bool getOnstationStatus();
  void stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg);

  void saveLastPosition();

  bool isValidNavigation(const std::string &pidFilePath, double start_time);

  bool startProcess(const std::string &command, const std::string &pidFilePath);
  bool stopProcess(const std::string &pidFilePath);
  
  void processMoveTarget();

  void target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg);
  MOVING_DATA getTargetPosition();

  bool getPrepareOdomFlag();
  pose getCurrentOdom();

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr localize_complete_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr odom_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr station_pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_status_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::Position>::SharedPtr req_target_sub_;
  

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reset_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tof_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr req_nomotion_local_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr req_estimatePose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr req_clear_costmap_pub_;
  
  rclcpp::TimerBase::SharedPtr odom_reset_timer_;
  rclcpp::TimerBase::SharedPtr sensor_off_timer_;

  geometry_msgs::msg::PoseWithCovarianceStamped robot_position_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped station_position_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped last_position_msg;
  std_msgs::msg::UInt8 odom_reset_cmd_;
  
  pose robot_pose;

  pose odom_;
  pose last_pose;
  pose station_pose;
  bool ready_odom = false;

  uint8_t odom_status;
  double reset_odom_start_time_;
  double arrived_goal_start_time_;
  double localize_start_time;
  uint8_t odom_reset_cnt_ = 0;
  bool bStartOnStation = false;
  bool bStartOdomReset = false;
  bool bOdomResetDone = false;
  bool bSendResetOdomCmd = false;
  bool bStartLocalizationStart = false;
  bool bLocalizationComplete = false;
  bool bOdomResetError = false;
  bool bLidarSensorOK = false;
  bool bMultiToFSensorOK = false;
  bool on_station_status = false;
  
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Time scan_callback_time_;

  ROBOT_STATE pre_state_id;
  ROBOT_STATE state_id;
  ROBOT_STATUS pre_status_id;
  ROBOT_STATUS status_id;
  state_cmd pre_cmd_ids;
  state_cmd cmd_ids;
  NODE_STATUS node_status_id;
  NAVI_STATE movingstate_id;
  NAVI_FAIL_REASON movingfail_id;
  MOVING_DATA movingData;

};

} // namespace airbot_state

#endif // NAVIGATION_HPP_
