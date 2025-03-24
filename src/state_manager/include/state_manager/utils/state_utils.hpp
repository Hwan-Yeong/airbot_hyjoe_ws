#ifndef STATEUTILS_HPP_
#define STATEUTILS_HPP_

#include <memory>
#include <functional>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_custom_msgs/action/manage_node.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int8.hpp"
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/empty.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h> 
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_custom_msgs/msg/station_data.hpp"
#include "robot_custom_msgs/msg/position.hpp"
#include "robot_custom_msgs/msg/camera_data.hpp"
#include "robot_custom_msgs/msg/camera_data_array.hpp"
#include "robot_custom_msgs/msg/move_n_rotation.hpp"
#include "robot_custom_msgs/msg/robot_state.hpp"
#include "robot_custom_msgs/msg/navi_state.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "state_defines.hpp"
#include "navi_defines.hpp"

// #include "state_manager/states/state_base.hpp"

namespace airbot_state {



class StateUtils{
public:
  StateUtils(std::shared_ptr<rclcpp::Node> node);
  
  void initializeData();
  void initInitPose();
  void initStationPose();
  void initParameters();
  void enableOdomcallback();
  void disableOdomcallback();

  void enableLocalizationcallback();
  void disableLocalizationcallback();

  void enableSensorcallback();
  void disableSensorcallback();

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odom_status_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void localizationComplete_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void stationPoseCallack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg);
  void cameraCallback(const robot_custom_msgs::msg::CameraDataArray::SharedPtr /*msg*/);
  void pathPlanDestinationCallback(const std_msgs::msg::Int8::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& params);
  

  double quaternion_to_euler(const geometry_msgs::msg::Quaternion &quat);
  void startMonitorOdomReset();
  void monitor_resetOdom();
  void stopMonitorOdom();

  void startSensorMonitor();
  void stopSensorMonitor();
  void monitor_sensor();

  void startLocalizationMonitor(LOCALIZATION_TYPE type);
  void stopLocalizationMonitor();
  void monitor_localization();

  void enableArrivedGoalSensorsOffTimer();
  void disableArrivedGoalSensorsOffTimer();
  void monitor_ArrivedGoal_SensorsOff();

  void reset_timerResetOdom();
  void reset_timerSensorMonitor();
  void reset_timerLocalization();
  bool isValidateResetOdom(const pose &odom);
  bool isStartOdomReset();
  bool getOdomResetDone();

  bool isStartLocalization();
  bool getLocalizationComplete();
  void publishStartOdomReset();
  void publishClearOdomReset();
  void publishLidarOff();
  void publishLidarOn();
  void publishMultiTofOff();
  void publishMultiTofOn();
  void publishCameraOff();
  void publishCameraOn();
  void publishAllSensorOff();
  void publishAllSensorOn();
  void publishClearCostMap();
  void publishLocalizeUndockPose();
  void publishLocalizePose();
  void publishLocalizeSavedPose();
  void publishVelocityCommand(double v, double w);
  void publishRobotState(ROBOT_STATE state);
  void publishRobotStatus(ROBOT_STATUS status);
  void publishNodeState(NODE_STATUS state);
  void publishMovingState(NAVI_STATE state, NAVI_FAIL_REASON reason);

  void publishManeuverOn();
  void publishManeuverOff();

  void publishMappingOn();
  void publishMappingOff();

  void publishSenSorManagerOn();
  void publishSenSorManagerOff();

  void setAllRobotStateIDs(ROBOT_STATE data_state, ROBOT_STATUS data_status, state_cmd data_cmd);

  void setStateID(const ROBOT_STATE &id);
  ROBOT_STATE getStateID();

  void setStatusID(const ROBOT_STATUS &id);
  ROBOT_STATUS getStatusID();

  void setRobotCMDID(const state_cmd &datas);
  state_cmd getRobotCMDID();

  void setMovingStateID(const NAVI_STATE &id ,const NAVI_FAIL_REASON &reason = NAVI_FAIL_REASON::VOID);
  NAVI_STATE getMovingStateID();

  void setNodeStatusID(const NODE_STATUS &id);
  NODE_STATUS getNodeStatusID();

  ROBOT_STATE getPreStateID();

  pose getRobotPose();
  pose getInitPose();
  pose getStationPose();
  
  void setStartOnStation(bool set);
  bool isStartOnStation();

  void setOdomResetError(bool set);
  bool isOdomResetError();
  void setLidarError(bool set);
  bool isLidarError();
  void setToFError(bool set);
  bool isToFError();

  void setCameraError(bool set);
  bool isCamreaError();

  void setLocalizationError(bool set);
  bool isLocalizationError();
  void setSensorReady(bool set);
  bool isSensorReady();
  int getPathPlanDestination();
  double getLocalizationStartTime();

  void setOnstationStatus(const bool &data);
  bool getOnstationStatus();
  void stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg);

  void saveLastPosition();

  void processMoveTarget();

  void move_target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg);
  void move_charger_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void move_rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg);
  MOVING_DATA getTargetPosition();

  bool getPrepareOdomFlag();
  pose getCurrentOdom();

  double getDistance(pose base, pose current);

  void publishMoveFailError();
  void publishAlternativeDestinationError();

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localize_complete_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr odom_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr station_pose_sub;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr path_plan_destination_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_status_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_data_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::Position>::SharedPtr req_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr move_charger_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::MoveNRotation>::SharedPtr req_rotation_target_sub_;
  

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reset_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tof_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr req_estimatePose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr req_clear_costmap_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_fail_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alternative_dest_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr direct_vel_pub_;

  rclcpp::Publisher<robot_custom_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr node_status_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::NaviState>::SharedPtr navi_state_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr maneuver_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mapping_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sensor_manager_cmd_pub_;
  
  rclcpp::TimerBase::SharedPtr odom_reset_timer_;
  rclcpp::TimerBase::SharedPtr sensor_monitor_timer_;
  rclcpp::TimerBase::SharedPtr localization_monitor_timer_;
  rclcpp::TimerBase::SharedPtr arrivedgoal_sensoroff_timer_;

  geometry_msgs::msg::PoseWithCovarianceStamped robot_position_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped station_position_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped last_position_msg;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  pose robot_pose;
  pose init_pose;
  pose odom_;
  pose last_pose;
  pose station_pose;

  //===== parameters =========
  bool use_camera_;
  bool use_multiTof_;
  double odom_reset_timeout_;
  double lidar_wait_timeout_;
  double tof_wait_timeout_;
  double camera_wait_timeout_;

  uint8_t odom_reset_retry_count_;
  uint8_t lidar_retry_count_;
  uint8_t tof_retry_count_;
  uint8_t camera_retry_count_;
  uint8_t localization_retry_count_;

  double direct_velocity_v_;
  double direct_velocity_w_;

  double undocking_distance_;
  uint8_t move_goal_retry_count_;
  double sensor_off_time_;
  //===== parameters ========= 

  bool ready_odom = false;

  uint8_t odom_status;
  double odom_reset_monitor_start_time_;
  double reset_odom_start_time_;
  double sensor_monitor_start_time_;
  double arrived_goal_start_time_;
  double lidarOn_time;
  double tofOn_time;
  double cameraOn_time;
  double localize_start_time;
  uint8_t odom_reset_cnt_ = 0;
  uint8_t lidar_retry_cnt = 0;
  uint8_t tof_retry_cnt = 0;
  uint8_t camera_retry_cnt = 0;
  uint8_t localization_retry_cnt = 0;
  bool bStartOnStation = false;
  bool bStartOdomReset = false;
  bool bOdomResetDone = false;
  bool bSendResetOdomCmd = false;
  bool bSendLidarCmd = false;
  bool bSendTofCmd = false;
  bool bSendCameraCmd = false;
  LOCALIZATION_TYPE Localtype;
  bool bStartLocalizationStart = false;
  bool bLocalizationComplete = false;
  bool bLocalizationFail = false;
  bool bOdomResetError = false;
  bool bLidarError = false;
  bool bTofError = false;
  bool bCameraError = false;
  bool bLoclizationError = false;
  bool bStartSensorOn = false;
  bool bLidarSensorOK = false;
  bool bMultiToFSensorOK = false;
  bool bCameraSensorOK = false;
  bool on_station_status = false;
  bool bSensorReady = false;
  int pathPlanDestination = 0;
  
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
  bool bTryMoveCharger;

  void send_node_goal(const NODE_STATUS &require_node);
  void setNodeClientStatus(const int &status);
  int getNodeClientStatus();
  int node_client_status = 0;
  rclcpp_action::Client<robot_custom_msgs::action::ManageNode>::SharedPtr node_client_;

  double getDirectVelocityV();
  double getDirectVelocityW();
  double getUndockingDistance();
  uint8_t getMoveGoalRetryCount();
  double getSensorOffTime();
  bool getMoveChargerFlag();

};

} // namespace airbot_state

#endif // NAVIGATION_HPP_
