#ifndef STATEUTILS_HPP_
#define STATEUTILS_HPP_

#include <memory>
#include <functional>
#include <fstream>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>  // C++17 이상

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_custom_msgs/action/manage_node.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int8.hpp"
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/empty.hpp"
#include <std_msgs/msg/int8_multi_array.hpp>
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
#include "robot_custom_msgs/msg/motor_status.hpp"
#include "robot_custom_msgs/msg/block_area.hpp"
#include "robot_custom_msgs/msg/block_area_list.hpp"
#include "robot_custom_msgs/msg/battery_status.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"

#include "state_defines.hpp"
#include "navi_defines.hpp"

#include <fcntl.h>      // open, O_*
#include <unistd.h>     // close, fsync
#include <sys/stat.h>   // fstat, mode_t
#include <cerrno>       // errno
#include <cstring>      // std::strerror

// #include "state_manager/states/state_base.hpp"

#define USE_LIDAR_STATE_CHECK 1

#define DOCK_START 0x01
#define DOCK_STOP 0x00

#define USE_JSLLOC 1

namespace airbot_state {

struct ImuData
{
  // 자세 (라디안)
  double roll;
  double pitch;
  double yaw;

  // 가속도 (m/s²)
  double ax;
  double ay;
  double az;
};

class StateUtils{
public:
  StateUtils(std::shared_ptr<rclcpp::Node> node);
  
  void initializeData();
  void clearRobotPoseData();
  void clearStationPoseData();
  void setInitPoseByStationPose();

  bool saveCurrentPoseJsonForRecvocery();
  bool loadRecoveryJasonFile();  

  void resetInitPose();
  void initializePose();
  void initParameters();
  #if USE_LIDAR_STATE_CHECK == 0
  void enableSensorcallback();
  void disableSensorcallback();
  #endif
  void enableManeuverCommand(bool cmd);
  void disableManeuverCommand();
  void maneuverStateMonitor();
  void perceptionStateMonitor();

  void batteryStatusCallback(const robot_custom_msgs::msg::BatteryStatus::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odom_status_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  bool odomResetWorkingCheck();
  void localizationComplete_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void localization_state_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void localizationModeCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void stationPoseCallack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  #if USE_LIDAR_STATE_CHECK == 0
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void scan_front_state_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void scan_back_state_callback(const std_msgs::msg::Bool::SharedPtr msg);
  #endif
  void tofCallback(const robot_custom_msgs::msg::TofData::SharedPtr msg);
  void cameraCallback(const robot_custom_msgs::msg::CameraDataArray::SharedPtr /*msg*/);
  void pathPlanDestinationCallback(const std_msgs::msg::Int8::SharedPtr msg);
  void inspection_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void maneuverStateCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
  void perceptionStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& params);
  void factory_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void motorCallback(const robot_custom_msgs::msg::MotorStatus::SharedPtr msg);
  void recoveryNaviBringupCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void mapCopyCallback(const std_msgs::msg::Empty::SharedPtr/* msg*/);
  void readyRobotCallback(const std_msgs::msg::Empty::SharedPtr/* msg*/);
  void mapInfoChangedCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/);

  void manualControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void monitorManualControlTimeout();
  void startManualControlMonitor();
  void stopManualControlMonitor();


  bool isFileExists(const std::string& path);
  bool createDefaultStationPoseFile(const std::string& path);
  void saveStationPoseToJson(const std::string& filename);
  bool loadStationPose(const std::string& file_path);
  bool isStationPoseDefaultValue(double x, double y, double theta);
  double quaternion_to_euler(const geometry_msgs::msg::Quaternion &quat);
  void startMonitorOdomReset();
  void monitor_resetOdom();
  void stopMonitorOdom();

  void startSensorMonitor();
  void stopSensorMonitor();
  void monitor_sensor();
  bool lidarSensorOnchecker();
  bool tofSensorOnchecker();
  bool cameraSensorOnchecker();

  void startLocalizationMonitor(LOCALIZATION_TYPE type);
  void stopLocalizationMonitor();
  void monitor_localization();

  void enableLocalizationStop();
  void disableLocalizationStop();
  void stopLocalizationTimer();

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
  void publishLocalizeRecoveryPose();
  void publishLocalizeStop();
  void publishVelocityCommand(double v, double w);
  void publishRobotState(ROBOT_STATE state, ROBOT_STATUS status);
  void publishRobotStatus(ROBOT_STATUS status);
  void publishNodeState(NODE_STATUS state);
  void publishMovingState(NAVI_STATE state, NAVI_FAIL_REASON reason);
  void publishLifeCycleOff();
  void publishRobotCommand(REQUEST_ROBOT_CMD robot_cmd_msg);
  void publishEmergencyStop(bool set);

  #if USE_LIDAR_STATE_CHECK == 0
  void publishScanMonitor(bool set);
  #endif
  void publishTFMonitor(bool set);
  void publishManeuverOn();
  void publishManeuverOff();

  void publishMappingStart();
  void publishMappingStop();

  void publishSenSorManagerOn();
  void publishSenSorManagerOff();
  //void publishMotorMode(uint8_t mode);
  void publishRemoteBlock(bool set);
  void publishRebootReadyComplete();
  void publishKeepOutEnable(bool set);
  void publishClearVirtualWall();

  void publishLocalizationMode(bool set);

  void setAllRobotStateIDs(ROBOT_STATE data_state, ROBOT_STATUS data_status, state_cmd data_cmd);

  void setStateID(const ROBOT_STATE &id,const ROBOT_STATUS &status);
  ROBOT_STATE getStateID();

  void setStatusID(const ROBOT_STATUS &id);
  ROBOT_STATUS getStatusID();

  void setRobotCMDID(const state_cmd &datas);
  state_cmd getRobotCMDID();

  void setMovingStateID(const NAVI_STATE &id ,const NAVI_FAIL_REASON &reason = NAVI_FAIL_REASON::VOID);
  NAVI_STATE getMovingStateID();
  NAVI_FAIL_REASON getMovingFailID();

  void setNodeStatusID(const NODE_STATUS &id);
  NODE_STATUS getNodeStatusID();
  ROBOT_STATE getPreStateID();

  void setReadyMoving(const READY_MOVING &set);
  READY_MOVING getReadyMoving();

  pose getRobotPose();
  pose getInitPose();
  pose getStationPose();
  void setMoveGoalPose(double x, double y, double theta);
  pose getMoveGoalPose();
  
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

  bool isManeuverDone();
  bool isManeuverCommunicateError();
  bool isPerceptionCommunicateError();

  void setOnstationStatus(const bool &data,uint8_t status);
  bool getOnstationStatus();
  void stationData_callback(const robot_custom_msgs::msg::StationData::SharedPtr msg);

  void saveLastPosition();

  void processMoveTarget();

  void move_target_callback(const robot_custom_msgs::msg::Position::SharedPtr msg);
  void move_charger_callback(const std_msgs::msg::Empty::SharedPtr);
  void move_rotation_callback(const robot_custom_msgs::msg::MoveNRotation::SharedPtr msg);

  bool isReservedMoving();
  void setReservedMoving(const bool &data);
  MOVING_DATA getTargetPosition();

  void startOdomCheck();
  bool getPrepareOdomFlag();
  pose getCurrentOdom();

  double normalize_angle(double angle);
  double getAngle(double base, double current);
  double getDistance(pose base, pose current);

  void publishMoveFailError();
  void publishAlternativeDestinationError();

  void startDocking();
  void stopDocking();

  void stopDriving();
  bool getStopDrivingFlag();

  void setFactoryMode(const bool &data);
  bool getFactoryMode();

  void monitorRPMStopTimer();
  void disableMonitorRPMStopTimer();

  void setReservePause(const bool &data);
  bool getReservePause();

  void setReserveDocking(const bool &data);
  bool getReserveDocking();

  void publishBatterySleep();
  uint8_t getStationReceiverStatus();
  uint8_t getStationShortSignal();
  uint8_t getStationLongSignal();
  uint8_t getBatteryPercentage();
  
  void publishLocalizationFailErrorStatus(const bool &data);

  void setRotatePauseFlag(const bool &data);
  bool getRotatePauseFlag();

  bool isRetryMoveGoal(NAVI_FAIL_REASON fail_reason);
  void goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);
  void goalResponseCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle);
  void moveToTarget(double x, double y, double theta);
  void pauseNavigation();
  void resumeNavigation();
  void cancelPreviousGoal();

  bool isStartCancelMoveGoal();
  bool isCancelCompleteMoveGoal();

  void enableMonitorSensorRecoveryTimer();
  void disableMonitorSensorRecoveryTimer();
  void monitorSensorRecovery();
  void setSensorRecoveryPause(const bool &set);
  bool getSensorRecoveryPause();

  void setNavibringUpRecoveryPause(const bool &set);
  bool getNavibringUpRecoveryPause();

  void setMappingResumeToErrorFlag(const bool &set);
  bool getMappingResumeToErrorFlag();

  void setAutoMappingCompleteState(const bool &set);
  bool isAutoMappingComplete();

  uint8_t getLowerBits(uint8_t byte);
  uint8_t getUpperBits(uint8_t byte);

  bool setMapParameters(const int &set);
  bool setMapChange(const int &set);
  bool getSetMapParametersDone();
  bool getSetMapChangeDone();
  bool checkMapFileExist(const std::string &path);

  rclcpp::Subscription<robot_custom_msgs::msg::BatteryStatus>::SharedPtr battery_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localize_complete_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr odom_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr station_pose_sub;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr path_plan_destination_sub;

  rclcpp::Subscription<robot_custom_msgs::msg::TofData>::SharedPtr tof_status_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::CameraDataArray>::SharedPtr camera_data_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::StationData>::SharedPtr station_data_sub;
  rclcpp::Subscription<robot_custom_msgs::msg::Position>::SharedPtr req_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr move_charger_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::MoveNRotation>::SharedPtr req_rotation_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr inspection_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr maneuver_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr perception_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr factory_mode_sub_;
  rclcpp::Subscription<robot_custom_msgs::msg::MotorStatus>::SharedPtr motor_status_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_navi_bringup_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_state_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_control_sub_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr ready_robot_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reset_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tof_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr req_estimatePose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr req_stop_localization_pub_;
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
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr life_cycle_cmd_pub_;
  
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr dock_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr req_robot_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr remote_block_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr keepout_enable_pub_;

  /* hjkim : 251001  localization mode
   - true : global localization allowed
   - false : local localization only   */
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_localization_mode_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_mode_sub_;

  #if USE_LIDAR_STATE_CHECK > 0
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scanHz_state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr lidar_state_sub_;
  #else
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_monitor_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_front_state_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_back_state_sub;
  #endif
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tf_monitor_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr reboot_ready_complete_pub_;

  rclcpp::Publisher<robot_custom_msgs::msg::BlockAreaList>::SharedPtr block_area_pub_;
  rclcpp::Publisher<robot_custom_msgs::msg::BlockAreaList>::SharedPtr block_wall_pub_;

  rclcpp::TimerBase::SharedPtr odom_reset_timer_;
  rclcpp::TimerBase::SharedPtr sensor_monitor_timer_;
  rclcpp::TimerBase::SharedPtr localization_monitor_timer_;
  rclcpp::TimerBase::SharedPtr stop_localization_timer_;
  rclcpp::TimerBase::SharedPtr arrivedgoal_sensoroff_timer_;
  rclcpp::TimerBase::SharedPtr stopdriving_rpmstop_timer_;
  rclcpp::TimerBase::SharedPtr maneuver_state_monitor_timer_;
  rclcpp::TimerBase::SharedPtr perception_state_monitor_timer_;
  rclcpp::TimerBase::SharedPtr monitor_sensor_recovery_timer_;
  rclcpp::TimerBase::SharedPtr manual_control_watchdog_timer_;

  geometry_msgs::msg::PoseWithCovarianceStamped robot_position_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped station_position_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped last_position_msg;
  geometry_msgs::msg::PoseWithCovarianceStamped recovery_init_pose_msg;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr batterySleep_cmd_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future_goal_handle_;
  std::shared_ptr<nav2_msgs::action::NavigateToPose::Goal> goal_msg;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions navi_goal_options;

  std::shared_ptr<rclcpp::AsyncParametersClient> control_param_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> global_costmap_param_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> local_costmap_param_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_server_client_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_server_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigator_state_sub_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr map_copy_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mapinfo_chaged_sub_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr recovery_local_sub_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr keepout_state_sub_;

  void recoveryLocalCallback(const std_msgs::msg::Int8::SharedPtr msg);
  void keepoutStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);

  void setMapServerNodeActive(const std_msgs::msg::Bool::SharedPtr msg);
  void setNaviNodeActive(const std_msgs::msg::Bool::SharedPtr msg);
  bool getMapServerNodeActive();
  bool getNaviNodeActive();

  bool map_server_activated = false;
  bool navigator_activated = false;

  bool set_map_parameters_done = false;
  bool set_map_change_done = false;
  
  pose robot_pose;
  pose init_pose;
  pose odom_;
  pose last_pose;
  pose last_odom;
  pose station_pose;
  pose move_goal;
  bool bSetRecoveryInitPose = false;
  pose recovery_init_pose;

  //===== parameters =========
  bool enable_lidar_onoff_;
  bool enable_tof_onoff_;
  bool enable_camera_onoff_;

  double odom_reset_timeout_;
  double lidar_wait_timeout_;
  double tof_wait_timeout_;
  double camera_wait_timeout_;

  uint8_t odom_reset_retry_count_;
  uint8_t lidar_retry_count_;
  uint8_t tof_retry_count_;
  uint8_t camera_retry_count_;
  uint8_t localization_retry_count_;
  uint8_t localization_check_error_count_;

  double direct_velocity_v_;
  double direct_velocity_w_;
  double factory_velocity_w_;

  double undocking_distance_;
  uint8_t move_goal_retry_count_;
  double sensor_off_time_;

  double return_charger_try_docking_distance_th_m_;
  //===== parameters ========= 

  bool ready_odom = false;

  uint8_t odom_status;

  uint8_t battery_percentage_;
  ImuData imu_;
  std::chrono::time_point<std::chrono::steady_clock> odom_reset_monitor_start_time_;
  std::chrono::time_point<std::chrono::steady_clock> reset_odom_start_time_;
  std::chrono::time_point<std::chrono::steady_clock> sensor_monitor_start_time_;
  std::chrono::time_point<std::chrono::steady_clock> arrived_goal_start_time_;
  std::chrono::time_point<std::chrono::steady_clock> lidarOn_time;
  std::chrono::time_point<std::chrono::steady_clock> tofOn_time;
  std::chrono::time_point<std::chrono::steady_clock> cameraOn_time;
  std::chrono::time_point<std::chrono::steady_clock> localize_start_time;
  std::chrono::time_point<std::chrono::steady_clock> keep_out_enable_start_time;
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

  #if USE_LIDAR_STATE_CHECK == 0
  uint8_t lidar_hz_check_cnt = 0;
  bool lidar_front_state_on = false;
  bool lidar_back_state_on = false;
  #endif
  bool bLidarSensorOK = false;
  bool bMultiToFSensorOK = false;
  bool bCameraSensorOK = false;
  bool on_station_status = false;
  bool bSensorReady = false;
  int pathPlanDestination = 0;

  bool maneuverCmd;
  bool bReadyManeuver;
  bool bReadyPerception;
  bool bReady_maneuver_perception;
  bool receivePerceptionState;
  bool maneuverCommunicateError;
  bool perceptionCommunicateError;

  bool bInspectionMode = false;
  bool bSkipLocalization = false; //alreadySensorOn
  bool bFactoryMode = false;
  bool bAutoMappingComplete = false;
  
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
  READY_MOVING readyMoving;
  MOVING_DATA movingData;
  bool bTryMoveCharger;

  KEEPOUT_STATE keepout_state;

  std_msgs::msg::UInt8 dock_cmd_;

  void send_node_goal(const NODE_STATUS &require_node);
  void setNodeClientStatus(const int &status);
  int getNodeClientStatus();
  int node_client_status = 0;
  rclcpp_action::Client<robot_custom_msgs::action::ManageNode>::SharedPtr node_client_;

  double getDirectVelocityV();
  double getDirectVelocityW();
  double getFactoryVelocityW();
  double getUndockingDistance();
  uint8_t getLocalErrorCount();
  uint8_t getMoveGoalRetryLimit();
  double getSensorOffTime();
  bool getMoveChargerFlag();
  bool isSkipLocalization();
  bool isLidarSensorOK();

  void setSkipLocalization(bool set);

  double auto_mapping_pub_startime_;
  double auto_mapping_pub_interval_;

  uint8_t docking_status;
  std::optional<std::chrono::steady_clock::time_point> ondock_start_time_;

  int16_t left_motor_rpm;
  int16_t right_motor_rpm;

  bool reserve_working_pause = false;
  bool stop_driving_done = true;
  uint8_t station_short_signal = 0;
  uint8_t station_long_signal = 0;
  uint8_t station_signal_receiver = 0;
  bool reserve_docking = false;

  bool bCheckTryDocking = false;
  void setCheckTryDocking(bool set);
  bool getCheckTryDocking();

  bool bLocalizationState;

  bool moving_pause_flag = false;
  void setMovingPauseFlag(bool set);
  bool getMovingPauseFlag();
  bool rotate_pause_flag = false;

  bool getManualControlFlag();
  bool manual_control_flag = false;
  std::chrono::time_point<std::chrono::steady_clock> last_manual_control_time_;

  std::chrono::time_point<std::chrono::steady_clock> send_goal_start_time;
  uint8_t try_move_target_count;
  void resetTryMoveTargetCount();

  bool bStartCancelMoveGoal = false;
  bool bCompleteCanceledMoveGoal = false;

  uint8_t reset_maneuver_sub_count;
  uint8_t reset_perception_sub_count;
  std_msgs::msg::Int8MultiArray maneuver_state_array; 
  std::chrono::time_point<std::chrono::steady_clock> maneuver_start_time;
  //std::chrono::time_point<std::chrono::steady_clock> getManeuverStartTime();
  double getOptionalSteadyClockRunningSeconds(const std::optional<std::chrono::steady_clock::time_point>& start_time_opt);
  double getSteadyClockRunningSeconds(const std::chrono::time_point<std::chrono::steady_clock> &start_time);

  double stop_check_start_time = 0.0;
  uint8_t bottom_left = 0;
  uint8_t bottom_right = 0;
  bool sensor_warning_pause = false;
  bool navibringup_recovery_pause = false;
  int recovery_flag = 0;

  bool mapping_resume_to_error_flag = false;
  bool bCheckAmclAfterLocalization = false;
  void setCheckAmclAfterLocalization();
  bool isInpectionMode();

  bool bStartMapLoad = false;
  bool bNeedToMapLoadAfterRecovery = false;
  std::chrono::steady_clock::time_point request_loadmap_start_time_;
  uint8_t retry_mapload = 0;
  void reserveMapLoadatferNavRecovery();
  bool isNeedToRunMapLoadAfterRecovery();
  void mapLoadAfterNavRecoveryProcess(int map_type);

  bool bReceivedMapCopy = false;
  void setMapCopyReceived();
  bool isMapCopyReceived();
  void resetMapCopyReceived();

  void map_saver();
  
  bool bDockingRunning = false;
  bool isRunningDocking();
  
  bool bRecoveryReboot = false;
  void setRecoveryRebootflag(bool set);
  bool getRecoveryRebootflag();

  bool removeRecoveryDirectory();
  bool checkRecoveryReboot();
  bool isRecoveryInitPoseExist();

  bool bSavingCurrentPose = false;
  rclcpp::TimerBase::SharedPtr save_current_pose_timer_;
  std::chrono::time_point<std::chrono::steady_clock> save_current_pose_start_time;
  void startSaveCurrentPoseTimer();
  void stopSaveCurrentPoseTimer();
  void runSaveCurrentPoseTimer();

  #if USE_LIDAR_STATE_CHECK > 0
  uint8_t lidarState = 0;
  bool isScanHzOk = false;
  void scanHzStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void lidarStateCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  #endif

  bool bCmdGlobalLocalizationMode = true; //hjkim : default is global allowed
  LOCALIZATION_MODE localizationMode = LOCALIZATION_MODE::VOID; //hjkim : default is global allowed : not receive(-1), local only(0), global allowed(1)
  LOCALIZATION_MODE getLocalizationMode();
  bool getCmdGlobalLocalizationMode();
  void setCmdGlobalLocalizationMode(bool localization_mode);

  bool bMapInfoChanged = false;
  bool getMapInfoChanged();
  bool bReservedSaveMapFile = false;
  void reserveSavingMapFile();
  bool isReservedSaveMapFile();
  void startSaveMapFile();
  bool copyFile(const std::string& source, const std::string& destination);
  geometry_msgs::msg::PoseWithCovarianceStamped getDeadReckoningLastPoseMsg();

  double getReturnChargerTryDockingDistanceThreshold();
  sensor_msgs::msg::LaserScan scan_data_;
  sensor_msgs::msg::LaserScan getScanData();
  float getDockingObstacleDistanceMargin();
  float getDockingObstacleAngleMarginMin();
  float getDockingObstacleAngleMarginMax();
  void backLidarObstacleDetctionOnOff(bool enable);

  void setKeepoutState(KEEPOUT_STATE state);
  KEEPOUT_STATE getKeepoutState();
  std::chrono::time_point<std::chrono::steady_clock> getKeepOutEnableStartTime(); 

};

} // namespace airbot_state

#endif // NAVIGATION_HPP_
