#ifndef STATEDEFINES_HPP_
#define STATEDEFINES_HPP_

#define RECEIVER_LEFT_DOCK_SIGNAL 0x10         // 0x1X
#define RECEIVER_LEFT_CENTER_DOCK_SIGNAL 0x20  // 0x2X
#define RECEIVER_RIGHT_CENTER_DOCK_SIGNAL 0x40 // 0x4X
#define RECEIVER_RIGHT_DOCK_SIGNAL 0x80        // 0x8X

#define SHORT_SIG_LEFT 0x01
#define SHORT_SIG_LEFT_CENTER 0x02
#define SHORT_SIG_CENTER 0x04
#define SHORT_SIG_RIGHT_CENTER 0x08
#define SHORT_SIG_RIGHT 0x10

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29577951308232)
#endif

#include <string>
namespace airbot_state {

enum class LOCALIZATION_TYPE {
  INIT_POSE,
  ROBOT_POSE,
  SAVED_POSE,
  RECOVERY_POSE,
};

inline std::string enumToString(LOCALIZATION_TYPE in) {
  std::string out;
  switch (in) {
  case LOCALIZATION_TYPE::INIT_POSE:
    out = std::string("INIT_POSE");
    break;
  case LOCALIZATION_TYPE::ROBOT_POSE:
    out = std::string("ROBOT_POSE");
    break;
  case LOCALIZATION_TYPE::SAVED_POSE:
    out = std::string("SAVED_POSE");
    break;
  case LOCALIZATION_TYPE::RECOVERY_POSE:
    out = std::string("RECOVERY_POSE");
    break;
  default:
    out = std::string("ERROR");
    break;
  }
  return out;
};

enum class NODE_STATUS {
  IDLE,
  AUTO_MAPPING,
  MANUAL_MAPPING,
  NAVI,
  FT_NAVI, // node manager에 켜라고 전달하기 위해 추가..
  FOLLOWING
};

inline std::string enumToString(NODE_STATUS in) {
  std::string out;
  switch (in) {
  case NODE_STATUS::IDLE:
    out = std::string("IDLE");
    break;
  case NODE_STATUS::AUTO_MAPPING:
    out = std::string("AUTO_MAPPING");
    break;
  case NODE_STATUS::MANUAL_MAPPING:
    out = std::string("MANUAL_MAPPING");
    break;
  case NODE_STATUS::NAVI:
    out = std::string("NAVI");
    break;
  case NODE_STATUS::FT_NAVI:
    out = std::string("FT_NAVI");
    break;
  case NODE_STATUS::FOLLOWING:
    out = std::string("FOLLOWING");
    break;
  }
  return out;
};

enum class NAVI_STATE : int {
  IDLE = 0,
  MOVE_GOAL,
  ARRIVED_GOAL,
  PAUSE,
  FAIL,
  START_ROTAION,
  ROTATION_COMPLETE,
  READY,
  ALTERNATE_GOAL,
  FOLLOWING,
  MANUAL_CONTROL,
};

inline std::string enumToString(NAVI_STATE in) {
  std::string out;
  switch (in) {
  case NAVI_STATE::IDLE:
    out = std::string("IDLE");
    break;
  case NAVI_STATE::READY:
    out = std::string("READY");
    break;
  case NAVI_STATE::MOVE_GOAL:
    out = std::string("MOVE_GOAL");
    break;
  case NAVI_STATE::ARRIVED_GOAL:
    out = std::string("ARRIVED_GOAL");
    break;
  case NAVI_STATE::ALTERNATE_GOAL:
    out = std::string("ALTERNATE_GOAL");
    break;
  case NAVI_STATE::PAUSE:
    out = std::string("PAUSE");
    break;
  case NAVI_STATE::FAIL:
    out = std::string("FAIL");
    break;
  case NAVI_STATE::START_ROTAION:
    out = std::string("START_ROTAION");
    break;
  case NAVI_STATE::ROTATION_COMPLETE:
    out = std::string("ROTATION_COMPLETE");
    break;
  case NAVI_STATE::FOLLOWING:
    out = std::string("FOLLOWING");
    break;
  case NAVI_STATE::MANUAL_CONTROL:
    out = std::string("MANUAL_CONTROL");
    break;
  }
  return out;
};

enum class NAVI_FAIL_REASON {
  VOID,
  NODE_OFF,
  SERVER_NO_ACTION,
  GOAL_ABORT,
  GOAL_REJECT,
  UNKNOWN,
};

inline std::string enumToString(NAVI_FAIL_REASON in) {
  std::string out;
  switch (in) {
  case NAVI_FAIL_REASON::VOID:
    out = std::string("VOID");
    break;
  case NAVI_FAIL_REASON::NODE_OFF:
    out = std::string("NODE_OFF");
    break;
  case NAVI_FAIL_REASON::SERVER_NO_ACTION:
    out = std::string("SERVER_NO_ACTION");
    break;
  case NAVI_FAIL_REASON::GOAL_ABORT:
    out = std::string("GOAL_ABORT");
    break;
  case NAVI_FAIL_REASON::GOAL_REJECT:
    out = std::string("GOAL_REJECT");
    break;
  case NAVI_FAIL_REASON::UNKNOWN:
    out = std::string("UNKNOWN");
    break;
  }
  return out;
};

enum class REQUEST_SOC_CMD {
  // SOC TRIGGER CMD
  VOID,
  START_AUTO_MAPPING,       // SOC 명령
  START_MANUAL_MAPPING,     // SOC 명령
  START_NAVIGATION,         // SOC 명령
  START_RETURN_CHARGER,     // SOC 명령
  START_DOCKING,            // SOC 명령
  START_CHARGING,           // SOC 명령
  PAUSE_WORKING,            // SOC 명령
  RESUME_WORKING,           // SOC 명령
  START_FACTORY_NAVIGATION, // reserve SOC명령
  STOP_WORKING,             //  SOC명령
  START_FOLLOWING,
  START_MANUAL_CONTROL,

  // reserve code
  // ROTATION
  // MANUAL_MOVING,
  // LIDAR_STOP,
  // LIDAR_START,
  // EMERGENCY_STOP,
};

inline std::string enumToString(const REQUEST_SOC_CMD in) {
  std::string out;
  switch (in) {
  case REQUEST_SOC_CMD::VOID:
    out = std::string("VOID");
    break;
  case REQUEST_SOC_CMD::START_AUTO_MAPPING:
    out = std::string("START_AUTO_MAPPING");
    break;
  case REQUEST_SOC_CMD::START_MANUAL_MAPPING:
    out = std::string("START_MANUAL_MAPPING");
    break;
  case REQUEST_SOC_CMD::START_NAVIGATION:
    out = std::string("START_NAVIGATION");
    break;
  case REQUEST_SOC_CMD::START_RETURN_CHARGER:
    out = std::string("START_RETURN_CHARGER");
    break;
  case REQUEST_SOC_CMD::START_DOCKING:
    out = std::string("START_DOCKING");
    break;
  case REQUEST_SOC_CMD::START_CHARGING:
    out = std::string("START_CHARGING");
    break;
  case REQUEST_SOC_CMD::PAUSE_WORKING:
    out = std::string("PAUSE_WORKING");
    break;
  case REQUEST_SOC_CMD::RESUME_WORKING:
    out = std::string("RESUME_WORKING");
    break;
  case REQUEST_SOC_CMD::START_FACTORY_NAVIGATION:
    out = std::string("START_FACTORY_NAVIGATION");
    break;
  case REQUEST_SOC_CMD::STOP_WORKING:
    out = std::string("STOP_WORKING");
    break;
  case REQUEST_SOC_CMD::START_FOLLOWING:
    out = std::string("START_FOLLOWING");
    break;
  case REQUEST_SOC_CMD::START_MANUAL_CONTROL:
    out = std::string("START_MANUAL_CONTROL");
    break;
  }
  return out;
};

enum class REQUEST_ROBOT_CMD {
  VOID,
  START_ONSTATION, // DOCKING인식시
  STOP_ONSTATION,  // Onstation에서 충전 단자 접촉해제시

  UNDOCKING_DONE_START_AUTO_MAPPING,   // UNDOCKING 완료시
  UNDOCKING_DONE_START_MANUAL_MAPPING, // UNDOCKING 완료시
  UNDOCKING_DONE_START_NAVIGATION,     // UNDOCKING 완료시
  UNDOCKING_DONE_START_FACTORY_NAVIGATION,
  UNDOCKING_DONE_START_MANUAL_CONTROL,
  DONE_AUTO_MAPPING, // AUTO_MAPPING완료시
  DONE_MANUAL_MAPPING,

  DONE_DOCKING, // DOCKING 완료 시

  DONE_RETURN_CHARGER,             // RETURN_CHARGER완료 시
  FAIL_RETURN_CHARGER_TRY_DOCKING, // RETURN_CHARGER FAIL 시
  UNDOCKING_DONE_START_FOLLOWING,
  CANCEL_FOLLOWING_NOT_MOVING,
  ERROR,
  OFFSTATION_TRY_DOCKING,
  READY_REBOOT,
};

inline std::string enumToString(const REQUEST_ROBOT_CMD in) {
  std::string out;
  switch (in) {
  case REQUEST_ROBOT_CMD::VOID:
    out = std::string("VOID");
    break;
  case REQUEST_ROBOT_CMD::START_ONSTATION:
    out = std::string("START_ONSTATION");
    break;
  case REQUEST_ROBOT_CMD::STOP_ONSTATION:
    out = std::string("STOP_ONSTATION");
    break;
  case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_AUTO_MAPPING:
    out = std::string("UNDOCKING_DONE_START_AUTO_MAPPING");
    break;
  case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_MAPPING:
    out = std::string("UNDOCKING_DONE_START_MANUAL_MAPPING");
    break;
  case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_NAVIGATION:
    out = std::string("UNDOCKING_DONE_START_NAVIGATION");
    break;
  case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_FACTORY_NAVIGATION:
    out = std::string("UNDOCKING_DONE_START_FACTORY_NAVIGATION");
    break;
  case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_MANUAL_CONTROL:
    out = std::string("UNDOCKING_DONE_START_MANUAL_CONTROL");
    break;
  case REQUEST_ROBOT_CMD::DONE_AUTO_MAPPING:
    out = std::string("DONE_AUTO_MAPPING");
    break;
  case REQUEST_ROBOT_CMD::DONE_MANUAL_MAPPING:
    out = std::string("DONE_MANUAL_MAPPING");
    break;
  case REQUEST_ROBOT_CMD::DONE_DOCKING:
    out = std::string("DONE_DOCKING");
    break;
  case REQUEST_ROBOT_CMD::DONE_RETURN_CHARGER:
    out = std::string("DONE_RETURN_CHARGER");
    break;
  case REQUEST_ROBOT_CMD::FAIL_RETURN_CHARGER_TRY_DOCKING:
    out = std::string("FAIL_RETURN_CHARGER_TRY_DOCKING");
    break;
  case REQUEST_ROBOT_CMD::ERROR:
    out = std::string("ERROR");
    break;
  case REQUEST_ROBOT_CMD::UNDOCKING_DONE_START_FOLLOWING:
    out = std::string("UNDOCKING_DONE_START_FOLLOWING");
    break;
  case REQUEST_ROBOT_CMD::CANCEL_FOLLOWING_NOT_MOVING:
    out = std::string("CANCEL_FOLLOWING_NOT_MOVING");
    break;
  case REQUEST_ROBOT_CMD::OFFSTATION_TRY_DOCKING:
    out = std::string("OFFSTATION_TRY_DOCKING");
    break;
  case REQUEST_ROBOT_CMD::READY_REBOOT:
    out = std::string("READY_REBOOT");
  }
  return out;
};

struct state_cmd {
  REQUEST_ROBOT_CMD robot_cmd = REQUEST_ROBOT_CMD::VOID;
  REQUEST_SOC_CMD soc_cmd = REQUEST_SOC_CMD::VOID;
}; //초기화 문제.

enum class ROBOT_STATE : int { // 현재 동작중인 STATE.
  IDLE = 0,
  AUTO_MAPPING,
  MANUAL_MAPPING,
  NAVIGATION,
  RETURN_CHARGER,
  DOCKING,
  UNDOCKING,
  ONSTATION,
  FACTORY_NAVIGATION,
  ERROR,
  FOLLOWING,
  MANUAL_CONTROL
};

inline std::string enumToString(const ROBOT_STATE in) {
  std::string out;
  switch (in) {
  case ROBOT_STATE::IDLE:
    out = std::string("IDLE");
    break;
  case ROBOT_STATE::MANUAL_MAPPING:
    out = std::string("MANUAL_MAPPING");
    break;
  case ROBOT_STATE::AUTO_MAPPING:
    out = std::string("AUTO_MAPPING");
    break;
  case ROBOT_STATE::NAVIGATION:
    out = std::string("NAVIGATION");
    break;
  case ROBOT_STATE::RETURN_CHARGER:
    out = std::string("RETURN_CHARGER");
    break;
  case ROBOT_STATE::DOCKING:
    out = std::string("DOCKING");
    break;
  case ROBOT_STATE::UNDOCKING:
    out = std::string("UNDOCKING");
    break;
  case ROBOT_STATE::ONSTATION:
    out = std::string("ONSTATION");
    break;
  case ROBOT_STATE::FACTORY_NAVIGATION:
    out = std::string("FACTORY_NAVIGATION");
    break;
  case ROBOT_STATE::ERROR:
    out = std::string("ERROR");
    break;
  case ROBOT_STATE::FOLLOWING:
    out = std::string("FOLLOWING");
    break;
  case ROBOT_STATE::MANUAL_CONTROL:
    out = std::string("MANUAL_CONTROL");
    break;
  }
  return out;
};

enum class ROBOT_STATUS : int {
  VOID = 0,
  READY,
  START,
  PAUSE,
  RESUME,
  COMPLETE,
  FAIL,
  RECOVERY_PAUSE
};

inline std::string enumToString(const ROBOT_STATUS in) {
  std::string out;
  switch (in) {
  case ROBOT_STATUS::READY:
    out = std::string("READY");
    break;
  case ROBOT_STATUS::START:
    out = std::string("START");
    break;
  case ROBOT_STATUS::PAUSE:
    out = std::string("PAUSE");
    break;
  case ROBOT_STATUS::RESUME:
    out = std::string("RESUME");
    break;
  case ROBOT_STATUS::COMPLETE:
    out = std::string("COMPLETE");
    break;
  case ROBOT_STATUS::FAIL:
    out = std::string("FAIL");
    break;
  case ROBOT_STATUS::RECOVERY_PAUSE:
    out = std::string("RECOVERY_PAUSE");
    break;
  default:
    out = std::string("VOID");
    break;
  }
  return out;
};

enum ROTATION_COMMAND {
  ROTATE_RELATIVE,
  ROTATE_ABSOLUTE,
  ROTATE_360_CCW,
  ROTATE_360_CW,
};

enum class READY_MAPPING : int {
  CHECK_SENSOR,
  CHECK_ODOM_RESET,
  CHECK_NODE_LAUNCH,
  REQUEST_MANEUVER_ON,
  CHECK_MANEUVER,
  COMPLETE,
  FAIL,
  STOP_NODE,
  CHECK_STOP_NODE,
  CHECK_KEEPOUT_STATE,
};

inline std::string enumToString(READY_MAPPING in) {
  std::string out;
  switch (in) {
  case READY_MAPPING::CHECK_NODE_LAUNCH:
    out = std::string("CHECK_NODE");
    break;
  case READY_MAPPING::CHECK_SENSOR:
    out = std::string("CHECK_SENSOR");
    break;
  case READY_MAPPING::CHECK_ODOM_RESET:
    out = std::string("CHECK_ODOM_RESET");
    break;
  case READY_MAPPING::REQUEST_MANEUVER_ON:
    out = std::string("REQUEST_MANEUVER_ON");
    break;
  case READY_MAPPING::CHECK_MANEUVER:
    out = std::string("CHECK_MANEUVER");
    break;
  case READY_MAPPING::COMPLETE:
    out = std::string("COMPLETE");
    break;
  case READY_MAPPING::FAIL:
    out = std::string("FAIL");
    break;
  case READY_MAPPING::STOP_NODE:
    out = std::string("STOP_NODE");
    break;
  case READY_MAPPING::CHECK_STOP_NODE:
    out = std::string("CHECK_STOP_NODE");
    break;
  case READY_MAPPING::CHECK_KEEPOUT_STATE:
    out = std::string("CHECK_KEEPOUT_STATE");
    break;           
  }
  return out;
};

enum class EXPLORE_STATUS : int {
  EXPLORE_VOID = -1,
  EXPLORE_NOT_READY,
  EXPLORE_IDLE,
  EXPLORE_CANCEL,
  EXPLORE_RUNNING,
  EXPLORE_FINISH,
  EXPLORE_ERROR, // node dead or exception case
};

inline std::string enumToString(EXPLORE_STATUS in) {
  std::string out;
  switch (in) {
  case EXPLORE_STATUS::EXPLORE_VOID:
    out = std::string("EXPLORE_VOID");
    break;
  case EXPLORE_STATUS::EXPLORE_NOT_READY:
    out = std::string("EXPLORE_NOT_READY");
    break;
  case EXPLORE_STATUS::EXPLORE_IDLE:
    out = std::string("EXPLORE_IDLE");
    break;
  case EXPLORE_STATUS::EXPLORE_CANCEL:
    out = std::string("EXPLORE_CANCEL");
    break;
  case EXPLORE_STATUS::EXPLORE_RUNNING:
    out = std::string("EXPLORE_RUNNING");
    break;
  case EXPLORE_STATUS::EXPLORE_FINISH:
    out = std::string("EXPLORE_FINISH");
    break;
  case EXPLORE_STATUS::EXPLORE_ERROR:
    out = std::string("EXPLORE_ERROR");
    break;
  }
  return out;
};

enum class ROTATE_STATE {
  ROTATE_IDLE,
  ROTATE_ODOM_SET,
  ROTATE_ODOM_CHECK,
  ROTATE_LIDAR_CHECK,
  ROTATE_START
};

inline std::string enumToString(ROTATE_STATE in) {
  std::string out;
  switch (in) {
  case ROTATE_STATE::ROTATE_IDLE:
    out = std::string("ROTATE_IDLE");
    break;
  case ROTATE_STATE::ROTATE_ODOM_SET:
    out = std::string("ROTATE_ODOM_SET");
    break;
  case ROTATE_STATE::ROTATE_ODOM_CHECK:
    out = std::string("ROTATE_ODOM_CHECK");
    break;
  case ROTATE_STATE::ROTATE_LIDAR_CHECK:
    out = std::string("ROTATE_LIDAR_CHECK");
    break;
  case ROTATE_STATE::ROTATE_START:
    out = std::string("ROTATE_START");
    break;
  }
  return out;
};

enum class LOCALIZATION_MODE {
  VOID = -1,
  LOCAL_ONLY,
  GLOBAL_ALLOWED,
};

inline std::string enumToString(LOCALIZATION_MODE in) {
  std::string out;
  switch (in) {
  case LOCALIZATION_MODE::VOID:
    out = std::string("VOID");
    break;
  case LOCALIZATION_MODE::LOCAL_ONLY:
    out = std::string("LOCAL_ONLY");
    break;
  case LOCALIZATION_MODE::GLOBAL_ALLOWED:
    out = std::string("GLOBAL_ALLOWED");
    break;
  }
  return out;
};

enum class READY_FOLLOWING : int {
  CHECK_NODE,
  CHECK_SENSOR,
  CHECK_ODOM_RESET,
  LAUNCH_FOLLOW_NODE,
  LAUNCH_LOCAL_NODE,
  READY_LOCALZIATION,
  START_LOCALIZATION,
  CHECK_LOCALIZATION,
  COMPLETE,
  FAIL,
  STOP_NODE,
  CHECK_STOP_NODE,
};

inline std::string enumToString(READY_FOLLOWING in) {
  std::string out;
  switch (in) {
  case READY_FOLLOWING::CHECK_NODE:
    out = std::string("CHECK_NODE");
    break;
  case READY_FOLLOWING::CHECK_SENSOR:
    out = std::string("CHECK_SENSOR");
    break;
  case READY_FOLLOWING::CHECK_ODOM_RESET:
    out = std::string("CHECK_ODOM_RESET");
    break;
  case READY_FOLLOWING::LAUNCH_FOLLOW_NODE:
    out = std::string("LAUNCH_FOLLOW_NODE");
    break;
  case READY_FOLLOWING::LAUNCH_LOCAL_NODE:
    out = std::string("LAUNCH_LOCAL_NODE");
    break;
  case READY_FOLLOWING::READY_LOCALZIATION:
    out = std::string("READY_LOCALZIATION");
    break;
  case READY_FOLLOWING::START_LOCALIZATION:
    out = std::string("START_LOCALIZATION");
    break;
  case READY_FOLLOWING::CHECK_LOCALIZATION:
    out = std::string("CHECK_LOCALIZATION");
    break;
  case READY_FOLLOWING::COMPLETE:
    out = std::string("COMPLETE");
    break;
  case READY_FOLLOWING::FAIL:
    out = std::string("FAIL");
    break;
  case READY_FOLLOWING::STOP_NODE:
    out = std::string("STOP_NODE");
    break;
  case READY_FOLLOWING::CHECK_STOP_NODE:
    out = std::string("CHECK_STOP_NODE");
  }
  return out;
};

enum class FOLLOWING_STATE {
  IDLE = 0,
  INITIALIZING,
  SEARCHING,
  SEARCHING_FAILED, //S12-1
  SEARCHING_COMPLETE,
  FOLLOWING,
  FOLLOWING_RESEARCHING,
  FOLLOWING_FAILED,//S12-1
  FOLLOWING_FAILED_KEEPOUT,//S12-2
  FOLLOWING_FAILED_OBSTACLE,//S12-3
  FOLLOWING_CANCELED,
  FOLLOWING_CANCELED_OTHER,
  FOLLOWING_CANCELED_NOT_MOVING,
  FOLLOWING_PAUSED,
  RESERSVED,
};

inline std::string enumToString(FOLLOWING_STATE in) {
  std::string out;
  switch (in) {
  case FOLLOWING_STATE::IDLE:
    out = std::string("IDLE");
    break;
  case FOLLOWING_STATE::INITIALIZING:
    out = std::string("INITIALIZING");
    break;
  case FOLLOWING_STATE::SEARCHING:
    out = std::string("SEARCHING");
    break;
  case FOLLOWING_STATE::SEARCHING_FAILED:
    out = std::string("SEARCHING_FAILED");
    break;
  case FOLLOWING_STATE::SEARCHING_COMPLETE:
    out = std::string("SEARCHING_COMPLETE");
    break;
  case FOLLOWING_STATE::FOLLOWING:
    out = std::string("FOLLOWING");
    break;
  case FOLLOWING_STATE::FOLLOWING_RESEARCHING:
    out = std::string("FOLLOWING_RESEARCHING");
    break;
  case FOLLOWING_STATE::FOLLOWING_FAILED:
    out = std::string("FOLLOWING_FAILED");
    break;
  case FOLLOWING_STATE::FOLLOWING_FAILED_KEEPOUT:
    out = std::string("FOLLOWING_FAILED_KEEPOUT");
    break;
  case FOLLOWING_STATE::FOLLOWING_FAILED_OBSTACLE:
    out = std::string("FOLLOWING_FAILED_OBSTACLE");
    break;
  case FOLLOWING_STATE::FOLLOWING_CANCELED:
    out = std::string("FOLLOWING_CANCELED");
    break;
  case FOLLOWING_STATE::FOLLOWING_CANCELED_OTHER:
      out = std::string("FOLLOWING_CANCELED_OTHER");
      break;
  case FOLLOWING_STATE::FOLLOWING_CANCELED_NOT_MOVING:
    out = std::string("FOLLOWING_CANCELED_NOT_MOVING");
    break;
  case FOLLOWING_STATE::FOLLOWING_PAUSED:
    out = std::string("FOLLOWING_PAUSED");
    break;
  case FOLLOWING_STATE::RESERSVED:
    out = std::string("RESERSVED");
    break;
  }
  return out;
};

enum class KEEPOUT_STATE {
  KEEPOUT_VOID,
  KEEPOUT_ENABLE,
  KEEPOUT_DISABLE,
};

inline std::string enumToString(KEEPOUT_STATE in){
  std::string out;
  switch (in) {
  case KEEPOUT_STATE::KEEPOUT_VOID:
    out = std::string("KEEPOUT_VOID");
    break;
  case KEEPOUT_STATE::KEEPOUT_ENABLE:
    out = std::string("KEEPOUT_ENABLED");
    break;
  case KEEPOUT_STATE::KEEPOUT_DISABLE:
    out = std::string("KEEPOUT_DISABLED");
    break;
  }
  return out;
};

enum class READY_MANUAL_CONTROL : int {
  CHECK_NODE,
  CHECK_SENSOR,
  CHECK_ODOM_RESET,
  LAUNCH_NODE,
  READY_LOCALZIATION,
  START_LOCALIZATION,
  CHECK_LOCALIZATION,
  COMPLETE,
  FAIL,
  STOP_NODE,
  CHECK_STOP_NODE,
};

inline std::string enumToString(READY_MANUAL_CONTROL in) {
  std::string out;
  switch (in) {
  case READY_MANUAL_CONTROL::CHECK_NODE:
    out = std::string("CHECK_NODE");
    break;
  case READY_MANUAL_CONTROL::CHECK_SENSOR:
    out = std::string("CHECK_SENSOR");
    break;
  case READY_MANUAL_CONTROL::CHECK_ODOM_RESET:
    out = std::string("CHECK_ODOM_RESET");
    break;
  case READY_MANUAL_CONTROL::LAUNCH_NODE:
    out = std::string("LAUNCH_NODE");
    break;
  case READY_MANUAL_CONTROL::READY_LOCALZIATION:
    out = std::string("READY_LOCALZIATION");
    break;
  case READY_MANUAL_CONTROL::START_LOCALIZATION:
    out = std::string("START_LOCALIZATION");
    break;
  case READY_MANUAL_CONTROL::CHECK_LOCALIZATION:
    out = std::string("CHECK_LOCALIZATION");
    break;
  case READY_MANUAL_CONTROL::COMPLETE:
    out = std::string("COMPLETE");
    break;
  case READY_MANUAL_CONTROL::FAIL:
    out = std::string("FAIL");
    break;
  case READY_MANUAL_CONTROL::STOP_NODE:
    out = std::string("STOP_NODE");
    break;
  case READY_MANUAL_CONTROL::CHECK_STOP_NODE:
    out = std::string("CHECK_STOP_NODE");
  }
  return out;
};

enum class MANUAL_CONTROL_STATE {
  IDLE = 0,
  INITIALIZING,
  RESERVED2,
  RESERVED3,
  RESERVED4,
  MANUAL_CONTROL,
  RESERVED6,
  RESERVED7,
  MANUAL_CONTROL_FAILED_KEEPOUT,//S13-2
  MANUAL_CONTROL_FAILED_OBSTACLE,//S13-3
  RESERVED10,
  RESERVED11,
  RESERVED12,
  RESERVED13,
  RESERVED14
};

inline std::string enumToString(MANUAL_CONTROL_STATE in) {
  std::string out;
  switch (in) {
  case MANUAL_CONTROL_STATE::IDLE:
    out = std::string("IDLE");
    break;
  case MANUAL_CONTROL_STATE::INITIALIZING:
    out = std::string("INITIALIZING");
    break;
  case MANUAL_CONTROL_STATE::MANUAL_CONTROL:
    out = std::string("MANUAL_CONTROL");
    break;
  case MANUAL_CONTROL_STATE::MANUAL_CONTROL_FAILED_KEEPOUT:
    out = std::string("MANUAL_CONTROL_FAILED_KEEPOUT");
    break;
  case MANUAL_CONTROL_STATE::MANUAL_CONTROL_FAILED_OBSTACLE:
    out = std::string("MANUAL_CONTROL_FAILED_OBSTACLE");
    break;
  default:
    out = std::string("RESERVED");
    break;
  return out;
  }
};

} // namespace airbot_state

#endif // NAVIGATION_HPP_
