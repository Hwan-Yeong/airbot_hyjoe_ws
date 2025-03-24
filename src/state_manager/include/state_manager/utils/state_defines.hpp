#ifndef STATEDEFINES_HPP_
#define STATEDEFINES_HPP_

#include <string>
namespace airbot_state {

enum class LOCALIZATION_TYPE {
  INIT_POSE,
  ROBOT_POSE,
  SAVED_POSE,
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
  FT_NAVI //node manager에 켜라고 전달하기 위해 추가..
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
  }
  return out;
};

enum class NAVI_FAIL_REASON {
  VOID,
  NODE_OFF,
  SERVER_NO_ACTION,
  GOAL_ABORT,
  GOAL_REJECT,
  UNKWON,
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
  case NAVI_FAIL_REASON::UNKWON:
    out = std::string("UNKWON");
    break;
  }
  return out;
};

enum class REQUEST_SOC_CMD {
  // SOC TRIGGER CMD
  VOID,
  START_AUTO_MAPPING,   // SOC 명령
  START_MANUAL_MAPPING, // SOC 명령
  START_NAVIGATION,     // SOC 명령
  START_RETURN_CHARGER, // SOC 명령
  START_DOCKING,        // SOC 명령
  START_CHARGING,       // SOC 명령
  PAUSE_NAVIGATION,     // SOC 명령
  RESUME_NAVIGATION,    // SOC 명령
  START_FACTORY_NAVIGATION, //reserve SOC명령
  STOP_WORKING //  SOC명령

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
  case REQUEST_SOC_CMD::PAUSE_NAVIGATION:
    out = std::string("PAUSE_NAVIGATION");
    break;
  case REQUEST_SOC_CMD::RESUME_NAVIGATION:
    out = std::string("RESUME_NAVIGATION");
    break;
  case REQUEST_SOC_CMD::START_FACTORY_NAVIGATION:
    out = std::string("START_FACTORY_NAVIGATION");
    break;
  case REQUEST_SOC_CMD::STOP_WORKING:
    out = std::string("STOP_WORKING");
    break;
  }
  return out;
};

enum class REQUEST_ROBOT_CMD{
  VOID,
  START_ONSTATION,      // DOCKING인식시
  STOP_ONSTATION, // Onstation에서 충전 단자 접촉해제시

  UNDOCKING_DONE_START_AUTO_MAPPING,   // UNDOCKING 완료시
  UNDOCKING_DONE_START_MANUAL_MAPPING, // UNDOCKING 완료시
  UNDOCKING_DONE_START_NAVIGATION,     // UNDOCKING 완료시
  UNDOCKING_DONE_START_FACTORY_NAVIGATION,
  DONE_AUTO_MAPPING, // AUTO_MAPPING완료시
  DONE_MANUAL_MAPPING,

  DONE_DOCKING, // DOCKING 완료 시

  DONE_RETURN_CHARGER, // RETURN_CHARGER완료 시
  FAIL_RETURN_CHARGER_TRY_DOCKING,  // RETURN_CHARGER FAIL 시

  ERROR
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
  }
  return out;
};

struct state_cmd{
  REQUEST_ROBOT_CMD robot_cmd;
  REQUEST_SOC_CMD soc_cmd;
};


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
  }
  return out;
};

enum class ROBOT_STATUS : int { 
  VOID = 0,
  READY,
  START,
  PAUSE,
  COMPLETE,
  FAIL,
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
  case ROBOT_STATUS::COMPLETE:
    out = std::string("COMPLETE");
    break;
  case ROBOT_STATUS::FAIL:
    out = std::string("FAIL");
    break;
  default:
    out = std::string("VOID");
    break;
  }
  return out;
};

enum ROTATION_COMMAND
{
    ROTATE_RELATIVE,
    ROTATE_ABSOLUTE,
    ROTATE_360_CCW,
    ROTATE_360_CW,
};

enum class READY_MAPPING : int {
  CHECK_SENSOR,
  CHECK_ODOM_RESET,
  LAUNCH_NODE,
  CHECK_NODE_LAUNCH,
  COMPLETE,
  FAIL,
};

inline std::string enumToString(READY_MAPPING in) {
  std::string out;
  switch (in) {
  case READY_MAPPING::LAUNCH_NODE:
    out = std::string("LAUCN_NODE");
    break;
  case READY_MAPPING::CHECK_NODE_LAUNCH:
    out = std::string("CHECK_NODE");
    break;
    case READY_MAPPING::CHECK_SENSOR:
    out = std::string("CHECK_SENSOR");
    break;    
  case READY_MAPPING::CHECK_ODOM_RESET:
    out = std::string("CHECK_ODOM_RESET");
    break;
  case READY_MAPPING::COMPLETE:
    out = std::string("COMPLETE");
    break;
  case READY_MAPPING::FAIL:
    out = std::string("FAIL");
    break;     
  }
  return out;
};

} // namespace airbot_state

#endif // NAVIGATION_HPP_
