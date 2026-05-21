#ifndef NAVIDEFINES_HPP_
#define NAVIDEFINES_HPP_

#include <string>
#include "state_defines.hpp"

namespace airbot_state {

  enum class READY_NAVIGATION : int {
    CHECK_NODE,
    CHECK_SENSOR,
    CHECK_ODOM_RESET,
    START_LOCALIZATION,
    CHECK_LOCALIZATION,
    COMPLETE,
    FAIL,
    LAUNCH_NODE,
    STOP_NODE,
    CHECK_STOP_NODE,
    NAVI_PARAM_CHECK,
    MAP_CHANGE,
    CHECK_NAV_ACTIVE
  };
  
  inline std::string enumToString(READY_NAVIGATION in) {
    std::string out;
    switch (in) {
    case READY_NAVIGATION::CHECK_NODE:
      out = std::string("CHECK_NODE");
      break;
      case READY_NAVIGATION::CHECK_SENSOR:
      out = std::string("CHECK_SENSOR");
      break;    
    case READY_NAVIGATION::CHECK_ODOM_RESET:
      out = std::string("CHECK_ODOM_RESET");
      break;
    case READY_NAVIGATION::START_LOCALIZATION:
      out = std::string("START_LOCALIZATION");
      break;
    case READY_NAVIGATION::CHECK_LOCALIZATION:
      out = std::string("CHECK_LOCALIZATION");
      break;  
    case READY_NAVIGATION::COMPLETE:
      out = std::string("COMPLETE");
      break;
    case READY_NAVIGATION::FAIL:
      out = std::string("FAIL");
      break;
    case READY_NAVIGATION::LAUNCH_NODE:
      out = std::string("LAUNCH_NODE");
      break;
    case READY_NAVIGATION::STOP_NODE:
      out = std::string("STOP_NODE");
      break;
    case READY_NAVIGATION::CHECK_STOP_NODE:
      out = std::string("CHECK_STOP_NODE");
      break;   
    case READY_NAVIGATION::NAVI_PARAM_CHECK:
      out = std::string("NAVI_PARAM_CHECK");
      break;
    case READY_NAVIGATION::MAP_CHANGE:
      out = std::string("MAP_CHANGE");
      break;  
    case READY_NAVIGATION::CHECK_NAV_ACTIVE:
      out = std::string("CHECK_NAV_ACTIVE");
      break;    
    }
    return out;
  };

  enum class READY_MOVING : int {
    IDLE,
    CHECK_SENSOR,
    CHECK_LOCALIZATION_MODE,	
    REQUEST_POSE_ESTIMATE,
    CHECK_POSE_ESTIMATE,
    REQUEST_MANEUVER_ON,
    CHECK_MANEUVER,
	  CHECK_PREVIOUS_GOAL,
	  SEND_GOAL,
    COMPLETE,
    CHECK_KEEPOUT_STATE,
  };

  inline std::string enumToString(READY_MOVING in) {
    std::string out;
    switch (in) {
    case READY_MOVING::IDLE:
      out = std::string("IDLE");
      break;  
    case READY_MOVING::CHECK_SENSOR:
      out = std::string("CHECK_SENSOR");
      break;
    case READY_MOVING::CHECK_LOCALIZATION_MODE:
      out = std::string("CHECK_LOCALIZATION_MODE");
      break;
    case READY_MOVING::REQUEST_POSE_ESTIMATE:
      out = std::string("REQUEST_POSE_ESTIMATE");
      break;
    case READY_MOVING::CHECK_POSE_ESTIMATE:
      out = std::string("CHECK_POSE_ESTIMATE");
      break;
    case READY_MOVING::REQUEST_MANEUVER_ON:
      out = std::string("REQUEST_MANEUVER_ON");
      break;
    case READY_MOVING::CHECK_MANEUVER:
      out = std::string("CHECK_MANEUVER");
      break;
	  case READY_MOVING::CHECK_PREVIOUS_GOAL:
      out = std::string("CHECK_PREVIOUS_GOAL");
      break;
	  case READY_MOVING::SEND_GOAL:
      out = std::string("SEND_GOAL");
      break;    
    case READY_MOVING::COMPLETE:
      out = std::string("COMPLETE");
      break;
    case READY_MOVING::CHECK_KEEPOUT_STATE:
      out = std::string("CHECK_KEEPOUT_STATE");
      break;
    }
    return out;
  };

  enum class GoalCancelResponse : int {
    ERROR_NONE = 0, // success
    ERROR_REJECTED = 1, // goal rejected
    ERROR_UNKNOWN_GOAL_ID = 2, // goal not found
    ERROR_GOAL_TERMINATED = 3 // goal has been canceled
  };

  inline std::string enumToString(GoalCancelResponse in) {
    std::string out;
    switch (in) {
    case GoalCancelResponse::ERROR_NONE:
      out = std::string("ERROR_NONE");
      break;
    case GoalCancelResponse::ERROR_REJECTED:
      out = std::string("ERROR_REJECTED");
      break;
    case GoalCancelResponse::ERROR_UNKNOWN_GOAL_ID:
      out = std::string("ERROR_UNKNOWN_GOAL_ID");
      break;
    case GoalCancelResponse::ERROR_GOAL_TERMINATED:
      out = std::string("ERROR_GOAL_TERMINATED");
      break;
    }
    return out;
  };

  struct pose {
    double x;
    double y;
    double theta;
    double timestamp;
  
    pose() : x(0.0), y(0.0), theta(0.0), timestamp(0.0) {}
  
    bool isInitPose() const {
      return (x == 0.0 && y == 0.0 && theta == 0.0 && timestamp == 0.0);
    }
  };

  struct MOVING_DATA
  {
    bool bStartMoving = false;
    uint8_t type;
    pose target_position;
  };
} // namespace airbot_state

#endif // NAVIDEFINES_HPP_
