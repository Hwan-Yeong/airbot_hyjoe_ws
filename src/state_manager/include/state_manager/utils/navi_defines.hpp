#ifndef NAVIDEFINES_HPP_
#define NAVIDEFINES_HPP_

#include <string>
#include "state_defines.hpp"

namespace airbot_state {

  enum class READY_NAVIGATION : int {
    LAUCN_NODE,
    CHECK_NODE,
    CHECK_SENSOR,
    CHECK_ODOM_RESET,
    REQUEST_POSE_ESTIMATE,
    CHECK_POSE_ESTIMATE,
    COMPLETE,
    FAIL,
  };
  
  inline std::string enumToString(READY_NAVIGATION in) {
    std::string out;
    switch (in) {
    case READY_NAVIGATION::LAUCN_NODE:
      out = std::string("LAUCN_NODE");
      break;
    case READY_NAVIGATION::CHECK_NODE:
      out = std::string("CHECK_NODE");
      break;
      case READY_NAVIGATION::CHECK_SENSOR:
      out = std::string("CHECK_SENSOR");
      break;    
    case READY_NAVIGATION::CHECK_ODOM_RESET:
      out = std::string("CHECK_ODOM_RESET");
      break;
    case READY_NAVIGATION::REQUEST_POSE_ESTIMATE:
      out = std::string("REQUEST_POSE_ESTIMATE");
      break;
    case READY_NAVIGATION::CHECK_POSE_ESTIMATE:
      out = std::string("CHECK_POSE_ESTIMATE");
      break;    
    case READY_NAVIGATION::COMPLETE:
      out = std::string("COMPLETE");
      break;
    case READY_NAVIGATION::FAIL:
      out = std::string("FAIL");
      break;     
    }
    return out;
  };

  enum class READY_MOVING : int {
    IDLE,
    CHECK_SENSOR,
    REQUEST_POSE_ESTIMATE,
    CHECK_POSE_ESTIMATE,
    COMPLETE,
    FAIL,
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
    case READY_MOVING::REQUEST_POSE_ESTIMATE:
      out = std::string("REQUEST_POSE_ESTIMATE");
      break;
    case READY_MOVING::CHECK_POSE_ESTIMATE:
      out = std::string("CHECK_POSE_ESTIMATE");
      break;
    case READY_MOVING::COMPLETE:
      out = std::string("COMPLETE");
      break;
      case READY_MOVING::FAIL:
      out = std::string("FAIL");
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
    pose target_position;
  };
} // namespace airbot_state

#endif // NAVIDEFINES_HPP_
