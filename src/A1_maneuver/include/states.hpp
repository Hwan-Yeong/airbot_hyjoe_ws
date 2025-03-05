#ifndef __STATES_H__
#define __STATES_H__

enum class ACTION_STATE {
  IDLE,
  STOP,
  BACK,
  FRONT,
};

enum class STOP_STATE {
  NO_STOP,
  DROP_OFF,
  ONE_D_TOF,
};

inline std::string enumToString(ACTION_STATE in) {
  std::string out;
  switch (in) {
  case ACTION_STATE::IDLE:
    out = std::string("IDLE");
    break;
  case ACTION_STATE::STOP:
    out = std::string("STOP");
    break;
  case ACTION_STATE::BACK:
    out = std::string("BACK");
    break;
  case ACTION_STATE::FRONT:
    out = std::string("FRONT");
    break;
  }
  return out;
};

inline std::string enumToString(STOP_STATE in) {
  std::string out;
  switch (in) {
  case STOP_STATE::NO_STOP:
    out = std::string("NO_STOP");
    break;
  case STOP_STATE::DROP_OFF:
    out = std::string("DROP_OFF");
    break;
  case STOP_STATE::ONE_D_TOF:
    out = std::string("ONE_D_TOF");
    break;
  }
  return out;
};
#endif
