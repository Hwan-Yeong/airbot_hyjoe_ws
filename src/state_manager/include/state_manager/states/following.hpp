#ifndef FOLLOWING_HPP_
#define FOLLOWING_HPP_

#include "state_manager/states/state_base.hpp"
#include "follow_msgs/msg/ui_client.hpp"
#include "follow_msgs/msg/detect.hpp"
namespace airbot_state {

enum class FOLLOW_CMD{
  FOLLOW_START = 1,
  FOLLOW_PAUSE,
  FOLLOW_RESUME,
  FOLLOW_STOP_SOC,
  FOLLOW_STOP_AMR
};

inline std::string enumToString(const FOLLOW_CMD in) {
  std::string out;
  switch (in) {
  case FOLLOW_CMD::FOLLOW_START:
    out = std::string("FOLLOW_START");
    break;
  case FOLLOW_CMD::FOLLOW_PAUSE:
    out = std::string("FOLLOW_PAUSE");
    break;
    case FOLLOW_CMD::FOLLOW_RESUME:
    out = std::string("FOLLOW_RESUME");
    break;
  case FOLLOW_CMD::FOLLOW_STOP_SOC:
    out = std::string("FOLLOW_STOP_SOC");
    break;
  case FOLLOW_CMD::FOLLOW_STOP_AMR:
    out = std::string("FOLLOW_STOP_AMR");
    break;
  }
  return out;
};


class Following : public stateBase {
public:

  Following(const int actionID, std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<StateUtils> &utils);

  virtual void pre_run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void run(const std::shared_ptr<StateUtils> &state_utils) override;
  virtual void post_run(const std::shared_ptr<StateUtils> &state_utils) override;

  // odom and odom reset
  ROBOT_STATUS processFollowingReady();
  void setReadyFollowing(READY_FOLLOWING set);
  void handleFollowFinish();
  void publishFollowCmd(const FOLLOW_CMD &cmd);
  int8_t localizationChecker();
  void followingStateCallback( const follow_msgs::msg::Detect::SharedPtr msg);

  rclcpp::Publisher<follow_msgs::msg::UiClient>::SharedPtr following_cmd_pub;
  rclcpp::Subscription<follow_msgs::msg::Detect>::SharedPtr following_state_sub_;

  READY_FOLLOWING ready_working;
  READY_FOLLOWING pre_ready_working;
  int16_t follow_status;
  int16_t pre_follow_status;
  int pub_cnt;
  bool bNeedsInitPoseLocalization = false;
  bool move_started = false;
  bool pause_flag = false;

  uint8_t retry_localization;
  uint8_t local_mode_retry_cnt = 0;
  
  std::chrono::time_point<std::chrono::steady_clock> localization_start_time;
};

} // namespace airbot_state

#endif // AUTOMAPPING_HPP_
