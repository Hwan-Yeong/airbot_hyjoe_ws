#ifndef __CLIMB_CHECKER_HPP__
#define __CLIMB_CHECKER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace A1::perception
{
// Forward declaration
class PerceptionNode;

class ClimbChecker
{
   public:
    ClimbChecker() = default;
    ClimbChecker(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);
    ~ClimbChecker() = default;

    void setClimb(bool is_climb);
    bool isClimb(void);

    void climbCheck(void);
    void publishClimbState(void);
    void reset(void);
    double computeExponentialWeightMovingAverage(double prev, double curr);

   private:
    float climbing_pitch_alpha{};
    float enable_climbing_threshold{};
    float disable_climbing_threshold{};
    float estimated_bias{};
    int climbing_timeout;
    rclcpp::Time climbing_time{};
    bool climb_flag{false};
    std::shared_ptr<PerceptionNode> node_ptr{};
};
}  // namespace A1::perception

#endif  // __CLIMB_CHECKER_HPP__