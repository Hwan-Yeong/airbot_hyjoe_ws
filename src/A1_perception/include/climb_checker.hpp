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
    ~ClimbChecker() = default;

    void setParams(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config);
    void setClimb(bool is_climb);
    bool isClimb();

    void climbCheck();
    double compute_exponential_weight_moving_average(double prev, double curr);
    void reset();

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