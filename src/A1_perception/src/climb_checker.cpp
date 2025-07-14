
#include "perception_node.hpp"

#include "climb_checker.hpp"
#include "yaml-cpp/yaml.h"

namespace A1::perception
{

ClimbChecker::ClimbChecker(std::shared_ptr<PerceptionNode> node_ptr_, const YAML::Node& config)
{
    this->node_ptr = node_ptr_;
    this->climbing_timeout = config["timeout_ms"].as<int>();
    this->climbing_pitch_alpha = config["pitch_alpha"].as<float>();
    this->enable_climbing_threshold = config["enable_climbing_threshold"].as<float>() * M_PI / 180;
    this->disable_climbing_threshold = config["disable_climbing_threshold"].as<float>() * M_PI / 180;
    this->enable_down_climbing_threshold = config["enable_down_climbing_threshold"].as<float>() * M_PI / 180;
    this->disable_down_climbing_threshold = config["disable_down_climbing_threshold"].as<float>() * M_PI / 180;
    this->estimated_bias = 0;
}

void ClimbChecker::reset()
{
    this->climb_flag = false;
    this->down_climb_flag = false;
    this->estimated_bias = 0;
}

bool ClimbChecker::isClimb()
{
    return this->climb_flag || this->down_climb_flag;
}

void ClimbChecker::climbCheck()
{
    auto node = this->node_ptr;
    float pitch = node->getIMUdata().pitch;

    if ((!this->climb_flag) && ((pitch - this->estimated_bias) < this->enable_climbing_threshold))
    {
        RCLCPP_INFO(
            node->get_logger(),
            "Remove All Drop off data, pause low obstacle detect by climb. Pitch(deg): %f",
            (pitch * 180 / M_PI));

        // this->setClimb(true);
        this->climb_flag = true;
        this->climbing_time = std::chrono::steady_clock::now();
    }
    else if (this->climb_flag && (pitch - this->estimated_bias) >= this->disable_climbing_threshold)
    {
        auto now = std::chrono::steady_clock::now();
        auto interval = std::chrono::milliseconds(static_cast<int64_t>(this->climbing_timeout));
        // if ((node->now() - this->climbing_time) > rclcpp::Duration::from_seconds(this->climbing_timeout / 1000.0))
        if ((now - this->climbing_time) > interval)
        {
            // this->setClimb(false);
            this->climb_flag = false;
            this->estimated_bias = 0;
        }
    }

    if (!this->down_climb_flag && ((pitch - this->estimated_bias) > this->enable_down_climbing_threshold))
    {
        RCLCPP_INFO(node->get_logger(), "Down climb state. Pitch(deg): %f", (pitch * 180 / M_PI));
        RCLCPP_INFO(node->get_logger(), "Remove all multi tof 6 rows data.");
        this->down_climb_flag = true;
        this->down_climbing_time = std::chrono::steady_clock::now();
    }
    else if (this->down_climb_flag && (pitch - this->estimated_bias) <= this->disable_down_climbing_threshold)
    {
        auto now = std::chrono::steady_clock::now();
        auto interval = std::chrono::milliseconds(static_cast<int64_t>(this->climbing_timeout));
        // if ((node->now() - this->down_climbing_time) > rclcpp::Duration::from_seconds(this->climbing_timeout /
        // 1000.0))
        if ((now - this->down_climbing_time) > interval)
        {
            this->down_climb_flag = false;
            this->estimated_bias = 0;
        }
    }

    if (this->isClimb())
    {
        node->clearDropOffLayerMap();
    }
    else
    {
        this->estimated_bias = this->computeExponentialWeightMovingAverage(this->estimated_bias, pitch);
    }

    this->publishClimbState();
}

void ClimbChecker::publishClimbState(void)
{
    auto node = this->node_ptr;
    std_msgs::msg::Bool climb_msg{};
    climb_msg.data = this->isClimb();
    auto pub_ptr =
        std::any_cast<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>>(node->getPublishers()["climb"]);
    pub_ptr->publish(climb_msg);
}

double ClimbChecker::computeExponentialWeightMovingAverage(double prev, double curr)
{
    return (1.0 - this->climbing_pitch_alpha) * prev + this->climbing_pitch_alpha * curr;
}
}  // namespace A1::perception