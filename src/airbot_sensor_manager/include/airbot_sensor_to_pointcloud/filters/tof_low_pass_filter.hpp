#ifndef __TOF_LOW_PASS_FILTER_HPP__
#define __TOF_LOW_PASS_FILTER_HPP__

#include <memory>
#include <array>
#include <vector>
#include "robot_custom_msgs/msg/tof_data.hpp"

#include "rclcpp/rclcpp.hpp"

class TofLowPassFilter
{
public:
    TofLowPassFilter()
        : alpha_(0.5), prev_left_(16, 0.0), prev_right_(16, 0.0), is_initialized_(false) {}

    void updateAlpha(double new_alpha)
    {
        alpha_ = new_alpha;
    }

    robot_custom_msgs::msg::TofData::SharedPtr update(const robot_custom_msgs::msg::TofData::SharedPtr& input_msg)
    {
        auto output = std::make_shared<robot_custom_msgs::msg::TofData>(*input_msg);

        if (!is_initialized_) {
            for (int i = 0; i < 16; ++i) {
                prev_left_[i] = input_msg->bot_left[i];
                prev_right_[i] = input_msg->bot_right[i];
            }
            is_initialized_ = true;
        }

        for (int i = 0; i < 16; ++i) {
            double raw_left = input_msg->bot_left[i];
            double raw_right = input_msg->bot_right[i];

            double filtered_left = alpha_ * raw_left + (1.0 - alpha_) * prev_left_[i];
            double filtered_right = alpha_ * raw_right + (1.0 - alpha_) * prev_right_[i];

            output->bot_left[i] = filtered_left;
            output->bot_right[i] = filtered_right;

            prev_left_[i] = filtered_left;
            prev_right_[i] = filtered_right;
        }

        return output;
    }

private:
    double alpha_;
    std::vector<double> prev_left_;
    std::vector<double> prev_right_;
    bool is_initialized_;
};

#endif // __TOF_LOW_PASS_FILTER_HPP__
