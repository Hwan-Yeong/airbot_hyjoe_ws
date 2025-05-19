#ifndef __TOF_MOVING_AVERAGE_FILTER_HPP__
#define __TOF_MOVING_AVERAGE_FILTER_HPP__

#include <memory>
#include <deque>
#include <vector>
#include "robot_custom_msgs/msg/tof_data.hpp"

#include "rclcpp/rclcpp.hpp"

class TofMovingAverageFilter
{
public:
    TofMovingAverageFilter()
        : window_size_(3)
    {
        left_history_.resize(16);
        right_history_.resize(16);
    }

    void updateWindowSize(int new_window_size)
    {
        window_size_ = new_window_size;
    }

    robot_custom_msgs::msg::TofData::SharedPtr update(const robot_custom_msgs::msg::TofData::SharedPtr& input_msg)
    {
        auto output = std::make_shared<robot_custom_msgs::msg::TofData>(*input_msg);

        for (int i = 0; i < 16; ++i) {
            double raw_left = input_msg->bot_left[i];
            double raw_right = input_msg->bot_right[i];

            if (raw_left > 1e-3) {
                left_history_[i].push_back(raw_left);
            } else {
                // nothing
            }

            if (raw_right > 1e-3) {
                right_history_[i].push_back(raw_right);
            } else {
                // nothing
            }

            if (static_cast<int>(left_history_[i].size()) > window_size_) {
                left_history_[i].pop_front();
            }
            if (static_cast<int>(right_history_[i].size()) > window_size_) {
                right_history_[i].pop_front();
            }

            raw_left<1e-3 ? 0.0 : output->bot_left[i] = average(left_history_[i]);
            raw_right<1e-3 ? 0.0 : output->bot_right[i] = average(right_history_[i]);
        }
        return output;
    }

private:
    int window_size_;
    std::vector<std::deque<double>> left_history_;
    std::vector<std::deque<double>> right_history_;

    double average(const std::deque<double>& values)
    {
        if (values.empty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (const auto& v : values) {
            sum += v;
        }
        return sum / static_cast<double>(values.size());
    }
};

#endif // __TOF_MOVING_AVERAGE_FILTER_HPP__
