#ifndef __TOF_COMPLEMENTARY_FILTER_HPP__
#define __TOF_COMPLEMENTARY_FILTER_HPP__

#include <memory>
#include <deque>
#include <vector>
#include "robot_custom_msgs/msg/tof_data.hpp"

#include "rclcpp/rclcpp.hpp"

class TofComplementaryFilter
{
public:
    TofComplementaryFilter()
    {
        comp_alpha_ = 0.9;
        lpf_alpha_ = 0.8;
        ma_window_size_ = 3;

        ma_buffer_left_.resize(16);
        ma_buffer_right_.resize(16);
        lpf_left_.resize(16, 0.0);
        lpf_right_.resize(16, 0.0);
    }

    void updateParams(double alpha_comp, double alpha_lpf, int window_size, std::vector<int>& enabled_row)
    {
        comp_alpha_ = alpha_comp;
        lpf_alpha_ = alpha_lpf;
        ma_window_size_ = window_size;

        valid_idx.fill(false);
        for (int row : enabled_row) {
            if (row >= 1 && row <= 4) {
                int base = (row - 1) * 4;
                for (int i = 0; i < 4; ++i) {
                    valid_idx[base + i] = true;
                }
            }
        }
    }

    robot_custom_msgs::msg::TofData::SharedPtr update(const robot_custom_msgs::msg::TofData::SharedPtr& input_msg)
    {
        auto output = std::make_shared<robot_custom_msgs::msg::TofData>(*input_msg);

        for (int i = 0; i < 16; ++i) {
            if (!valid_idx[i]) continue;

            double raw_left = input_msg->bot_left[i];
            double raw_right = input_msg->bot_right[i];

            if (raw_left > 1e-3) {
                lpf_left_[i] = lpf_alpha_ * raw_left + (1.0 - lpf_alpha_) * lpf_left_[i];

                ma_buffer_left_[i].push_back(raw_left);
                if (static_cast<int>(ma_buffer_left_[i].size()) > ma_window_size_) {
                    ma_buffer_left_[i].pop_front();
                }

                double ma_avg = average(ma_buffer_left_[i]);

                output->bot_left[i] = comp_alpha_ * lpf_left_[i] + (1.0 - comp_alpha_) * ma_avg;
            } else {
                output->bot_left[i] = 0.0;
            }

            if (raw_right > 1e-3) {
                lpf_right_[i] = lpf_alpha_ * raw_right + (1.0 - lpf_alpha_) * lpf_right_[i];

                ma_buffer_right_[i].push_back(raw_right);
                if (static_cast<int>(ma_buffer_right_[i].size()) > ma_window_size_) {
                    ma_buffer_right_[i].pop_front();
                }

                double ma_avg = average(ma_buffer_right_[i]);

                output->bot_right[i] = comp_alpha_ * lpf_right_[i] + (1.0 - comp_alpha_) * ma_avg;
            } else {
                output->bot_right[i] = 0.0;
            }
        }

        return output;
    }

private:
    int ma_window_size_;
    double comp_alpha_;
    double lpf_alpha_;
    std::array<bool, 16> valid_idx;

    std::vector<double> lpf_left_;
    std::vector<double> lpf_right_;

    std::vector<std::deque<double>> ma_buffer_left_;
    std::vector<std::deque<double>> ma_buffer_right_;

    double average(const std::deque<double>& values)
    {
        if (values.empty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (double val : values) {
            sum += val;
        }
        return sum / static_cast<double>(values.size());
    }
};

#endif // __TOF_COMPLEMENTARY_FILTER_HPP__
