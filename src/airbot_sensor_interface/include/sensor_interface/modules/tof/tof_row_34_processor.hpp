#ifndef __TOF_ROW_34_PROCESSOR_HPP__
#define __TOF_ROW_34_PROCESSOR_HPP__

#include <cmath>
#include <array>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"

/**
 * @brief: 낮은장애물 감지 알고리즘에 필요한 멀티존 ToF의 3, 4행 데이터를 처리하는 클래스입니다.
 */
class TofRow34Processor
{
public:
    TofRow34Processor();
    ~TofRow34Processor();

    std_msgs::msg::Float64MultiArray getTofRow34DiffData(const robot_custom_msgs::msg::TofData::SharedPtr msg);

private:
    std::array<double, 4> pre_left_tof_3 = {0.0};
    std::array<double, 4> pre_right_tof_3 = {0.0};
    std::array<double, 4> cur_left_tof_3 = {0.0};
    std::array<double, 4> cur_right_tof_3 = {0.0};
    std::array<double, 4> cur_left_tof_4 = {0.0};
    std::array<double, 4> cur_right_tof_4 = {0.0};
};

#endif // __TOF_ROW_34_PROCESSOR_HPP__