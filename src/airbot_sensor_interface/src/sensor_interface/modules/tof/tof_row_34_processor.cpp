#include "rclcpp/rclcpp.hpp"
#include "sensor_interface/modules/tof/tof_row_34_processor.hpp"

TofRow34Processor::TofRow34Processor()
{
}

TofRow34Processor::~TofRow34Processor()
{
}
std_msgs::msg::Float64MultiArray TofRow34Processor::getTofRow34DiffData(const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    // ######################## 타임 스탬프 검사 ########################
    // 이전 타임스탬프를 안전하게 초기화
    static rclcpp::Time last_timestamp(0, 0, RCL_ROS_TIME);
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    rclcpp::Time now = ros_clock.now();

    // 메시지 타임스탬프가 0이면 유효하지 않은 데이터로 간주하고 무시
    if (msg->timestamp.sec == 0 && msg->timestamp.nanosec == 0) {
        // RCLCPP_WARN(rclcpp::get_logger("TofRow34Processor"), "Received invalid timestamp! Skipping frame...");
        return std_msgs::msg::Float64MultiArray();
    }

    // sec 값이 정상적으로 할당되지 않았다면 스킵
    if (msg->timestamp.sec < 1577836800) { // 2020-01-01 00:00:00 UTC 이전이면 비정상
        // RCLCPP_WARN(rclcpp::get_logger("TofRow34Processor"), "Received invalid timestamp (too old)! Skipping frame...");
        return std_msgs::msg::Float64MultiArray();
    }

    // 중복 데이터 방지: 같은 타임스탬프의 데이터가 연속으로 들어오면 처리하지 않음
    if (msg->timestamp == last_timestamp) {
        // RCLCPP_WARN(rclcpp::get_logger("TofRow34Processor"), "Duplicate ToF data received, skipping...");
        return std_msgs::msg::Float64MultiArray();
    }

    last_timestamp = msg->timestamp;  // 최신 데이터 타임스탬프 갱신
    // ###########################################################

    // 1x16 Array
    // {L_3_prev - L_3_curr (4개), R_3_prev - R_3_curr (4개), L_3 - L_4 (4개), R_3 - R_4 (4개)}
    std_msgs::msg::Float64MultiArray output_msg;
    output_msg.data.resize(16);

    // 현재값 업데디트
    cur_left_tof_3 = {msg->bot_left[8], msg->bot_left[9], msg->bot_left[10], msg->bot_left[11]};
    cur_right_tof_3 = {msg->bot_right[8], msg->bot_right[9], msg->bot_right[10], msg->bot_right[11]};
    cur_left_tof_4 = {msg->bot_left[12], msg->bot_left[13], msg->bot_left[14], msg->bot_left[15]};
    cur_right_tof_4 = {msg->bot_right[12], msg->bot_right[13], msg->bot_right[14], msg->bot_right[15]};

    // 이전 3행 데이터와 현재 3행 데이터의 차이 (L, R)
    for (size_t i = 0; i < 4; i++)
    {
        output_msg.data[i] = pre_left_tof_3[i] - cur_left_tof_3[i];         // L_3_prev - L_3_curr
        output_msg.data[i + 4] = pre_right_tof_3[i] - cur_right_tof_3[i];   // R_3_prev - R_3_curr
    }

    // 현재 3행과 4행의 차이 (L, R)
    for (size_t i = 0; i < 4; i++)
    {
        output_msg.data[i + 8] = cur_left_tof_3[i] - cur_left_tof_4[i];  // L_3 - L_4
        output_msg.data[i + 12] = cur_right_tof_3[i] - cur_right_tof_4[i]; // R_3 - R_4
    }

    // 이전값 업데이트
    pre_left_tof_3 = cur_left_tof_3;
    pre_right_tof_3 = cur_right_tof_3;

    // 로깅
    std::ostringstream log_stream;
    log_stream << "ToF Row 34 Diff Data: [";
    for (size_t i = 0; i < output_msg.data.size(); i++)
    {
        log_stream << output_msg.data[i];
        if (i < output_msg.data.size() - 1)
            log_stream << ", ";
    }
    log_stream << "]";

    // RCLCPP_INFO(rclcpp::get_logger("TofRow34Processor"), "%s", log_stream.str().c_str());

    return output_msg;
}