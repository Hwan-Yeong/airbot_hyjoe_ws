#pragma once

#include <memory>

#include <deque>
#include <ctime>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"

#include "utils/json.hpp"
#include "utils/common_struct.hpp"
#include "cloud_converter/cloud_converter.hpp"

namespace sensor_manager {

using json = nlohmann::ordered_json;

class MultizoneTofCalibrator {
  public:
    MultizoneTofCalibrator(rclcpp::Logger logger, const tTofCalibrationParam& param);

    const std::array<float, 6>& getResultArray() const { return mtof_calib_result_array_; } //Left,Right 모두 완료되었을 때만 호출해야 함
    void setCalibrationDone(TOF_SIDE side, bool is_done);
    bool isCalibrationDone(TOF_SIDE side) const;
    void setCalibrationState(MTOF_CALIB_STATE state);
    MTOF_CALIB_STATE getCalibrationState() const;
    void setConverter(CloudConverterPtr converter);
    uint8_t makeMTofState(TOF_SIDE side, MTOF_CALIB_RESULT state);
    void reset();
    MTOF_CALIB_RESULT update(MTOF_CALIB_DATA& calib_result,
      const robot_custom_msgs::msg::TofData::SharedPtr msg,
      TOF_SIDE side);

  private:
    MTOF_CALIB_RESULT processCalibration(MTOF_CALIB_DATA& calib_result,
      const robot_custom_msgs::msg::TofData::SharedPtr msg,
      TOF_SIDE side);

    void writeSelfTestCalibFile(TOF_SIDE side, MTOF_CALIB_RESULT resultCode);
    bool checkFileExist(std::string path, std::deque<std::string> &buffer);
    void createJsonData(json &j, TOF_SIDE side, MTOF_CALIB_RESULT resultCode);
    void writeDataFile(const std::string& path, const std::deque<std::string>& buffer, const json& output_data);

    double truncate_to_n(double value, int n);

    rclcpp::Logger logger_;
    tTofCalibrationParam mtof_calib_cfg_;
    CloudConverterPtr converter_ = nullptr;

    MTOF_CALIB_STATE calib_state_ = MTOF_CALIB_STATE::INACTIVE; //isActiveMToFCalibration
    bool is_left_done_ = false; //bLeftMToFCalibrationSet
    bool is_right_done_ = false; //bRightMToFCalibrationSet
    tMToFCalibSession calib_session_; //샘플링cnt, raw데이터, 변환데이터, 결과데이터 등 주요 변수 저장 멤버변수
    std::array<float, 6> mtof_calib_result_array_{};
};

} // namespace sensor_manager
