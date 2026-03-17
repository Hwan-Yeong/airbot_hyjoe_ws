#pragma once

#include <ctime>
#include <deque>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "cloud_converter/cloud_converter_strategy.hpp"
#include "robot_custom_msgs/msg/tof_data.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "utils/common_struct.hpp"
#include "utils/json.hpp"

namespace sensor_manager {

using json = nlohmann::ordered_json;

class MultizoneTofCalibrator {
 public:
  MultizoneTofCalibrator(rclcpp::Logger logger,
                         const TofCalibrationParam& param);

  /**
   * @brief Returns calibration results (6-data array) upon success.
   *
   * @return {L_13, L_14, L_15, R_13, R_14, R_15} float type array.
   * @note Should only be called when both Left and Right are completed.
   */
  const std::array<float, 6>& GetResultArray() const {
    return mtof_calib_result_array_;
  }

  /**
   * @brief Sets L/R Calibration completion status.
   *
   * @param[in] side Left or Right.
   * @param[in] is_done Completion status (true: Done, false: Not Done).
   */
  void SetCalibrationDone(TofSide side, bool is_done);

  /**
   * @brief Gets L/R Calibration completion status.
   *
   * @param[in] side Left or Right.
   * @return Completion status (true: Done, false: Not Done).
   */
  bool IsCalibrationDone(TofSide side) const;

  /**
   * @brief Sets the Calibration operation state.
   *
   * @param[in] state (INACTIVE: Inactive, ACTIVE_LEFT: Left in progress,
   * ACTIVE_RIGHT: Right in progress).
   */
  void SetCalibrationState(MToFCalibState state);

  /**
   * @brief Gets the Calibration operation state.
   *
   * @return Calibration operation state.
   */
  MToFCalibState GetCalibrationState() const;

  /**
   * @brief Sets the converter used for Calibration coordinate conversion.
   *        (Receives L/R Converter pointer and sets it).
   */
  void SetConverter(CloudConverterPtr converter);

  /**
   * @brief Returns state information converted to the agreed protocol based on Calibration results.
   *
   * @param[in] side Left or Right.
   * @param[in] state Calibration state.
   * @return Returns L/R state information saved as HEX in int type.
   */
  uint8_t MakeMTofState(TofSide side, MToFCalibResult state);

  /**
   * @brief Initializes class member variables.
   */
  void Reset();

  /**
   * @brief Calibration update function.
   *
   * @param[in] calib_result Calibration result data pointer, directly updates calibration result data.
   * @param[in] msg tof data.
   * @param[in] side Left or Right.
   * @return Returns calibration result state (running / pass / fail).
   */
  MToFCalibResult Update(MToFCalibData& calib_result,
                         const robot_custom_msgs::msg::TofData::SharedPtr msg,
                         TofSide side);

 private:
  /**
   * @brief Main Calibration logic operating inside the update function.
   *
   * @param[in] calib_result Calibration result data pointer, directly updates calibration result data.
   * @param[in] msg tof data.
   * @param[in] side Left or Right.
   * @return Returns calibration result state (running / pass / fail).
   */
  MToFCalibResult ProcessCalibration(
      MToFCalibData& calib_result,
      const robot_custom_msgs::msg::TofData::SharedPtr msg, TofSide side);

  /**
   * @brief Function to save Calibration data in json format.
   *
   * @param side Left or Right.
   * @param resultCode tof calibration result.
   */
  void WriteSelfTestCalibFile(TofSide side, MToFCalibResult resultCode);

  /**
   * @brief Function to check if a file exists and save up to 10 lines of data from the file to the buffer.
   *
   * @param path Json file path.
   * @param buffer exist data in file.
   * @return true, File open success.
   * @return false, Fail to open file or fail to create file.
   */
  bool CheckFileExist(std::string path, std::deque<std::string>& buffer);

  /**
   * @brief Function to create json data.
   * Format : {"time": "YY-MM-DD HH:MM:SS", "side": "Left"/"Right", "result":
   * "PASS"/"FAILE", "failCode": "0xXX", "data": [x.xxx, x.xxx, x.xxx]}
   *
   * @param j nlohmann::ordered_json.
   */
  void CreateJsonData(json& j, TofSide side, MToFCalibResult resultCode);

  /**
   * @brief Function to save json data to a file.
   *
   * @param path json file path.
   * @param buffer exist data in json file.
   * @param output_data new data to write file.
   */
  void WriteDataFile(const std::string& path,
                     const std::deque<std::string>& buffer,
                     const json& output_data);

  /**
   * @brief Truncates to n decimal places.
   *
   * @param value double, original real value.
   * @param n int, decimal places to keep.
   * @return double, real value truncated to n decimal places.
   */
  double TruncateToN(double value, int n);

  rclcpp::Logger logger_;
  TofCalibrationParam mtof_calib_cfg_;
  CloudConverterPtr converter_ = nullptr;

  MToFCalibState calib_state_ = MToFCalibState::kInactive;
  bool is_left_done_ = false;
  bool is_right_done_ = false;
  MToFCalibSession
      calib_session_;  // Member variable storing key variables such as sampling
                       // cnt, raw data, converted data, result data, etc.
  std::array<float, 6> mtof_calib_result_array_{};
};

}  // namespace sensor_manager


