#include "utils/multizone_tof_calibrator.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace sensor_manager {

MultizoneTofCalibrator::MultizoneTofCalibrator(rclcpp::Logger logger,
                                               const TofCalibrationParam& param)
    : logger_(logger), mtof_calib_cfg_(param) {
  mtof_calib_result_array_.fill(0.0f);
  RCLCPP_INFO(logger_,
              "[MultizoneTofCalibrator] initialized with method: %s",
              mtof_calib_cfg_.method.c_str());
}

void MultizoneTofCalibrator::SetCalibrationDone(TofSide side, bool is_done) {
  if (side == TofSide::kLeft)
    is_left_done_ = is_done;
  else if (side == TofSide::kRight)
    is_right_done_ = is_done;
}

bool MultizoneTofCalibrator::IsCalibrationDone(TofSide side) const {
  return (side == TofSide::kLeft) ? is_left_done_ : is_right_done_;
}

void MultizoneTofCalibrator::SetCalibrationState(MToFCalibState state) {
  calib_state_ = state;
  if (state != MToFCalibState::kInactive) {
    calib_session_.Reset();
  }
}

MToFCalibState MultizoneTofCalibrator::GetCalibrationState() const {
  return calib_state_;
}

void MultizoneTofCalibrator::SetConverter(CloudConverterPtr converter) {
  converter_ = converter;
}

void MultizoneTofCalibrator::Reset() {
  calib_session_.Reset();
  is_left_done_ = false;
  is_right_done_ = false;
  calib_state_ = MToFCalibState::kInactive;
}

MToFCalibResult MultizoneTofCalibrator::Update(
    MToFCalibData& calib_result,
    const robot_custom_msgs::msg::TofData::SharedPtr msg, TofSide side) {
  MToFCalibResult ret = ProcessCalibration(calib_result, msg, side);

  if (ret != MToFCalibResult::kRunning) {
    RCLCPP_INFO(logger_, "[Calibration: %s] SIDE: %s",
                EnumToString(ret).c_str(), EnumToString(calib_state_).c_str());

    calib_state_ = MToFCalibState::kInactive;

    if (ret == MToFCalibResult::kPass) {
      if (side == TofSide::kLeft) {
        is_left_done_ = true;
      }
      if (side == TofSide::kRight) {
        is_right_done_ = true;
      }
    }

    // save log after calibration done
    WriteSelfTestCalibFile(side, ret);
  }

  return ret;
}

MToFCalibResult MultizoneTofCalibrator::ProcessCalibration(
    MToFCalibData& calib_result,
    const robot_custom_msgs::msg::TofData::SharedPtr msg, TofSide side) {
  MToFCalibResult ret = MToFCalibResult::kRunning;

  if (converter_ == nullptr) {
    RCLCPP_ERROR(logger_,
                 "[MultizoneTofCalibrator] Converter is not set-up yet!");
    return MToFCalibResult::kFailUnknown;
  }

  // 1. Threshold TF Conversion
  auto pnp_min_msg = std::make_shared<robot_custom_msgs::msg::TofData>();
  auto pnp_max_msg = std::make_shared<robot_custom_msgs::msg::TofData>();

  auto fill_tof_msg = [](auto& tof_msg, float val) {
    for (int i = 0; i < 16; ++i) {
      tof_msg->bot_left[i] = val;
      tof_msg->bot_right[i] = val;
    }
  };

  fill_tof_msg(pnp_min_msg, mtof_calib_cfg_.pass_min_value);
  fill_tof_msg(pnp_max_msg, mtof_calib_cfg_.pass_max_value);

  auto min_th_arr = converter_->CalibrationConvert(
      static_cast<const void*>(pnp_min_msg.get()));
  auto max_th_arr = converter_->CalibrationConvert(
      static_cast<const void*>(pnp_max_msg.get()));

  // 2. Input Data TF Conversion
  auto current_data_arr =
      converter_->CalibrationConvert(static_cast<const void*>(msg.get()));

  if (current_data_arr.data.size() < 3 || min_th_arr.data.size() < 3) {
    RCLCPP_ERROR(logger_, "[MultizoneTofCalibrator] Data size mismatch!");
    return MToFCalibResult::kFailUnknown;
  }

  // 3. Session Reset and Data Update Check
  if (calib_session_.is_finish_sampling && calib_session_.sample_count != 0) {
    calib_session_.Reset();
    RCLCPP_INFO(logger_,
                "[MultizoneTofCalibrator] Starting session for %s. Method: %s, "
                "Target Samples: %d",
                (side == TofSide::kLeft ? "LEFT" : "RIGHT"),
                mtof_calib_cfg_.method.c_str(), mtof_calib_cfg_.sampling_count);
  }

  // Increment attempt count (regardless of validity)
  calib_session_.attempt_count++;

  bool any_valid = false;
  for (int i = 0; i < 3; ++i) {
    int idx = calib_session_.kTargetIndices[i];
    float raw_val = (side == TofSide::kLeft) ? msg->bot_left[idx]
                                             : msg->bot_right[idx];

    if (raw_val > 1e-6 && !std::isnan(raw_val)) {
      any_valid = true;
      break;
    }
  }

  // FAIL if all values are invalid
  if (!any_valid) {
    if (calib_session_.attempt_count > 64) {
      RCLCPP_ERROR(logger_,
                   "[MultizoneTofCalibrator] FAIL: No valid TOF data. "
                   "attempts=%d, calib result: FAIL_DATA_NON_RENEWAL",
                   calib_session_.attempt_count);
      calib_session_.is_finish_sampling = true;
      return MToFCalibResult::kFailDataNonRenewal;
    }
    return MToFCalibResult::kRunning;
  }

  // Data Update Check and Accumulation (Integrated into for loop)
  for (int i = 0; i < 3; ++i) {
    int target_idx = calib_session_.kTargetIndices[i];  // 13, 14, 15
    float raw_val = (side == TofSide::kLeft) ? msg->bot_left[target_idx]
                                             : msg->bot_right[target_idx];

    // Skip 0.0 or nan data
    if (raw_val <= 1e-6 || std::isnan(raw_val)) {
      return MToFCalibResult::kRunning;
    }

    // Check data renewal
    if (!calib_session_.origins[i].empty() &&
        std::abs(calib_session_.origins[i].back() - raw_val) < 1e-6f) {
      calib_session_.non_renewal_counts[i]++;
    } else {
      calib_session_.non_renewal_counts[i] = 0;
    }

    if (calib_session_.non_renewal_counts[i] >
        mtof_calib_cfg_.data_non_renewal_count) {
      RCLCPP_ERROR(logger_,
                   "[MultizoneTofCalibrator] FAIL: Data not renewing on idx %d",
                   target_idx);
      return MToFCalibResult::kFailDataNonRenewal;
    }

    // Push data
    calib_session_.origins[i].push_back(raw_val);
    calib_session_.samples[i].push_back(current_data_arr.data[i]);
  }
  calib_session_.sample_count++;

  if (calib_session_.sample_count % 100 == 0) {
    RCLCPP_INFO(logger_, "[MultizoneTofCalibrator] Progress: %d/%d...",
                calib_session_.sample_count, mtof_calib_cfg_.sampling_count);
  }

  // 4. Result Judgment
  if (calib_session_.sample_count >= mtof_calib_cfg_.sampling_count) {
    // Calculate Statistics (Max / Median)
    for (int i = 0; i < 3; ++i) {
      auto& v = calib_session_.samples[i];
      if (mtof_calib_cfg_.method == "Max") {
        calib_session_.stats[i] = *std::max_element(v.begin(), v.end());
      } else {
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        calib_session_.stats[i] = v[v.size() / 2];
      }
    }

    // Stability Check (idx 14)
    float min_val =
        *std::min_element(calib_session_.samples[1].begin(),
                          calib_session_.samples[1].end());
    float max_val =
        *std::max_element(calib_session_.samples[1].begin(),
                          calib_session_.samples[1].end());
    float diff = max_val - min_val;

    // Set data for UDP transmission
    if (mtof_calib_cfg_.method == "Max") {
      calib_result.SetMinValue(side, min_val, min_th_arr.data[1]);
      calib_result.SetMaxValue(side, max_val, max_th_arr.data[1]);
    } else if (mtof_calib_cfg_.method == "Median") {
      calib_result.SetMedianValue(side, calib_session_.stats[1]);
    }

    // Result Logging (Simple Style)
    RCLCPP_INFO(logger_,
                "[Calibration Result] Method: %s | Samples: %d\n"
                "  idx_13: %.3f\n"
                "  idx_14: %.3f\n"
                "  idx_15: %.3f",
                mtof_calib_cfg_.method.c_str(), calib_session_.sample_count,
                calib_session_.stats[0], calib_session_.stats[1],
                calib_session_.stats[2]);

    // Range Check
    bool out_of_range = false;
    for (int i = 0; i < 3; ++i) {
      if (calib_session_.stats[i] < min_th_arr.data[i] ||
          calib_session_.stats[i] > max_th_arr.data[i]) {
        out_of_range = true;
        break;
      }
    }

    if (out_of_range) {
      RCLCPP_INFO(logger_,
                  "[Calibration: FAIL_OUT_OF_RANGE]\n"
                  "  idx_13: %.3f (min_th: %.3f, max_th: %.3f)\n"
                  "  idx_14: %.3f (min_th: %.3f, max_th: %.3f)\n"
                  "  idx_15: %.3f (min_th: %.3f, max_th: %.3f)",
                  calib_session_.stats[0], min_th_arr.data[0],
                  max_th_arr.data[0], calib_session_.stats[1],
                  min_th_arr.data[1], max_th_arr.data[1],
                  calib_session_.stats[2], min_th_arr.data[2],
                  max_th_arr.data[2]);
      ret = MToFCalibResult::kFailOutOfRange;
    } else if (diff > mtof_calib_cfg_.pass_diff_th) {
      RCLCPP_INFO(logger_,
                  "[Calibration: FAIL_UNSTABLE_RANGE] Diff: %.4f (Th: %.4f)",
                  diff, mtof_calib_cfg_.pass_diff_th);
      ret = MToFCalibResult::kFailUnstableRange;
    } else {
      RCLCPP_INFO(logger_,
                  "[Calibration: PASS] Side %s successfully calibrated.",
                  (side == TofSide::kLeft ? "LEFT" : "RIGHT"));
      ret = MToFCalibResult::kPass;

      // Save result data
      int offset = (side == TofSide::kLeft) ? 0 : 3;
      for (int i = 0; i < 3; ++i) {
        mtof_calib_result_array_[offset + i] = calib_session_.stats[i];
      }
      calib_result.SetCalibValue(side, calib_session_.stats[0],
                                 calib_session_.stats[1],
                                 calib_session_.stats[2]);
    }
    // calib_session_.sample_count = 0;
    calib_session_.is_finish_sampling = true;
  }

  return ret;
}

/**
 * @brief Left (LSB) / Right (MSB)
 *   Running:         0x01 (Left), 0x10 (Right)
 *   Complete:        0x02 (Left), 0x20 (Right)
 *   Out of Range:    0x03 (Left), 0x30 (Right)
 *   Unstable Range:  0x04 (Left), 0x40 (Right)
 *   Data non renewal:0x08 (Left), 0x80 (Right)
 */
uint8_t MultizoneTofCalibrator::MakeMTofState(TofSide side,
                                              MToFCalibResult state) {
  uint8_t value = 0;

  switch (state) {
    case MToFCalibResult::kRunning:
      value = 0x01;
      break;
    case MToFCalibResult::kPass:
      value = 0x02;
      break;
    case MToFCalibResult::kFailOutOfRange:
      value = 0x03;
      break;
    case MToFCalibResult::kFailUnstableRange:
      value = 0x04;
      break;
    // case MToFCalibResult::kFailTimeOut:          value = 0x08; break;
    case MToFCalibResult::kFailDataNonRenewal:
      value = 0x08;
      break;
    default:
      value = 0x00;
      break;
  }

  if (side == TofSide::kRight) {
    value = (value & 0x0F) << 4;  // Left 0x0?, Right 0x?0
  }

  return value;
}

void MultizoneTofCalibrator::WriteSelfTestCalibFile(TofSide side,
                                                    MToFCalibResult resultCode) {
  std::deque<std::string> buffer;
  std::string tof_calib_file_path =
      "/home/airbot/app_rw/log/MultiCalibration.json";

  if (!CheckFileExist(tof_calib_file_path, buffer)) {
    return;
  }

  json j;
  CreateJsonData(j, side, resultCode);

  WriteDataFile(tof_calib_file_path, buffer, j);
}

bool MultizoneTofCalibrator::CheckFileExist(std::string path,
                                            std::deque<std::string>& buffer) {
  std::ifstream read_file(path);
  int max_lines = 10;

  if (!read_file.good()) {
    RCLCPP_WARN(logger_, "There is no file in path = %s", path.c_str());
    // Create new file if not exists
    std::ofstream make_new_file(path);
    if (!make_new_file) {
      RCLCPP_ERROR(logger_, "Fail to make new file in path = %s", path.c_str());
      read_file.close();
      return false;
    }
    make_new_file.close();
  }

  RCLCPP_INFO(logger_, "File open success.");

  std::string line;
  while (std::getline(read_file, line)) {
    buffer.push_back(line);
    // If the file contains more than 10 lines, delete the oldest content
    if ((int)buffer.size() >= max_lines) {
      buffer.pop_front();
    }
  }
  read_file.close();

  return true;
}

void MultizoneTofCalibrator::CreateJsonData(json& j, TofSide side,
                                            MToFCalibResult resultCode) {
  json tof_data;

  time_t now = time(0);
  tm* ltm = localtime(&now);
  std::ostringstream oss;
  oss << std::put_time(ltm, "%Y-%m-%d %H:%M:%S");
  std::string time_str = oss.str();

  j["time"] = time_str;
  j["side"] = ((side == TofSide::kLeft) ? "Left" : "Right");
  if (resultCode == MToFCalibResult::kPass) {
    j["result"] = "PASS";
  } else {
    j["result"] = "FAIL";
    j["failCode"] = static_cast<int>(resultCode);
  }

  if (side == TofSide::kLeft) {
    for (uint8_t i = 0; i < 3; i++) {
      j["data"].push_back(TruncateToN(mtof_calib_result_array_[i], 3));
    }
  } else if (side == TofSide::kRight) {
    for (uint8_t i = 3; i < (uint8_t)mtof_calib_result_array_.size(); i++) {
      j["data"].push_back(TruncateToN(mtof_calib_result_array_[i], 3));
    }
  }
}

void MultizoneTofCalibrator::WriteDataFile(
    const std::string& path, const std::deque<std::string>& buffer,
    const json& output_data) {
  std::ofstream output_file(path);
  if (!output_file.is_open()) {
    RCLCPP_ERROR(logger_, "Fail to open file for writing, path = %s",
                 path.c_str());
    return;
  }

  for (const auto& line : buffer) {
    output_file << line << std::endl;
  }
  output_file << output_data << std::endl;
  output_file.flush();  // kernel buffer
  output_file.close();

  int fd = ::open(path.c_str(), O_WRONLY | O_APPEND);
  if (fd != -1) {
    // kernel buffer to disk
    ::fsync(fd);
    ::close(fd);
  } else {
    RCLCPP_ERROR(logger_, "Fail to open fsync");
  }

  RCLCPP_INFO(logger_, "File write success.");
}

double MultizoneTofCalibrator::TruncateToN(double value, int n) {
  double scale = std::pow(10.0, n);
  return std::round(value * scale) / scale;
}

}  // namespace sensor_manager
