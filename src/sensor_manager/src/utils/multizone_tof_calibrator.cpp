#include "utils/multizone_tof_calibrator.hpp"

MultizoneTofCalibrator::MultizoneTofCalibrator(rclcpp::Logger logger, const tTofCalibrationParam& param)
    : logger_(logger)
    , mtof_calib_cfg_(param)
{
    mtof_calib_result_array_.fill(0.0f);
    RCLCPP_INFO(logger_, "[MultizoneTofCalibrator] initialized with method: %s", mtof_calib_cfg_.method.c_str());
}

void MultizoneTofCalibrator::setCalibrationDone(TOF_SIDE side, bool is_done)
{
    if (side == TOF_SIDE::LEFT) is_left_done_ = is_done;
    else if (side == TOF_SIDE::RIGHT) is_right_done_ = is_done;
}

bool MultizoneTofCalibrator::isCalibrationDone(TOF_SIDE side) const
{
    return (side == TOF_SIDE::LEFT) ? is_left_done_ : is_right_done_;
}

void MultizoneTofCalibrator::setCalibrationState(MTOF_CALIB_STATE state)
{
    calib_state_ = state;
    if (state != MTOF_CALIB_STATE::INACTIVE) {
        calib_session_.reset();
    }
}

MTOF_CALIB_STATE MultizoneTofCalibrator::getCalibrationState() const
{
    return calib_state_;
}

void MultizoneTofCalibrator::reset()
{
    calib_session_.reset();
    is_left_done_ = false;
    is_right_done_ = false;
    calib_state_ = MTOF_CALIB_STATE::INACTIVE;
}

MTOF_CALIB_RESULT MultizoneTofCalibrator::update(MTOF_CALIB_DATA& calib_result, const robot_custom_msgs::msg::TofData::SharedPtr msg, TOF_SIDE side)
{
    MTOF_CALIB_RESULT ret = processCalibration(calib_result, msg);

    if (ret != MTOF_CALIB_RESULT::RUNNING) {
        RCLCPP_INFO(logger_,
            "[Calibration: %s] SIDE: %s",
            enumToString(ret).c_str(),
            enumToString(calib_state_).c_str()
        );

        calib_state_ = MTOF_CALIB_STATE::INACTIVE;

        if (ret == MTOF_CALIB_RESULT::PASS) {
            if (side == TOF_SIDE::LEFT) {
                is_left_done_ = true;
            }
            if (side == TOF_SIDE::RIGHT) {
                is_right_done_ = true;
            }
        }

        // save log after calibration done
        writeSelfTestCalibFile(side, ret);
    }

    return ret;
}

MTOF_CALIB_RESULT MultizoneTofCalibrator::processCalibration(MTOF_CALIB_DATA& calib_result, const robot_custom_msgs::msg::TofData::SharedPtr msg)
{
    MTOF_CALIB_RESULT ret = MTOF_CALIB_RESULT::RUNNING;

    TOF_SIDE side;
    std::string calib_converter_key;

    if (calib_state_ == MTOF_CALIB_STATE::ACTIVE_LEFT) {
        side = TOF_SIDE::LEFT;
        calib_converter_key = "tof_multi_left";
    } else if (calib_state_ == MTOF_CALIB_STATE::ACTIVE_RIGHT) {
        side = TOF_SIDE::RIGHT;
        calib_converter_key = "tof_multi_right";
    } else {
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    auto calib_converter = converters_.find(calib_converter_key);
    if (calib_converter == converters_.end() || !calib_converter->second) {
        RCLCPP_WARN(logger_, "[Calib] No converter found for: %s", calib_converter_key.c_str());
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    // 1. 임계값(Threshold) TF 변환 계산
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

    auto min_th_arr = calib_converter->second->calibration_convert(static_cast<const void*>(pnp_min_msg.get()));
    auto max_th_arr = calib_converter->second->calibration_convert(static_cast<const void*>(pnp_max_msg.get()));

    // 2. 입력 데이터 TF 변환
    auto current_data_arr = calib_converter->second->calibration_convert(static_cast<const void*>(msg.get()));

    if (current_data_arr.data.size() < 3 || min_th_arr.data.size() < 3) {
        RCLCPP_ERROR(logger_, "[Calib] Data size mismatch!");
        return MTOF_CALIB_RESULT::FAIL_UNKNOWN;
    }

    // 3. 세션 초기화 및 데이터 갱신 체크
    if (calib_session_.sample_count == 0) {
        RCLCPP_INFO(logger_, "[Calib] Starting session for %s. Method: %s, Target Samples: %d", 
                    (side == TOF_SIDE::LEFT ? "LEFT" : "RIGHT"), mtof_calib_cfg_.method.c_str(), mtof_calib_cfg_.sampling_count);
        calib_session_.reset();
    }

    // 갱신 체크 및 데이터 축적 (for 루프로 통합)
    for (int i = 0; i < 3; ++i) {
        int target_idx = calib_session_.TARGET_INDICES[i]; // 13, 14, 15
        float raw_val = (side == TOF_SIDE::LEFT) ? msg->bot_left[target_idx] : msg->bot_right[target_idx];

        // 데이터 갱신 여부 확인
        if (!calib_session_.origins[i].empty() && std::abs(calib_session_.origins[i].back() - raw_val) < 1e-6f) {
            calib_session_.non_renewal_counts[i]++;
        } else {
            calib_session_.non_renewal_counts[i] = 0;
        }

        if (calib_session_.non_renewal_counts[i] > mtof_calib_cfg_.data_non_renewal_count) {
            RCLCPP_ERROR(logger_, "[Calib] FAIL: Data not renewing on idx %d", target_idx);
            return MTOF_CALIB_RESULT::FAIL_DATA_NON_RENEWAL;
        }

        // 데이터 push
        calib_session_.origins[i].push_back(raw_val);
        calib_session_.samples[i].push_back(current_data_arr.data[i]);
    }
    calib_session_.sample_count++;

    if (calib_session_.sample_count % 100 == 0) {
        RCLCPP_INFO(logger_, "[Calib] Progress: %d/%d...", calib_session_.sample_count, mtof_calib_cfg_.sampling_count);
    }

    // 4. 결과 판정
    if (calib_session_.sample_count >= mtof_calib_cfg_.sampling_count) {
        // 통계값 계산 (Max / Median)
        for (int i = 0; i < 3; ++i) {
            if (mtof_calib_cfg_.method == "Max") {
                calib_session_.stats[i] = *std::max_element(calib_session_.samples[i].begin(), calib_session_.samples[i].end());
            } else {
                auto& v = calib_session_.samples[i];
                std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
                calib_session_.stats[i] = v[v.size() / 2];
            }
        }

        // 안정성 체크 (idx 14 기준)
        float min_val = *std::min_element(calib_session_.samples[1].begin(), calib_session_.samples[1].end());
        float max_val = *std::max_element(calib_session_.samples[1].begin(), calib_session_.samples[1].end());
        float diff = max_val - min_val;

        // 결과 로깅 (심플 스타일)
        RCLCPP_INFO(
            logger_,
            "[Calibration Result] Method: %s | Samples: %d\n"
            "  idx_13: %.3f\n"
            "  idx_14: %.3f\n"
            "  idx_15: %.3f",
            mtof_calib_cfg_.method.c_str(), calib_session_.sample_count,
            calib_session_.stats[0], calib_session_.stats[1], calib_session_.stats[2]
        );

        // 범위 검사
        bool out_of_range = false;
        for (int i = 0; i < 3; ++i) {
            if (calib_session_.stats[i] < min_th_arr.data[i] || calib_session_.stats[i] > max_th_arr.data[i]) {
                out_of_range = true;
                break;
            }
        }

        if (out_of_range) {
            RCLCPP_INFO(
                logger_,
                "[Calibration: FAIL_OUT_OF_RANGE]\n"
                "  idx_13: %.3f (min_th: %.3f, max_th: %.3f)\n"
                "  idx_14: %.3f (min_th: %.3f, max_th: %.3f)\n"
                "  idx_15: %.3f (min_th: %.3f, max_th: %.3f)",
                calib_session_.stats[0], min_th_arr.data[0], max_th_arr.data[0],
                calib_session_.stats[1], min_th_arr.data[1], max_th_arr.data[1],
                calib_session_.stats[2], min_th_arr.data[2], max_th_arr.data[2]
            );
            ret = MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE;
        }
        else if (diff > mtof_calib_cfg_.pass_diff_th) {
            RCLCPP_INFO(logger_, "[Calibration: FAIL_UNSTABLE_RANGE] Diff: %.4f (Th: %.4f)", diff, mtof_calib_cfg_.pass_diff_th);
            ret = MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE;
        }
        else {
            RCLCPP_INFO(logger_, "[Calibration: PASS] Side %s successfully calibrated.", (side == TOF_SIDE::LEFT ? "LEFT" : "RIGHT"));
            ret = MTOF_CALIB_RESULT::PASS;

            // 결과 데이터 저장
            int offset = (side == TOF_SIDE::LEFT) ? 0 : 3;
            for (int i = 0; i < 3; ++i) {
                mtof_calib_result_array_[offset + i] = calib_session_.stats[i];
            }
            calib_result.setCalibValue(side, calib_session_.stats[0], calib_session_.stats[1], calib_session_.stats[2]);
        }
        calib_session_.sample_count = 0;
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
uint8_t MultizoneTofCalibrator::makeMTofState(TOF_SIDE side, MTOF_CALIB_RESULT state)
{
    uint8_t value = 0;

    switch (state) {
        case MTOF_CALIB_RESULT::RUNNING:                value = 0x01; break;
        case MTOF_CALIB_RESULT::PASS:                   value = 0x02; break;
        case MTOF_CALIB_RESULT::FAIL_OUT_OF_RANGE:      value = 0x03; break;
        case MTOF_CALIB_RESULT::FAIL_UNSTABLE_RANGE:    value = 0x04; break;
        // case MTOF_CALIB_RESULT::FAIL_TIME_OUT:          value = 0x08; break;
        case MTOF_CALIB_RESULT::FAIL_DATA_NON_RENEWAL:  value = 0x08; break;
        default:                                        value = 0x00; break;
    }

    if (side == TOF_SIDE::RIGHT) {
        value = (value & 0x0F) << 4;  // Left 0x0?, Right 0x?0
    }

    return value;
}

/**
  * @brief Save tof self test calibration data in json format
  *
  * @param side enum Class, Left or Right
  * @param resultCode uint8_t, tof calibration result
  */
void MultizoneTofCalibrator::writeSelfTestCalibFile(TOF_SIDE side, MTOF_CALIB_RESULT resultCode)
{
    std::deque<std::string> buffer;
    std::string tof_calib_file_path = "/home/airbot/app_rw/log/MultiCalibration.json";

    if (!checkFileExist(tof_calib_file_path, buffer)){
        return;
    }

    json j;
    createJsonData(j, side, resultCode);

    writeDataFile(tof_calib_file_path, buffer, j);
}

/**
 * @brief Check file exist, and save exist data in buffer
 *
 * @param path Json file path
 * @param buffer exist data in file
 * @return true, File open success
 * @return false, Fail to open file or fail to create file
 */
bool MultizoneTofCalibrator::checkFileExist(std::string path, std::deque<std::string> &buffer)
{
    std::ifstream read_file(path);
    int max_lines = 10;

    if (!read_file.good()){
        RCLCPP_WARN(logger_, "There is no file in path = %s", path.c_str());
        // 파일 없을 시 새로 생성
        std::ofstream make_new_file(path);
        if (!make_new_file){
            RCLCPP_ERROR(logger_, "Fail to make new file in path = %s", path.c_str());
            read_file.close();
            return false;
        }
        make_new_file.close();
    }

    RCLCPP_INFO(logger_, "File open success.");

    std::string line;
    while (std::getline(read_file, line)){
        buffer.push_back(line);
        // 만약 파일에 쓰여져있는 내용이 10줄 이상이라면 오래된 내용 삭제
        if ((int)buffer.size() >= max_lines){
            buffer.pop_front();
        }
    }
    read_file.close();

    return true;
}

/**
 * @brief Create json data.
 * Format : {"time": "YY-MM-DD HH:MM:SS", "side": "Left"/"Right", "result": "PASS"/"FAILE", "failCode": "0xXX", "data": [x.xxx, x.xxx, x.xxx]}
 *
 * @param j nlohmann::ordered_json
 */
void MultizoneTofCalibrator::createJsonData(json &j, TOF_SIDE side, MTOF_CALIB_RESULT resultCode)
{
    json tof_data;

    time_t now = time(0);
    tm* ltm = localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(ltm, "%Y-%m-%d %H:%M:%S");
    std::string time_str = oss.str();

    j["time"] = time_str;
    j["side"] = ((side == TOF_SIDE::LEFT)? "Left" : "Right");
    if (resultCode == MTOF_CALIB_RESULT::PASS){
        j["result"] = "PASS";
    }
    else{
        j["result"] = "FAIL";
        j["failCode"] = resultCode;
    }

    if (side == TOF_SIDE::LEFT){
        for (uint8_t i=0; i<3; i++){
            j["data"].push_back(truncate_to_n(mtof_calib_result_array_[i], 3));
        }
    }
    else if (side == TOF_SIDE::RIGHT){
        for (uint8_t i=3; i<(uint8_t)mtof_calib_result_array_.size(); i++){
            j["data"].push_back(truncate_to_n(mtof_calib_result_array_[i], 3));
        }
    }
}

/**
 * @brief Save json data to file
 *
 * @param path json file path
 * @param buffer exist data in json file
 * @param output_data new data to wirte file
 */
void MultizoneTofCalibrator::writeDataFile(const std::string& path, const std::deque<std::string>& buffer, const json& output_data)
{
    std::ofstream output_file(path);
    if (!output_file.is_open()){
        RCLCPP_ERROR(logger_, "Fail to open file for writing, path = %s", path.c_str());
        return;
    }

    for (const auto &line : buffer){
        output_file << line << std::endl;
    }
    output_file << output_data << std::endl;
    output_file.flush(); // kernel buffer
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

/**
 * @brief 소수점 n자리 이하 버림
 *
 * @param value double, 원본 실수값
 * @param n int, n자리 이하 버림값
 * @return double, n자리 이하 버려진 실수값
 */
double MultizoneTofCalibrator::truncate_to_n(double value, int n)
{
    double scale = std::pow(10.0, n);
    return std::round(value * scale) / scale;
}
