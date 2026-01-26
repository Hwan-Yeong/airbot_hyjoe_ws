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

    /**
     * @brief Calibration 성공 시, 보정 결과 (6개 데이터 array) 전달 함수
     * 
     * @return {L_13, L_14, L_15, R_13, R_14, R_15} float 형 array
     * @note Left,Right 모두 완료되었을 때만 호출해야 함
     */
    const std::array<float, 6>& getResultArray() const { return mtof_calib_result_array_; }

    /**
     * @brief L/R Calibration 완료 상태 세팅 함수
     * 
     * @param[in] side Left or Right
     * @param[in] is_done 완료 여부 (true: 완료, false: 미완료)
     */
    void setCalibrationDone(TOF_SIDE side, bool is_done);

    /**
     * @brief L/R Calibration 완료 상태 전달 함수
     * 
     * @param[in] side Left or Right
     * @return 완료 여부 (true: 완료, false: 미완료)
     */
    bool isCalibrationDone(TOF_SIDE side) const;

    /**
     * @brief Calibration 동작의 상태 세팅 함수
     * 
     * @param[in] state (INACTIVE: 미동작, ACTIVE_LEFT: 왼쪽 진행 중, ACTIVE_RIGHT: 오른쪽 진행 중)
     */
    void setCalibrationState(MTOF_CALIB_STATE state);

    /**
     * @brief Calibration 동작의 상태 전달 함수
     */
    MTOF_CALIB_STATE getCalibrationState() const;

    /**
     * @brief Calibration 좌표변환에 사용할 converter 세팅 함수 (L/R Converter 포인터 전달받아 세팅)
     */
    void setConverter(CloudConverterPtr converter);

    /**
     * @brief Calibration 결과에 따라 약속된 프로토콜로 변환한 상태 정보 반환 함수
     * 
     * @param[in] side Left or Right
     * @param[in] state Calibration 상태
     * @return HEX로 저장한 L/R 상태 정보를 int 형으로 반환
     */
    uint8_t makeMTofState(TOF_SIDE side, MTOF_CALIB_RESULT state);

    /**
     * @brief 클래스 멤버 변수 초기화
     */
    void reset();

    /**
     * @brief Calibration update 함수
     * 
     * @param[in] calib_result Calibration 결과 데이터 포인터, 캘리브레이션 결과 데이터를 직접 업데이트한다.
     * @param[in] msg tof data
     * @param[in] side Left or Right
     * @return Calibration 결과 상태 반환 (running / pass / fail)
     */
    MTOF_CALIB_RESULT update(MTOF_CALIB_DATA& calib_result,
      const robot_custom_msgs::msg::TofData::SharedPtr msg,
      TOF_SIDE side);

  private:
    /**
     * @brief update 함수 내부에서 동작하는 Calibration 메인 로직
     * 
     * @param[in] calib_result Calibration 결과 데이터 포인터, 캘리브레이션 결과 데이터를 직접 업데이트한다.
     * @param[in] msg tof data
     * @param[in] side Left or Right
     * @return Calibration 결과 상태 반환 (running / pass / fail)
     */
    MTOF_CALIB_RESULT processCalibration(MTOF_CALIB_DATA& calib_result,
      const robot_custom_msgs::msg::TofData::SharedPtr msg,
      TOF_SIDE side);

    /**
     * @brief json 포맷으로 Calibration 데이터를 저장하는 함수
     *
     * @param side Left or Right
     * @param resultCode tof calibration result
     */
    void writeSelfTestCalibFile(TOF_SIDE side, MTOF_CALIB_RESULT resultCode);

    /**
     * @brief 파일이 있는지 확인하고 파일에 있는 데이터를 최대 10줄까지만 buffer에 저장하는 함수
     *
     * @param path Json file path
     * @param buffer exist data in file
     * @return true, File open success
     * @return false, Fail to open file or fail to create file
     */
    bool checkFileExist(std::string path, std::deque<std::string> &buffer);

    /**
     * @brief json data 생성 함수
     * Format : {"time": "YY-MM-DD HH:MM:SS", "side": "Left"/"Right", "result": "PASS"/"FAILE", "failCode": "0xXX", "data": [x.xxx, x.xxx, x.xxx]}
     *
     * @param j nlohmann::ordered_json
     */
    void createJsonData(json &j, TOF_SIDE side, MTOF_CALIB_RESULT resultCode);

    /**
     * @brief json data 를 file로 저장하는 함수
     *
     * @param path json file path
     * @param buffer exist data in json file
     * @param output_data new data to wirte file
     */
    void writeDataFile(const std::string& path, const std::deque<std::string>& buffer, const json& output_data);

    /**
     * @brief 소수점 n자리 이하 버림
     *
     * @param value double, 원본 실수값
     * @param n int, n자리 이하 버림값
     * @return double, n자리 이하 버려진 실수값
     */
    double truncate_to_n(double value, int n);

    rclcpp::Logger logger_;
    tTofCalibrationParam mtof_calib_cfg_;
    CloudConverterPtr converter_ = nullptr;

    MTOF_CALIB_STATE calib_state_ = MTOF_CALIB_STATE::INACTIVE;
    bool is_left_done_ = false;
    bool is_right_done_ = false;
    tMToFCalibSession calib_session_; //샘플링cnt, raw데이터, 변환데이터, 결과데이터 등 주요 변수 저장 멤버변수
    std::array<float, 6> mtof_calib_result_array_{};
};

} // namespace sensor_manager
