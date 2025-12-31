#ifndef __TOF_UTILS__
#define __TOF_UTILS__

#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "utils/common_struct.hpp"

namespace sensor_manager {

class TofUtils
{
public:
    TofUtils();
    ~TofUtils();

    /**
     * @brief ToF 좌표 계산에 사용되는 tan 계산 함수
     * @param[in] sub_cell_idx_array 8x8 Multi ToF의  타겟 cell index
     * @param[in] fov Multi ToF FOV [rad]
     * @param[in] y_tan_out 업데이트 될 y축 tan array
     * @param[in] z_tan_out 업데이트 될 z축 tan array
     * @param[in] logger Logging을 위한 logger 전달
     * @return 합쳐진 결과 sensor_msgs::msg::PointCloud2 메시지
     */
    void updateSubCellIndexArray(
        const std::vector<int>& sub_cell_idx_array,
        double fov,
        std::vector<double>& y_tan_out,
        std::vector<double>& z_tan_out,
        rclcpp::Logger logger
    );

private:
};

} // namespace sensor_manager

#endif // TOF_UTILS