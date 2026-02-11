#pragma once

#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "utils/common_struct.hpp"

namespace sensor_manager {

class TofUtils {
 public:
  TofUtils();
  ~TofUtils();

  /**
   * @brief tan calculation function used for ToF coordinate calculation.
   *
   * @param[in] sub_cell_idx_array Target cell index of 8x8 Multi ToF.
   * @param[in] fov Multi ToF FOV [rad].
   * @param[in] y_tan_out Updated y-axis tan array.
   * @param[in] z_tan_out Updated z-axis tan array.
   * @param[in] logger Logger passed for logging.
   */
  void UpdateSubCellIndexArray(const std::vector<int>& sub_cell_idx_array,
                               double fov, std::vector<double>& y_tan_out,
                               std::vector<double>& z_tan_out,
                               rclcpp::Logger logger);

 private:
};

}  // namespace sensor_manager


