#include "utils/tof_utils.hpp"

#include <sstream>

namespace sensor_manager {

TofUtils::TofUtils() {}
TofUtils::~TofUtils() {}

void TofUtils::UpdateSubCellIndexArray(
    const std::vector<int>& sub_cell_idx_array, double fov,
    std::vector<double>& y_tan_out, std::vector<double>& z_tan_out,
    rclcpp::Logger logger) {
  // =========== Create masked 8x8 matrix based on user input ===========
  bool masked_mat[8][8] = {false};

  // =========== Input sub cell index logging ===========
  std::ostringstream oss;
  oss << "\n==== Input sub_cell_idx_array ====\n";
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      int idx = r * 4 + c;
      if (idx < static_cast<int>(sub_cell_idx_array.size())) {
        oss << std::setw(2) << std::setfill('0') << sub_cell_idx_array[idx]
            << " ";
      } else {
        oss << "-- ";
      }
    }
    oss << "\n";
  }
  RCLCPP_INFO(logger, "%s", oss.str().c_str());
  oss.str("");
  oss.clear();

  // =========== Masked matrix setting ===========
  for (int idx : sub_cell_idx_array) {
    if (idx >= 0 && idx < 64) {
      int row = idx / 8;
      int col = idx % 8;
      masked_mat[row][col] = true;
    } else {
      RCLCPP_WARN(logger,
                  "Each value in sub_cell_idx_array must be between 0 and 63. "
                  "Invalid input idx: %d",
                  idx);
    }
  }

  // =========== Masked matrix logging ===========
  oss << "\n==== Masked 8x8 Matrix ====\n";
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      oss << (masked_mat[i][j] ? "1 " : "0 ");
    }
    oss << "\n";
  }
  RCLCPP_INFO(logger, "%s", oss.str().c_str());
  oss.str("");
  oss.clear();

  // =========== Calculate y_tan, z_tan based on true cells ===========
  y_tan_out.clear();
  z_tan_out.clear();

  std::vector<std::pair<int, int>> true_indices;
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      if (masked_mat[i][j]) {
        true_indices.emplace_back(i, j);
      }
    }
  }

  if (true_indices.size() != 16) {
    RCLCPP_INFO(logger, "Expected 16 true values in mask, but got %zu",
                true_indices.size());
  }

  for (const auto& [i, j] : true_indices) {
    double y = std::tan(fov * ((7 - 2 * j) / 16.0));
    double z = std::tan(fov * ((7 - 2 * i) / 16.0));
    y_tan_out.emplace_back(y);
    z_tan_out.emplace_back(z);
  }
}

}  // namespace sensor_manager
