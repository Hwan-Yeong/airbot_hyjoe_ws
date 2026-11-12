#include "kebi_follow_tracker/hungarian.hpp"

#include <limits>

namespace kebi_follow {
namespace tracker {

// 퍼텐셜을 쓰는 최단 증가 경로 헝가리안, O(rows^2 * cols). 1 부터 세는 내부 인덱스.
void AssignmentSolver::solve(const std::vector<double>& cost, int rows,
                             int cols, std::vector<int>& row_to_col) {
  row_to_col.assign(rows, -1);
  if (rows <= 0 || cols <= 0 || rows > cols) {
    return;
  }
  const double inf = std::numeric_limits<double>::infinity();
  u_.assign(rows + 1, 0.0);
  v_.assign(cols + 1, 0.0);
  p_.assign(cols + 1, 0);
  way_.assign(cols + 1, 0);
  for (int i = 1; i <= rows; ++i) {
    p_[0] = i;
    int j0 = 0;
    minv_.assign(cols + 1, inf);
    used_.assign(cols + 1, 0);
    do {
      used_[j0] = 1;
      const int i0 = p_[j0];
      double delta = inf;
      int j1 = 0;
      const double* row = cost.data() + static_cast<std::size_t>(i0 - 1) * cols;
      for (int j = 1; j <= cols; ++j) {
        if (used_[j]) {
          continue;
        }
        const double cur = row[j - 1] - u_[i0] - v_[j];
        if (cur < minv_[j]) {
          minv_[j] = cur;
          way_[j] = j0;
        }
        if (minv_[j] < delta) {
          delta = minv_[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= cols; ++j) {
        if (used_[j]) {
          u_[p_[j]] += delta;
          v_[j] -= delta;
        } else {
          minv_[j] -= delta;
        }
      }
      j0 = j1;
    } while (p_[j0] != 0);
    do {
      const int j1 = way_[j0];
      p_[j0] = p_[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  for (int j = 1; j <= cols; ++j) {
    const int i = p_[j];
    if (i > 0 &&
        cost[static_cast<std::size_t>(i - 1) * cols + (j - 1)] < kForbidden) {
      row_to_col[i - 1] = j - 1;
    }
  }
}

}  // namespace tracker
}  // namespace kebi_follow
