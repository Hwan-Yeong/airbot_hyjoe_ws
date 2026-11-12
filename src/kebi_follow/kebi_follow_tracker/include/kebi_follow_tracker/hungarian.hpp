// 최소 비용 할당 (헝가리안, 직사각 행렬 rows <= cols). 버퍼를 재사용해 매 호출 할당이 없다.
#pragma once

#include <vector>

namespace kebi_follow {
namespace tracker {

class AssignmentSolver {
 public:
  // 이 값 이상인 비용은 금지된 짝으로 본다
  static constexpr double kForbidden = 1e9;

  // cost: 행 우선 rows x cols (rows <= cols). row_to_col[i] = 열 또는 -1 (금지 짝만 남은 경우)
  void solve(const std::vector<double>& cost, int rows, int cols,
             std::vector<int>& row_to_col);

 private:
  std::vector<double> u_;
  std::vector<double> v_;
  std::vector<double> minv_;
  std::vector<int> p_;
  std::vector<int> way_;
  std::vector<char> used_;
};

}  // namespace tracker
}  // namespace kebi_follow
