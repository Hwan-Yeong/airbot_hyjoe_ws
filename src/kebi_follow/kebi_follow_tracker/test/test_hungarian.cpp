// 최소 비용 할당 시험 (전수 탐색과 비교)
#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <vector>

#include "kebi_follow_tracker/hungarian.hpp"

namespace kt = kebi_follow::tracker;

namespace {
constexpr double kF = kt::AssignmentSolver::kForbidden;

double totalCost(const std::vector<double>& c, int cols,
                 const std::vector<int>& a) {
  double s = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i] >= 0) {
      s += c[i * cols + a[i]];
    }
  }
  return s;
}

// 모든 행이 서로 다른 열을 갖는 배정 중 최소 비용 (금지 짝은 쓰지 않는다), 깊이 우선 전수 탐색
void search(const std::vector<double>& c, int rows, int cols, int row,
            double acc, std::vector<char>& used, double& best) {
  if (row == rows) {
    best = std::min(best, acc);
    return;
  }
  for (int j = 0; j < cols; ++j) {
    const double v = c[row * cols + j];
    if (used[j] || v >= kF) {
      continue;
    }
    used[j] = 1;
    search(c, rows, cols, row + 1, acc + v, used, best);
    used[j] = 0;
  }
}

double bruteForce(const std::vector<double>& c, int rows, int cols) {
  std::vector<char> used(cols, 0);
  double best = 1e300;
  search(c, rows, cols, 0, 0.0, used, best);
  return best;
}
}  // namespace

TEST(Hungarian, SquareExample) {
  const std::vector<double> c = {4, 1, 3, 2, 0, 5, 3, 2, 2};
  kt::AssignmentSolver s;
  std::vector<int> a;
  s.solve(c, 3, 3, a);
  EXPECT_DOUBLE_EQ(totalCost(c, 3, a), 5.0);  // (0,1) (1,0) (2,2)
  EXPECT_EQ(a[0], 1);
  EXPECT_EQ(a[1], 0);
  EXPECT_EQ(a[2], 2);
}

TEST(Hungarian, RectangularAndForbidden) {
  // 행 1 은 열 3 만 허용
  const std::vector<double> c = {1, 9, 9, 9, kF, kF, kF, 2};
  kt::AssignmentSolver s;
  std::vector<int> a;
  s.solve(c, 2, 4, a);
  EXPECT_EQ(a[0], 0);
  EXPECT_EQ(a[1], 3);
}

TEST(Hungarian, AllForbiddenRowIsUnassigned) {
  const std::vector<double> c = {kF, kF, kF, 1, 2, 3};
  kt::AssignmentSolver s;
  std::vector<int> a;
  s.solve(c, 2, 3, a);
  EXPECT_EQ(a[0], -1);
  EXPECT_EQ(a[1], 0);
}

TEST(Hungarian, NegativeCostsAllowed) {
  const std::vector<double> c = {-10, -2, -9, -8};
  kt::AssignmentSolver s;
  std::vector<int> a;
  s.solve(c, 2, 2, a);
  EXPECT_DOUBLE_EQ(totalCost(c, 2, a), -18.0);
}

TEST(Hungarian, MatchesBruteForceOnRandomMatrices) {
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> cost(-12.0, 10.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  kt::AssignmentSolver s;
  std::vector<int> a;
  for (int trial = 0; trial < 300; ++trial) {
    const int rows = 1 + trial % 4;
    const int cols = rows + trial % 3;
    std::vector<double> c(rows * cols);
    for (auto& v : c) {
      v = uni(rng) < 0.2 ? kF : cost(rng);
    }
    // "측정 없음" 열을 붙여 항상 완전 배정이 있게 한다 (추적기와 같은 구성)
    const int width = cols + rows;
    std::vector<double> full(rows * width, kF);
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        full[i * width + j] = c[i * cols + j];
      }
      full[i * width + cols + i] = 5.0;
    }
    s.solve(full, rows, width, a);
    EXPECT_NEAR(totalCost(full, width, a), bruteForce(full, rows, width), 1e-9)
        << "trial " << trial;
  }
}
