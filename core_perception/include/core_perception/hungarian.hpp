// 割り当て問題のハンガリアン法。行数が列数以下の前提で、行ごとの割り当て列を返す。

#ifndef CORE_PERCEPTION__HUNGARIAN_HPP_
#define CORE_PERCEPTION__HUNGARIAN_HPP_

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace core_perception
{
// cost[r][c] の総和が最小になるよう、各行に別々の列を割り当てる
inline std::vector<std::size_t> solveAssignment(const std::vector<std::vector<double>> & cost)
{
  const std::size_t rows = cost.size();
  if (rows == 0U) {
    return {};
  }
  const std::size_t columns = cost.front().size();
  constexpr double kInfinity = std::numeric_limits<double>::max();
  std::vector<double> row_potential(rows + 1U, 0.0);
  std::vector<double> column_potential(columns + 1U, 0.0);
  // column_row[c] は列 c に割り当てた行（1 始まり、0 は未割り当て）
  std::vector<std::size_t> column_row(columns + 1U, 0U);
  std::vector<std::size_t> previous(columns + 1U, 0U);
  for (std::size_t r = 1U; r <= rows; ++r) {
    column_row[0] = r;
    std::size_t current = 0U;
    std::vector<double> reduced(columns + 1U, kInfinity);
    std::vector<bool> used(columns + 1U, false);
    do {
      used[current] = true;
      const std::size_t row = column_row[current];
      double delta = kInfinity;
      std::size_t next = 0U;
      for (std::size_t c = 1U; c <= columns; ++c) {
        if (used[c]) {
          continue;
        }
        const double candidate =
          cost[row - 1U][c - 1U] - row_potential[row] - column_potential[c];
        if (candidate < reduced[c]) {
          reduced[c] = candidate;
          previous[c] = current;
        }
        if (reduced[c] < delta) {
          delta = reduced[c];
          next = c;
        }
      }
      for (std::size_t c = 0U; c <= columns; ++c) {
        if (used[c]) {
          row_potential[column_row[c]] += delta;
          column_potential[c] -= delta;
        } else {
          reduced[c] -= delta;
        }
      }
      current = next;
    } while (column_row[current] != 0U);
    do {
      const std::size_t before = previous[current];
      column_row[current] = column_row[before];
      current = before;
    } while (current != 0U);
  }
  std::vector<std::size_t> assignment(rows, 0U);
  for (std::size_t c = 1U; c <= columns; ++c) {
    if (column_row[c] != 0U) {
      assignment[column_row[c] - 1U] = c - 1U;
    }
  }
  return assignment;
}
}  // namespace core_perception

#endif  // CORE_PERCEPTION__HUNGARIAN_HPP_
