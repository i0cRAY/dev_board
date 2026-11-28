#pragma once

#include <optional>
#include <vector>

#include "core/config.h"
#include "core/decision_core.h"

// 判斷核心內部用：細格和 evaluate（dev_board_spec.md §5 第 6、7 步、§6.2）。不是公開介面
namespace dev_board::terrain {

// 一個細格的點，x、y 是水平座標系（公尺），h 往上為正
struct Cell {
    int valid = 0;
    int invalid = 0;
    // 有效點的 Moments（§5 第 6 步）
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_h = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double sum_yy = 0.0;
    double sum_xh = 0.0;
    double sum_yh = 0.0;
    double sum_hh = 0.0;
    std::vector<double> heights;  // 有效點的 h，finish() 算完 h20 就清掉
    double h20 = 0.0;             // 有效點 h 的第 low_percentile 百分位（§5 第 6 步），finish() 之後才有
};

// 以水平座標系原點為中心、每邊 2·half 格的細格。格子 (ix, iy) 是 ix·s ≤ x < (ix + 1)·s、
// iy·s ≤ y < (iy + 1)·s，−half ≤ ix, iy < half
class Grid {
public:
    Grid(int half, double cell_size);

    int half() const { return half_; }
    double cell_size() const { return cell_size_; }
    const Cell& at(int ix, int iy) const;

    // 落在範圍外（含 NaN、inf）的點不收
    void add_valid(double x, double y, double h);
    void add_invalid(double x, double y);
    // 點都放完之後呼叫一次：每格算 h20，推薦和 drop_check 共用（§5 第 6 步）
    void finish(double low_percentile);

private:
    Cell* find(double x, double y);

    int half_;
    double cell_size_;
    std::vector<Cell> cells_;
};

// 偶數個時取中間兩個的平均（§6.1、§6.2）。不整個排序：一幀有上萬個點
double median(std::vector<double> values);

// §6.1：正下方 Z 裡有高度的細格（有效點 ≥ 1）的平均高度取中位數，取負號；不到一半時量不到
std::optional<double> agl(const Grid& grid, int zone_cells);

// 正下方的 Z（中心在原點）和它的外圈。grid 要涵蓋 Z、外圈和外圈外面一格。
// 回傳空的代表合格（§6.2）；0 個點的格子算 UNKNOWN（§7）
std::optional<CheckReason> evaluate(const Grid& grid, const Config& config);

}  // namespace dev_board::terrain
