#include "core/terrain.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "core/angle.h"

namespace dev_board::terrain {

Grid::Grid(int half, double cell_size)
    : half_(half), cell_size_(cell_size), cells_(static_cast<std::size_t>(2 * half) * static_cast<std::size_t>(2 * half)) {}

const Cell& Grid::at(int ix, int iy) const {
    return cells_[static_cast<std::size_t>(ix + half_) * static_cast<std::size_t>(2 * half_) +
                  static_cast<std::size_t>(iy + half_)];
}

Cell* Grid::find(double x, double y) {
    // 先用浮點數比範圍再轉整數：很遠的點或 NaN 直接轉 int 是未定義行為
    const double fx = std::floor(x / cell_size_);
    const double fy = std::floor(y / cell_size_);
    if (!(fx >= -half_ && fx < half_ && fy >= -half_ && fy < half_)) {
        return nullptr;
    }
    return const_cast<Cell*>(&at(static_cast<int>(fx), static_cast<int>(fy)));
}

void Grid::add_valid(double x, double y, double h) {
    Cell* cell = find(x, y);
    if (cell == nullptr) {
        return;
    }
    ++cell->valid;
    cell->sum_x += x;
    cell->sum_y += y;
    cell->sum_h += h;
    cell->sum_xx += x * x;
    cell->sum_xy += x * y;
    cell->sum_yy += y * y;
    cell->sum_xh += x * h;
    cell->sum_yh += y * h;
    cell->sum_hh += h * h;
    cell->heights.push_back(h);
}

void Grid::add_invalid(double x, double y) {
    Cell* cell = find(x, y);
    if (cell != nullptr) {
        ++cell->invalid;
    }
}

void Grid::finish(double low_percentile) {
    for (Cell& cell : cells_) {
        std::vector<double>& heights = cell.heights;
        if (heights.empty()) {
            continue;
        }
        // 由低到高第 max(ceil(p·n) − 1, 0) 個。設定檔改成其他百分位時，例如 0.28 × 25 算出來是
        // 7.000000000000001，不減一點會多算一個
        const double rank = std::ceil(low_percentile * static_cast<double>(heights.size()) - 1e-9) - 1.0;
        const auto index = static_cast<std::ptrdiff_t>(std::max(rank, 0.0));
        std::nth_element(heights.begin(), heights.begin() + index, heights.end());
        cell.h20 = heights[static_cast<std::size_t>(index)];
        heights = {};
    }
}

double median(std::vector<double> values) {
    const std::size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
    if (values.size() % 2 == 1) {
        return values[mid];
    }
    return (*std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid)) + values[mid]) / 2.0;
}

namespace {

// 有效點 < 3 或無效比例 > 50%（§6.2 第 1 步）。0 個點的格子也是
bool unknown(const Cell& cell, const Config::Evaluate& e) {
    return cell.valid < e.min_points_per_cell ||
           cell.invalid > e.max_cell_invalid_ratio * (cell.valid + cell.invalid);
}

double mean_h(const Cell& cell) { return cell.sum_h / cell.valid; }

// h = a·x + b·y + c，另外帶擬合用的點到平面的殘差 RMS
struct Plane {
    double a;
    double b;
    double c;
    double rms;
    double at(double x, double y) const { return a * x + b * y + c; }
};

// 格子座標 (ix, iy) 的一組格子
using Cells = std::vector<std::pair<int, int>>;

// 用這些格子所有點的 Moments 做最小平方擬合。點共線時沒有解
std::optional<Plane> fit(const Grid& grid, const Cells& cells) {
    double n = 0, sx = 0, sy = 0, sh = 0, sxx = 0, sxy = 0, syy = 0, sxh = 0, syh = 0, shh = 0;
    for (const auto& [ix, iy] : cells) {
        const Cell& c = grid.at(ix, iy);
        n += c.valid;
        sx += c.sum_x;
        sy += c.sum_y;
        sh += c.sum_h;
        sxx += c.sum_xx;
        sxy += c.sum_xy;
        syy += c.sum_yy;
        sxh += c.sum_xh;
        syh += c.sum_yh;
        shh += c.sum_hh;
    }
    // 先減掉平均再解，數值比較穩
    const double mx = sx / n;
    const double my = sy / n;
    const double mh = sh / n;
    const double cxx = sxx - n * mx * mx;
    const double cxy = sxy - n * mx * my;
    const double cyy = syy - n * my * my;
    const double cxh = sxh - n * mx * mh;
    const double cyh = syh - n * my * mh;
    const double chh = shh - n * mh * mh;
    const double det = cxx * cyy - cxy * cxy;
    if (!(det > 1e-9 * cxx * cyy)) {
        return std::nullopt;
    }
    const double a = (cxh * cyy - cyh * cxy) / det;
    const double b = (cyh * cxx - cxh * cxy) / det;
    // 殘差平方和 = Σ(h − a·x − b·y − c)²，用減掉平均後的 Moments 展開
    const double residual = chh - 2.0 * a * cxh - 2.0 * b * cyh + a * a * cxx + 2.0 * a * b * cxy + b * b * cyy;
    return Plane{a, b, mh - a * mx - b * my, std::sqrt(std::max(residual, 0.0) / n)};
}

// mask（每邊 2·half 格，從 −half 開始）裡全是 true 的最大正方形邊長（格數）
int largest_square(const std::vector<bool>& mask, int half) {
    const int side = 2 * half;
    std::vector<int> size(mask.size(), 0);
    int best = 0;
    for (int i = 0; i < side; ++i) {
        for (int j = 0; j < side; ++j) {
            const std::size_t at = static_cast<std::size_t>(i * side + j);
            if (!mask[at]) {
                continue;
            }
            int s = 1;
            if (i > 0 && j > 0) {
                s = 1 + std::min({size[at - side], size[at - 1], size[at - side - 1]});
            }
            size[at] = s;
            best = std::max(best, s);
        }
    }
    return best;
}

}  // namespace

std::optional<double> agl(const Grid& grid, int zone_cells) {
    const int zh = zone_cells / 2;
    std::vector<double> heights;
    for (int ix = -zh; ix < zh; ++ix) {
        for (int iy = -zh; iy < zh; ++iy) {
            const Cell& cell = grid.at(ix, iy);
            if (cell.valid > 0) {
                heights.push_back(mean_h(cell));
            }
        }
    }
    if (heights.size() * 2 < static_cast<std::size_t>(zone_cells * zone_cells)) {
        return std::nullopt;
    }
    return -median(heights);
}

std::optional<CheckReason> evaluate(const Grid& grid, const Config& config) {
    const Config::Evaluate& e = config.evaluate;
    const int zh = config.zone.width_cells / 2;
    const int zone_cells = config.zone.width_cells * config.zone.width_cells;

    int known = 0;
    for (int ix = -zh; ix < zh; ++ix) {
        for (int iy = -zh; iy < zh; ++iy) {
            if (!unknown(grid.at(ix, iy), e)) {
                ++known;
            }
        }
    }
    if (known * 2 < zone_cells) {
        return CheckReason::invalid_depth;
    }

    // 第 2 步：基準面
    const double d = e.hole_depth_m;
    std::vector<double> heights;
    for (int ix = -zh; ix < zh; ++ix) {
        for (int iy = -zh; iy < zh; ++iy) {
            if (!unknown(grid.at(ix, iy), e)) {
                heights.push_back(mean_h(grid.at(ix, iy)));
            }
        }
    }
    const double h_med = median(heights);
    Cells inliers;
    for (int ix = -zh; ix < zh; ++ix) {
        for (int iy = -zh; iy < zh; ++iy) {
            const Cell& c = grid.at(ix, iy);
            if (!unknown(c, e) && std::abs(mean_h(c) - h_med) < d) {
                inliers.emplace_back(ix, iy);
            }
        }
    }
    if (static_cast<int>(inliers.size()) * 2 < zone_cells) {
        return CheckReason::hole;
    }
    std::optional<Plane> plane = fit(grid, inliers);
    if (!plane) {
        return CheckReason::hole;
    }
    // 排除坑格後重新擬合：平均低不到 D 的坑格會把平面往下拉，其他坑格就更不像坑
    const double s = grid.cell_size();
    for (int round = 0; round < e.refit_rounds; ++round) {
        Cells kept;
        for (const auto& [ix, iy] : inliers) {
            if (plane->at((ix + 0.5) * s, (iy + 0.5) * s) - grid.at(ix, iy).h20 < d) {
                kept.emplace_back(ix, iy);
            }
        }
        inliers = kept;
        if (static_cast<int>(inliers.size()) * 2 < zone_cells) {
            return CheckReason::hole;
        }
        plane = fit(grid, inliers);
        if (!plane) {
            return CheckReason::hole;
        }
    }

    // 第 3 步：坡度
    if (std::atan(std::hypot(plane->a, plane->b)) > e.max_slope_deg * kDegree) {
        return CheckReason::slope;
    }

    // 第 4 步：粗糙度，扣掉降採樣後的深度雜訊
    const double noise = e.depth_noise_m;
    if (std::sqrt(std::max(0.0, plane->rms * plane->rms - noise * noise)) > e.max_roughness_m) {
        return CheckReason::rough;
    }

    // 第 5 步：Z、外圈和外圈外面一格的 LOW/UNKNOWN、HIGH（細格中心的基準面高度）
    const int g = grid.half();
    const int zr = zh + config.zone.ring_cells;
    std::vector<char> low(static_cast<std::size_t>(4 * g * g), 0);
    const auto low_at = [&](int ix, int iy) -> char& {
        return low[static_cast<std::size_t>((ix + g) * 2 * g + iy + g)];
    };
    const auto is_low = [&](int ix, int iy) { return low_at(ix, iy) != 0; };
    bool high_in_zone = false;
    bool high_in_ring = false;
    for (int ix = -g; ix < g; ++ix) {
        for (int iy = -g; iy < g; ++iy) {
            const Cell& c = grid.at(ix, iy);
            if (unknown(c, e)) {
                low_at(ix, iy) = 1;
                continue;
            }
            const double base = plane->at((ix + 0.5) * s, (iy + 0.5) * s);
            low_at(ix, iy) = base - c.h20 >= d;
            const bool in_zone = ix >= -zh && ix < zh && iy >= -zh && iy < zh;
            const bool in_ring = !in_zone && ix >= -zr && ix < zr && iy >= -zr && iy < zr;
            if (mean_h(c) - base >= d) {
                high_in_zone = high_in_zone || in_zone;
                high_in_ring = high_in_ring || in_ring;
            }
        }
    }

    // 規則 A：Z∪外圈的 LOW/UNKNOWN 放得下箱子短邊的正方形。正方形本身是 4 連通的，所以不用先找連通區
    std::vector<bool> low_inside(static_cast<std::size_t>(4 * zr * zr), false);
    for (int ix = -zr; ix < zr; ++ix) {
        for (int iy = -zr; iy < zr; ++iy) {
            low_inside[static_cast<std::size_t>((ix + zr) * 2 * zr + iy + zr)] = is_low(ix, iy);
        }
    }
    if (largest_square(low_inside, zr) >= ceil_cells(config.zone.box_short_side_m, s)) {
        return CheckReason::hole;
    }
    // 規則 B：外圈最外面一圈的 LOW/UNKNOWN，緊貼的外面那格也是 LOW/UNKNOWN（含 0 個點）
    for (int i = -zr; i < zr; ++i) {
        const bool out = (is_low(-zr, i) && is_low(-zr - 1, i)) || (is_low(zr - 1, i) && is_low(zr, i)) ||
                         (is_low(i, -zr) && is_low(i, -zr - 1)) || (is_low(i, zr - 1) && is_low(i, zr));
        if (out) {
            return CheckReason::hole;
        }
    }
    // 規則 C：外圈有凸起
    if (high_in_ring) {
        return CheckReason::hole;
    }
    // 規則 D：Z 裡有凸起
    if (high_in_zone) {
        return CheckReason::rough;
    }

    // 第 7 步：Z 的無效比例。第 1 步已確定 Z 有一半以上的格子有 ≥ 3 個有效點，分母不會是 0
    int valid = 0;
    int invalid = 0;
    for (int ix = -zh; ix < zh; ++ix) {
        for (int iy = -zh; iy < zh; ++iy) {
            valid += grid.at(ix, iy).valid;
            invalid += grid.at(ix, iy).invalid;
        }
    }
    if (invalid > e.max_zone_invalid_ratio * (valid + invalid)) {
        return CheckReason::invalid_depth;
    }
    return std::nullopt;
}

}  // namespace dev_board::terrain
