#include "core/decision_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace dev_board {
namespace {

// 一個細格裡的有效點
struct Cell {
    int valid = 0;
    double sum_h = 0.0;
};

// 偶數個時取中間兩個的平均（§6.1）
double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    return values.size() % 2 == 1 ? values[mid] : (values[mid - 1] + values[mid]) / 2.0;
}

}  // namespace

DecisionCore::DecisionCore(const Config& config) : config_(config) {
    const Intrinsics& k = config_.camera.depth;
    for (int u = 0; u < k.width; ++u) {
        ray_x_.push_back((u + 0.5 - k.cx) / k.fx);
    }
    for (int v = 0; v < k.height; ++v) {
        ray_y_.push_back((v + 0.5 - k.cy) / k.fy);
    }
}

Results DecisionCore::on_attitude(const Attitude& attitude, Now) {
    attitude_ = attitude;
    return {};
}

Results DecisionCore::on_position(const Position& position, Now) {
    position_ = position;
    return {};
}

Results DecisionCore::on_depth(const DepthFrame& frame, Now) {
    const Intrinsics& k = config_.camera.depth;
    if (frame.width != k.width || frame.height != k.height) {
        throw std::invalid_argument("DecisionCore::on_depth: frame size does not match camera.depth");
    }
    if (frame.depth_mm.size() != static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height)) {
        throw std::invalid_argument("DecisionCore::on_depth: depth_mm is not width x height");
    }
    depth_seen_ = true;
    const DataState now_state = state();
    if (now_state != DataState::ok) {
        return {Height{std::nullopt, now_state}};
    }
    return {Height{agl(frame, *attitude_), DataState::ok}};
}

Results DecisionCore::on_tick(Now) {
    const DataState now_state = state();
    if (now_state != DataState::ok) {
        return {Height{std::nullopt, now_state}};
    }
    return {};
}

// 依序 no_depth → no_px4 → ok（§9.2 heartbeat）
DataState DecisionCore::state() const {
    if (!depth_seen_) {
        return DataState::no_depth;
    }
    if (!attitude_ || !position_) {
        return DataState::no_px4;
    }
    return DataState::ok;
}

// §5 第 3～5 步把每個有效點放進正下方 Z 的細格，再照 §6.1 取中位數
std::optional<double> DecisionCore::agl(const DepthFrame& frame, const Attitude& attitude) const {
    const double cr = std::cos(attitude.roll);
    const double sr = std::sin(attitude.roll);
    const double cp = std::cos(attitude.pitch);
    const double sp = std::sin(attitude.pitch);
    // 機身 FRD → 水平座標系：R = Ry(pitch)·Rx(roll)
    const auto rotate = [&](double x, double y, double z, double& out_x, double& out_y, double& out_z) {
        const double y1 = cr * y - sr * z;
        const double z1 = sr * y + cr * z;
        out_x = cp * x + sp * z1;
        out_y = y1;
        out_z = cp * z1 - sp * x;
    };
    const std::array<double, 3>& t = config_.camera.t_cam;
    double ox = 0.0, oy = 0.0, oz = 0.0;
    rotate(t[0], t[1], t[2], ox, oy, oz);

    const double s = config_.grid.cell_size_m;
    const int cells = config_.zone.width_cells;
    const int half = cells / 2;
    std::vector<Cell> zone(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells));
    const Intrinsics& k = config_.camera.depth;
    for (int v = 0; v < k.height; ++v) {
        for (int u = 0; u < k.width; ++u) {
            const std::uint16_t mm = frame.depth_mm[static_cast<std::size_t>(v) * k.width + u];
            if (mm == 0) {
                continue;
            }
            const double z = mm / 1000.0;
            // 相機光學座標 → 機身 FRD：(x_b, y_b, z_b) = (−y_c, x_c, z_c)
            double qx = 0.0, qy = 0.0, qz = 0.0;
            rotate(-z * ray_y_[v], z * ray_x_[u], z, qx, qy, qz);
            const int ix = static_cast<int>(std::floor((qx + ox) / s)) + half;
            const int iy = static_cast<int>(std::floor((qy + oy) / s)) + half;
            if (ix < 0 || ix >= cells || iy < 0 || iy >= cells) {
                continue;
            }
            Cell& cell = zone[static_cast<std::size_t>(ix) * cells + iy];
            ++cell.valid;
            cell.sum_h += -qz;  // h 不加 o.z，從相機量起
        }
    }

    std::vector<double> heights;
    for (const Cell& cell : zone) {
        if (cell.valid > 0) {
            heights.push_back(cell.sum_h / cell.valid);
        }
    }
    if (heights.size() * 2 < zone.size()) {
        return std::nullopt;
    }
    return -median(heights);
}

}  // namespace dev_board
