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

// 依 sim time 排好的 PX4 緩衝，加入時丟掉比最新一筆早超過 keep_ms 的
template <class Sample>
void remember(std::vector<Sample>& buffer, const Sample& sample, int keep_ms) {
    const auto later = [](const Sample& a, const Sample& b) { return a.time_boot_ms < b.time_boot_ms; };
    buffer.insert(std::upper_bound(buffer.begin(), buffer.end(), sample, later), sample);
    const std::int64_t oldest = static_cast<std::int64_t>(buffer.back().time_boot_ms) - keep_ms;
    buffer.erase(buffer.begin(),
                 std::find_if(buffer.begin(), buffer.end(), [oldest](const Sample& s) {
                     return static_cast<std::int64_t>(s.time_boot_ms) >= oldest;
                 }));
}

// 夾住時間 t 的前後兩筆，t = before + a·(after − before)。剛好相等時 before、after 是同一筆
template <class Sample>
struct Bracket {
    const Sample* before;
    const Sample* after;
    double a;
};

template <class Sample>
std::optional<Bracket<Sample>> bracket(const std::vector<Sample>& buffer, std::int64_t t) {
    const auto after = std::find_if(buffer.begin(), buffer.end(), [t](const Sample& s) {
        return static_cast<std::int64_t>(s.time_boot_ms) >= t;
    });
    if (after == buffer.end()) {
        return std::nullopt;  // 後一筆還沒到
    }
    if (static_cast<std::int64_t>(after->time_boot_ms) == t) {
        return Bracket<Sample>{&*after, &*after, 0.0};
    }
    if (after == buffer.begin()) {
        return std::nullopt;  // 比緩衝區最舊的還早
    }
    const Sample& before = *(after - 1);
    const double a = static_cast<double>(t - static_cast<std::int64_t>(before.time_boot_ms)) /
                     static_cast<double>(after->time_boot_ms - before.time_boot_ms);
    return Bracket<Sample>{&before, &*after, a};
}

double lerp(double from, double to, double a) { return from + a * (to - from); }

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
    remember(attitudes_, attitude, config_.timing.px4_buffer_ms);
    return {};
}

Results DecisionCore::on_position(const Position& position, Now) {
    remember(positions_, position, config_.timing.px4_buffer_ms);
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
    // 03 只用到 roll、pitch；位置要夾得住才算能用。yaw、位置、速度的內插在第一個用到的 ticket 做
    const std::int64_t t = static_cast<std::int64_t>(frame.timestamp_ms);
    const auto attitude = bracket(attitudes_, t);
    if (!attitude || !bracket(positions_, t)) {
        return {};
    }
    Attitude at_frame = *attitude->before;
    at_frame.roll = lerp(attitude->before->roll, attitude->after->roll, attitude->a);
    at_frame.pitch = lerp(attitude->before->pitch, attitude->after->pitch, attitude->a);
    return {Height{agl(frame, at_frame), DataState::ok}};
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
    if (attitudes_.empty() || positions_.empty()) {
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
