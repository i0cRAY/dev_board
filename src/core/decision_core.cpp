#include "core/decision_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "core/angle.h"
#include "core/terrain.h"

namespace dev_board {
namespace {

using Vec3 = std::array<double, 3>;

// 角度的上限多留一點：MAVLink 是 float，最接近 π 的 float 比 double 的 π 大約 9e-8
constexpr double kMaxAngle = kPi + 1e-6;

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

// 每個原因的幀數，只列出現過的
std::map<CheckReason, int> count_reasons(const std::vector<FrameCheck>& frames) {
    std::map<CheckReason, int> counts;
    for (const FrameCheck& f : frames) {
        if (f.reason) {
            ++counts[*f.reason];
        }
    }
    return counts;
}

// 機身 FRD ↔ 水平座標系：R = Ry(pitch)·Rx(roll)，不含 yaw（§5 第 5 步）
class Tilt {
public:
    explicit Tilt(const Attitude& attitude)
        : cr_(std::cos(attitude.roll)),
          sr_(std::sin(attitude.roll)),
          cp_(std::cos(attitude.pitch)),
          sp_(std::sin(attitude.pitch)) {}

    // R·b
    Vec3 to_horizontal(const Vec3& b) const {
        const double y = cr_ * b[1] - sr_ * b[2];
        const double z = sr_ * b[1] + cr_ * b[2];
        return {cp_ * b[0] + sp_ * z, y, cp_ * z - sp_ * b[0]};
    }
    // Rᵀ·h
    Vec3 to_body(const Vec3& h) const {
        const double x = cp_ * h[0] - sp_ * h[2];
        const double z = sp_ * h[0] + cp_ * h[2];
        return {x, cr_ * h[1] + sr_ * z, cr_ * z - sr_ * h[1]};
    }

private:
    double cr_;
    double sr_;
    double cp_;
    double sp_;
};

}  // namespace

// §5 第 3～7 步：把每個有效點放進細格，無效像素沿射線打到中位數高度的水平面，算進落點那格的無效數。
// 細格涵蓋 Z、外圈和外圈外面一格
terrain::Grid DecisionCore::project(const DepthFrame& frame, const Attitude& attitude) const {
    const Tilt tilt(attitude);
    const Vec3 o = tilt.to_horizontal(config_.camera.t_cam);

    terrain::Grid grid(config_.zone.width_cells / 2 + config_.zone.ring_cells + 1, config_.grid.cell_size_m);
    const Intrinsics& k = config_.camera.depth;
    std::vector<double> heights;
    for (int v = 0; v < k.height; ++v) {
        for (int u = 0; u < k.width; ++u) {
            const std::uint16_t mm = frame.depth_mm[static_cast<std::size_t>(v) * k.width + u];
            if (mm == 0) {
                continue;
            }
            const double z = mm / 1000.0;
            // 相機光學座標 → 機身 FRD：(x_b, y_b, z_b) = (−y_c, x_c, z_c)
            const Vec3 q = tilt.to_horizontal({-z * ray_y_[v], z * ray_x_[u], z});
            grid.add_valid(q[0] + o[0], q[1] + o[1], -q[2]);  // h 不加 o.z，從相機量起
            heights.push_back(-q[2]);
        }
    }
    if (heights.empty()) {
        return grid;  // 沒有中位數：所有細格都是 0 個點，沒有 h20 要算
    }

    // 第 7 步：深度 1 的射線方向打到 h = h_med，要往下（d.z > 0）而且 h_med 在相機下面才打得到
    const double h_med = terrain::median(heights);
    for (int v = 0; v < k.height; ++v) {
        for (int u = 0; u < k.width; ++u) {
            if (frame.depth_mm[static_cast<std::size_t>(v) * k.width + u] != 0) {
                continue;
            }
            const Vec3 d = tilt.to_horizontal({-ray_y_[v], ray_x_[u], 1.0});
            const double z = -h_med / d[2];
            if (!(d[2] > 0.0 && z > 0.0)) {
                continue;
            }
            grid.add_invalid(z * d[0] + o[0], z * d[1] + o[1]);
        }
    }
    grid.finish(config_.evaluate.low_percentile);
    return grid;
}

DecisionCore::DecisionCore(const Config& config) : config_(config) {
    const Intrinsics& k = config_.camera.depth;
    for (int u = 0; u < k.width; ++u) {
        ray_x_.push_back((u + 0.5 - k.cx) / k.fx);
    }
    for (int v = 0; v < k.height; ++v) {
        ray_y_.push_back((v + 0.5 - k.cy) / k.fy);
    }
}

Results DecisionCore::on_attitude(const Attitude& attitude, Now now) {
    const Results expired{{}, expire(now)};
    // roll 或 pitch 超過 ±kMaxAngle（含 NaN、inf）整筆不收：進到後面會變成未定義的浮點數轉整數，
    // 很大的有限值內插時也會溢位。yaw 在第一個用到的 ticket（06）一起檢查
    if (!(std::abs(attitude.roll) <= kMaxAngle) || !(std::abs(attitude.pitch) <= kMaxAngle)) {
        return expired;
    }
    remember(attitudes_, attitude, config_.timing.px4_buffer_ms);
    return expired;
}

Results DecisionCore::on_position(const Position& position, Now now) {
    const Results expired{{}, expire(now)};
    // vx、vy 不是有限值整筆不收：內插出 NaN 時 moving 的比較不成立，在移動也會判合格。
    // north、east、down 在第一個用到的 ticket（06）一起檢查
    if (!std::isfinite(position.vx_mps) || !std::isfinite(position.vy_mps)) {
        return expired;
    }
    remember(positions_, position, config_.timing.px4_buffer_ms);
    return expired;
}

Results DecisionCore::on_depth(const DepthFrame& frame, Now now) {
    const Intrinsics& k = config_.camera.depth;
    if (frame.width != k.width || frame.height != k.height) {
        throw std::invalid_argument("DecisionCore::on_depth: frame size does not match camera.depth");
    }
    if (frame.depth_mm.size() != static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height)) {
        throw std::invalid_argument("DecisionCore::on_depth: depth_mm is not width x height");
    }
    // 期限到了這一幀就不算進確認：先回 no_depth
    std::optional<DropCheck> check = expire(now);
    depth_seen_ = true;
    last_frame_timestamp_ = frame.timestamp_ms;
    const DataState now_state = state();
    if (now_state != DataState::ok) {
        return {Height{std::nullopt, now_state}, check};
    }
    // 用到 roll、pitch、vx、vy；位置要夾得住才算能用。yaw 和 north、east、down 的內插在 06 做
    const std::int64_t t = static_cast<std::int64_t>(frame.timestamp_ms);
    const auto attitude = bracket(attitudes_, t);
    const auto position = bracket(positions_, t);
    if (!attitude || !position) {
        return {{}, check};
    }
    Attitude at_frame = *attitude->before;
    at_frame.roll = lerp(attitude->before->roll, attitude->after->roll, attitude->a);
    at_frame.pitch = lerp(attitude->before->pitch, attitude->after->pitch, attitude->a);
    Position position_at_frame = *position->before;
    position_at_frame.vx_mps = lerp(position->before->vx_mps, position->after->vx_mps, position->a);
    position_at_frame.vy_mps = lerp(position->before->vy_mps, position->after->vy_mps, position->a);
    const terrain::Grid grid = project(frame, at_frame);
    const std::optional<double> height = terrain::agl(grid, config_.zone.width_cells);
    if (check_) {
        check = collect(frame, check_frame(grid, height, at_frame, position_at_frame));
    }
    return {Height{height, DataState::ok}, check};
}

Results DecisionCore::on_check_request(const CheckRequest& request, Now now) {
    const Results expired{{}, expire(now)};
    // 只跟等待中的確認比：比它小或相同（重送）的不取代它（05 的決定 O1）
    if (check_ && request.request_id <= check_->request_id) {
        return expired;
    }
    check_ = PendingCheck{request.request_id, now + std::chrono::milliseconds(config_.drop_check.max_wait_ms), {}};
    return expired;
}

// §7 每幀一個結果：依序檢查，第一個成立的就是這一幀的原因
std::optional<CheckReason> DecisionCore::check_frame(const terrain::Grid& grid, std::optional<double> agl,
                                                     const Attitude& attitude, const Position& position) const {
    const double tilt_angle = std::acos(std::cos(attitude.roll) * std::cos(attitude.pitch));
    if (tilt_angle > config_.limits.max_tilt_deg * kDegree) {
        return CheckReason::tilted;
    }
    if (std::hypot(position.vx_mps, position.vy_mps) > config_.drop_check.max_speed_mps) {
        return CheckReason::moving;
    }
    if (agl && *agl > config_.drop_check.drop_max_height_m) {
        return CheckReason::too_high;
    }
    if (agl && (*agl < config_.limits.min_distance_m || !sees_zone_and_ring(*agl, attitude))) {
        return CheckReason::too_low;
    }
    return terrain::evaluate(grid, config_);
}

// Z＋外圈的四個角 (±(Zw/2 + r), ±(Zw/2 + r))、深度 agl，用 §8.5 的公式投影到深度影像，都要在畫面裡（§7）
bool DecisionCore::sees_zone_and_ring(double agl, const Attitude& attitude) const {
    const Tilt tilt(attitude);
    const Vec3 o = tilt.to_horizontal(config_.camera.t_cam);
    const Intrinsics& k = config_.camera.depth;
    const double a = config_.zone.width_m / 2.0 + config_.zone.ring_width_m;
    for (const double x : {-a, a}) {
        for (const double y : {-a, a}) {
            const Vec3 b = tilt.to_body({x - o[0], y - o[1], agl});
            // 機身 FRD → 相機光學座標：(x_c, y_c, z_c) = (y_b, −x_b, z_b)
            const double u = k.fx * b[1] / b[2] + k.cx;
            const double v = k.fy * -b[0] / b[2] + k.cy;
            if (!(b[2] > 0.0 && u >= 0.0 && u < k.width && v >= 0.0 && v < k.height)) {
                return false;
            }
        }
    }
    return true;
}

// 收進確認中的一幀；同一個 frame_id 只算一次。收滿時照 §7 合併
std::optional<DropCheck> DecisionCore::collect(const DepthFrame& frame, std::optional<CheckReason> reason) {
    if (!check_) {
        return std::nullopt;
    }
    std::vector<FrameCheck>& frames = check_->frames;
    if (std::any_of(frames.begin(), frames.end(),
                    [&](const FrameCheck& f) { return f.frame_id == frame.frame_id; })) {
        return std::nullopt;
    }
    frames.push_back({frame.frame_id, reason});
    if (frames.size() < static_cast<std::size_t>(config_.drop_check.frames)) {
        return std::nullopt;
    }
    // §7：合格幀數 ≥ min_pass 就合格；否則取最多的原因，平手時取排前面的（CheckReason 的順序）
    DropCheck result{check_->request_id, frame.timestamp_ms, true, std::nullopt, {}, frames};
    const std::map<CheckReason, int> counts = count_reasons(frames);
    const auto passed = std::count_if(frames.begin(), frames.end(), [](const FrameCheck& f) { return !f.reason; });
    if (passed < config_.drop_check.min_pass) {
        result.ok = false;
        result.counts = counts;
        const auto most = std::max_element(counts.begin(), counts.end(),
                                           [](const auto& a, const auto& b) { return a.second < b.second; });
        result.reason = most->first;
    }
    check_.reset();
    return result;
}

std::optional<DropCheck> DecisionCore::expire(Now now) {
    if (!check_ || now < check_->deadline) {
        return std::nullopt;
    }
    DropCheck result{check_->request_id, last_frame_timestamp_, false, CheckReason::no_depth,
                     count_reasons(check_->frames), check_->frames};
    check_.reset();
    return result;
}

Results DecisionCore::on_tick(Now now) {
    std::optional<DropCheck> expired = expire(now);
    const DataState now_state = state();
    if (now_state != DataState::ok) {
        return {Height{std::nullopt, now_state}, expired};
    }
    return {{}, expired};
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

}  // namespace dev_board
