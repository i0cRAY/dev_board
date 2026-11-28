#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <map>
#include <optional>
#include <utility>
#include <random>
#include <string>
#include <vector>

#include "core/config.h"
#include "core/decision_core.h"
#include "synthetic/synthetic_depth.h"

using namespace dev_board;
using synthetic::Pose;
using synthetic::Scene;

namespace {

constexpr const char* kConfigPath = "config/dev_board.json";
constexpr double kDegree = 3.14159265358979323846 / 180.0;

Now at(int ms) { return Now{} + std::chrono::milliseconds(ms); }

Pose hover(double height_m) {
    Pose pose;
    pose.down_m = -height_m;
    return pose;
}

// 水平、t_cam = 0、離平地 height_m 時，像素中心落在 x0 ≤ x < x1、y0 ≤ y < y1（公尺，x 往前、y 往右）
// 的降採樣像素。x(v) = −h·(v + 0.5 − cy)/fy，y(u) = h·(u + 0.5 − cx)/fx
synthetic::PixelRect pixels_over(const Config& config, double height_m, double x0, double x1, double y0,
                                 double y1) {
    const Intrinsics& k = config.camera.depth;
    const int u0 = static_cast<int>(std::ceil(y0 * k.fx / height_m + k.cx - 0.5));
    const int u1 = static_cast<int>(std::ceil(y1 * k.fx / height_m + k.cx - 0.5));
    const int v0 = static_cast<int>(std::floor(k.cy - 0.5 - x1 * k.fy / height_m)) + 1;
    const int v1 = static_cast<int>(std::floor(k.cy - 0.5 - x0 * k.fy / height_m)) + 1;
    return {u0, v0, u1 - u0, v1 - v0};
}

// 0.5 × 0.5 m 的坑對齊細格，在 Z 正中央的 2×2 格
synthetic::Pit centre_pit(double depth_m) { return {{-0.25, 0.25, -0.25, 0.25}, depth_m}; }

std::string name(const std::optional<CheckReason>& reason) {
    if (!reason) {
        return "ok";
    }
    switch (*reason) {
        case CheckReason::hole: return "hole";
        case CheckReason::rough: return "rough";
        case CheckReason::slope: return "slope";
        case CheckReason::invalid_depth: return "invalid_depth";
        case CheckReason::too_low: return "too_low";
        case CheckReason::too_high: return "too_high";
        case CheckReason::tilted: return "tilted";
        case CheckReason::moving: return "moving";
        case CheckReason::no_depth: return "no_depth";
    }
    return "?";
}

DepthFrame frame_of(const Config& config, std::uint32_t frame_id, const std::vector<std::uint16_t>& depth,
                    std::uint64_t timestamp_ms = 0) {
    return {frame_id, timestamp_ms, config.camera.depth.width, config.camera.depth.height, depth};
}

// 按「確認」後依序送 depths 的每一張（frame_id 1、2、…，sim time 100、200、… ms），回傳最後一幀產生的結果。
// 前面的幀不能有結果。vx、vy 是 PX4 的水平速度（北、東）
std::optional<DropCheck> drop_check(const Config& config, const Pose& pose,
                                    const std::vector<std::vector<std::uint16_t>>& depths, double vx = 0.0,
                                    double vy = 0.0) {
    DecisionCore core(config);
    for (const std::uint32_t t : {0u, 1000u}) {
        core.on_attitude({t, pose.roll, pose.pitch, pose.yaw}, at(0));
        core.on_position({t, pose.north_m, pose.east_m, pose.down_m, vx, vy}, at(0));
    }
    core.on_check_request({7}, at(0));
    std::optional<DropCheck> result;
    for (std::size_t i = 0; i < depths.size(); ++i) {
        const auto id = static_cast<std::uint32_t>(i + 1);
        Results results = core.on_depth(frame_of(config, id, depths[i], 100u * id), at(30 * (i + 1)));
        if (i + 1 < depths.size()) {
            REQUIRE_FALSE(results.drop_check.has_value());
        } else {
            result = results.drop_check;
        }
    }
    return result;
}

// 同一張深度送 drop_check.frames 幀
std::optional<DropCheck> drop_check(const Config& config, const Pose& pose, const std::vector<std::uint16_t>& depth) {
    return drop_check(config, pose,
                      std::vector<std::vector<std::uint16_t>>(static_cast<std::size_t>(config.drop_check.frames), depth));
}

std::optional<DropCheck> drop_check(const Config& config, const Scene& scene, const Pose& pose,
                                    const synthetic::Options& options = {}) {
    return drop_check(config, pose, synthetic::synthesize_depth(scene, config.camera, pose, options));
}

// 每一幀的結果都一樣時，確認結果就是那個結果
std::string verdict_of(const Config& config, const std::optional<DropCheck>& result) {
    REQUIRE(result.has_value());
    REQUIRE(result->frames.size() == static_cast<std::size_t>(config.drop_check.frames));
    for (const FrameCheck& frame : result->frames) {
        CHECK(name(frame.reason) == name(result->reason));
    }
    CHECK(result->ok == !result->reason.has_value());
    return name(result->reason);
}

std::string verdict(const Config& config, const Scene& scene, const Pose& pose,
                    const synthetic::Options& options = {}) {
    return verdict_of(config, drop_check(config, scene, pose, options));
}

std::string verdict(const Config& config, const Pose& pose, const std::vector<std::uint16_t>& depth) {
    return verdict_of(config, drop_check(config, pose, depth));
}

// 水平、t_cam = 0、離平地 height_m 時，像素中心落在格子 (ix, iy)（ix·s ≤ x < (ix + 1)·s，y 同理）的
// 降採樣像素（v·width + u）。只取深度乘上 scale 之後仍落在同一格的，用來把點往下移而不換格
std::vector<std::size_t> pixels_in(const Config& config, double height_m, int ix, int iy, double scale = 1.0) {
    const Intrinsics& k = config.camera.depth;
    const double s = config.grid.cell_size_m;
    std::vector<std::size_t> pixels;
    for (int v = 0; v < k.height; ++v) {
        for (int u = 0; u < k.width; ++u) {
            const double x = -height_m * (v + 0.5 - k.cy) / k.fy;
            const double y = height_m * (u + 0.5 - k.cx) / k.fx;
            const bool in = std::floor(x / s) == ix && std::floor(y / s) == iy &&
                            std::floor(x * scale / s) == ix && std::floor(y * scale / s) == iy;
            if (in) {
                pixels.push_back(static_cast<std::size_t>(v * k.width + u));
            }
        }
    }
    return pixels;
}

// 場景表寫「A，或先 B」時兩個都可以
bool one_of(const std::string& result, std::initializer_list<const char*> expected) {
    for (const char* reason : expected) {
        if (result == reason) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST_CASE("drop check: flat level ground passes") {
    const Config config = load_config(kConfigPath);
    const std::optional<DropCheck> result = drop_check(config, {}, hover(3.0));
    REQUIRE(result.has_value());
    CHECK(result->request_id == 7);
    CHECK(result->ok);
    CHECK_FALSE(result->reason.has_value());
    CHECK(result->counts.empty());
    CHECK(result->frame_timestamp == 1000);
    REQUIRE(result->frames.size() == 10);
    for (std::size_t i = 0; i < result->frames.size(); ++i) {
        CHECK(result->frames[i].frame_id == i + 1);
        CHECK_FALSE(result->frames[i].reason.has_value());
    }
}


// 傾角 = acos(cos(roll)·cos(pitch))，門檻 5°（§7）
TEST_CASE("drop check: tilted more than the limit") {
    const Config config = load_config(kConfigPath);
    Pose pose = hover(3.0);
    SUBCASE("roll 6 degrees") {
        pose.roll = 6.0 * kDegree;
        CHECK(verdict(config, {}, pose) == "tilted");
    }
    SUBCASE("roll 4 and pitch 4 degrees tilt 5.66 degrees") {
        pose.roll = 4.0 * kDegree;
        pose.pitch = 4.0 * kDegree;
        CHECK(verdict(config, {}, pose) == "tilted");
    }
    SUBCASE("roll 4 degrees passes") {
        pose.roll = 4.0 * kDegree;
        CHECK(verdict(config, {}, pose) == "ok");
    }
}

// 水平速度 hypot(vx, vy)，門檻 0.3 m/s（§7）
TEST_CASE("drop check: moving faster than the limit") {
    const Config config = load_config(kConfigPath);
    const Pose pose = hover(3.0);
    const std::vector<std::vector<std::uint16_t>> flat(
        static_cast<std::size_t>(config.drop_check.frames), synthetic::synthesize_depth({}, config.camera, pose));
    SUBCASE("1 m/s north is moving") {
        CHECK(verdict_of(config, drop_check(config, pose, flat, 1.0, 0.0)) == "moving");
    }
    SUBCASE("1 m/s east is moving") {
        CHECK(verdict_of(config, drop_check(config, pose, flat, 0.0, 1.0)) == "moving");
    }
    SUBCASE("0.25 m/s north and 0.25 m/s east is 0.35 m/s, moving") {
        CHECK(verdict_of(config, drop_check(config, pose, flat, 0.25, 0.25)) == "moving");
    }
    SUBCASE("0 passes") { CHECK(verdict_of(config, drop_check(config, pose, flat, 0.0, 0.0)) == "ok"); }
}

// 速度不是有限值的位置整筆不收：收了會內插出 NaN，NaN > 0.3 不成立，在移動也會判合格。
// 0 和 1000 ms 是 1 m/s，50 ms 多一筆壞的
TEST_CASE("drop check: a position whose speed is not finite is ignored") {
    const Config config = load_config(kConfigPath);
    const Pose pose = hover(3.0);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, pose);
    Position bad{50, 0.0, 0.0, pose.down_m, 1.0, 0.0};
    SUBCASE("vx NaN") { bad.vx_mps = std::nan(""); }
    SUBCASE("vx inf") { bad.vx_mps = HUGE_VAL; }
    SUBCASE("vy -inf") { bad.vy_mps = -HUGE_VAL; }
    DecisionCore core(config);
    core.on_attitude({0, 0.0, 0.0, 0.0}, at(0));
    core.on_attitude({1000, 0.0, 0.0, 0.0}, at(0));
    core.on_position({0, 0.0, 0.0, pose.down_m, 1.0, 0.0}, at(0));
    core.on_position({1000, 0.0, 0.0, pose.down_m, 1.0, 0.0}, at(0));
    core.on_position(bad, at(0));
    core.on_check_request({3}, at(0));
    std::optional<DropCheck> result;
    for (std::uint32_t id = 1; id <= 10; ++id) {
        result = core.on_depth(frame_of(config, id, flat, 100u * id), at(0)).drop_check;
    }
    CHECK(verdict_of(config, result) == "moving");
}

// PX4 的速度內插到幀時間：sim time 0 → 1000 ms 從 0 加速到 0.55 m/s，第 k 幀（k·100 ms）是 0.055·k m/s
TEST_CASE("drop check: the speed of each frame is interpolated to the frame time") {
    const Config config = load_config(kConfigPath);
    const Pose pose = hover(3.0);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, pose);
    double vx = 0.0;
    double vy = 0.0;
    SUBCASE("north") { vx = 0.55; }
    SUBCASE("east") { vy = 0.55; }
    DecisionCore core(config);
    core.on_attitude({0, 0.0, 0.0, 0.0}, at(0));
    core.on_attitude({1000, 0.0, 0.0, 0.0}, at(0));
    core.on_position({0, 0.0, 0.0, pose.down_m, 0.0, 0.0}, at(0));
    core.on_position({1000, 0.0, 0.0, pose.down_m, vx, vy}, at(0));
    core.on_check_request({3}, at(0));
    std::optional<DropCheck> result;
    for (std::uint32_t id = 1; id <= 10; ++id) {
        result = core.on_depth(frame_of(config, id, flat, 100u * id), at(0)).drop_check;
    }
    REQUIRE(result.has_value());
    REQUIRE(result->frames.size() == 10);
    for (std::size_t i = 0; i < 10; ++i) {
        CAPTURE(i);
        CHECK(name(result->frames[i].reason) == std::string(i < 5 ? "ok" : "moving"));
    }
}

// agl > drop_max_height（5 m）是 too_high；量不到 agl 時跳過這一步（§7）
TEST_CASE("drop check: height above flat ground") {
    const Config config = load_config(kConfigPath);
    SUBCASE("3 m passes") { CHECK(verdict(config, {}, hover(3.0)) == "ok"); }
    SUBCASE("5 m passes") { CHECK(verdict(config, {}, hover(5.0)) == "ok"); }
    SUBCASE("7.5 m is too_high") { CHECK(verdict(config, {}, hover(7.5)) == "too_high"); }
    SUBCASE("12 m is too_high") { CHECK(verdict(config, {}, hover(12.0)) == "too_high"); }
    SUBCASE("18 m is too_high") { CHECK(verdict(config, {}, hover(18.0)) == "too_high"); }
    SUBCASE("25 m is beyond the camera, invalid_depth") { CHECK(verdict(config, {}, hover(25.0)) == "invalid_depth"); }
}

// Z＋外圈的四個角 (±1.25, ±1.25) m 投影到深度影像，有任何一個在畫面外就是 too_low（§7）。
// vfov/2 ≈ 35.4°：水平時約 1.76 m 以上前後兩個角才在畫面裡；pitch 4° 時約 2.05 m；
// 相機往前 0.3 m 時後面的角離相機 1.55 m，約 2.18 m
TEST_CASE("drop check: too low to see all of Z and its ring") {
    const Config config = load_config(kConfigPath);
    Pose pose;
    SUBCASE("pitch 4 degrees at 1.5 m is too_low") {
        pose = hover(1.5);
        pose.pitch = 4.0 * kDegree;
        CHECK(verdict(config, {}, pose) == "too_low");
    }
    SUBCASE("pitch 4 degrees at 2.5 m passes") {
        pose = hover(2.5);
        pose.pitch = 4.0 * kDegree;
        CHECK(verdict(config, {}, pose) == "ok");
    }
    SUBCASE("1.9 m passes level and is too_low at pitch 4 degrees") {
        pose = hover(1.9);
        CHECK(verdict(config, {}, pose) == "ok");
        pose.pitch = 4.0 * kDegree;
        CHECK(verdict(config, {}, pose) == "too_low");
    }
    SUBCASE("1.9 m is too_low at pitch -4 degrees") {
        pose = hover(1.9);
        pose.pitch = -4.0 * kDegree;
        CHECK(verdict(config, {}, pose) == "too_low");
    }
    SUBCASE("2 m passes with the camera at the centre and is too_low with it 0.3 m forward") {
        pose = hover(2.0);
        CHECK(verdict(config, {}, pose) == "ok");
        Config forward = config;
        forward.camera.t_cam = {0.3, 0.0, 0.0};
        CHECK(verdict(forward, {}, pose) == "too_low");
    }
    // 相機往前 0.3 m，後面的角離相機 1.55 m。抬頭（pitch > 0）時相機往前看，後面的角更偏
    SUBCASE("camera 0.3 m forward at 2.3 m: pitch -4 degrees passes, pitch 4 degrees is too_low") {
        Config forward = config;
        forward.camera.t_cam = {0.3, 0.0, 0.0};
        pose = hover(2.3);
        pose.pitch = -4.0 * kDegree;
        CHECK(verdict(forward, {}, pose) == "ok");
        pose.pitch = 4.0 * kDegree;
        CHECK(verdict(forward, {}, pose) == "too_low");
    }
    // 相機往右 0.6 m，左邊的角離相機 1.85 m（hfov/2 ≈ 43.5°）。右邊往下（roll > 0）時機腹朝左，相機往左看
    SUBCASE("camera 0.6 m right at 2.1 m: roll 4 degrees passes, roll -4 degrees is too_low") {
        Config right = config;
        right.camera.t_cam = {0.0, 0.6, 0.0};
        pose = hover(2.1);
        pose.roll = 4.0 * kDegree;
        CHECK(verdict(right, {}, pose) == "ok");
        pose.roll = -4.0 * kDegree;
        CHECK(verdict(right, {}, pose) == "too_low");
    }
}

// 視野夠寬時四個角在 0.2 m 也看得到，仍因為 agl < 0.3 m 是 too_low
TEST_CASE("drop check: closer than the minimum distance is too_low") {
    Config config = load_config(kConfigPath);
    config.camera.depth.fx = config.camera.depth.fy = 8.0;
    config.camera.rgb.fx = config.camera.rgb.fy = 32.0;
    SUBCASE("0.2 m is too_low") { CHECK(verdict(config, {}, hover(0.2)) == "too_low"); }
    SUBCASE("0.5 m passes") { CHECK(verdict(config, {}, hover(0.5)) == "ok"); }
}

// 一幀依序檢查 tilted → moving → too_high／too_low → evaluate，第一個成立的就是這一幀的原因（§7）
TEST_CASE("drop check: each frame reports the first check that fails") {
    const Config config = load_config(kConfigPath);
    SUBCASE("tilted and moving is tilted") {
        Pose pose = hover(3.0);
        pose.roll = 6.0 * kDegree;
        const std::vector<std::vector<std::uint16_t>> flat(
            static_cast<std::size_t>(config.drop_check.frames), synthetic::synthesize_depth({}, config.camera, pose));
        CHECK(verdict_of(config, drop_check(config, pose, flat, 1.0, 0.0)) == "tilted");
    }
    SUBCASE("moving and too high is moving") {
        const Pose pose = hover(7.5);
        const std::vector<std::vector<std::uint16_t>> flat(
            static_cast<std::size_t>(config.drop_check.frames), synthetic::synthesize_depth({}, config.camera, pose));
        CHECK(verdict_of(config, drop_check(config, pose, flat, 1.0, 0.0)) == "moving");
    }
}

// 0.5 × 0.5 m 的坑對齊細格，在 Z 正中央的 2×2 格
TEST_CASE("drop check: a pit in the middle of Z") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("0.3 m deep is rough") {
        scene.pits.push_back(centre_pit(0.3));
        CHECK(verdict(config, scene, hover(3.0)) == "rough");
    }
    SUBCASE("0.3 m deep passes when the depth noise accounts for the roughness") {
        Config noisy = config;
        noisy.evaluate.depth_noise_m = 0.1;
        scene.pits.push_back(centre_pit(0.3));
        CHECK(verdict(noisy, scene, hover(3.0)) == "ok");
    }
    SUBCASE("0.5 m deep is a hole") {
        scene.pits.push_back(centre_pit(0.5));
        CHECK(verdict(config, scene, hover(3.0)) == "hole");
    }
}

TEST_CASE("drop check: sloped ground") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("9 degrees passes") {
        scene.rise_per_north = std::tan(9.0 * kDegree);
        CHECK(verdict(config, scene, hover(3.0)) == "ok");
    }
    SUBCASE("11 degrees is slope") {
        scene.rise_per_east = std::tan(11.0 * kDegree);
        CHECK(verdict(config, scene, hover(3.0)) == "slope");
    }
}

// 0.5 × 0.5 m、高 0.6 m 的石頭對齊細格，在 Z 裡
TEST_CASE("drop check: a rock in Z is rough") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.blocks.push_back({{0.0, 0.5, 0.0, 0.5}, 0.6});
    CHECK(verdict(config, scene, hover(3.0)) == "rough");
}

// 外圈是 Z 往外 0.75 m：x 或 y 在 0.5～1.25 m（含負的那邊）
TEST_CASE("drop check: a standing person in the ring is a hole") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.blocks.push_back({{0.75, 1.05, -0.25, 0.25}, 1.7});
    CHECK(verdict(config, scene, hover(3.0)) == "hole");
}

// 0.2 m 寬、比箱子窄，規則 A 抓不到；延伸到外圈外面，規則 B 抓得到
TEST_CASE("drop check: a narrow slit running from Z out of the ring is a hole") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("forward") { scene.pits.push_back({{0.0, 10.0, 0.0, 0.2}, 1.0}); }
    SUBCASE("back") { scene.pits.push_back({{-10.0, 0.0, 0.0, 0.2}, 1.0}); }
    SUBCASE("right") { scene.pits.push_back({{0.0, 0.2, 0.0, 10.0}, 1.0}); }
    SUBCASE("left") { scene.pits.push_back({{0.0, 0.2, -10.0, 0.0}, 1.0}); }
    CHECK(verdict(config, scene, hover(3.0)) == "hole");
}

TEST_CASE("drop check: pixels set to 0") {
    const Config config = load_config(kConfigPath);
    synthetic::Options options;
    SUBCASE("4 separate cells in Z are invalid_depth") {
        options.invalid = {pixels_over(config, 3.0, -0.5, -0.25, -0.5, -0.25),
                           pixels_over(config, 3.0, -0.5, -0.25, 0.25, 0.5),
                           pixels_over(config, 3.0, 0.25, 0.5, -0.5, -0.25),
                           pixels_over(config, 3.0, 0.25, 0.5, 0.25, 0.5)};
        CHECK(verdict(config, {}, hover(3.0), options) == "invalid_depth");
    }
    SUBCASE("2x2 cells in Z are a hole") {
        options.invalid = {pixels_over(config, 3.0, 0.0, 0.5, 0.0, 0.5)};
        CHECK(verdict(config, {}, hover(3.0), options) == "hole");
    }
    SUBCASE("a row from Z out of the ring is a hole") {
        options.invalid = {pixels_over(config, 3.0, 0.0, 0.25, 0.0, 1.5)};
        CHECK(verdict(config, {}, hover(3.0), options) == "hole");
    }
    SUBCASE("all of Z is invalid_depth") {
        options.invalid = {pixels_over(config, 3.0, -0.5, 0.5, -0.5, 0.5)};
        CHECK(verdict(config, {}, hover(3.0), options) == "invalid_depth");
    }
    SUBCASE("one cell at the outer edge of the ring passes") {
        options.invalid = {pixels_over(config, 3.0, 1.0, 1.25, 0.0, 0.25)};
        CHECK(verdict(config, {}, hover(3.0), options) == "ok");
    }
}

// 0.5 × 0.5 m 的坑對齊細格，整個在外圈裡
TEST_CASE("drop check: a pit in the ring") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("0.3 m deep passes") {
        scene.pits.push_back({{0.5, 1.0, 0.0, 0.5}, 0.3});
        CHECK(verdict(config, scene, hover(3.0)) == "ok");
    }
    SUBCASE("0.5 m deep is a hole") {
        scene.pits.push_back({{0.5, 1.0, 0.0, 0.5}, 0.5});
        CHECK(verdict(config, scene, hover(3.0)) == "hole");
    }
}

// Z 的邊界在 x = 0.5 m：坑一半在 Z、一半在外圈
TEST_CASE("drop check: a 0.5 m pit across the edge of Z") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.pits.push_back({{0.25, 0.75, -0.25, 0.25}, 1.0});
    SUBCASE("alone") {}
    SUBCASE("with a narrow slit attached") { scene.pits.push_back({{0.25, 0.45, 0.25, 1.0}, 1.0}); }
    CHECK(one_of(verdict(config, scene, hover(3.0)), {"hole", "rough"}));
}

// 沒對齊細格：部分在坑裡的格子平均低不到 D，會被當成內點把平面往下拉，要排除後重新擬合才看得出坑
TEST_CASE("drop check: an unaligned 0.5 m pit across the edge of Z is a hole") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.pits.push_back({{-0.75, -0.25, -0.4, 0.1}, 0.5});
    CHECK(verdict(config, scene, hover(3.0)) == "hole");
}

TEST_CASE("drop check: a 0.5 m wide, 1 m deep crack") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("aligned with the cells") { scene.pits.push_back({{0.25, 0.75, -10.0, 10.0}, 1.0}); }
    SUBCASE("not aligned") { scene.pits.push_back({{0.35, 0.85, -10.0, 10.0}, 1.0}); }
    CHECK(one_of(verdict(config, scene, hover(3.0)), {"hole", "slope", "rough"}));
}

// 離坑底 3 m，坑深 1 m
TEST_CASE("drop check: Z on the bottom of a 1 m deep pit") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("1.5 m wide is a hole") {
        scene.pits.push_back({{-0.75, 0.75, -0.75, 0.75}, 1.0});
        CHECK(verdict(config, scene, hover(2.0)) == "hole");
    }
    SUBCASE("2 m wide is a hole") {
        scene.pits.push_back({{-1.0, 1.0, -1.0, 1.0}, 1.0});
        CHECK(verdict(config, scene, hover(2.0)) == "hole");
    }
    SUBCASE("3 m wide passes (known limitation)") {
        scene.pits.push_back({{-1.5, 1.5, -1.5, 1.5}, 1.0});
        CHECK(verdict(config, scene, hover(2.0)) == "ok");
    }
}

// 0.2 × 0.2 m、深 1 m，在一格的中間，四周是平地
TEST_CASE("drop check: an isolated 0.2 m pit in the ring passes") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("innermost cell") { scene.pits.push_back({{0.525, 0.725, 0.025, 0.225}, 1.0}); }
    SUBCASE("outermost cell") { scene.pits.push_back({{1.025, 1.225, 0.025, 0.225}, 1.0}); }
    CHECK(verdict(config, scene, hover(3.0)) == "ok");
}

// 10 m 高的建築，離底下的地面 3 m
TEST_CASE("drop check: a 10 m building") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("middle of the roof passes") {
        scene.blocks.push_back({{-20.0, 20.0, -20.0, 20.0}, 10.0});
        CHECK(verdict(config, scene, hover(13.0)) == "ok");
    }
    SUBCASE("roof edge 0.25 m from Z is a hole") {
        scene.blocks.push_back({{-20.0, 0.75, -20.0, 20.0}, 10.0});
        CHECK(verdict(config, scene, hover(13.0)) == "hole");
    }
    SUBCASE("roof edge 0.45 m from Z is a hole") {
        scene.blocks.push_back({{-20.0, 0.95, -20.0, 20.0}, 10.0});
        CHECK(verdict(config, scene, hover(13.0)) == "hole");
    }
    SUBCASE("ground 0.25 m from the wall is a hole") {
        scene.blocks.push_back({{0.75, 20.0, -20.0, 20.0}, 10.0});
        CHECK(verdict(config, scene, hover(3.0)) == "hole");
    }
    SUBCASE("ground far from the building passes") {
        scene.blocks.push_back({{20.0, 40.0, -20.0, 20.0}, 10.0});
        CHECK(verdict(config, scene, hover(3.0)) == "ok");
    }
}

// 1.7 × 0.45 m、高 0.25 m，比 D 矮
TEST_CASE("drop check: a person lying down") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    SUBCASE("in the ring passes (known limitation)") {
        scene.blocks.push_back({{0.7, 1.15, -0.85, 0.85}, 0.25});
        CHECK(verdict(config, scene, hover(3.0)) == "ok");
    }
    SUBCASE("in Z is slope or rough") {
        scene.blocks.push_back({{-0.225, 0.225, -0.85, 0.85}, 0.25});
        CHECK(one_of(verdict(config, scene, hover(3.0)), {"slope", "rough"}));
    }
}

// 漏判：坑整個在 Z＋外圈（x、y 都在 −1.25～1.25 m）裡卻判合格（GLOSSARY.md）。M1 spec 的小模擬是 2.5%、4%
TEST_CASE("drop check: misses 10% or fewer of 100 random 0.5 m pits at 4 m") {
    const Config config = load_config(kConfigPath);
    for (const double noise : {0.0, 0.02}) {
        CAPTURE(noise);
        std::mt19937 random(20261128);
        // 自己換算成 0～1，不用 uniform_real_distribution：它的算法各家標準函式庫不同
        const auto uniform = [&random](double low, double high) {
            return low + (high - low) * (static_cast<double>(random()) / 4294967296.0);
        };
        int missed = 0;
        for (int i = 0; i < 100; ++i) {
            const double north = uniform(-1.25, 0.75);
            const double east = uniform(-1.25, 0.75);
            Scene scene;
            scene.pits.push_back({{north, north + 0.5, east, east + 0.5}, 0.5});
            synthetic::Options options;
            options.noise_stddev_m = noise;
            options.seed = static_cast<std::uint32_t>(i);
            if (verdict(config, scene, hover(4.0), options) == "ok") {
                ++missed;
            }
        }
        MESSAGE("missed " << missed << " of 100");
        CHECK(missed <= 10);
    }
}


// 請求之前收到的是坑，之後是平地：只算之後的
TEST_CASE("drop check: only frames after the request count, each frame_id once") {
    const Config config = load_config(kConfigPath);
    const Pose pose = hover(3.0);
    Scene pit;
    pit.pits.push_back(centre_pit(0.5));
    const std::vector<std::uint16_t> before = synthetic::synthesize_depth(pit, config.camera, pose);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, pose);

    DecisionCore core(config);
    core.on_attitude({0, 0.0, 0.0, 0.0}, at(0));
    core.on_position({0, 0.0, 0.0, pose.down_m, 0.0, 0.0}, at(0));
    for (std::uint32_t id = 1; id <= 10; ++id) {
        CHECK_FALSE(core.on_depth(frame_of(config, id, before), at(0)).drop_check.has_value());
    }
    core.on_check_request({8}, at(0));
    for (std::uint32_t id = 11; id <= 19; ++id) {
        CHECK_FALSE(core.on_depth(frame_of(config, id, flat), at(0)).drop_check.has_value());
    }
    CHECK_FALSE(core.on_depth(frame_of(config, 19, flat), at(0)).drop_check.has_value());
    const std::optional<DropCheck> result = core.on_depth(frame_of(config, 20, flat), at(0)).drop_check;
    REQUIRE(result.has_value());
    CHECK(result->request_id == 8);
    CHECK(result->ok);
    REQUIRE(result->frames.size() == 10);
    CHECK(result->frames.front().frame_id == 11);
    CHECK(result->frames.back().frame_id == 20);
    CHECK_FALSE(core.on_depth(frame_of(config, 21, flat), at(0)).drop_check.has_value());
}

// 收幀途中又收到 check_request：只跟等待中的確認比，比它大才取代（§7、05 的決定 O1）
TEST_CASE("drop check: a second request while frames are being collected") {
    const Config config = load_config(kConfigPath);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, hover(3.0));
    DecisionCore core(config);
    core.on_attitude({0, 0.0, 0.0, 0.0}, at(0));
    core.on_position({0, 0.0, 0.0, -3.0, 0.0, 0.0}, at(0));
    std::uint32_t id = 0;
    // 送 count 幀，回傳有結果的那些
    const auto send = [&](int count) {
        std::vector<DropCheck> results;
        for (int i = 0; i < count; ++i) {
            ++id;
            if (std::optional<DropCheck> result = core.on_depth(frame_of(config, id, flat), at(0)).drop_check) {
                results.push_back(*result);
            }
        }
        return results;
    };
    core.on_check_request({9}, at(0));
    CHECK(send(4).empty());

    SUBCASE("a smaller request_id does not replace it") {
        core.on_check_request({7}, at(0));
        const std::vector<DropCheck> results = send(10);
        REQUIRE(results.size() == 1);
        CHECK(results[0].request_id == 9);
        REQUIRE(results[0].frames.size() == 10);
        CHECK(results[0].frames.front().frame_id == 1);
        CHECK(results[0].frames.back().frame_id == 10);
    }
    SUBCASE("a larger request_id replaces it, and only the new one has a result") {
        core.on_check_request({11}, at(0));
        const std::vector<DropCheck> results = send(14);
        REQUIRE(results.size() == 1);
        CHECK(results[0].request_id == 11);
        REQUIRE(results[0].frames.size() == 10);
        CHECK(results[0].frames.front().frame_id == 5);
        CHECK(results[0].frames.back().frame_id == 14);
    }
    // 去重和結果快取在 M2，核心不記得已算完的 request_id
    SUBCASE("once it has a result, a smaller request_id starts a new check") {
        REQUIRE(send(6).size() == 1);
        core.on_check_request({3}, at(0));
        const std::vector<DropCheck> results = send(10);
        REQUIRE(results.size() == 1);
        CHECK(results[0].request_id == 3);
        CHECK(results[0].frames.front().frame_id == 11);
    }
    // server 沒收到 ack 每 300 ms 重送一次，不能每次都清掉已收的幀
    SUBCASE("the same request_id sent again keeps the frames already collected") {
        core.on_check_request({9}, at(0));
        const std::vector<DropCheck> results = send(10);
        REQUIRE(results.size() == 1);
        CHECK(results[0].request_id == 9);
        REQUIRE(results[0].frames.size() == 10);
        CHECK(results[0].frames.front().frame_id == 1);
        CHECK(results[0].frames.back().frame_id == 10);
    }
}

// 深度 5 FPS：請求後 200、400、…、1400 ms 收到 7 幀（前 2 幀是坑），1.5 秒（本機時鐘）收不滿 10 幀
TEST_CASE("drop check: not enough frames within 1.5 s is no_depth") {
    const Config config = load_config(kConfigPath);
    const Pose pose = hover(3.0);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, pose);
    Scene pit;
    pit.pits.push_back(centre_pit(0.5));
    const std::vector<std::uint16_t> hole = synthetic::synthesize_depth(pit, config.camera, pose);
    DecisionCore core(config);
    core.on_check_request({4}, at(0));
    for (std::uint32_t id = 1; id <= 7; ++id) {
        const Now now = at(200 * static_cast<int>(id));
        core.on_attitude({200 * id, 0.0, 0.0, 0.0}, now);
        core.on_position({200 * id, 0.0, 0.0, pose.down_m, 0.0, 0.0}, now);
        const DepthFrame frame = frame_of(config, id, id <= 2 ? hole : flat, 200u * id);
        REQUIRE_FALSE(core.on_depth(frame, now).drop_check.has_value());
    }
    CHECK_FALSE(core.on_tick(at(1499)).drop_check.has_value());

    const std::optional<DropCheck> result = core.on_tick(at(1500)).drop_check;
    REQUIRE(result.has_value());
    CHECK(result->request_id == 4);
    CHECK_FALSE(result->ok);
    CHECK(name(result->reason) == "no_depth");
    CHECK(result->counts == std::map<CheckReason, int>{{CheckReason::hole, 2}});
    CHECK(result->frame_timestamp == 1400);
    CHECK(result->frames.size() == 7);
}

// §9.2：從沒收過深度幀時 frame_timestamp 是空的
TEST_CASE("drop check: no depth frame at all is no_depth without a frame_timestamp") {
    const Config config = load_config(kConfigPath);
    DecisionCore core(config);
    core.on_check_request({1}, at(0));
    const std::optional<DropCheck> result = core.on_tick(at(1500)).drop_check;
    REQUIRE(result.has_value());
    CHECK(name(result->reason) == "no_depth");
    CHECK_FALSE(result->frame_timestamp.has_value());
    CHECK(result->counts.empty());
    CHECK(result->frames.empty());
}

// 還沒收過 PX4 時深度幀不能用，期限到了一樣回 no_depth
TEST_CASE("drop check: a frame without PX4 data still answers no_depth once 1.5 s have passed") {
    const Config config = load_config(kConfigPath);
    DecisionCore core(config);
    core.on_check_request({1}, at(0));
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, hover(3.0));
    const Results results =
        core.on_depth(frame_of(config, 1, flat, 100), at(1500));
    REQUIRE(results.drop_check.has_value());
    CHECK(name(results.drop_check->reason) == "no_depth");
}

// 1.5 秒的期限每個事件都檢查：請求後 100～900 ms 收到 9 幀，1.5 秒時的第一個事件就回 no_depth
TEST_CASE("drop check: every event answers no_depth once 1.5 s have passed") {
    const Config config = load_config(kConfigPath);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, hover(3.0));
    DecisionCore core(config);
    core.on_check_request({6}, at(0));
    for (std::uint32_t id = 1; id <= 9; ++id) {
        const Now now = at(100 * static_cast<int>(id));
        core.on_attitude({100 * id, 0.0, 0.0, 0.0}, now);
        core.on_position({100 * id, 0.0, 0.0, -3.0, 0.0, 0.0}, now);
        REQUIRE_FALSE(core.on_depth(frame_of(config, id, flat, 100u * id), now).drop_check.has_value());
    }
    Results results;
    SUBCASE("a 10th frame that arrives at 1.5 s does not count") {
        core.on_attitude({1000, 0.0, 0.0, 0.0}, at(1000));
        core.on_position({1000, 0.0, 0.0, -3.0, 0.0, 0.0}, at(1000));
        results = core.on_depth(frame_of(config, 10, flat, 1000), at(1500));
    }
    SUBCASE("attitude") { results = core.on_attitude({1000, 0.0, 0.0, 0.0}, at(1500)); }
    SUBCASE("an attitude that is not accepted") { results = core.on_attitude({1000, 1e9, 0.0, 0.0}, at(1500)); }
    SUBCASE("a frame whose time cannot be interpolated") {
        results = core.on_depth(frame_of(config, 10, flat, 5000), at(1500));
    }
    SUBCASE("position") { results = core.on_position({1000, 0.0, 0.0, -3.0, 0.0, 0.0}, at(1500)); }
    SUBCASE("a check request with a smaller request_id") {
        results = core.on_check_request({2}, at(1500));
        // 舊的已經到期，新的照常開始收幀
        std::optional<DropCheck> next;
        for (std::uint32_t id = 10; id <= 19; ++id) {
            core.on_attitude({100 * id, 0.0, 0.0, 0.0}, at(1500));
            core.on_position({100 * id, 0.0, 0.0, -3.0, 0.0, 0.0}, at(1500));
            next = core.on_depth(frame_of(config, id, flat, 100u * id), at(1500)).drop_check;
        }
        REQUIRE(next.has_value());
        CHECK(next->request_id == 2);
        CHECK(next->ok);
    }
    REQUIRE(results.drop_check.has_value());
    CHECK(results.drop_check->request_id == 6);
    CHECK(name(results.drop_check->reason) == "no_depth");
    CHECK(results.drop_check->frames.size() == 9);
    CHECK(results.drop_check->counts.empty());
}

TEST_CASE("drop check: when every frame fails the same way, counts has that reason for all frames") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.pits.push_back(centre_pit(0.5));
    const std::optional<DropCheck> result = drop_check(config, scene, hover(3.0));
    REQUIRE(result.has_value());
    CHECK_FALSE(result->ok);
    CHECK(result->counts == std::map<CheckReason, int>{{CheckReason::hole, 10}});
}

// 10 幀的結果不同時照 §7 合併：合格 ≥ 8 幀就合格，否則取最多的原因，平手時 hole 優先
TEST_CASE("drop check: frames that disagree are merged") {
    const Config config = load_config(kConfigPath);
    const Pose pose = hover(3.0);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, pose);
    Scene pit;
    pit.pits.push_back(centre_pit(0.5));
    const std::vector<std::uint16_t> hole = synthetic::synthesize_depth(pit, config.camera, pose);
    Scene slope;
    slope.rise_per_east = std::tan(11.0 * kDegree);
    const std::vector<std::uint16_t> steep = synthetic::synthesize_depth(slope, config.camera, pose);
    // 失敗的幀放前面，合格的放後面
    const auto frames = [](std::initializer_list<std::pair<int, const std::vector<std::uint16_t>*>> runs) {
        std::vector<std::vector<std::uint16_t>> depths;
        for (const auto& [count, depth] : runs) {
            depths.insert(depths.end(), static_cast<std::size_t>(count), *depth);
        }
        return depths;
    };

    SUBCASE("8 passing and 2 hole frames pass") {
        const std::optional<DropCheck> result = drop_check(config, pose, frames({{2, &hole}, {8, &flat}}));
        REQUIRE(result.has_value());
        CHECK(result->ok);
        CHECK_FALSE(result->reason.has_value());
        CHECK(result->counts.empty());
        REQUIRE(result->frames.size() == 10);
        CHECK(name(result->frames[0].reason) == "hole");
        CHECK(name(result->frames[9].reason) == "ok");
    }
    SUBCASE("7 passing and 3 hole frames are a hole") {
        const std::optional<DropCheck> result = drop_check(config, pose, frames({{3, &hole}, {7, &flat}}));
        REQUIRE(result.has_value());
        CHECK_FALSE(result->ok);
        CHECK(name(result->reason) == "hole");
        CHECK(result->counts == std::map<CheckReason, int>{{CheckReason::hole, 3}});
    }
    // 兩種順序都跑：hole 贏是因為優先順序，不是因為先到或後到
    const auto hole_wins_the_tie = [&](const std::vector<std::vector<std::uint16_t>>& depths) {
        const std::optional<DropCheck> result = drop_check(config, pose, depths);
        REQUIRE(result.has_value());
        CHECK_FALSE(result->ok);
        CHECK(name(result->reason) == "hole");
        CHECK(result->counts == std::map<CheckReason, int>{{CheckReason::hole, 3}, {CheckReason::slope, 3}});
    };
    SUBCASE("3 slope and 3 hole frames tie, and hole wins, slope first") {
        hole_wins_the_tie(frames({{3, &steep}, {3, &hole}, {4, &flat}}));
    }
    SUBCASE("3 slope and 3 hole frames tie, and hole wins, hole first") {
        hole_wins_the_tie(frames({{3, &hole}, {3, &steep}, {4, &flat}}));
    }
}

// 飛行狀態的原因也照 §7 合併。平手的順序 invalid_depth > too_low > too_high > tilted > moving，
// 每一對相鄰的都跑兩種順序：贏是因為優先順序，不是因為先到或後到
TEST_CASE("drop check: flight state reasons are merged with the same rule") {
    const Config config = load_config(kConfigPath);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, hover(3.0));
    const std::vector<std::uint16_t> high = synthetic::synthesize_depth({}, config.camera, hover(7.5));
    const std::vector<std::uint16_t> low = synthetic::synthesize_depth({}, config.camera, hover(1.5));
    const std::vector<std::uint16_t> beyond = synthetic::synthesize_depth({}, config.camera, hover(25.0));
    struct Frame {
        const std::vector<std::uint16_t>* depth;
        double roll;
        double vx;
    };
    const Frame ok{&flat, 0.0, 0.0};
    const Frame tilted{&flat, 6.0 * kDegree, 0.0};
    const Frame moving{&flat, 0.0, 1.0};
    const Frame too_high{&high, 0.0, 0.0};
    const Frame too_low{&low, 0.0, 0.0};
    const Frame invalid{&beyond, 0.0, 0.0};
    // 第 k 幀（sim time k·100 ms）之前送剛好在那個時間的姿態和位置
    const auto check = [&](std::initializer_list<std::pair<int, Frame>> runs) {
        DecisionCore core(config);
        core.on_check_request({5}, at(0));
        std::optional<DropCheck> result;
        std::uint32_t id = 0;
        for (const auto& [count, frame] : runs) {
            for (int i = 0; i < count; ++i) {
                ++id;
                core.on_attitude({100 * id, frame.roll, 0.0, 0.0}, at(0));
                core.on_position({100 * id, 0.0, 0.0, -3.0, frame.vx, 0.0}, at(0));
                result = core.on_depth(frame_of(config, id, *frame.depth, 100u * id), at(0)).drop_check;
            }
        }
        REQUIRE(id == 10);
        REQUIRE(result.has_value());
        return *result;
    };
    const auto wins = [&](const Frame& first, const Frame& second, CheckReason winner, CheckReason loser) {
        for (const DropCheck& result : {check({{3, first}, {3, second}, {4, ok}}),
                                        check({{3, second}, {3, first}, {4, ok}})}) {
            CHECK_FALSE(result.ok);
            CHECK(name(result.reason) == name(winner));
            CHECK(result.counts == std::map<CheckReason, int>{{winner, 3}, {loser, 3}});
        }
    };

    SUBCASE("8 passing and 2 tilted frames pass") {
        const DropCheck result = check({{2, tilted}, {8, ok}});
        CHECK(result.ok);
        CHECK(result.counts.empty());
    }
    SUBCASE("7 passing and 3 moving frames are moving") {
        const DropCheck result = check({{3, moving}, {7, ok}});
        CHECK_FALSE(result.ok);
        CHECK(name(result.reason) == "moving");
        CHECK(result.counts == std::map<CheckReason, int>{{CheckReason::moving, 3}});
    }
    SUBCASE("invalid_depth beats too_low") { wins(invalid, too_low, CheckReason::invalid_depth, CheckReason::too_low); }
    SUBCASE("too_low beats too_high") { wins(too_low, too_high, CheckReason::too_low, CheckReason::too_high); }
    SUBCASE("too_high beats tilted") { wins(too_high, tilted, CheckReason::too_high, CheckReason::tilted); }
    SUBCASE("tilted beats moving") { wins(tilted, moving, CheckReason::tilted, CheckReason::moving); }
}

// 3 m 高時每格 7 × 7 個像素。2×2 格每格 30 個改成 0：剩 19 個有效點，但無效比例 61%
TEST_CASE("drop check: cells with more than half of their pixels invalid are UNKNOWN") {
    const Config config = load_config(kConfigPath);
    std::vector<std::uint16_t> depth = synthetic::synthesize_depth({}, config.camera, hover(3.0));
    for (const auto& [ix, iy] : {std::pair{0, 0}, {0, 1}, {1, 0}, {1, 1}}) {
        const std::vector<std::size_t> pixels = pixels_in(config, 3.0, ix, iy);
        REQUIRE(pixels.size() == 49);
        for (std::size_t i = 0; i < 30; ++i) {
            depth[pixels[i]] = 0;
        }
    }
    CHECK(verdict(config, hover(3.0), depth) == "hole");
}

// 10.5 m 高時每格 2 × 2 個像素。2×2 格每格改 2 個成 0：剩 2 個有效點，無效比例剛好 50%。
// 高度只是為了每格的像素少，投放高度上限放寬到 11 m，才不會先判 too_high
TEST_CASE("drop check: cells with fewer than 3 valid points are UNKNOWN") {
    Config config = load_config(kConfigPath);
    config.drop_check.drop_max_height_m = 11.0;
    std::vector<std::uint16_t> depth = synthetic::synthesize_depth({}, config.camera, hover(10.5));
    for (const auto& [ix, iy] : {std::pair{0, 0}, {0, 1}, {1, 0}, {1, 1}}) {
        const std::vector<std::size_t> pixels = pixels_in(config, 10.5, ix, iy);
        REQUIRE(pixels.size() == 4);
        depth[pixels[0]] = 0;
        depth[pixels[1]] = 0;
    }
    CHECK(verdict(config, hover(10.5), depth) == "hole");
}

// 正中央 2×2 格每格 49 個點，第 20 百分位是由低到高第 ceil(0.2 × 49) = 10 個。把 k 個點的深度加 1 m
// （只挑加深後仍落在同一格的像素）：10 個時 h20 是低的點，四格都是 LOW；9 個時不是 LOW，
// 但那些點讓 Z 的粗糙度超過 R
TEST_CASE("drop check: LOW looks at the 20th percentile of a cell's heights") {
    const Config config = load_config(kConfigPath);
    const auto with_deep_points = [&](std::size_t count) {
        std::vector<std::uint16_t> depth = synthetic::synthesize_depth({}, config.camera, hover(3.0));
        for (const auto& [ix, iy] : {std::pair{-1, -1}, {-1, 0}, {0, -1}, {0, 0}}) {
            REQUIRE(pixels_in(config, 3.0, ix, iy).size() == 49);
            const std::vector<std::size_t> pixels = pixels_in(config, 3.0, ix, iy, 4.0 / 3.0);
            REQUIRE(pixels.size() >= count);
            for (std::size_t i = 0; i < count; ++i) {
                depth[pixels[i]] = 4000;
            }
        }
        return depth;
    };
    CHECK(verdict(config, hover(3.0), with_deep_points(10)) == "hole");
    CHECK(verdict(config, hover(3.0), with_deep_points(9)) == "rough");
}

// Z 的 4×4 格裡 9 格（棋盤格 8 格加 1 格，沒有 2×2）各有 15 個點深 0.6 m：平均低 0.18 m，一開始是內點；
// 第 20 百分位是低的點，排除後內點剩 7 格，少於一半
TEST_CASE("drop check: too few cells left after removing pit cells is a hole") {
    const Config config = load_config(kConfigPath);
    std::vector<std::uint16_t> depth = synthetic::synthesize_depth({}, config.camera, hover(3.0));
    for (int ix = -2; ix < 2; ++ix) {
        for (int iy = -2; iy < 2; ++iy) {
            if ((ix + iy) % 2 != 0 && !(ix == -2 && iy == -1)) {
                continue;
            }
            REQUIRE(pixels_in(config, 3.0, ix, iy).size() == 49);
            const std::vector<std::size_t> pixels = pixels_in(config, 3.0, ix, iy, 3.6 / 3.0);
            REQUIRE(pixels.size() >= 15);
            for (std::size_t i = 0; i < 15; ++i) {
                depth[pixels[i]] = 3600;
            }
        }
    }
    CHECK(verdict(config, hover(3.0), depth) == "hole");
}

// Z 是 x 在 −0.5～0.5 m 的 4 排
TEST_CASE("drop check: at least half of Z must have depth") {
    const Config config = load_config(kConfigPath);
    synthetic::Options options;
    SUBCASE("half of Z without depth goes on to the rules") {
        options.invalid = {pixels_over(config, 3.0, -0.5, 0.0, -0.5, 0.5)};
        CHECK(verdict(config, {}, hover(3.0), options) == "hole");
    }
    SUBCASE("three quarters of Z without depth is invalid_depth") {
        options.invalid = {pixels_over(config, 3.0, -0.5, 0.25, -0.5, 0.5)};
        CHECK(verdict(config, {}, hover(3.0), options) == "invalid_depth");
    }
}

// Z 前半在 0.5 m 高的台上：8 格 h = −2.5、8 格 h = −3。h_med 取中間兩格的平均，所有格子都是內點，
// 擬合出很斜的面（§6.2「台階會被算成坡度」）
TEST_CASE("drop check: a 0.5 m step across the middle of Z is slope") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.blocks.push_back({{0.0, 10.0, -10.0, 10.0}, 0.5});
    CHECK(verdict(config, scene, hover(3.0)) == "slope");
}

// 畫面右邊有 5 m 深的坑（在細格範圍外）。無效像素要打到中位數高度（平地），不是最低的點
TEST_CASE("drop check: invalid pixels land at the median height of the frame") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.pits.push_back({{-10.0, 10.0, 1.75, 2.75}, 5.0});
    synthetic::Options options;
    options.invalid = {pixels_over(config, 3.0, -0.5, -0.25, -0.5, -0.25),
                       pixels_over(config, 3.0, -0.5, -0.25, 0.25, 0.5),
                       pixels_over(config, 3.0, 0.25, 0.5, -0.5, -0.25),
                       pixels_over(config, 3.0, 0.25, 0.5, 0.25, 0.5)};
    CHECK(verdict(config, scene, hover(3.0), options) == "invalid_depth");
}

// 外圈到 1.25 m 為止，人站在外面那一格
TEST_CASE("drop check: a standing person just outside the ring passes") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.blocks.push_back({{1.3, 1.45, -0.25, 0.25}, 1.7});
    CHECK(verdict(config, scene, hover(3.0)) == "ok");
}

// 0.2 m 寬、比箱子窄，兩端都在外圈裡，沒有延伸到外面
TEST_CASE("drop check: a narrow slit that stays inside the ring passes") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.pits.push_back({{-1.0, 1.0, 0.025, 0.225}, 1.0});
    CHECK(verdict(config, scene, hover(3.0)) == "ok");
}

// 9° 的斜面往東北升高，Z 裡沿著西北—東南的對角線少 4 格：剩下的格子 x、y 有相關，
// 擬合要用到 Σxy 才會是 9°（不然會超過 10° 判 slope）
TEST_CASE("drop check: the plane fit handles cells missing along a diagonal") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    const double rise = std::tan(9.0 * kDegree) / std::sqrt(2.0);
    scene.rise_per_north = rise;
    scene.rise_per_east = rise;
    synthetic::Options options;
    options.invalid = {pixels_over(config, 3.0, 0.25, 0.5, -0.5, -0.25),
                       pixels_over(config, 3.0, 0.0, 0.25, -0.25, 0.0),
                       pixels_over(config, 3.0, -0.25, 0.0, 0.0, 0.25),
                       pixels_over(config, 3.0, -0.5, -0.25, 0.25, 0.5)};
    CHECK(verdict(config, scene, hover(3.0), options) == "invalid_depth");
}

// 收幀途中按第二次（較大的 request_id）：之前收的幀不算，只回新的
TEST_CASE("drop check: a newer request during collection starts over") {
    const Config config = load_config(kConfigPath);
    const Pose pose = hover(3.0);
    const std::vector<std::uint16_t> flat = synthetic::synthesize_depth({}, config.camera, pose);

    DecisionCore core(config);
    core.on_attitude({0, 0.0, 0.0, 0.0}, at(0));
    core.on_position({0, 0.0, 0.0, pose.down_m, 0.0, 0.0}, at(0));
    core.on_check_request({8}, at(0));
    for (std::uint32_t id = 1; id <= 5; ++id) {
        CHECK_FALSE(core.on_depth(frame_of(config, id, flat), at(0)).drop_check.has_value());
    }
    core.on_check_request({9}, at(0));
    for (std::uint32_t id = 6; id <= 14; ++id) {
        CHECK_FALSE(core.on_depth(frame_of(config, id, flat), at(0)).drop_check.has_value());
    }
    const std::optional<DropCheck> result = core.on_depth(frame_of(config, 15, flat), at(0)).drop_check;
    REQUIRE(result.has_value());
    CHECK(result->request_id == 9);
    REQUIRE(result->frames.size() == 10);
    CHECK(result->frames.front().frame_id == 6);
}
