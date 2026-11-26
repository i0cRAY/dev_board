#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
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

DepthFrame frame_of(const Config& config, const Scene& scene, const Pose& pose,
                    std::uint64_t sim_ms, const synthetic::Options& options = {}) {
    return {1, sim_ms, config.camera.depth.width, config.camera.depth.height,
            synthetic::synthesize_depth(scene, config.camera, pose, options)};
}

// 姿態、位置、深度幀都在 sim time 0：時間剛好相等
std::optional<double> agl_of(const Config& config, const Scene& scene, const Pose& pose,
                             const synthetic::Options& options = {}) {
    DecisionCore core(config);
    core.on_attitude({0, pose.roll, pose.pitch, pose.yaw}, at(0));
    core.on_position({0, pose.north_m, pose.east_m, pose.down_m, 0.0, 0.0}, at(0));
    const Results results = core.on_depth(frame_of(config, scene, pose, 0, options), at(5));
    REQUIRE(results.height.has_value());
    CHECK(results.height->state == DataState::ok);
    return results.height->agl;
}

}  // namespace

TEST_CASE("height: agl over flat level ground is the true height") {
    const Config config = load_config(kConfigPath);
    for (const double height : {1.5, 3.0, 7.5, 12.0, 18.0}) {
        CAPTURE(height);
        const std::optional<double> agl = agl_of(config, {}, hover(height));
        REQUIRE(agl.has_value());
        CHECK(std::abs(*agl - height) < 0.005);
    }
}

TEST_CASE("height: agl is empty when the ground is beyond the camera's range") {
    const Config config = load_config(kConfigPath);
    CHECK_FALSE(agl_of(config, {}, hover(25.0)).has_value());
}

// 3 m 高時一個像素約 3.6 cm。u < 80 是 y < 0 的半邊 Z；u 80～86、v 53～59 剛好蓋住
// x、y 都在 0～0.25 m 的那一格（像素中心離格線至少 1.7 cm）。無效像素不能當成深度 0
// 的點：那些點會落在原點，也就是這一格
TEST_CASE("height: agl needs at least half of Z's cells to have a valid point") {
    const Config config = load_config(kConfigPath);
    synthetic::Options options;
    options.invalid = {{0, 0, 80, 120}};
    const std::optional<double> half = agl_of(config, {}, hover(3.0), options);
    REQUIRE(half.has_value());
    CHECK(std::abs(*half - 3.0) < 0.005);

    options.invalid.push_back({80, 53, 7, 7});
    CHECK_FALSE(agl_of(config, {}, hover(3.0), options).has_value());
}

// 前半個 Z 在 1 m 高的台上：8 格 h = −2、8 格 h = −3
TEST_CASE("height: an even number of cells takes the mean of the middle two") {
    const Config config = load_config(kConfigPath);
    Scene scene;
    scene.blocks = {{{0.0, 10.0, -10.0, 10.0}, 1.0}};
    const std::optional<double> agl = agl_of(config, scene, hover(3.0));
    REQUIRE(agl.has_value());
    CHECK(std::abs(*agl - 2.5) < 0.005);
}

// Z 在兩個 1 m 高的台中間 1.2 m 寬的地面上。7.5 m 高傾斜 5° 時，點的水平位置要是沒有補償，
// 會差約 1.3 m，台上的點就落進 Z
TEST_CASE("height: tilt is compensated in where the points land") {
    const Config config = load_config(kConfigPath);
    Pose pose = hover(7.5);
    Scene scene;
    SUBCASE("pitch") {
        scene.blocks = {{{0.6, 10.0, -10.0, 10.0}, 1.0}, {{-10.0, -0.6, -10.0, 10.0}, 1.0}};
        SUBCASE("nose up") { pose.pitch = 5.0 * kDegree; }
        SUBCASE("nose down") { pose.pitch = -5.0 * kDegree; }
    }
    SUBCASE("roll") {
        scene.blocks = {{{-10.0, 10.0, 0.6, 10.0}, 1.0}, {{-10.0, 10.0, -10.0, -0.6}, 1.0}};
        SUBCASE("right wing down") { pose.roll = 5.0 * kDegree; }
        SUBCASE("right wing up") { pose.roll = -5.0 * kDegree; }
    }
    const std::optional<double> agl = agl_of(config, scene, pose);
    REQUIRE(agl.has_value());
    CHECK(std::abs(*agl - 7.5) < 0.005);
}

TEST_CASE("height: agl is the vertical distance when the drone tilts up to 6 degrees") {
    const Config config = load_config(kConfigPath);
    for (const double height : {3.0, 7.5}) {
        for (const auto& [roll, pitch] : {std::pair{6.0, 0.0}, {0.0, 6.0}, {-6.0, -6.0}, {4.0, -6.0}}) {
            CAPTURE(height);
            CAPTURE(roll);
            CAPTURE(pitch);
            Pose pose = hover(height);
            pose.roll = roll * kDegree;
            pose.pitch = pitch * kDegree;
            const std::optional<double> agl = agl_of(config, {}, pose);
            REQUIRE(agl.has_value());
            CHECK(std::abs(*agl - height) < 0.005);
        }
    }
}

// 相機正下方是 1 m 深的坑，機身中心正下方的 Z 是平地。agl 從相機量起（§5）
TEST_CASE("height: agl is measured below the body centre when t_cam is offset") {
    Config config = load_config(kConfigPath);
    SUBCASE("forward") {
        config.camera.t_cam = {1.0, 0.0, 0.0};
        Scene scene;
        scene.pits = {{{0.75, 5.0, -5.0, 5.0}, 1.0}};
        Pose pose = hover(3.0);
        SUBCASE("level") {}
        SUBCASE("tilted") {
            pose.roll = 5.0 * kDegree;
            pose.pitch = -5.0 * kDegree;
        }
        // 機頭朝下時相機在機身中心前面 1 m，跟著低 sin(5°)
        const double camera_height = 3.0 + std::sin(pose.pitch);
        const std::optional<double> agl = agl_of(config, scene, pose);
        REQUIRE(agl.has_value());
        CHECK(std::abs(*agl - camera_height) < 0.005);
    }
    SUBCASE("right") {
        config.camera.t_cam = {0.0, 1.0, 0.0};
        Scene scene;
        scene.pits = {{{-5.0, 5.0, 0.75, 5.0}, 1.0}};
        Pose pose = hover(3.0);
        SUBCASE("level") {}
        SUBCASE("tilted") {
            pose.roll = -5.0 * kDegree;
            pose.pitch = 5.0 * kDegree;
        }
        // 右翼抬起時相機在機身中心右邊 1 m，跟著高 sin(5°)·cos(5°)
        const double camera_height = 3.0 - std::sin(pose.roll) * std::cos(pose.pitch);
        const std::optional<double> agl = agl_of(config, scene, pose);
        REQUIRE(agl.has_value());
        CHECK(std::abs(*agl - camera_height) < 0.005);
    }
}

TEST_CASE("height: over a roof or a pit bottom agl is the distance to the ground below") {
    const Config config = load_config(kConfigPath);
    SUBCASE("7.5 m above a 10 m roof") {
        Scene scene;
        scene.blocks = {{{-10.0, 10.0, -10.0, 10.0}, 10.0}};
        const std::optional<double> agl = agl_of(config, scene, hover(17.5));
        REQUIRE(agl.has_value());
        CHECK(std::abs(*agl - 7.5) < 0.005);
    }
    SUBCASE("3 m above the ground, over a 3 x 3 x 1 m pit") {
        Scene scene;
        scene.pits = {{{-1.5, 1.5, -1.5, 1.5}, 1.0}};
        const std::optional<double> agl = agl_of(config, scene, hover(3.0));
        REQUIRE(agl.has_value());
        CHECK(std::abs(*agl - 4.0) < 0.005);
    }
}

TEST_CASE("height: a frame whose size does not match camera.depth is rejected") {
    const Config config = load_config(kConfigPath);
    DecisionCore core(config);
    core.on_attitude({0, 0.0, 0.0, 0.0}, at(0));
    core.on_position({0, 0.0, 0.0, -3.0, 0.0, 0.0}, at(0));
    DepthFrame frame = frame_of(config, {}, hover(3.0), 0);
    SUBCASE("width") {
        frame.width = 320;
        frame.depth_mm.resize(320 * 120);
    }
    SUBCASE("height") {
        frame.height = 60;
        frame.depth_mm.resize(160 * 60);
    }
    SUBCASE("data") { frame.depth_mm.pop_back(); }
    CHECK_THROWS_AS(core.on_depth(frame, at(5)), std::invalid_argument);
}
