#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "synthetic/synthetic_depth.h"

using namespace dev_board;
using namespace dev_board::synthetic;

namespace {

constexpr double kDegree = 3.14159265358979323846 / 180.0;

// 640×480、hfov 87°，跟 config/dev_board.json 一樣，但測試不依賴那個檔案
Config::Camera camera() {
    Config::Camera c{};
    c.depth = {160, 120, 84.3024, 84.3024, 80.0, 60.0};
    c.rgb = {640, 480, 337.2096, 337.2096, 320.0, 240.0};
    c.near_m = 0.1;
    c.far_m = 20.0;
    c.t_cam = {0.0, 0.0, 0.0};
    return c;
}

Pose at_height(double height_m) {
    Pose pose;
    pose.down_m = -height_m;
    return pose;
}

int pixel(const std::vector<std::uint16_t>& frame, int u, int v) {
    REQUIRE(frame.size() == 160 * 120);
    return frame[static_cast<std::size_t>(v * 160 + u)];
}

Footprint square_at(double north, double east, double half) {
    return {north - half, north + half, east - half, east + half};
}

bool all_equal(const std::vector<std::uint16_t>& frame, int mm) {
    return std::all_of(frame.begin(), frame.end(), [mm](int value) { return value == mm; });
}

// 影像正中央的 4 個像素都接近 mm。傾斜時上下左右差幾公分，所以給容差
void check_center(const std::vector<std::uint16_t>& frame, int mm) {
    for (const int u : {79, 80}) {
        for (const int v : {59, 60}) {
            CAPTURE(u);
            CAPTURE(v);
            CHECK(std::abs(pixel(frame, u, v) - mm) <= 50);
        }
    }
}

}  // namespace

TEST_CASE("synthetic: level flat ground has the same z-depth everywhere") {
    const auto frame = synthesize_depth({}, camera(), at_height(5.0));
    REQUIRE(frame.size() == 160 * 120);
    CHECK(all_equal(frame, 5000));
}

TEST_CASE("synthetic: ground beyond far is 0") {
    CHECK(all_equal(synthesize_depth({}, camera(), at_height(20.0)), 20000));
    CHECK(all_equal(synthesize_depth({}, camera(), at_height(20.1)), 0));
    CHECK(all_equal(synthesize_depth({}, camera(), at_height(25.0)), 0));
}

// 深度 4 m 時 1 m 對到 84.3 個原始像素。凸起的頂面剛好蓋到 102.25 或 102.75 個原始像素：
// 第 102 個像素的中心在 102.5，蓋到時第 25 個降採樣像素（100～103）有 3 行是頂面，
// 沒蓋到時只有 2 行，一半一半取較遠的地面。像素中心差半格、縮放或方向錯了都會變。
TEST_CASE("synthetic: pixel centres are at u+0.5, v+0.5; image up is the nose, right is right") {
    const double covered = 4.0 * 102.75 / 337.2096;
    const double missed = 4.0 * 102.25 / 337.2096;
    const auto frame_with = [](Footprint footprint) {
        Scene scene;
        scene.blocks = {{footprint, 1.0}};
        return synthesize_depth(scene, camera(), at_height(5.0));
    };
    // 蓋到的那一邊看到頂面，其他三邊沒蓋到
    const auto north = frame_with({-missed, covered, -missed, missed});
    CHECK(pixel(north, 80, 34) == 4000);
    CHECK(pixel(north, 80, 85) == 5000);
    CHECK(pixel(north, 54, 60) == 5000);
    CHECK(pixel(north, 105, 60) == 5000);
    const auto south = frame_with({-covered, missed, -missed, missed});
    CHECK(pixel(south, 80, 34) == 5000);
    CHECK(pixel(south, 80, 85) == 4000);
    const auto west = frame_with({-missed, missed, -covered, missed});
    CHECK(pixel(west, 54, 60) == 4000);
    CHECK(pixel(west, 105, 60) == 5000);
    const auto east = frame_with({-missed, missed, -missed, covered});
    CHECK(pixel(east, 54, 60) == 5000);
    CHECK(pixel(east, 105, 60) == 4000);
    // 再往外的像素都是地面，往內的都是頂面
    CHECK(pixel(east, 106, 60) == 5000);
    CHECK(pixel(east, 104, 60) == 4000);
}

TEST_CASE("synthetic: position moves the camera over the scene") {
    Scene scene;
    scene.blocks = {{square_at(3.0, -7.0, 0.3), 1.0}};
    Pose pose = at_height(5.0);
    pose.north_m = 3.0;
    pose.east_m = -7.0;
    check_center(synthesize_depth(scene, camera(), pose), 4000);
    pose.north_m = 2.0;
    check_center(synthesize_depth(scene, camera(), pose), 5000);
}

// 頂面在相機下方 5 m。傾斜 30° 時光軸打在 5·tan30° = 2.887 m 外，深度 5/cos30° = 5.774 m；
// 沒打到凸起的話會是 6.93 m
TEST_CASE("synthetic: tilt points the optical axis where expected") {
    Pose pose = at_height(6.0);
    double north = 0.0;
    double east = 0.0;
    int mm = 5774;
    SUBCASE("nose up looks forward") {
        pose.pitch = 30 * kDegree;
        north = 2.887;
    }
    SUBCASE("right side down looks left") {
        pose.roll = 30 * kDegree;
        east = -2.887;
    }
    SUBCASE("nose up, heading east") {
        pose.pitch = 30 * kDegree;
        pose.yaw = 90 * kDegree;
        east = 2.887;
    }
    SUBCASE("right side down, heading east") {
        pose.roll = 30 * kDegree;
        pose.yaw = 90 * kDegree;
        north = 2.887;
    }
    SUBCASE("roll is applied before pitch") {
        // 光軸 = Ry·Rx·(0,0,1)：往前 5·tan30°，往左 5·tan30°/cos30°，深度 5/cos²30°
        pose.roll = 30 * kDegree;
        pose.pitch = 30 * kDegree;
        north = 2.887;
        east = -3.333;
        mm = 6667;
    }
    Scene scene;
    scene.blocks = {{square_at(north, east, 0.15), 1.0}};
    check_center(synthesize_depth(scene, camera(), pose), mm);
}

TEST_CASE("synthetic: yaw turns the image: what is east appears ahead when heading east") {
    Scene scene;
    scene.blocks = {{square_at(0.0, 2.0, 0.2), 1.0}};
    Pose pose = at_height(5.0);
    pose.yaw = 90 * kDegree;
    const auto frame = synthesize_depth(scene, camera(), pose);
    CHECK(pixel(frame, 80, 60 - 42) == 4000);  // 2 m ÷ 4 m × 84.3 = 42 個像素
    CHECK(pixel(frame, 80 + 42, 60) == 5000);
}

// roll 10° 的平地：depth = 5 / (x·sin10° + cos10°)，x 是像素中心的 (u+0.5−cx)/fx
TEST_CASE("synthetic: tilted flat ground is nearer on the lower side") {
    Pose pose = at_height(5.0);
    pose.roll = 10 * kDegree;
    const auto frame = synthesize_depth({}, camera(), pose);
    CHECK(pixel(frame, 120, 60) == 4682);
    CHECK(pixel(frame, 39, 60) == 5549);
}

TEST_CASE("synthetic: pixels looking above the horizon are 0") {
    Pose pose = at_height(5.0);
    pose.pitch = 80 * kDegree;
    const auto frame = synthesize_depth({}, camera(), pose);
    CHECK(pixel(frame, 80, 0) == 0);
    CHECK(pixel(frame, 80, 119) > 5000);  // 影像下緣還看得到地面
}

TEST_CASE("synthetic: a tall block is seen above the horizon") {
    Scene scene;
    scene.blocks = {{{5.0, 6.0, -5.0, 5.0}, 30.0}};
    Pose pose = at_height(5.0);
    pose.pitch = 80 * kDegree;
    const auto frame = synthesize_depth(scene, camera(), pose);
    CHECK(pixel(frame, 80, 30) == 4783);  // 牆在 5 m 外：5 / (sin80° + cos80°·117.5/337.2)
}

TEST_CASE("synthetic: blocks behind the camera and around it") {
    Scene scene;
    SUBCASE("a tall block to the south does not hide the ground to the north") {
        scene.blocks = {{{-3.0, -2.0, -1.0, 1.0}, 20.0}};
        CHECK(pixel(synthesize_depth(scene, camera(), at_height(5.0)), 80, 20) == 5000);
    }
    SUBCASE("a camera inside a block sees nothing") {
        scene.blocks = {{square_at(0.0, 0.0, 1.0), 6.0}};
        CHECK(all_equal(synthesize_depth(scene, camera(), at_height(5.0)), 0));
    }
}

TEST_CASE("synthetic: rays parallel to the sides of blocks and pits") {
    // 光軸剛好在像素中心、沒有降採樣，中央的射線垂直往下
    Config::Camera c = camera();
    c.rgb = {9, 9, 10.0, 10.0, 4.5, 4.5};
    c.depth = c.rgb;
    Scene scene;
    const auto center = [&] { return synthesize_depth(scene, c, at_height(5.0))[4 * 9 + 4]; };
    SUBCASE("block underneath") {
        scene.blocks = {{square_at(0.0, 0.0, 1.0), 1.0}};
        CHECK(center() == 4000);
    }
    SUBCASE("block to one side") {
        scene.blocks = {{square_at(0.0, 2.0, 1.0), 1.0}, {square_at(0.0, -2.0, 1.0), 1.0}};
        CHECK(center() == 5000);
    }
    SUBCASE("pit underneath") {
        scene.pits = {{square_at(0.0, 0.0, 1.0), 1.0}};
        CHECK(center() == 6000);
    }
    SUBCASE("pit to one side") {
        scene.pits = {{square_at(2.0, 0.0, 1.0), 1.0}, {square_at(-2.0, 0.0, 1.0), 1.0}};
        CHECK(center() == 5000);
    }
    // 射線剛好在邊上，算 (邊 − 原點) ÷ 方向會變成 0/0
    SUBCASE("exactly on an edge") {
        for (const Footprint& f : {Footprint{0, 1, -1, 1}, {-1, 0, -1, 1}, {-1, 1, 0, 1}, {-1, 1, -1, 0}}) {
            CAPTURE(f.north_min_m);
            CAPTURE(f.north_max_m);
            CAPTURE(f.east_min_m);
            scene = {};
            scene.blocks = {{f, 1.0}};
            CHECK(center() == 4000);
            scene = {};
            scene.pits = {{f, 1.0}};
            CHECK(center() == 6000);
        }
    }
}

TEST_CASE("synthetic: t_cam is rotated with the body") {
    Config::Camera c = camera();
    Scene scene;
    Pose pose = at_height(5.0);
    SUBCASE("level: lower camera, and right is south when heading east") {
        c.t_cam = {0.0, 2.0, 0.3};
        pose.yaw = 90 * kDegree;
        scene.blocks = {{square_at(-2.0, 0.0, 0.2), 1.0}};
        check_center(synthesize_depth(scene, c, pose), 3700);
    }
    SUBCASE("forward is north when heading north") {
        c.t_cam = {2.0, 0.0, 0.0};
        scene.blocks = {{square_at(2.0, 0.0, 0.2), 1.0}};
        check_center(synthesize_depth(scene, c, pose), 4000);
    }
    SUBCASE("nose up 30 degrees lifts a camera 1 m ahead by 0.5 m") {
        c.t_cam = {1.0, 0.0, 0.0};
        pose.pitch = 30 * kDegree;
        check_center(synthesize_depth(scene, c, pose), 6351);  // 5.5 / cos30°
    }
}

TEST_CASE("synthetic: sloped ground") {
    Scene scene;
    SUBCASE("rising north") {
        scene.rise_per_north = std::tan(10 * kDegree);
        const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
        CHECK(pixel(frame, 80, 20) == 4620);  // 5 / (1 + tan10°·157.5/337.2)
        CHECK(pixel(frame, 80, 99) == 5452);
    }
    SUBCASE("rising east") {
        scene.rise_per_east = std::tan(10 * kDegree);
        const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
        CHECK(pixel(frame, 119, 60) == 4620);
        CHECK(pixel(frame, 40, 60) == 5452);
    }
    SUBCASE("ground passes through the world origin") {
        scene.rise_per_north = 0.1;
        scene.rise_per_east = 0.2;
        Pose pose = at_height(5.0);
        pose.north_m = 4.0;   // 地面高 0.4 m
        pose.east_m = -2.0;   // 地面低 0.4 m，抵銷
        check_center(synthesize_depth(scene, camera(), pose), 5000);
        pose.east_m = 3.0;
        check_center(synthesize_depth(scene, camera(), pose), 4000);
    }
    SUBCASE("block tops and pit floors follow the ground") {
        scene.rise_per_north = 0.1;
        Pose pose = at_height(5.0);
        pose.north_m = 10.0;  // 地面高 1 m
        scene.blocks = {{square_at(10.0, 0.0, 0.5), 0.5}};
        check_center(synthesize_depth(scene, camera(), pose), 3500);
        scene.blocks.clear();
        scene.pits = {{square_at(10.0, 0.0, 0.5), 0.5}};
        check_center(synthesize_depth(scene, camera(), pose), 4500);
    }
}

TEST_CASE("synthetic: block top and side") {
    Scene scene;
    scene.blocks = {{{-1.0, 1.0, 1.0, 2.0}, 2.0}};
    const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
    CHECK(pixel(frame, 80, 60) == 5000);
    CHECK(pixel(frame, 101, 60) == 3944);  // 側面在東 1 m：depth = 337.2 / 85.5
    CHECK(pixel(frame, 120, 60) == 3000);  // 頂面
    CHECK(pixel(frame, 140, 60) == 5000);  // 凸起後面的地面
    CHECK(pixel(frame, 120, 10) == 5000);  // 頂面前方是地面（頂面只到北 1 m）
}

TEST_CASE("synthetic: the nearest of several blocks is seen") {
    Scene scene;
    scene.blocks = {{square_at(0.0, 0.0, 0.2), 2.0}, {square_at(0.0, 0.0, 1.0), 1.0}};
    const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
    check_center(frame, 3000);
    std::reverse(scene.blocks.begin(), scene.blocks.end());
    CHECK(synthesize_depth(scene, camera(), at_height(5.0)) == frame);
}

TEST_CASE("synthetic: pit floor and wall") {
    Scene scene;
    scene.pits = {{square_at(0.0, 0.0, 1.0), 1.0}};
    const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
    CHECK(pixel(frame, 80, 60) == 6000);
    CHECK(pixel(frame, 95, 60) == 5483);  // 壁在東 1 m：depth = 337.2 / 61.5
    CHECK(pixel(frame, 64, 60) == 5483);
    CHECK(pixel(frame, 80, 44) == 5483);
    CHECK(pixel(frame, 80, 75) == 5483);
    CHECK(pixel(frame, 100, 60) == 5000);
}

TEST_CASE("synthetic: ground hides a pit that lies further along the ray") {
    Scene scene;
    scene.pits = {{{-1.0, 1.0, 1.0, 2.0}, 1.0}};
    const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
    CHECK(pixel(frame, 95, 60) == 5000);  // 打到東 0.9 m 的地面
    CHECK(pixel(frame, 100, 60) == 6000);
}

TEST_CASE("synthetic: no wall between touching or overlapping pits") {
    Scene scene;
    SUBCASE("touching") { scene.pits = {{{-1, 1, -1, 1}, 1.0}, {{-1, 1, 1, 2}, 1.0}}; }
    SUBCASE("overlapping") { scene.pits = {{{-1, 1, -1, 1}, 1.0}, {{-1, 1, 0.5, 2}, 1.0}}; }
    SUBCASE("listed the other way round") { scene.pits = {{{-1, 1, 1, 2}, 1.0}, {{-1, 1, -1, 1}, 1.0}}; }
    SUBCASE("three in a row, middle one last") {
        scene.pits = {{{-1, 1, -1, 0.95}, 1.0}, {{-1, 1, 1.02, 2}, 1.0}, {{-1, 1, 0.95, 1.02}, 1.0}};
    }
    const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
    CHECK(pixel(frame, 95, 60) == 6000);  // 只有一個坑時這裡是壁
}

TEST_CASE("synthetic: no wall between touching pits whose edges differ by a rounding error") {
    Scene scene;
    const double edge = 0.1 + 0.2;
    REQUIRE(edge != 0.3);
    SUBCASE("in order") { scene.pits = {{{-1, 1, -1, 0.3}, 1.0}, {{-1, 1, edge, 2}, 1.0}}; }
    SUBCASE("other way round") { scene.pits = {{{-1, 1, edge, 2}, 1.0}, {{-1, 1, -1, 0.3}, 1.0}}; }
    CHECK(pixel(synthesize_depth(scene, camera(), at_height(5.0)), 84, 60) == 6000);
}

TEST_CASE("synthetic: a deeper pit next to a shallow one has a wall up to the shallow floor") {
    Scene scene;
    scene.pits = {{{-1, 1, -1, 1}, 1.0}, {{-1, 1, 1, 2}, 0.3}};
    const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
    CHECK(pixel(frame, 95, 60) == 5483);   // 深坑的壁
    CHECK(pixel(frame, 100, 60) == 5300);  // 淺坑底
}

TEST_CASE("synthetic: a block inside a pit stands on the pit floor") {
    Scene scene;
    scene.pits = {{square_at(0.0, 0.0, 1.0), 1.0}};
    scene.blocks = {{{-0.5, 0.5, 0.5, 0.8}, 0.2}};
    const auto frame = synthesize_depth(scene, camera(), at_height(5.0));
    CHECK(pixel(frame, 80, 60) == 6000);
    CHECK(pixel(frame, 87, 60) == 5715);  // 地面以下的側面：depth = 0.5 × 337.2 / 29.5
}

TEST_CASE("synthetic: noise is repeatable, and added before downsampling") {
    const auto noisy = [](double stddev, std::uint32_t seed) {
        Options options;
        options.noise_stddev_m = stddev;
        options.seed = seed;
        return synthesize_depth({}, camera(), at_height(5.0), options);
    };
    const auto frame = noisy(0.02, 1);
    CHECK(frame == noisy(0.02, 1));
    CHECK(frame != noisy(0.02, 2));
    CHECK(all_equal(noisy(0.0, 1), 5000));

    double sum = 0.0;
    double squares = 0.0;
    for (const int mm : frame) {
        sum += mm - 5000;
        squares += (mm - 5000) * (mm - 5000);
    }
    const double n = static_cast<double>(frame.size());
    const double mean = sum / n;
    const double stddev = std::sqrt(squares / n - mean * mean);
    CHECK(std::abs(mean) < 2.5);  // 16 個取第 8 個，會偏遠約 0.08σ = 1.5 mm
    CHECK(stddev > 0.28 * 20.0);  // 4×4 中位數把獨立雜訊降到約 0.31 倍（§6.2）
    CHECK(stddev < 0.34 * 20.0);
}

TEST_CASE("synthetic: chosen regions become 0") {
    Options options;
    options.invalid = {{10, 20, 3, 2}, {159, 119, 1, 1}};
    const auto frame = synthesize_depth({}, camera(), at_height(5.0), options);
    for (int v = 0; v < 120; ++v) {
        for (int u = 0; u < 160; ++u) {
            const bool cleared = (u >= 10 && u < 13 && v >= 20 && v < 22) || (u == 159 && v == 119);
            if (pixel(frame, u, v) != (cleared ? 0 : 5000)) {
                CAPTURE(u);
                CAPTURE(v);
                CHECK(pixel(frame, u, v) == (cleared ? 0 : 5000));
            }
        }
    }
}

TEST_CASE("synthetic: follows the camera sizes in the config") {
    Config::Camera c = camera();
    c.depth = {320, 240, 168.6048, 168.6048, 160.0, 120.0};
    const auto frame = synthesize_depth({}, c, at_height(5.0));
    CHECK(frame.size() == 320 * 240);
    CHECK(all_equal(frame, 5000));
}

// fx ≠ fy、光軸不在中央，頂面中心在原始像素 u = 250 + 1.2/4·400 = 370、v = 260 − 1.0/4·300 = 185
TEST_CASE("synthetic: uses fx, fy, cx and cy separately") {
    Config::Camera c = camera();
    c.rgb = {640, 480, 400.0, 300.0, 250.0, 260.0};
    c.depth = {160, 120, 100.0, 75.0, 62.5, 65.0};
    Scene scene;
    scene.blocks = {{square_at(1.0, 1.2, 0.2), 1.0}};
    const auto frame = synthesize_depth(scene, c, at_height(5.0));
    CHECK(pixel(frame, 92, 46) == 4000);
    // 頂面是 u 350～390、v 170～200，這些在它外面
    CHECK(pixel(frame, 92, 41) == 5000);
    CHECK(pixel(frame, 98, 46) == 5000);
    CHECK(pixel(frame, 80, 46) == 5000);
}

TEST_CASE("synthetic: uses near and far from the config") {
    Config::Camera c = camera();
    const auto run = [&c](double near_m, double far_m, double height_m) {
        c.near_m = near_m;
        c.far_m = far_m;
        return synthesize_depth({}, c, at_height(height_m));
    };
    CHECK(all_equal(run(4.5, 5.5, 5.0), 5000));
    CHECK(all_equal(run(5.5, 20.0, 5.0), 0));
    CHECK(all_equal(run(0.1, 4.5, 5.0), 0));
    CHECK(all_equal(run(0.1, 15.1, 15.1), 15100));
}

TEST_CASE("synthetic: rejects scenes and poses that make no sense") {
    const auto run = [](const Scene& scene, const Pose& pose, const Options& options) {
        synthesize_depth(scene, camera(), pose, options);
    };
    const auto with_block = [](Footprint f, double height) { Scene s; s.blocks = {{f, height}}; return s; };
    const auto with_pit = [](Footprint f, double depth) { Scene s; s.pits = {{f, depth}}; return s; };
    const auto with_invalid = [](PixelRect rect) { Options o; o.invalid = {rect}; return o; };
    const Footprint ok{-1, 1, -1, 1};
    const Pose pose = at_height(5.0);

    CHECK_NOTHROW(run(with_block(ok, 1.0), pose, with_invalid({0, 0, 160, 120})));
    CHECK_NOTHROW(run(with_pit(ok, 1.0), pose, {}));
    CHECK_THROWS_AS(run({}, at_height(0.0), {}), std::invalid_argument);
    CHECK_THROWS_AS(run({}, at_height(-1.0), {}), std::invalid_argument);
    CHECK_THROWS_AS(run({}, at_height(std::nan("")), {}), std::invalid_argument);
    CHECK_THROWS_AS(run(with_block(ok, 0.0), pose, {}), std::invalid_argument);
    CHECK_THROWS_AS(run(with_pit(ok, -1.0), pose, {}), std::invalid_argument);
    CHECK_THROWS_AS(run(with_block({1, -1, -1, 1}, 1.0), pose, {}), std::invalid_argument);
    CHECK_THROWS_AS(run(with_pit({-1, 1, 1, -1}, 1.0), pose, {}), std::invalid_argument);
    for (const PixelRect rect : {PixelRect{-1, 0, 2, 1}, {0, -1, 1, 2}, {159, 0, 2, 1},
                                 {0, 119, 1, 2}, {0, 0, 0, 1}, {0, 0, 1, 0}}) {
        CHECK_THROWS_AS(run({}, pose, with_invalid(rect)), std::invalid_argument);
    }

    Options options;
    options.noise_stddev_m = -0.01;
    CHECK_THROWS_AS(run({}, pose, options), std::invalid_argument);
}
