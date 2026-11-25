#include <doctest/doctest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "downsample/downsample.h"

using namespace dev_board;

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kNearM = 0.1F;
constexpr float kFarM = 20.0F;

// 一個 4×4 的區塊縮成一個像素
std::uint16_t block(const std::vector<float>& depth_m) {
    REQUIRE(depth_m.size() == 16);
    const auto frame = downsample_depth(depth_m.data(), {4, 4}, {1, 1}, 0.1, 20.0);
    REQUIRE(frame.size() == 1);
    return frame[0];
}

// 前 count 個像素是 a，其他是 b
std::vector<float> mixed(int count, float a, float b) {
    std::vector<float> depth_m(16, b);
    for (int i = 0; i < count; ++i) {
        depth_m[static_cast<std::size_t>(i)] = a;
    }
    return depth_m;
}

}  // namespace

TEST_CASE("downsample converts metres to rounded millimetres") {
    CHECK(block(mixed(16, 5.0F, 0)) == 5000);
    CHECK(block(mixed(16, 1.2346F, 0)) == 1235);
    CHECK(block(mixed(16, 1.2344F, 0)) == 1234);
}

TEST_CASE("downsample keeps depths at near and far") {
    CHECK(block(mixed(16, kNearM, 0)) == 100);
    CHECK(block(mixed(16, kFarM, 0)) == 20000);
    // 0.1 轉成 float 會變大一點，用 0.25 才看得出有沒有包含邊界
    const std::vector<float> at_near(16, 0.25F);
    CHECK(downsample_depth(at_near.data(), {4, 4}, {1, 1}, 0.25, 20.0)[0] == 250);
}

// 15.1 轉成 float 會變大、0.7 會變小，剛好在邊界上的深度還是要算有效
TEST_CASE("downsample keeps depths at near and far that float cannot represent exactly") {
    const auto run = [](float depth_m, double near_m, double far_m) {
        return downsample_depth(&depth_m, {1, 1}, {1, 1}, near_m, far_m)[0];
    };
    REQUIRE(static_cast<double>(15.1F) > 15.1);
    REQUIRE(static_cast<double>(0.7F) < 0.7);
    CHECK(run(15.1F, 0.7, 15.1) == 15100);
    CHECK(run(std::nextafter(15.1F, kInf), 0.7, 15.1) == 0);
    CHECK(run(0.7F, 0.7, 15.1) == 700);
    CHECK(run(std::nextafter(0.7F, 0.0F), 0.7, 15.1) == 0);
    CHECK(run(65.535F, 0.7, 65.535) == 65535);
    CHECK(run(std::nextafter(65.535F, kInf), 0.7, 65.535) == 0);
}

// NaN 只驗結果：x86 和 aarch64 的 lround(NaN) 轉成 uint16 剛好也是 0，拿掉檢查這裡也不會失敗
TEST_CASE("downsample outputs 0 for depths that are not finite or outside near and far") {
    for (const float invalid : {kInf, -kInf, kNan, -1.0F, 0.0F, 0.05F, 25.0F, 70.0F}) {
        CAPTURE(invalid);
        CHECK(block(mixed(16, invalid, 0)) == 0);
    }
}

TEST_CASE("downsample takes an existing value: the farther one of an even split") {
    CHECK(block(mixed(8, 3.0F, 13.0F)) == 13000);
    CHECK(block(mixed(8, 13.0F, 3.0F)) == 13000);  // 跟像素順序無關
    CHECK(block(mixed(9, 3.0F, 13.0F)) == 3000);
    CHECK(block(mixed(7, 13.0F, 3.0F)) == 3000);
}

TEST_CASE("downsample ignores invalid pixels, and needs half of them valid") {
    CHECK(block(mixed(7, 5.0F, kInf)) == 0);
    CHECK(block(mixed(8, 5.0F, kInf)) == 5000);
    CHECK(block(mixed(7, kNan, 5.0F)) == 5000);
    CHECK(block(mixed(9, -1.0F, 5.0F)) == 0);
}

TEST_CASE("downsample: scale follows the two sizes") {
    // 640×480 的每個像素各自不同，看得出取了哪一塊
    constexpr ImageSize kFull{640, 480};
    std::vector<float> depth_m;
    for (int v = 0; v < kFull.height; ++v) {
        for (int u = 0; u < kFull.width; ++u) {
            depth_m.push_back(static_cast<float>(1000 + u + 20 * v) / 1000.0F);
        }
    }

    SUBCASE("640x480 to 160x120") {
        const auto frame = downsample_depth(depth_m.data(), kFull, {160, 120}, 0.1, 20.0);
        REQUIRE(frame.size() == 160 * 120);
        // 左上角 4×4：1000～1003、1020～1023、1040～1043、1060～1063，第 8 個是 1040
        CHECK(frame[0] == 1040);
        CHECK(frame[159] == 1040 + 636);
        CHECK(frame[160] == 1040 + 80);
        CHECK(frame[160 * 120 - 1] == 1040 + 636 + 20 * 476);
    }
    SUBCASE("640x480 to 320x240") {
        const auto frame = downsample_depth(depth_m.data(), kFull, {320, 240}, 0.1, 20.0);
        REQUIRE(frame.size() == 320 * 240);
        // 左上角 2×2：1000、1001、1020、1021，第 2 個是 1020
        CHECK(frame[0] == 1020);
        CHECK(frame[319] == 1020 + 638);
        CHECK(frame[320] == 1020 + 40);
        CHECK(frame[320 * 240 - 1] == 1020 + 638 + 20 * 478);
    }
    SUBCASE("half of 2x2 is 2 valid pixels") {
        depth_m[0] = depth_m[1] = kInf;
        CHECK(downsample_depth(depth_m.data(), kFull, {320, 240}, 0.1, 20.0)[0] == 1021);
        depth_m[640] = kInf;
        CHECK(downsample_depth(depth_m.data(), kFull, {320, 240}, 0.1, 20.0)[0] == 0);
    }
}

TEST_CASE("downsample rejects sizes it cannot scale and ranges that do not fit in uint16") {
    const std::vector<float> depth_m(17 * 17, 5.0F);
    const auto run = [&](ImageSize from, ImageSize to, double near_m, double far_m) {
        downsample_depth(depth_m.data(), from, to, near_m, far_m);
    };
    CHECK_NOTHROW(run({16, 16}, {4, 4}, 0.1, 20.0));
    CHECK_NOTHROW(run({16, 16}, {4, 4}, 0.1, 65.535));
    for (const ImageSize to : {ImageSize{5, 4}, {4, 5}, {8, 4}, {32, 32}, {0, 4}, {4, 0}, {-4, -4}}) {
        CAPTURE(to.width);
        CAPTURE(to.height);
        CHECK_THROWS_AS(run({16, 16}, to, 0.1, 20.0), std::invalid_argument);
    }
    CHECK_THROWS_AS(run({17, 16}, {4, 4}, 0.1, 20.0), std::invalid_argument);
    CHECK_THROWS_AS(run({16, 17}, {4, 4}, 0.1, 20.0), std::invalid_argument);
    CHECK_THROWS_AS(run({0, 0}, {4, 4}, 0.1, 20.0), std::invalid_argument);
    CHECK_THROWS_AS(run({16, 16}, {4, 4}, 0.1, 65.536), std::invalid_argument);
    CHECK_THROWS_AS(run({16, 16}, {4, 4}, 0.1, 70.0), std::invalid_argument);
    CHECK_THROWS_AS(run({16, 16}, {4, 4}, 0.0, 20.0), std::invalid_argument);
    CHECK_THROWS_AS(run({16, 16}, {4, 4}, 20.0, 20.0), std::invalid_argument);
    CHECK_THROWS_AS(run({16, 16}, {4, 4}, 0.1, static_cast<double>(kNan)), std::invalid_argument);
}
