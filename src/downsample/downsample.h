#pragma once

#include <cstdint>
#include <vector>

namespace dev_board {

constexpr double kMaxFarM = 65.535;  // uint16 公釐的上限

// 一張深度影像的大小（像素）
struct ImageSize {
    int width;
    int height;
};

// 原始 z-depth（公尺，逐列排列）→ 縮小後的公釐，0 = 無效（dev_board_spec.md §2）。
// 縮小倍數由兩個大小決定，兩邊的倍數要相同而且整除，否則丟 std::invalid_argument。
// 橋接程式和合成深度共用，所以不依賴 Config。depth_m 要有 from.width × from.height 個值。
std::vector<std::uint16_t> downsample_depth(const float* depth_m, ImageSize from, ImageSize to,
                                            double near_m, double far_m);

}  // namespace dev_board
