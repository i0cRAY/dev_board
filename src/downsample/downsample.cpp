#include "downsample/downsample.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dev_board {
namespace {

// 0 = 無效。先檢查範圍再轉型：inf、NaN、負數直接轉成整數是未定義行為。
// 用 float 比較：15.1 轉成 float 會變大，跟 double 比的話剛好在 far 的深度會變無效。
// far 轉成 float 頂多大 0.004 mm，進位後不會超過 uint16。
std::uint16_t to_mm(float depth_m, float near_m, float far_m) {
    if (!(depth_m >= near_m && depth_m <= far_m)) {  // 寫成 ! 才會擋掉 NaN
        return 0;
    }
    return static_cast<std::uint16_t>(std::lround(static_cast<double>(depth_m) * 1000.0));
}

int scale_of(ImageSize from, ImageSize to) {
    if (to.width < 1 || to.height < 1 || from.width < to.width ||
        from.width % to.width != 0 || from.height % to.height != 0 ||
        from.width / to.width != from.height / to.height) {
        throw std::invalid_argument("downsample_depth: both sides must shrink by the same integer");
    }
    return from.width / to.width;
}

}  // namespace

std::vector<std::uint16_t> downsample_depth(const float* depth_m, ImageSize from, ImageSize to,
                                            double near_m, double far_m) {
    const int scale = scale_of(from, to);
    if (!(near_m > 0.0 && near_m < far_m && far_m <= kMaxFarM)) {
        throw std::invalid_argument("downsample_depth: need 0 < near < far <= 65.535 m");
    }

    const float near_f = static_cast<float>(near_m);
    const float far_f = static_cast<float>(far_m);
    const std::size_t block_size = static_cast<std::size_t>(scale) * static_cast<std::size_t>(scale);
    std::vector<std::uint16_t> frame;
    frame.reserve(static_cast<std::size_t>(to.width) * static_cast<std::size_t>(to.height));
    std::vector<std::uint16_t> valid;
    valid.reserve(block_size);
    for (int v = 0; v < to.height; ++v) {
        for (int u = 0; u < to.width; ++u) {
            valid.clear();
            for (int row = v * scale; row < (v + 1) * scale; ++row) {
                for (int col = u * scale; col < (u + 1) * scale; ++col) {
                    const std::size_t index =
                        static_cast<std::size_t>(row) * static_cast<std::size_t>(from.width) +
                        static_cast<std::size_t>(col);
                    const std::uint16_t mm = to_mm(depth_m[index], near_f, far_f);
                    if (mm != 0) {
                        valid.push_back(mm);
                    }
                }
            }
            if (valid.size() * 2 < block_size) {
                frame.push_back(0);
                continue;
            }
            // 取實際存在的值，不要平均；偶數個時取較遠的，偏安全
            const auto middle = valid.begin() + static_cast<std::ptrdiff_t>(valid.size() / 2);
            std::nth_element(valid.begin(), middle, valid.end());
            frame.push_back(*middle);
        }
    }
    return frame;
}

}  // namespace dev_board
