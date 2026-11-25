#pragma once

#include <cstdint>
#include <vector>

#include "core/config.h"

// 不用 Gazebo 就能產生深度幀：測試和回放程式用，開發板主程式不連結。
namespace dev_board::synthetic {

// 水平範圍（世界座標，公尺）
struct Footprint {
    double north_min_m;
    double north_max_m;
    double east_min_m;
    double east_max_m;
};

// 長方體凸起：頂面平行於地面，高 height_m，四面垂直；一直延伸到地面下，所以跟坑重疊時會從坑底立起來
struct Block {
    Footprint footprint;
    double height_m;
};

// 直壁坑：坑底平行於地面，深 depth_m；重疊或相鄰的坑之間沒有牆
struct Pit {
    Footprint footprint;
    double depth_m;
};

// 地面通過世界原點（down = 0），可以傾斜：往北、往東每公尺升高多少公尺
struct Scene {
    double rise_per_north = 0.0;
    double rise_per_east = 0.0;
    std::vector<Block> blocks;
    std::vector<Pit> pits;
};

// 機身中心的位置和姿態，跟 LOCAL_POSITION_NED、ATTITUDE 一樣（公尺、rad）
struct Pose {
    double north_m = 0.0;
    double east_m = 0.0;
    double down_m = 0.0;  // 往下為正，在地面上空是負的
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

// 降採樣後的像素，從 (u, v) 開始、寬 width、高 height
struct PixelRect {
    int u;
    int v;
    int width;
    int height;
};

struct Options {
    double noise_stddev_m = 0.0;  // 高斯雜訊，加在原始解析度的深度上再降採樣
    std::uint32_t seed = 0;       // 同一個種子，雜訊相同
    std::vector<PixelRect> invalid;  // 降採樣後改成 0 的區域
};

// 射線投射出 camera.rgb 解析度的 z-depth，再用 downsample_depth 變成 camera.depth 大小的公釐。
// 相機裝在機身中心偏 camera.t_cam 的地方往下看，影像上方是機頭（§5）。
// 相機不在地面上方、凸起或坑的大小不合理、invalid 超出影像時丟 std::invalid_argument。
std::vector<std::uint16_t> synthesize_depth(const Scene& scene, const Config::Camera& camera,
                                            const Pose& pose, const Options& options = {});

}  // namespace dev_board::synthetic
