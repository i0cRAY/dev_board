#include "synthetic/synthetic_depth.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

#include "downsample/downsample.h"

namespace dev_board::synthetic {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTouchT = 1e-9;  // 兩個坑的邊只差捨入誤差（0.3 和 0.1 + 0.2）也算相鄰

using Vec3 = std::array<double, 3>;

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }

// 機身 FRD → 世界 NED：Rz(yaw)·Ry(pitch)·Rx(roll)
class Rotation {
public:
    explicit Rotation(const Pose& p)
        : cr_(std::cos(p.roll)), sr_(std::sin(p.roll)), cp_(std::cos(p.pitch)),
          sp_(std::sin(p.pitch)), cy_(std::cos(p.yaw)), sy_(std::sin(p.yaw)) {}

    Vec3 operator*(const Vec3& b) const {
        const double y = cr_ * b[1] - sr_ * b[2];
        const double z = sr_ * b[1] + cr_ * b[2];
        const double forward = cp_ * b[0] + sp_ * z;
        const double down = cp_ * z - sp_ * b[0];
        return {cy_ * forward - sy_ * y, sy_ * forward + cy_ * y, down};
    }

private:
    double cr_, sr_, cp_, sp_, cy_, sy_;
};

// 射線上的一段，t 就是 z-depth
struct Interval {
    double begin;
    double end;
};

// 座標 (north, east, 離地高度)。離地高度是線性的，所以射線還是直線，地面變成 up = 0、
// 凸起和坑變成軸對齊的長方體。
struct Ray {
    Vec3 origin;
    Vec3 direction;
};

Interval intersect(Interval a, Interval b) {
    return {std::max(a.begin, b.begin), std::min(a.end, b.end)};
}

Interval slab(const Ray& ray, std::size_t axis, double min, double max) {
    const double o = ray.origin[axis];
    const double d = ray.direction[axis];
    if (d == 0.0) {
        return o >= min && o <= max ? Interval{-kInf, kInf} : Interval{kInf, -kInf};
    }
    const double t1 = (min - o) / d;
    const double t2 = (max - o) / d;
    return {std::min(t1, t2), std::max(t1, t2)};
}

Interval prism(const Ray& ray, const Footprint& f, double up_min, double up_max) {
    return intersect(intersect(slab(ray, 0, f.north_min_m, f.north_max_m),
                               slab(ray, 1, f.east_min_m, f.east_max_m)),
                     slab(ray, 2, up_min, up_max));
}

// 從地面進到坑裡之後，在各個坑裡一路往前走，走到沒有坑的地方就是壁或坑底
double through_pits(const Ray& ray, const std::vector<Pit>& pits, double t) {
    for (;;) {
        double farthest = t;
        for (const Pit& pit : pits) {
            const Interval inside = prism(ray, pit.footprint, -pit.depth_m, 0.0);
            if (inside.begin <= t + kTouchT) {
                farthest = std::max(farthest, inside.end);
            }
        }
        if (!(farthest > t)) {  // 寫成 ! 才不會被 NaN 卡住
            return t;
        }
        t = farthest;
    }
}

double hit(const Ray& ray, const Scene& scene) {
    double nearest = kInf;
    if (ray.direction[2] < 0.0) {
        nearest = through_pits(ray, scene.pits, -ray.origin[2] / ray.direction[2]);
    }
    for (const Block& block : scene.blocks) {
        const Interval inside = prism(ray, block.footprint, -kInf, block.height_m);
        if (inside.begin <= inside.end && inside.end >= 0.0) {
            nearest = std::min(nearest, inside.begin);  // 相機在凸起裡面時是負的，降採樣後是 0
        }
    }
    return nearest;
}

void require(bool ok, const char* problem) {
    if (!ok) {
        throw std::invalid_argument(std::string("synthesize_depth: ") + problem);
    }
}

bool has_area(const Footprint& f) {
    return f.north_min_m < f.north_max_m && f.east_min_m < f.east_max_m;
}

void check(const Scene& scene, const Options& options, const Intrinsics& depth) {
    for (const Block& block : scene.blocks) {
        require(has_area(block.footprint) && block.height_m > 0.0, "bad block");
    }
    for (const Pit& pit : scene.pits) {
        require(has_area(pit.footprint) && pit.depth_m > 0.0, "bad pit");
    }
    require(options.noise_stddev_m >= 0.0, "noise must be >= 0");
    for (const PixelRect& r : options.invalid) {
        require(r.u >= 0 && r.v >= 0 && r.width > 0 && r.height > 0 &&
                    r.width <= depth.width - r.u && r.height <= depth.height - r.v,
                "invalid region outside the image");
    }
}

std::vector<float> render(const Scene& scene, const Config::Camera& camera, const Pose& pose) {
    const Rotation rotation(pose);
    const auto up = [&scene](const Vec3& ned) {
        return -ned[2] - scene.rise_per_north * ned[0] - scene.rise_per_east * ned[1];
    };
    const Vec3 position = Vec3{pose.north_m, pose.east_m, pose.down_m} + rotation * camera.t_cam;
    Ray ray{{position[0], position[1], up(position)}, {}};
    require(ray.origin[2] > 0.0, "camera must be above the ground");

    const Intrinsics& k = camera.rgb;
    std::vector<float> depth_m;
    depth_m.reserve(static_cast<std::size_t>(k.width) * static_cast<std::size_t>(k.height));
    for (int v = 0; v < k.height; ++v) {
        for (int u = 0; u < k.width; ++u) {
            const double x = (u + 0.5 - k.cx) / k.fx;
            const double y = (v + 0.5 - k.cy) / k.fy;
            // 相機光學座標 (x, y, 1) → 機身 (−y, x, 1)（§5）；z 是 1，所以 t 就是 z-depth
            const Vec3 d = rotation * Vec3{-y, x, 1.0};
            ray.direction = {d[0], d[1], up(d)};
            depth_m.push_back(static_cast<float>(hit(ray, scene)));
        }
    }
    return depth_m;
}

void add_noise(std::vector<float>& depth_m, double stddev_m, std::uint32_t seed) {
    if (stddev_m == 0.0) {
        return;
    }
    std::mt19937 generator(seed);
    std::normal_distribution<double> noise(0.0, stddev_m);
    for (float& d : depth_m) {
        d = static_cast<float>(static_cast<double>(d) + noise(generator));
    }
}

}  // namespace

std::vector<std::uint16_t> synthesize_depth(const Scene& scene, const Config::Camera& camera,
                                            const Pose& pose, const Options& options) {
    check(scene, options, camera.depth);
    std::vector<float> depth_m = render(scene, camera, pose);
    add_noise(depth_m, options.noise_stddev_m, options.seed);
    std::vector<std::uint16_t> frame = downsample_depth(
        depth_m.data(), {camera.rgb.width, camera.rgb.height},
        {camera.depth.width, camera.depth.height}, camera.near_m, camera.far_m);
    for (const PixelRect& r : options.invalid) {
        for (int v = r.v; v < r.v + r.height; ++v) {
            const auto row = frame.begin() + static_cast<std::ptrdiff_t>(v) * camera.depth.width + r.u;
            std::fill(row, row + r.width, std::uint16_t{0});
        }
    }
    return frame;
}

}  // namespace dev_board::synthetic
