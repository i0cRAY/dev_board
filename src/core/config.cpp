#include "core/config.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <utility>

namespace dev_board {
namespace {

using nlohmann::json;

constexpr double kMaxLengthM = 100.0;
constexpr int kMaxMs = 3600 * 1000;

[[noreturn]] void fail(const std::string& path, const std::string& problem) {
    throw ConfigError(path + ": " + problem);
}

std::string to_text(double value) {
    std::ostringstream out;
    out << value;
    return out.str();
}

// 讀一個 JSON object，記住讀過的欄位，finish() 時剩下的就是未知欄位。
class Section {
public:
    Section(const json& object, std::string path) : object_(object), path_(std::move(path)) {
        if (!object_.is_object()) {
            fail(path_.empty() ? "config" : path_, "must be an object");
        }
    }

    Section section(const std::string& key) { return Section(get(key), path_of(key)); }

    double number(const std::string& key, double min, double max) {
        const json& value = get(key);
        if (!value.is_number()) {
            fail(path_of(key), "must be a number");
        }
        return in_range(key, value.get<double>(), min, max);
    }

    // min 不含在內
    double positive(const std::string& key, double max) {
        const double value = number(key, 0.0, max);
        if (value <= 0.0) {
            fail(path_of(key), "must be > 0");
        }
        return value;
    }

    int integer(const std::string& key, int min, int max) {
        const json& value = get(key);
        if (!value.is_number_integer()) {
            fail(path_of(key), "must be an integer");
        }
        // 先用 double 檢查範圍，轉成 int 時才不會溢位
        return static_cast<int>(in_range(key, value.get<double>(), min, max));
    }

    int port(const std::string& key) { return integer(key, 1, 65535); }

    std::string text(const std::string& key) {
        const json& value = get(key);
        if (!value.is_string() || value.get<std::string>().empty()) {
            fail(path_of(key), "must be a non-empty string");
        }
        return value.get<std::string>();
    }

    std::array<double, 3> vector3(const std::string& key) {
        const json& value = get(key);
        if (!value.is_array() || value.size() != 3) {
            fail(path_of(key), "must be an array of 3 numbers");
        }
        std::array<double, 3> result{};
        for (std::size_t i = 0; i < 3; ++i) {
            if (!value[i].is_number()) {
                fail(path_of(key), "must be an array of 3 numbers");
            }
            result[i] = in_range(key, value[i].get<double>(), -kMaxLengthM, kMaxLengthM);
        }
        return result;
    }

    void finish() const {
        for (const auto& item : object_.items()) {
            if (used_.count(item.key()) == 0) {
                fail(path_of(item.key()), "unknown field");
            }
        }
    }

    std::string path_of(const std::string& key) const {
        return path_.empty() ? key : path_ + "." + key;
    }

private:
    const json& get(const std::string& key) {
        const auto it = object_.find(key);
        if (it == object_.end()) {
            fail(path_of(key), "missing");
        }
        used_.insert(key);
        return *it;
    }

    double in_range(const std::string& key, double value, double min, double max) const {
        if (!(value >= min && value <= max)) {
            fail(path_of(key), "must be between " + to_text(min) + " and " + to_text(max));
        }
        return value;
    }

    const json& object_;
    std::string path_;
    std::set<std::string> used_;
};

Config::Network read_network(Section s) {
    Config::Network n{};
    n.depth_host = s.text("depth_host");
    n.depth_port = s.port("depth_port");
    n.mavlink_port = s.port("mavlink_port");
    n.server_host = s.text("server_host");
    n.server_port = s.port("server_port");
    n.command_port = s.port("command_port");
    if (n.command_port == n.mavlink_port) {
        fail(s.path_of("command_port"), "must differ from mavlink_port (both are UDP listeners)");
    }
    s.finish();
    return n;
}

Intrinsics read_intrinsics(Section s) {
    constexpr int kMaxPixels = 10000;
    Intrinsics i{};
    i.width = s.integer("width", 1, kMaxPixels);
    i.height = s.integer("height", 1, kMaxPixels);
    i.fx = s.positive("fx", 1e5);
    i.fy = s.positive("fy", 1e5);
    i.cx = s.number("cx", 0.0, i.width);
    i.cy = s.number("cy", 0.0, i.height);
    s.finish();
    return i;
}

Config::Camera read_camera(Section s) {
    Config::Camera c{};
    c.depth = read_intrinsics(s.section("depth"));
    c.rgb = read_intrinsics(s.section("rgb"));
    c.near_m = s.positive("near_m", kMaxLengthM);
    c.far_m = s.positive("far_m", 65.535);  // uint16 公釐的上限
    c.t_cam = s.vector3("t_cam");
    if (c.near_m >= c.far_m) {
        fail(s.path_of("near_m"), "must be less than far_m");
    }
    s.finish();
    return c;
}

Config::Grid read_grid(Section s) {
    Config::Grid g{};
    g.cell_size_m = s.number("cell_size_m", 0.01, kMaxLengthM);
    s.finish();
    return g;
}

Config::Zone read_zone(Section s, double cell) {
    Config::Zone z{};
    z.width_cells = 2 * ceil_cells(s.positive("width_m", kMaxLengthM), 2 * cell);
    z.ring_cells = ceil_cells(s.positive("ring_width_m", kMaxLengthM), cell);
    z.width_m = z.width_cells * cell;
    z.ring_width_m = z.ring_cells * cell;
    z.box_short_side_m = s.positive("box_short_side_m", kMaxLengthM);
    s.finish();
    return z;
}

Config::Evaluate read_evaluate(Section s) {
    Config::Evaluate e{};
    e.hole_depth_m = s.positive("hole_depth_m", kMaxLengthM);
    e.max_slope_deg = s.positive("max_slope_deg", 90.0);
    e.max_roughness_m = s.positive("max_roughness_m", kMaxLengthM);
    e.depth_noise_m = s.number("depth_noise_m", 0.0, kMaxLengthM);
    e.max_zone_invalid_ratio = s.number("max_zone_invalid_ratio", 0.0, 1.0);
    e.max_cell_invalid_ratio = s.number("max_cell_invalid_ratio", 0.0, 1.0);
    e.min_points_per_cell = s.integer("min_points_per_cell", 1, 10000);
    e.low_percentile = s.number("low_percentile", 0.0, 1.0);
    e.refit_rounds = s.integer("refit_rounds", 0, 100);
    s.finish();
    return e;
}

Config::Limits read_limits(Section s) {
    Config::Limits l{};
    l.min_distance_m = s.number("min_distance_m", 0.0, kMaxLengthM);
    l.max_tilt_deg = s.positive("max_tilt_deg", 90.0);
    s.finish();
    return l;
}

Config::Suggest read_suggest(Section s) {
    Config::Suggest g{};
    g.neighbor_rings = s.integer("neighbor_rings", 0, 100);
    g.max_distance_m = s.positive("max_distance_m", kMaxLengthM);
    g.exclude_radius_m = s.number("exclude_radius_m", 0.0, kMaxLengthM);
    g.exclude_expire_ms = s.integer("exclude_expire_ms", 1, kMaxMs);
    s.finish();
    return g;
}

Config::DropCheck read_drop_check(Section s) {
    Config::DropCheck d{};
    d.drop_max_height_m = s.positive("drop_max_height_m", kMaxLengthM);
    d.max_speed_mps = s.positive("max_speed_mps", 100.0);
    d.frames = s.integer("frames", 1, 1000);
    d.min_pass = s.integer("min_pass", 1, 1000);
    d.max_wait_ms = s.integer("max_wait_ms", 1, kMaxMs);
    if (d.min_pass > d.frames) {
        fail(s.path_of("min_pass"), "must not exceed frames");
    }
    s.finish();
    return d;
}

Config::Timing read_timing(Section s) {
    Config::Timing t{};
    t.data_max_age_ms = s.integer("data_max_age_ms", 1, kMaxMs);
    t.px4_buffer_ms = s.integer("px4_buffer_ms", 1, kMaxMs);
    t.interpolation_max_ms = s.integer("interpolation_max_ms", 1, kMaxMs);
    t.tcp_recv_timeout_ms = s.integer("tcp_recv_timeout_ms", 1, kMaxMs);
    t.tcp_connect_timeout_ms = s.integer("tcp_connect_timeout_ms", 1, kMaxMs);
    s.finish();
    return t;
}

// 深度是 RGB 降採樣來的，內參要用同樣的比例縮小（§4）。比例不寫死，§10 可能改傳 320×240。
void check_depth_intrinsics(const Intrinsics& depth, const Intrinsics& rgb) {
    constexpr double kTolerancePx = 0.1;  // 手抄時的四捨五入
    if (rgb.width % depth.width != 0) {
        fail("camera.depth.width", "must divide camera.rgb.width");
    }
    const int scale = rgb.width / depth.width;
    if (depth.height * scale != rgb.height) {
        fail("camera.depth.height", "must be camera.rgb.height / " + std::to_string(scale));
    }
    const auto check = [scale](const char* name, double depth_value, double rgb_value) {
        if (std::abs(depth_value - rgb_value / scale) > kTolerancePx) {
            fail(std::string("camera.depth.") + name, "must be camera.rgb." + std::string(name) +
                     " / " + std::to_string(scale) + " = " + to_text(rgb_value / scale));
        }
    };
    check("fx", depth.fx, rgb.fx);
    check("fy", depth.fy, rgb.fy);
    check("cx", depth.cx, rgb.cx);
    check("cy", depth.cy, rgb.cy);
}

// 相機範圍要比最近、最遠距離寬，才報得出 too_low、too_high（§11）。
void check_distances(const Config& c) {
    const double nearest_limit = std::min(c.suggest.max_distance_m, c.drop_check.drop_max_height_m);
    const double farthest_limit = std::max(c.suggest.max_distance_m, c.drop_check.drop_max_height_m);
    if (c.limits.min_distance_m >= nearest_limit) {
        fail("limits.min_distance_m",
             "must be less than suggest.max_distance_m and drop_check.drop_max_height_m");
    }
    if (c.camera.near_m >= c.limits.min_distance_m) {
        fail("camera.near_m", "must be less than limits.min_distance_m");
    }
    if (c.camera.far_m <= farthest_limit) {
        fail("camera.far_m",
             "must be greater than suggest.max_distance_m and drop_check.drop_max_height_m");
    }
}

}  // namespace

int ceil_cells(double length, double unit) {
    // 1.1 / 0.1 算出來是 11.000000000000002，不減一點會多進一格
    constexpr double kEpsilon = 1e-9;
    return static_cast<int>(std::ceil(length / unit - kEpsilon));
}

Config parse_config(const std::string& text) {
    json root;
    try {
        root = json::parse(text, nullptr, /*allow_exceptions=*/true, /*ignore_comments=*/true);
    } catch (const json::exception& error) {
        throw ConfigError(std::string("invalid JSON: ") + error.what());
    }

    Section s(root, "");
    Config c{};
    c.network = read_network(s.section("network"));
    c.camera = read_camera(s.section("camera"));
    c.grid = read_grid(s.section("grid"));
    c.zone = read_zone(s.section("zone"), c.grid.cell_size_m);
    c.evaluate = read_evaluate(s.section("evaluate"));
    c.limits = read_limits(s.section("limits"));
    c.suggest = read_suggest(s.section("suggest"));
    c.drop_check = read_drop_check(s.section("drop_check"));
    c.timing = read_timing(s.section("timing"));
    s.finish();
    check_depth_intrinsics(c.camera.depth, c.camera.rgb);
    check_distances(c);
    return c;
}

Config load_config(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    std::ostringstream text;
    if (!file || !(text << file.rdbuf())) {
        throw ConfigError(path + ": cannot read file, or it is empty");
    }
    try {
        return parse_config(text.str());
    } catch (const ConfigError& error) {
        throw ConfigError(path + ": " + error.what());
    }
}

}  // namespace dev_board
