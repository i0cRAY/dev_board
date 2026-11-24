#pragma once

#include <array>
#include <stdexcept>
#include <string>

namespace dev_board {

// 設定檔有錯。訊息包含欄位路徑，例如 "zone.width_m: must be > 0"。
struct ConfigError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct Intrinsics {
    int width;
    int height;
    double fx;
    double fy;
    double cx;
    double cy;
};

// 對應 dev_board_spec.md §11，另加連線和內參。
struct Config {
    struct Network {
        std::string depth_host;  // 橋接程式
        int depth_port;          // TCP，開發板主動連
        int mavlink_port;        // UDP，開發板收
        std::string server_host;
        int server_port;   // UDP，開發板 → server
        int command_port;  // UDP，server → 開發板
    } network;

    struct Camera {
        Intrinsics depth;  // 降採樣後
        Intrinsics rgb;
        double near_m;
        double far_m;
        std::array<double, 3> t_cam;  // 相機相對機身中心，機身 FRD
    } camera;

    struct Grid {
        double cell_size_m;  // s
    } grid;

    struct Zone {
        double width_m;       // Zw，已進位到 2s 的倍數
        double ring_width_m;  // r，已進位到 s 的倍數
        int width_cells;
        int ring_cells;
        double box_short_side_m;
    } zone;

    struct Evaluate {
        double hole_depth_m;  // D
        double max_slope_deg;
        double max_roughness_m;  // R
        double depth_noise_m;    // σz，降採樣後的深度
        double max_zone_invalid_ratio;
        double max_cell_invalid_ratio;
        int min_points_per_cell;
        double low_percentile;  // h20
        int refit_rounds;
    } evaluate;

    struct Limits {  // 推薦和 drop_check 共用
        double min_distance_m;
        double max_tilt_deg;
    } limits;

    struct Suggest {
        int neighbor_rings;  // k
        double max_distance_m;
        double exclude_radius_m;
        int exclude_expire_ms;  // sim time
    } suggest;

    struct DropCheck {
        double drop_max_height_m;
        double max_speed_mps;
        int frames;  // N
        int min_pass;
        int max_wait_ms;
    } drop_check;

    struct Timing {
        int data_max_age_ms;
        int px4_buffer_ms;
        int interpolation_max_ms;
        int tcp_recv_timeout_ms;
        int tcp_connect_timeout_ms;
    } timing;
};

// length 需要幾個 unit 才蓋得住（§10 的進位）。兩個都要 > 0。
int ceil_cells(double length, double unit);

Config parse_config(const std::string& text);
Config load_config(const std::string& path);

}  // namespace dev_board
