#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "core/config.h"

namespace dev_board {

namespace terrain {
class Grid;
}

// 本機單調時鐘的「現在」。核心不讀時鐘，每個事件由呼叫端帶進來。
using Now = std::chrono::steady_clock::time_point;

// MAVLink ATTITUDE（rad）
struct Attitude {
    std::uint32_t time_boot_ms;
    double roll;
    double pitch;
    double yaw;
};

// MAVLink LOCAL_POSITION_NED（公尺、公尺/秒）
struct Position {
    std::uint32_t time_boot_ms;
    double north_m;
    double east_m;
    double down_m;
    double vx_mps;
    double vy_mps;
};

// 降採樣後的深度幀，逐列排列的公釐，0 = 無效（dev_board_spec.md §4）
struct DepthFrame {
    std::uint32_t frame_id;
    std::uint64_t timestamp_ms;  // Gazebo sim time，跟 time_boot_ms 同一個零點
    int width;
    int height;
    std::vector<std::uint16_t> depth_mm;
};

// 資料狀態，判法同 §9.2 heartbeat 的 state
enum class DataState { ok, no_depth, no_px4 };

// §9.2 `height` 的 payload，另外帶資料狀態
struct Height {
    std::optional<double> agl;  // 公尺；量不到或資料不能用時是空的
    DataState state;
};

// §9.2 `check_request` 的 payload
struct CheckRequest {
    std::uint32_t request_id;
};

// §7 drop_check 不合格的原因，依平手時的優先順序排。04 只做地面判斷，飛行狀態的原因在 05
enum class CheckReason { hole, rough, slope, invalid_depth };

// 一幀的判斷（診斷資料，不進 JSON）
struct FrameCheck {
    std::uint32_t frame_id;
    std::optional<CheckReason> reason;  // 空的 = 合格
};

// §9.2 `drop_check` 的 payload，另外附每一幀的判斷
struct DropCheck {
    std::uint32_t request_id;
    std::uint64_t frame_timestamp;          // 最後一幀的 sim time
    bool ok;
    std::optional<CheckReason> reason;      // 合格時是空的
    std::map<CheckReason, int> counts;      // 只列出現過的原因；合格時是空的
    std::vector<FrameCheck> frames;
};

// 一個事件這次產生的結果，可能都是空的
struct Results {
    std::optional<Height> height;
    std::optional<DropCheck> drop_check;
};

// 判斷核心：餵事件，拿結果。不開執行緒、不讀時鐘、不碰網路，也不保證可以多執行緒同時呼叫。
class DecisionCore {
public:
    explicit DecisionCore(const Config& config);

    Results on_attitude(const Attitude& attitude, Now now);
    Results on_position(const Position& position, Now now);
    // 尺寸跟 config.camera.depth 不同、或 depth_mm 的長度不是 width × height 時丟 std::invalid_argument
    Results on_depth(const DepthFrame& frame, Now now);
    // 投放前確認：用之後收到的 drop_check.frames 個不同 frame_id 的幀檢查正下方（§7）
    Results on_check_request(const CheckRequest& request, Now now);
    // 時間前進。資料狀態不是 ok 時回傳空的高度
    Results on_tick(Now now);

private:
    // §5 第 3～7 步：把一幀放進正下方的細格
    terrain::Grid project(const DepthFrame& frame, const Attitude& attitude) const;
    DataState state() const;
    std::optional<DropCheck> collect(const DepthFrame& frame, std::optional<CheckReason> reason);

    // 收幀中的確認
    struct PendingCheck {
        std::uint32_t request_id;
        std::vector<FrameCheck> frames;
    };

    Config config_;
    std::vector<double> ray_x_;  // 反投影查表 (u + 0.5 − cx) / fx
    std::vector<double> ray_y_;  // (v + 0.5 − cy) / fy
    bool depth_seen_ = false;
    std::vector<Attitude> attitudes_;  // 依 time_boot_ms 排好，留最近 px4_buffer_ms
    std::vector<Position> positions_;
    std::optional<PendingCheck> check_;
};

}  // namespace dev_board
