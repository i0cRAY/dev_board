# 專案骨架 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 PC 上能編譯、測試，並從設定檔讀入 §11 參數的 C++ 專案。

**Architecture:** `dev_board_core` 靜態庫（不碰網路）＋ `dev_board` 執行檔＋ doctest 測試。第三方函式庫以原始碼放在 `third_party/`。

**Tech Stack:** C++17、CMake、nlohmann/json、doctest、c_library_v2

**Spec:** [設計](../specs/2026-11-24-project-skeleton-design.md)、[dev_board_spec.md](../../../dev_board_spec.md) §10、§11、§14

## Global Constraints

- C++17；建置時不連網；函式庫以 `SYSTEM` include
- §11 參數不寫死在程式裡，未知欄位要報錯
- 不加 M1～M3 的程式或空檔案；不 commit（使用者沒要求）

## Review Focus

- `1.1 / 0.1` 這種浮點誤差：進位不能多一格 → Task 2
- 整數欄位填 3.7 或 `true`：報錯，不能默默截斷 → Task 2
- 打錯的欄位名：報錯並指出完整路徑 → Task 2
- 兩個 UDP 接收 port 填一樣：啟動時報錯 → Task 2
- 設定檔不存在或 JSON 語法錯：印出檔名、exit 1，不能 crash → Task 3

### Task 1: 建置系統與第三方函式庫

**Files:** `CMakeLists.txt`、`CMakePresets.json`、`cmake/h1v1.cmake`、`third_party/*`、`tests/{CMakeLists.txt,test_main.cpp,test_third_party.cpp}`、`.gitignore`

**Interfaces:** Produces: CMake target `nlohmann_json`、`doctest`、`mavlink`（INTERFACE）

- [x] 抓鎖定版本，記錄來源與 SHA256 到 `third_party/README.md`，保留授權檔
- [x] 測試：MAVLink `ATTITUDE`、`LOCAL_POSITION_NED` 打包後逐 byte 解析回原值；JSON `null` 與註解可解析
- [x] `cmake --preset pc-debug && cmake --build --preset pc-debug && ctest --preset pc-debug` 通過

### Task 2: Config

**Files:** `src/core/config.{h,cpp}`、`tests/test_config.cpp`

**Interfaces:** Produces: `Config parse_config(const std::string& text)`、`Config load_config(const std::string& path)`、`struct ConfigError : std::runtime_error`、`int ceil_cells(double length, double unit)`

- [x] 先寫測試並確認失敗：
  - r = 0.35、Zw = 1.2、s = 0.25 → 0.5、1.5（§14）；r = 0.5、Zw = 1 不變；r = 1.1、s = 0.1 → 11 格
  - 缺欄位、未知欄位、整數填 3.7、`true` 當數字、port 0／65536、`min_pass` > `frames`、near ≥ far、UDP port 相同 → `ConfigError`，訊息含欄位路徑
  - repo 內的 `config/dev_board.json` 可讀，值等於 §11
- [x] 實作到全部通過

### Task 3: 主程式

**Files:** `config/dev_board.json`、`src/dev_board/main.cpp`、`README.md`

- [x] `dev_board <config>`：成功印進位後的 r、Zw 並 exit 0；設定錯誤印到 stderr 並 exit 1；參數數量錯誤 exit 2
- [x] 手動驗證 pc-debug、pc-release、錯誤情況；`h1v1` 用假前綴驗證工具鏈檔的機制
