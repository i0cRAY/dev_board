# 專案骨架設計

> 狀態：使用者 2026-11-24 同意。對應 [roadmap.md](../../../roadmap.md) M0 的「建專案」和「設定檔」，`§` 指 [dev_board_spec.md](../../../dev_board_spec.md)。

## 目的

讓 M1（演算法）和 M2（通訊）能直接開工：PC 上能編譯、能跑測試，§11 的參數都從設定檔讀。

- **不做**：M1、M2、M3 的任何功能，也不先開空檔案或定介面
- **失敗的樣子**：設定檔寫錯卻沒人發現；在沒有網路的電腦或交叉編譯時編不起來

## 做法

| 做法 | 取捨 | 結果 |
| ---- | ---- | ---- |
| 最小可執行骨架，函式庫原始碼放進 repo | repo 多約 8 MB；不用連網、不用安裝 | **採用** |
| CMake FetchContent 下載函式庫 | repo 小；Gazebo 組的電腦和交叉編譯都要連網 | 不採用 |
| 先開好 M1、M2 的所有模組 | 看起來完整；介面還沒用過就定死 | 不採用 |

## 內容

- C++17＋CMake（≥ 3.21）。preset：`pc-debug`（ASan＋UBSan）、`pc-release`、`h1v1`
- `h1v1` 的編譯器前綴和 sysroot 由環境變數給，SDK 到手前無法驗證
- 函式庫：nlohmann/json 3.12.0、doctest 2.5.3、mavlink/c_library_v2 commit `d2062a25`（沒有 release，只放 `common`、`standard`、`minimal`）
- `src/core/`：不碰網路的函式庫，目前只有 `Config`
- `src/dev_board/main.cpp`：讀設定、印出進位後的值就結束
- 設定檔 `config/dev_board.json`：JSON、允許註解。缺欄位、未知欄位、型別錯、超出範圍，都在啟動時報錯並指出欄位
- 啟動時進位（§10）：`r = ceil(r/s)·s`，`Zw = ceil(Zw/2s)·2s`

## 我決定的事（可以改）

- 暫定值：r = 0.5 m（§14 的基準）、R = 0.1 m（§12 小模擬）、σz = 0（Gazebo 預設無雜訊，且不扣雜訊偏保守）；TCP 15000、UDP 板→server 15001、UDP server→板 15002，IP 都是 localhost
- 不放 `hfov`：由內參算得出來，兩邊都放會互相矛盾。vfov 由 `2·atan(height / 2fy)` 得到
- 不放外圈滑動角：選做，還沒有程式會讀
- §9 的重送次數等通訊常數不在 §11，M2 再決定
