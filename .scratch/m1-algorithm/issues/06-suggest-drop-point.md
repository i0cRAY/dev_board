# 06: 推薦投放點

Status: ready-for-agent
Blocked by: 04

**What to build:** 操作員按「推薦」，核心掃過畫面上所有位置，回傳最近而且周圍也合格的投放點和全部座標欄位；找不到時回傳原因和各原因的數量。

**Seams:** 判斷核心

- [ ] 前置檢查、搜尋範圍、選點、輸出欄位、找不到的原因照 dev_board_spec §8.2～8.6；margin 和距離都相同時用固定順序，距離用格數的整數平方比較
- [ ] 診斷資料有每個位置的中心、判斷、外圈外面一格是否都有點，不進 §9.2 的欄位
- [ ] 平地 7.5 m，roll 0°／3°／4.9°，雜訊 0 和 2 cm：誤擋 ≤ 5%（只算外圈外面一格也都有點的位置）
- [ ] 平地 7.4 m 水平、roll 3°、pitch 3°、`t_cam` 往右 0.15 m：搜尋到的位置集合跟測試用幾何算的相同（傾斜時排除離格線 5 mm 內像素影響的位置）；水平且 `t_cam` = 0 時前後左右對稱
- [ ] 機頭朝北、東、南、西，`north`、`east`、`down` 照公式；姿態時間夾住幀時間時 yaw 179° 到 −179° 的中間 → ±180°、170° 到 −170° 的 1/4 → 175°（從世界座標觀察）
- [ ] 穿過 (`pixel_u`, `pixel_v`) 中心的射線落點跟 Z 中心差小於一個像素的寬度；u、v 是捨去後的整數
- [ ] 正下方有 2 × 2 × 1 m 的坑：選到 margin 滿分中最近的；寬 2.75 m 的長條屋頂（高 10 m、離屋頂 7.5 m）：選到 margin 最大再比距離的
- [ ] 到處是坑：`terrain`，`counts` 四個原因都列、數量跟診斷一致
- [ ] 12、18 m `too_high`；25 m `invalid_depth` 且 `agl` 空的；傾角 6° `tilted`（`agl` 照常有值）；視野放不下 → `too_low`
- [ ] 正下方像素改成 0、旁邊平地：找得到，`height_diff_m` 是空的
- [ ] 同一份資料推薦兩次結果相同
- [ ] `pc-release` 記錄 7.5 m 平地的推薦時間，寫在這張 ticket 的 Comments，只當參考
- [ ] `pc-debug`、`pc-release`、`h1v1`＋qemu 測試通過

## Comments

2026-11-26（03 留下的事）：03 的 `on_attitude` 對 roll、pitch 超過 ±(π + 1e-6)（含 NaN、inf）的姿態整筆不收。06 開始用 yaw 和位置的值，要用同樣的上限檢查 yaw（機頭朝南時 yaw 在 ±π，float 的 π 比 double 的大一點，所以不能用剛好 π 當上限），位置、速度檢查是不是有限值，一樣整筆不收，並補測試（含 yaw = float 的 ±π 要收）。

2026-11-28（05 做了一部分）：05 的 `on_position` 已經檢查 `vx`、`vy` 是有限值（不是就整筆不收），也有測試。06 只要再補 north、east、down 和 yaw 的檢查。

2026-11-28（05 review 的 ST5）：確認的 1.5 秒期限在每個事件開頭用 `expire(now)` 檢查。06 新增 `on_suggest_request`（或其他事件）時也要加這一行，並在 `tests/test_drop_check.cpp` 的「drop check: every event answers no_depth once 1.5 s have passed」補一個 SUBCASE。
