# 05: 投放前確認：飛行狀態、合併、收幀

Status: ready-for-agent
Blocked by: 04

**What to build:** 「確認」的完整行為：太傾斜、在移動、太高、太低時告訴操作員；10 幀合併成一個結果和最主要的原因；收不到足夠的幀時 1.5 秒內回答；收幀途中再按只回新的。

**Seams:** 判斷核心

- [ ] 每幀依序 `tilted` → `moving` → `too_high`／`too_low` → `evaluate`，照 dev_board_spec §7；`agl` 是空的就跳過高度那一步
- [ ] `too_low` 用四個角投影的判法，傾角和 `t_cam` 都算進去
- [ ] 平地 pitch 4°：1.5 m `too_low`、2.5 m 合格
- [ ] 平地 3 m 合格、7.5 m `too_high`；12、18 m `too_high`；25 m `invalid_depth`
- [ ] 傾角 6° `tilted`；水平速度 1 m/s `moving`、0 合格
- [ ] 合格 ≥ 8 幀 → `ok`；10 幀 8＋2 `hole` → `ok`；7＋3 `hole` → `hole`；4＋3 `hole`＋3 `slope` → `hole`（平手順序照 §7）
- [ ] `counts` 只列出現過的原因；`ok` 時沒有 `reason` 和 `counts`；`frame_timestamp` 用最後一幀
- [ ] 還沒算完收到更大的 `request_id`：只回新的；較小的不取代新的
- [ ] 深度 5 FPS：1.5 秒時回 `no_depth`，`counts` 是已收到的統計
- [ ] `pc-debug`、`pc-release`、`h1v1`＋qemu 測試通過

## Comments

2026-11-28（04 的 review 留下來的）：
- 「較小的不取代新的」也要涵蓋**同一個 `request_id` 重送**：現在 `on_check_request` 一律取代等待中的確認，同一個 id 重送會清掉已收的幀重收；server 每 300 ms 重送一次時可能永遠收不滿（04 review 的 AD1、SP2）。
- 先決定比較的對象：跟等待中的那筆比，還是跟歷史最大的比（§9.1 寫「比目前最大的還小」）。跟歷史最大的比時，server 重啟後 `request_id` 從 1 開始，核心要有辦法知道 `session_id` 換了，不然之後的請求都會被丟掉。
- 這條規則來自 §9.1 的 UDP 重送和去重。開發板 ↔ server 的協定還沒跟 server 組確認（server 組員 2026-11-23 把 Godot ↔ server 改成二進位），協定改了（例如沒有重送、沒有 `request_id`、改用 TCP）這一格要跟著改或拿掉。
- 「合格 ≥ 8 幀」那一格的三個例子（8＋2、7＋3、3＋3 平手兩種順序）04 已經寫成測試：`tests/test_drop_check.cpp` 的 `drop check: frames that disagree are merged`（04 的 Q1 決定）。那一格先不打勾：05 加上飛行狀態後，要確認 `tilted`、`moving` 等原因也照同一個規則合併。

2026-11-28（實作前的決定）：
- 比較的對象選 O1：只跟**等待中**的確認比。比它大就取代；比它小或相同（含同一個 id 重送）不理；沒有等待中的確認時，任何 id 都開始新的確認。已算完的請求重送、`session_id` 換了，由 M2 的去重和結果快取處理（M1 spec 的 Out of Scope）。理由：核心介面不用改；O2 要讓 `CheckRequest` 帶 `session_id`，等於先把還沒跟 server 組定的 §9.1 協定寫進核心。

2026-11-28（實作）：
- 所有格子的場景都符合預期，沒有調門檻或預期。「合格 ≥ 8 幀」那一格：`tilted`、`moving`、`too_high`、`too_low`、`invalid_depth` 也照同一個規則合併，平手順序每一對相鄰的都跑兩種順序（`flight state reasons are merged with the same rule`）。
- `too_low` 的四個角用 §8.5 的公式反算：`p_b = Rᵀ·(x − o.x, y − o.y, agl)`，連續座標 `0 ≤ u < 寬`、`0 ≤ v < 高` 才算在畫面裡。`agl < min_distance`（0.3 m）也是 `too_low`；預設視野下 0.3 m 時四個角早就在畫面外，所以測試用 fx = 8 的寬視野相機才看得到這條。
- 姿態和速度都內插到幀時間。`vx`、`vy` 不是有限值的位置整筆不收：內插出 NaN 時 `hypot > 0.3` 不成立，在移動也會判合格（06、08 已記下）。
- 1.5 秒的期限從收到請求的 `now` 算，每個事件開頭都檢查，到了就在那個事件的結果回 `no_depth`；期限到了才來的幀不算進去。`no_depth` 的 `frame_timestamp` 用那之前最後收到的幀，從沒收過深度幀是空的（`DropCheck::frame_timestamp` 改成 optional，§9.2）。
- 改到的舊測試：10.5 m 的「少於 3 點算 UNKNOWN」只是要每格像素少，投放高度上限放寬到 11 m，不然先判 `too_high`；04 的「收幀途中較大的 request_id」刪掉，05 的 subcase 測同一件事、還多檢查舊的沒有結果。
- 測不到、留著的程式（mutation 仍是綠的）：
  - 四個角的 `z_c > 0`：傾角 ≤ 5°、`agl` ≥ 0.3 m 時角都在相機前面。留著是為了不拿負的 `z_c` 去除，以及傾角上限放寬的設定。
  - `u < 寬` 跟 `u ≤ 寬`：連續座標剛好等於邊界的輸入做不出來。
  - `o = R·t_cam` 跟沒轉的 `t_cam`：傾角 ≤ 5° 時差 `t_cam·(1 − cos 5°)`，約 0.4%，場景分不出來。
- 時間：`pc-debug` 完整測試約 54 秒；`pc-release` 約 4 秒；`h1v1`＋qemu 約 61 秒。
- QA（獨立的 subagent）全部通過。觀察（不違反 spec）：`no_depth` 的 `counts` 只數失敗的幀，收到 7 幀都合格時是 `{}`，跟一幀都沒收到一樣，server 分不出來（§7 就是這樣寫）；PX4 比幀晚一點到時那一幀直接丟掉，要等 08 的「內插最多等 100 ms」。

2026-11-28（review 決定）：
- review 只有 P3。ST1（`DepthFrame` 寫法重複 11 份）、ST2（04 和 05 的重複測試）、ST3（命名）、ST4（只轉呼叫的 `rotate`）、SP1（速度壞掉的位置整筆不收，08 要知道）都已修正或記下。
- ST5（`expire(now)` 在 5 個事件函式各寫一次，新增事件時可能忘了）：選 O1，保持現狀，06 的 Comments 記下新增事件要加這行和對應的測試。理由：改成包裝函式也擋不住「新事件忘了包」，真正擋得住的是「every event answers no_depth」那個測試。
- AD1（server 在 1.5 秒內重啟時，新 session 從 1 開始的 `request_id` 會被等待中的確認擋掉，核心沒有方法讓 M2 清掉它）：選 O2，核心不改，記在 roadmap 的 M2。理由跟 O1 一樣：協定（有沒有 `session_id`、重送）還沒跟 server 組定，現在加 `reset_check()` 可能用不到或要改。
