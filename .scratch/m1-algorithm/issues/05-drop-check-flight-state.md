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
