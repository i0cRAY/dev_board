# 07: 排除清單和換一個

Status: ready-for-agent
Blocked by: 06

**What to build:** 操作員按「換一個」拿到明顯不同的點；確認過不合格的地方 5 分鐘內不再被推薦。清單存世界座標，無人機移動、轉向後仍然對得上。

**Seams:** 判斷核心

- [ ] `exclude_last` 收到就先排除上一次推薦的點，再做前置檢查；被拒絕（例如 `tilted`）也算排除
- [ ] 上一次推薦的點 = 最近一次 `ok=true` 的推薦點；排除後清掉；中間失敗的推薦不影響它；沒有的話忽略 `exclude_last`，照常搜尋
- [ ] drop_check 的 `reason` 是 `slope`／`hole`／`rough` 時加入 Z 中心的世界座標（最後一幀），其他原因不加入
- [ ] 到期用 sim time：drop_check 用最後一幀的時間，`exclude_last` 用最後已知的 sim time；5 分鐘後移除
- [ ] 平地 7.5 m「推薦」再「換一個」：第二點離第一點 ≥ 2 m；5 分鐘後可再推薦第一點
- [ ] 7.5 m 推薦後傾斜時「換一個」（`tilted`），停穩再「推薦」：離原來那一點 ≥ 2 m
- [ ] 3 m 確認判 `hole` 後升到 7.5 m「推薦」：避開 2 m 內；確認判 `invalid_depth`、`moving` 等不會避開
- [ ] 推薦後機身移動並轉向 90°，被排除的仍是同一個世界位置
- [ ] 全部位置都被排除時 `excluded`，數量進 `counts`
- [ ] `pc-debug`、`pc-release`、`h1v1`＋qemu 測試通過
