# 01: 落地偏移、編譯選項、文件對齊

Status: ready-for-agent

**What to build:** 設定檔填量到的落地偏移，程式自己算出外圈寬（進位後再多一格）。PC 和 H1V1 的浮點結果不會因為乘加合併而不同。所有文件跟 [M1 spec](../spec.md) 的 Q1～Q8 和「行為上的補充」一致，其他組員不會照舊的數字做。

**Seams:** 設定檔

- [ ] `zone.ring_width_m` 改名為 `zone.landing_offset_m`（值 0.5 不變，標暫定），載入後的外圈寬是 `ceil(落地偏移/s)·s + s`
- [ ] 落地偏移 0.35 m → r 0.75 m、3 格；0.5 m → r 0.75 m；`Zw` 1.2 m → 1.5 m 不變；0、負數、舊欄位名稱都會報錯並指出欄位
- [ ] 所有目標都加 `-ffp-contract=off`
- [ ] `pc-debug`、`pc-release` 測試通過；`h1v1` preset 編得過、qemu 測試通過
- [ ] dev_board_spec.md 照 M1 spec「要改的文件」改完（§3、§5、§6.1、§6.2、§8.1～8.5、§8.8、§10～§14），r = 0.75 m 的數字（`too_low` 約 1.8 m、最小乾淨寬度 3 m、投放高度 ≥ 2.5 m）都更新；§10、§12 裡 r = 0.5 m 量的表格有標明
- [ ] §14 的場景跟 M1 spec 的場景表一致；`roll 5°` 改成 4.9°
- [ ] discussion.md：A2、P5 的投放高度 2.5～5 m、A4 的誤擋定義、r 的實驗說明、3 × 3 m 的坑
- [ ] roadmap.md：回放紀錄移到 M2、M1 的場景增加、M2 和 M3 註明複驗、M4 的 r 那一列、第 7 節的 r、滑動角移到「不在這次範圍」
- [ ] 用 GLOSSARY.md 的詞：落地偏移跟外圈寬（`r`）不混用
