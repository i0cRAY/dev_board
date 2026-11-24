# 第三方函式庫

原始碼直接放進 repo，建置不用連網。不要手動修改。

| 目錄 | 版本 | 來源 | SHA256／commit | 授權 |
| ---- | ---- | ---- | -------------- | ---- |
| `nlohmann/` | 3.12.0 | https://github.com/nlohmann/json release 的 `json.hpp` | `aaf127c04cb31c406e5b04a63f1ae89369fccde6d8fa7cdda1ed4f32dfc5de63` | MIT |
| `doctest/` | 2.5.3 | https://github.com/doctest/doctest tag `v2.5.3` 的 `doctest/doctest.h` | `cfd518a3ef90f67e1f3ba514df23fb3627437de1a2feeba78cf5062a40021421` | MIT |
| `mavlink/` | 無 release | https://github.com/mavlink/c_library_v2 的 `common`、`standard`、`minimal` 和最上層標頭，拿掉 `testsuite.h` | `d2062a25f05fc85213830427058401b0ec345430` | 該 repo 沒有授權檔，見 https://github.com/mavlink/mavlink |
