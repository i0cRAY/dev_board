# XR 救災無人機：開發板（H1V1）

規格 [dev_board_spec.md](dev_board_spec.md)、進度 [roadmap.md](roadmap.md)、跨組問題 [discussion.md](discussion.md)。

## 編譯與測試（PC）

需要 CMake ≥ 3.21、Ninja、支援 C++17 的 GCC 或 Clang。不用連網。

```bash
cmake --preset pc-debug
```

```bash
cmake --build --preset pc-debug
```

```bash
ctest --preset pc-debug
```

```bash
build/pc-debug/dev_board config/dev_board.json
```

`pc-debug` 開 ASan＋UBSan；量時間用 `pc-release`。

## 交叉編譯（H1V1）

還沒用真的 NT98690 SDK 驗證過。先設 `H1V1_TOOLCHAIN_PREFIX`（必要時加 `H1V1_SYSROOT`），再用 `h1v1` preset，見 [cmake/h1v1.cmake](cmake/h1v1.cmake)。

## 目錄

| 路徑 | 內容 |
| ---- | ---- |
| `config/dev_board.json` | §11 參數、連線、相機內參 |
| `src/core/` | 不碰網路的函式庫 |
| `src/dev_board/` | 開發板主程式 |
| `tests/` | 單元測試 |
| `third_party/` | 第三方函式庫原始碼 |
