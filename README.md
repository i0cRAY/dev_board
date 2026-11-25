# XR 救災無人機：開發板

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

2026-11-24 用課程的工具鏈（GCC 10.4、glibc 2.35）編譯、在 qemu 跑過測試，還沒在板子上驗證。

從課程的 Docker image 把工具鏈複製到本機（約 813 MB，只要做一次；不要放進 repo）：

```bash
mkdir -p ~/toolchains && docker run --rm --network none --user "$(id -u):$(id -g)" -v ~/toolchains:/out --entrypoint cp nt98690-course:d6d4ddb0c -a /opt/arm/aarch64-ca73-linux-gnueabihf-10.4.0 /out/
```

編譯（`H1V1_TOOLCHAIN_PREFIX` 會存進 cache，之後不用再設；sysroot 工具鏈內建，見 [cmake/h1v1.cmake](cmake/h1v1.cmake)）：

```bash
H1V1_TOOLCHAIN_PREFIX=~/toolchains/aarch64-ca73-linux-gnueabihf-10.4.0/bin/aarch64-ca73-linux-gnu- cmake --preset h1v1
```

```bash
cmake --build --preset h1v1
```

在 PC 上用 qemu 跑測試（要裝 `qemu-user-static`）：

```bash
qemu-aarch64-static -L ~/toolchains/aarch64-ca73-linux-gnueabihf-10.4.0/aarch64-ca73-linux-gnu/sysroot build/h1v1/tests/dev_board_tests
```

## 目錄

| 路徑 | 內容 |
| ---- | ---- |
| `config/dev_board.json` | §11 參數、連線、相機內參 |
| `src/core/` | 不碰網路的函式庫 |
| `src/downsample/` | 深度降採樣，橋接程式和合成深度共用，不依賴其他程式 |
| `src/synthetic/` | 合成深度：不用 Gazebo 產生深度幀，測試和回放程式用 |
| `src/dev_board/` | 開發板主程式 |
| `tests/` | 單元測試 |
| `third_party/` | 第三方函式庫原始碼 |
