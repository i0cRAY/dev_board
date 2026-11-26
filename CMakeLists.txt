cmake_minimum_required(VERSION 3.21)
project(dev_board LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DEV_BOARD_SANITIZE "Enable ASan and UBSan" OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
# aarch64 的 GCC 預設會把乘加合併成一個指令，門檻附近的結果會跟 PC 不同
add_compile_options(-ffp-contract=off)
if(DEV_BOARD_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# 第三方函式庫都只有標頭檔（third_party/README.md）
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json SYSTEM INTERFACE third_party)
add_library(doctest INTERFACE)
target_include_directories(doctest SYSTEM INTERFACE third_party)
add_library(mavlink INTERFACE)
target_include_directories(mavlink SYSTEM INTERFACE third_party)

add_library(dev_board_core STATIC src/core/config.cpp src/core/decision_core.cpp)
target_include_directories(dev_board_core PUBLIC src)
target_link_libraries(dev_board_core PRIVATE nlohmann_json)

# 橋接程式也會連結，所以不能依賴其他函式庫
add_library(depth_downsample STATIC src/downsample/downsample.cpp)
target_include_directories(depth_downsample PUBLIC src)

# 測試和回放程式用，開發板主程式不連結
add_library(synthetic_depth STATIC src/synthetic/synthetic_depth.cpp)
target_include_directories(synthetic_depth PUBLIC src)
target_link_libraries(synthetic_depth PRIVATE depth_downsample)

# 每幀 30 萬條射線，不最佳化的話場景測試會慢 5～10 倍；ASan、UBSan 照樣有開
target_compile_options(depth_downsample PRIVATE $<$<CONFIG:Debug>:-O2>)
target_compile_options(synthetic_depth PRIVATE $<$<CONFIG:Debug>:-O2>)

add_executable(dev_board src/dev_board/main.cpp)
target_link_libraries(dev_board PRIVATE dev_board_core)

enable_testing()
add_subdirectory(tests)
