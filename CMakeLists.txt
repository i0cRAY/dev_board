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

add_library(dev_board_core STATIC src/core/config.cpp)
target_include_directories(dev_board_core PUBLIC src)
target_link_libraries(dev_board_core PRIVATE nlohmann_json)

add_executable(dev_board src/dev_board/main.cpp)
target_link_libraries(dev_board PRIVATE dev_board_core)

enable_testing()
add_subdirectory(tests)
