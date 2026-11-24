add_executable(dev_board_tests
  test_main.cpp
  test_config.cpp
  test_third_party.cpp)
target_link_libraries(dev_board_tests PRIVATE dev_board_core doctest mavlink nlohmann_json)

# 測試會讀 config/dev_board.json，從專案根目錄執行
add_test(NAME dev_board_tests COMMAND dev_board_tests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
