cmake_minimum_required(VERSION 3.16)
project(fusion_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(fusion
  src/assignment.cpp
  src/ekf.cpp
  src/experiment.cpp
  src/metrics.cpp
  src/sensors.cpp
  src/tracker.cpp
  src/traffic.cpp)
target_include_directories(fusion PUBLIC include)
target_compile_options(fusion PRIVATE -Wall -Wextra -Wpedantic -Wshadow)

add_executable(fusion_eval src/main.cpp)
target_link_libraries(fusion_eval PRIVATE fusion)
target_compile_options(fusion_eval PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(fusion_tests tests/test_main.cpp)
target_link_libraries(fusion_tests PRIVATE fusion)
target_compile_options(fusion_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME unit COMMAND fusion_tests)
