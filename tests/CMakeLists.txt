cmake_minimum_required(VERSION 3.10)
project(HikDriverTests)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
FetchContent_MakeAvailable(googletest)

find_package(OpenCV REQUIRED)

# Build the test executable.
# We compile HikDriver.cpp against the mock SDK header instead of the real one.
# The mock_mv_camera.cpp provides all SDK function implementations.
add_executable(test_hik_driver
    test_hik_driver.cpp
    mock_mv_camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/HikDriver.cpp
)

# IMPORTANT: The tests/ directory is listed BEFORE include/ so that
# HikDriver.cpp's  #include "MvCameraControl.h"  resolves to
# tests/MvCameraControl.h (the mock wrapper) instead of the real SDK header.
target_include_directories(test_hik_driver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}          # mock MvCameraControl.h wrapper
    ${CMAKE_CURRENT_SOURCE_DIR}/../include  # HikDriver.h
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(test_hik_driver
    GTest::gtest_main
    ${OpenCV_LIBS}
)

include(GoogleTest)
gtest_discover_tests(test_hik_driver)
