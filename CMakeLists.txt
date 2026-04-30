cmake_minimum_required(VERSION 3.10)
project(HikPikachu)

set(CMAKE_CXX_STANDARD 17)

# 1. 找 OpenCV和Eigen3（kalman用）
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)

# 2. 找海康 SDK (根据你电脑实际路径修改)
set(MVS_INCLUDE_DIR /opt/MVS/include)
set(MVS_LIBRARY_DIR /opt/MVS/lib/64)

# 3. 头文件
include_directories(
    ./include
    ${OpenCV_INCLUDE_DIRS}
    ${MVS_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenVINO_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# 4. 库文件
link_directories(${MVS_LIBRARY_DIR})

# 5. 编译源
file(GLOB SOURCES "src/*.cpp")
add_executable(app ${SOURCES})

# 6. 链接库
target_link_libraries(app
    ${OpenCV_LIBS}
    openvino::runtime
    yaml-cpp
    MvCameraControl
    pthread
    dl
)