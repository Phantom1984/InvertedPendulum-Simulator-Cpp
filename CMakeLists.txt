# 设置最低版本的 CMake
cmake_minimum_required(VERSION 3.10)

# 设置项目名称和使用的编程语言
project(InvertedPendulumControl CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)

# 设置头文件目录
include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories("D:/Program Files/eigen-3.4.0")

# 指定源文件
set(SOURCES
    src/main.cpp
    src/pid.cpp
    src/pendulum.cpp
    src/lqr.cpp
)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})
