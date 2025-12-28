cmake_minimum_required(VERSION 3.10)
project(ForwardKinematics LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成编译数据库，这是关键！
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找 Eigen3 库
find_package(Eigen3 REQUIRED)

# 包含头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(forward_kinematics
    src/ForwardKinematics.cpp
    src/main.cpp
)

# 链接 Eigen3
target_link_libraries(forward_kinematics
    Eigen3::Eigen
)