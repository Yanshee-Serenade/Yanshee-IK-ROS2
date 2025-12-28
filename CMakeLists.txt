cmake_minimum_required(VERSION 3.10)
project(ForwardKinematics LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成编译数据库
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找 Eigen3 库
find_package(Eigen3 REQUIRED)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 包含头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

# 添加主可执行文件
add_executable(forward_kinematics
    src/ForwardKinematics.cpp
    src/main.cpp
)

# 链接 Eigen3
target_link_libraries(forward_kinematics
    Eigen3::Eigen
)

# 添加动态库
add_library(forward_kinematics_dylib SHARED
    src/ForwardKinematics.cpp
    src/ForwardKinematicsInterface.cpp
)

# 设置动态库版本和属性
set_target_properties(forward_kinematics_dylib PROPERTIES
    VERSION 1.0
    SOVERSION 1
    OUTPUT_NAME "forward_kinematics"
    MACOSX_RPATH ON
)

# 链接动态库的依赖
target_link_libraries(forward_kinematics_dylib
    Eigen3::Eigen
)

# 添加测试可执行文件
add_executable(forward_kinematics_test
    src/ForwardKinematics.cpp
    test/ForwardKinematicsTest.cpp
)

# 链接测试可执行文件的依赖
target_link_libraries(forward_kinematics_test
    Eigen3::Eigen
)

# 启用测试（如果使用CTest）
enable_testing()
add_test(NAME ForwardKinematicsTest COMMAND forward_kinematics_test)