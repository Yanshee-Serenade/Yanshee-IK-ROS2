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

add_executable(inverse_kinematics_demo
    src/ForwardKinematics.cpp
    src/InverseKinematics.cpp
    src/InverseKinematicsInterface.cpp
    src/main.cpp
)

target_link_libraries(inverse_kinematics_demo
    Eigen3::Eigen
)

# 添加正向运动学测试
add_executable(forward_kinematics_test
    src/ForwardKinematics.cpp
    test/ForwardKinematicsTest.cpp
)

# 链接正向运动学测试的依赖
target_link_libraries(forward_kinematics_test
    Eigen3::Eigen
)

# 添加逆运动学测试
add_executable(inverse_kinematics_test
    src/ForwardKinematics.cpp
    src/InverseKinematics.cpp
    test/InverseKinematicsTest.cpp
)

# 链接逆运动学测试的依赖
target_link_libraries(inverse_kinematics_test
    Eigen3::Eigen
)

# 修改动态库部分，支持跨平台
add_library(yanshee_kinematics SHARED
    src/ForwardKinematics.cpp
    src/ForwardKinematicsInterface.cpp
    src/InverseKinematics.cpp
    src/InverseKinematicsInterface.cpp
)

# 根据平台设置不同的输出名称
if(APPLE)
    set_target_properties(yanshee_kinematics PROPERTIES
        OUTPUT_NAME "yanshee_kinematics"
        SUFFIX ".dylib"
        MACOSX_RPATH ON
    )
elseif(UNIX AND NOT APPLE)
    set_target_properties(yanshee_kinematics PROPERTIES
        OUTPUT_NAME "yanshee_kinematics"
        SUFFIX ".so"
    )
endif()

target_link_libraries(yanshee_kinematics
    Eigen3::Eigen
)

# 启用测试（如果使用CTest）
enable_testing()
add_test(NAME ForwardKinematicsTest COMMAND forward_kinematics_test)
add_test(NAME InverseKinematicsTest COMMAND inverse_kinematics_test)

# 添加性能测试
add_executable(performance_test
    src/ForwardKinematics.cpp
    src/InverseKinematics.cpp
    test/PerformanceTest.cpp
)

target_link_libraries(performance_test
    Eigen3::Eigen
)

# 可选：添加到测试套件
add_test(NAME PerformanceTest COMMAND performance_test)