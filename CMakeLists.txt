cmake_minimum_required(VERSION 3.10)
project(yanshee_ik LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成编译数据库
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ROS2 build tools
find_package(ament_cmake REQUIRED)

# 查找 Eigen3 库
find_package(Eigen3 REQUIRED)

# 在find_package之后添加
# 检查编译器类型并设置最高优化
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  # GCC的极致优化
  add_compile_options(-Ofast)           # 比-O3更激进
  add_compile_options(-march=native)
  add_compile_options(-mtune=native)
  add_compile_options(-ffast-math)
  add_compile_options(-flto)
  add_compile_options(-funroll-loops)
  add_compile_options(-fomit-frame-pointer)
  add_compile_options(-fno-signed-zeros)
  add_compile_options(-fno-trapping-math)
  add_compile_options(-fassociative-math)
  add_link_options(-flto)
  
  # 特定于性能的警告
  add_compile_options(-Wno-maybe-uninitialized)
  
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang")
  # Clang的极致优化
  add_compile_options(-O3)
  add_compile_options(-march=native)
  add_compile_options(-ffast-math)
  add_compile_options(-flto=thin)       # Clang的薄LTO
  add_compile_options(-funroll-loops)
  add_compile_options(-fvectorize)
  add_compile_options(-fslp-vectorize)
  add_link_options(-flto=thin)
  
elseif(MSVC)
  # MSVC的极致优化
  add_compile_options(/Ox)              # 最大优化
  add_compile_options(/GL)              # 全程序优化
  add_compile_options(/arch:AVX2)       # 使用AVX2
  add_compile_options(/fp:fast)
  add_compile_options(/Ob2)             # 内联任何合适的函数
  add_compile_options(/Oi)              # 生成内部函数
  add_compile_options(/Ot)              # 优先速度
  add_link_options(/LTCG)               # 链接时代码生成
  add_link_options(/OPT:REF)            # 消除未使用的函数和数据
  add_link_options(/OPT:ICF)            # 相同的COMDAT折叠
  
else()
  message(WARNING "Unknown compiler ${CMAKE_CXX_COMPILER_ID}, using default -O3")
  add_compile_options(-O3)
  
endif()

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 包含头文件目录
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

# 导出动态库
add_library(yanshee_kinematics SHARED
    src/ForwardArmKinematics.cpp
    src/ForwardArmKinematicsInterface.cpp
    src/InverseArmKinematics.cpp
    src/InverseArmKinematicsInterface.cpp
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
add_executable(inverse_performance_test
    src/ForwardKinematics.cpp
    src/InverseKinematics.cpp
    test/InversePerformanceTest.cpp
)

target_link_libraries(inverse_performance_test
    Eigen3::Eigen
)

# 添加性能测试
add_executable(forward_performance_test
    src/ForwardKinematics.cpp
    src/InverseKinematics.cpp
    test/ForwardPerformanceTest.cpp
)

target_link_libraries(forward_performance_test
    Eigen3::Eigen
)

# 添加性能测试
add_executable(inverse_arm_performance_test
    src/ForwardArmKinematics.cpp
    src/InverseArmKinematics.cpp
    test/InverseArmPerformanceTest.cpp
)

target_link_libraries(inverse_arm_performance_test
    Eigen3::Eigen
)

# Install the shared library
install(TARGETS yanshee_kinematics
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)

# ROS2 package hooks
ament_package()