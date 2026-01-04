# Yanshee Inverse Kinematics

## Usage

1. Build with CMake
2. [Optional] Run tests and benchmarks with CMake
3. Copy dynamic library to `Plugins` folder in Unity 
  - GNU/Linux: `build/lib/libyanshee_kinematics.so`
  - MacOS: `build/lib/libyanshee_kinematics.dylib` 

## Introduction (zh-cn)

逆运动学求解是人形机器人控制中的核心计算任务，其性能直接决定了机器人动作的实时性与精确性。为确保求解效率与精度满足高频率控制的需求（求解耗时低于1毫秒），本研究采用C++语言开发核心求解算法，并通过CMake构建系统将其编译为平台无关的动态链接库。该模块封装了复杂的运动学约束与数值求解过程，为上层控制逻辑提供高性能的调用接口。

该模块的实现遵循清晰的层次化设计，主要包含两个核心类：ForwardKinematics（正向运动学）和InverseKinematics（逆运动学）。ForwardKinematics类负责建立机器人腿部的精确运动学模型。在初始化时，根据传入的腿类型（左腿或右腿），它会加载对应腿部的所有几何参数，包括各关节的初始位置、旋转轴向量以及关节之间的固定连杆长度。该类提供的关键方法computeAllJointPositions能够根据给定的四个关节角度（θ₁, θ₂, θ₃, θ₄），利用标准的D-H参数法或几何变换，依次计算出从髋部到脚踝所有中间关节以及末端执行器的三维坐标。同时，它还负责管理各关节的物理运动范围限制，为逆解过程提供可行性验证。

InverseKinematics类则封装了从三维目标脚部位置反推关节角度的复杂求解算法。由于人形机器人腿部通常存在运动学耦合（例如本模型中θ₂ = θ₃ + θ₄，且θ₅ = -θ₁），解析解难以直接获得或存在多解。因此，本模块创新性地采用了一种混合求解策略，结合了解析推导与高效数值搜索。算法首先利用几何关系，从目标位置的X、Z坐标中解析出关节θ₁的潜在候选解，这大大缩小了搜索空间。然后，针对每个候选的θ₁，算法在一个由θ₃和θ₄构成的二维参数空间内，执行一种自适应精度的网格搜索。搜索过程分为两步：首先进行低密度的全局粗搜，定位误差较小的区域；然后在该区域附近进行高密度的局部细搜，以精确定位最优解。这种方法在保证求解精度的同时（平均位置误差小于1毫米），极大地提升了计算效率。

为方便Unity（C#）等高级语言环境调用，模块通过ForwardKinematicsInterface.h和InverseKinematicsInterface.h头文件暴露了纯C语言接口。这些接口函数（如inverse_kinematics）内部实例化运动学对象并调用对应的求解方法，处理了内存分配与数据格式转换的细节，最终将求解得到的关节角度（弧度值）通过输出数组返回。性能测试报告显示，在标准网格密度（Grid Size=12）下，该模块在Apple M4平台上的平均求解时间仅为0.16毫秒（右腿）和0.11毫秒（左腿），完全满足实时控制需求。通过CMake脚本，该模块可被编译为libyanshee_kinematics.so（Linux）或libyanshee_kinematics.dylib（macOS）等动态库，方便集成到不同的机器人软件系统中。

## Performance Test Results

Running on MacBook Air M4:

```
=========================================
  Inverse Kinematics Performance Tests
       Grid Size = 12, 100 tests
=========================================

=== Right Leg Performance Test (Grid Size = 12) ===
Initialized right leg kinematics
Link lengths (all joints at 0 position):
  Link 1-2: 0.044095
  Link 2-3: 0.0500281
  Link 3-4: 0.0599502
  Link 4-5: 0.0375876


Test 1:
  Original angles: -0.5649, 0.0192, -0.3580, 0.3772
  Solved angles:   -0.5649, 0.0208, -0.3706, 0.3914
  Position error:  0.8801 mm
  Angle error:     0.5449 °
  Execution time:  0.1800 ms

Test 2:
  Original angles: 0.9938, 0.9948, 0.0442, 0.9506
  Solved angles:   0.9938, 1.9397, 1.7664, 0.1733
  Position error:  0.6344 mm
  Angle error:     60.5208 °
  Execution time:  0.1940 ms

Test 3:
  Original angles: -0.3266, -0.3196, 0.5204, -0.8400
  Solved angles:   -0.3266, -0.3281, 0.5017, -0.8297
  Position error:  0.2713 mm
  Angle error:     0.6583 °
  Execution time:  0.2010 ms

Test 4:
  Original angles: -0.1879, -0.6711, 0.3077, -0.9788
  Solved angles:   -0.1879, -0.0228, 1.5047, -1.5275
  Position error:  0.7714 mm
  Angle error:     42.0477 °
  Execution time:  0.1970 ms

Test 5:
  Original angles: -1.1664, 1.0691, 0.2506, 0.8185
  Solved angles:   -1.1664, 1.0675, 0.2400, 0.8275
  Position error:  0.4791 mm
  Angle error:     0.4026 °
  Execution time:  0.1910 ms

=== Right Leg Various Positions Test ===
Initialized right leg kinematics
Link lengths (all joints at 0 position):
  Link 1-2: 0.0441
  Link 2-3: 0.0500
  Link 3-4: 0.0600
  Link 4-5: 0.0376

Test Case           Time(ms)    PosErr(mm)  AngErr(°)  Valid     
------------------------------------------------------------------
Zero Position       0.13        1.001       0.867       ✓       
Extended            0.20        0.785       47.388      ✓       
Bent                0.23        1.061       0.638       ✓       
Max Theta1          0.24        0.755       1.209       ✓       
Min Theta1          0.21        0.755       1.209       ✓       
Complex 1           0.15        0.389       42.334      ✓       
Complex 2           0.11        1.260       1.433       ✓       
------------------------------------------------------------------
Average time: 0.18 ms
Success rate: 7/7 (100.00%)

=== Left Leg Performance Test (Grid Size = 12) ===
Initialized left leg kinematics
Link lengths (all joints at 0 position):
  Link 1-2: 0.04
  Link 2-3: 0.05
  Link 3-4: 0.06
  Link 4-5: 0.04


Test 1:
  Original angles: 0.4251, -0.8610, 0.2694, -1.1305
  Solved angles:   0.4251, -1.9833, -1.8100, -0.1733
  Position error:  7.4772 mm
  Angle error:     73.0360 °
  Execution time:  0.1030 ms

Test 2:
  Original angles: 1.0090, 0.1658, -1.5197, 1.6855
  Solved angles:   1.0090, 0.8514, -0.2836, 1.1350
  Position error:  1.0780 mm
  Angle error:     43.4574 °
  Execution time:  0.0960 ms

Test 3:
  Original angles: -1.2646, -0.5244, 0.2937, -0.8181
  Solved angles:   -1.2646, -0.5442, 0.2833, -0.8275
  Position error:  1.2975 mm
  Angle error:     0.6946 °
  Execution time:  0.1270 ms

Test 4:
  Original angles: 0.5192, -1.7678, -1.7383, -0.0296
  Solved angles:   0.5192, -0.8494, -0.0656, -0.7839
  Position error:  0.2645 mm
  Angle error:     58.7837 °
  Execution time:  0.1160 ms

Test 5:
  Original angles: 0.5197, 0.0681, -0.2401, 0.3082
  Solved angles:   0.5197, -0.6750, -1.5919, 0.9169
  Position error:  0.5900 mm
  Angle error:     47.5094 °
  Execution time:  0.1250 ms

=== Left Leg Various Positions Test ===
Initialized left leg kinematics
Link lengths (all joints at 0 position):
  Link 1-2: 0.0438
  Link 2-3: 0.0497
  Link 3-4: 0.0595
  Link 4-5: 0.0387

Test Case           Time(ms)    PosErr(mm)  AngErr(°)  Valid     
------------------------------------------------------------------
Zero Position       0.10        0.996       0.867       ✓       
Extended            0.10        0.450       0.226       ✓       
Bent                0.10        0.731       35.766      ✓       
Max Theta1          0.09        0.963       1.220       ✓       
Min Theta1          0.09        0.963       1.220       ✓       
Complex 1           0.09        1.636       0.876       ✓       
Complex 2           0.09        0.738       48.924      ✓       
------------------------------------------------------------------
Average time: 0.09 ms
Success rate: 7/7 (100.00%)

================================================
           PERFORMANCE TEST REPORT
          (Grid Size = 12, 100 tests)
================================================

Metric                   Right Leg      Left Leg       Unit           
----------------------------------------------------------------------
Avg Execution Time       0.16           0.11           ms             
Min Execution Time       0.10           0.08           ms             
Max Execution Time       0.36           0.23           ms             
Time Std Dev             0.04           0.02           ms             
----------------------------------------------------------------------
Avg Position Error       0.735          0.817          mm             
Min Position Error       0.044          0.072          mm             
Max Position Error       5.527          7.477          mm             
----------------------------------------------------------------------
Avg Angle Error          8.609          12.682         °             
Max Angle Error          71.753         73.036         °             
----------------------------------------------------------------------
Success Count            100            100                           
Success Rate             100.0          %100.0          %               
----------------------------------------------------------------------

PERFORMANCE EVALUATION:
1. Real-time Capability:
   ✓ Suitable for real-time control (< 10ms)

2. Accuracy Assessment:
   ✓ High accuracy (< 1mm avg error)

3. Reliability:
   ✓ High reliability (> 90% success rate)
Results saved to right_leg_performance.csv
Results saved to left_leg_performance.csv

=========================================
  Performance tests completed successfully!
=========================================
```





# 逆运动学模块关键组件详解

## 1. 模块架构设计

### 1.1 层次化架构
```
├── 应用层 (C# Unity)
│   └── RobotSolver.cs (封装器，负责坐标转换)
├── 接口层 (C ABI)
│   ├── ForwardKinematicsInterface.h
│   └── InverseKinematicsInterface.h
├── 核心算法层 (C++)
│   ├── ForwardKinematics.cpp
│   └── InverseKinematics.cpp
└── 数学基础层
    └── Eigen 线性代数库
```

### 1.2 数据流设计
```
目标位置(Unity坐标系) 
    → RobotSolver坐标转换 
    → C接口函数调用 
    → 混合求解算法 
    → 关节角度(弧度) 
    → Unity驱动关节
```

## 2. 关键算法

### 2.1 混合求解策略
本模块的核心创新在于结合了**解析推导**与**数值搜索**的优势：

#### 2.1.1 θ₁解析推导
```cpp
// 关键几何关系
const double delta_x = targetPos.x() - p1.x();
const double delta_z = targetPos.z() - p1.z();
const double r = sqrt(delta_x * delta_x + delta_z * delta_z);
const double phi = atan2(delta_z, delta_x);
const double alpha = acos(x_rel_const / r);

// 两个解析解
double theta1_1 = -phi + alpha;  // 第一种构型
double theta1_2 = -phi - alpha;  // 第二种构型
```

**算法特点**：
- 直接从目标位置的X-Z平面投影计算θ₁
- 有效将5维搜索问题降为2维
- 物理意义明确，对应机器人腿部的两种可能弯曲方向

#### 2.1.2 自适应网格搜索
采用**两级搜索策略**优化计算效率：

```cpp
// 第一级：粗网格搜索（全局探索）
double theta3_step_coarse = (theta3_upper - theta3_lower) / coarseDensity;
double theta4_step_coarse = (theta4_upper - theta4_lower) / coarseDensity;

// 第二级：细网格搜索（局部优化）
double theta3_min_fine = max(theta3_lower, best_theta3_coarse - search_radius);
double theta3_max_fine = min(theta3_upper, best_theta3_coarse + search_radius);
```

**搜索参数**：
- 粗搜索密度：gridDensity（默认12）
- 细搜索密度：gridDensity
- 搜索半径：最大网格步长
- 误差阈值：1e-2（10毫米）

### 2.2 运动学约束处理
机器人腿部存在特殊的几何约束，算法需要正确处理：

#### 2.2.1 耦合关节关系
```cpp
double theta2 = theta3 + theta4;  // θ₂ = θ₃ + θ₄ 约束
double theta5 = -theta1;          // θ₅ = -θ₁ 约束
```

#### 2.2.2 关节限位检查
```cpp
bool checkJointLimits(double theta1, double theta2, 
                     double theta3, double theta4) const {
    // 检查所有关节角度是否在物理允许范围内
    return (theta1 >= limits.theta1_lower && theta1 <= limits.theta1_upper) &&
           (theta2 >= limits.theta2_lower && theta2 <= limits.theta2_upper) &&
           (theta3 >= limits.theta3_lower && theta3 <= limits.theta3_upper) &&
           (theta4 >= limits.theta4_lower && theta4 <= limits.theta4_upper);
}
```

## 3. 核心类与函数

### 3.1 ForwardKinematics类
**职责**：建立精确的运动学模型，计算正运动学

#### 关键方法：
- `computeAllJointPositions()`：计算所有关节位置
- `validateLinkLengths()`：验证连杆长度恒定性
- `getJointLimits()`：获取关节运动范围

#### 关键技术：
```cpp
// 创建齐次变换矩阵
Eigen::Matrix4d createTransform(const Eigen::Vector3d& translation,
                               const Eigen::Vector3d& axis,
                               double angle) const {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    // 旋转部分
    Eigen::AngleAxisd rotation(angle, axis.normalized());
    T.block<3,3>(0,0) = rotation.toRotationMatrix();
    // 平移部分
    T.block<3,1>(0,3) = translation;
    return T;
}
```

### 3.2 InverseKinematics类
**职责**：实现逆运动学求解算法

#### 关键方法：
- `solve()`：主求解函数，整合整个求解流程
- `computeTheta1Solutions()`：计算θ₁解析解
- `gridSearchForTheta3Theta4()`：执行网格搜索

#### 求解流程：
```
1. 输入目标位置 [x, y, z]
2. 计算θ₁的所有解析解
   ↓
3. 对每个θ₁候选解：
   - 检查关节限位
   - 执行二级网格搜索(θ₃, θ₄)
     ├─ 粗搜索：全局定位
     └─ 细搜索：局部优化
   ↓
4. 选择误差最小的有效解
5. 计算θ₂ = θ₃ + θ₄, θ₅ = -θ₁
6. 返回所有关节角度
```

### 3.3 C接口函数
**职责**：提供跨语言调用接口

#### 关键接口：
```c
// 正运动学接口
void compute_last_joint_position(int is_right_leg, double theta1, double theta2,
                                double theta3, double theta4, double* result);

// 逆运动学主接口
int inverse_kinematics(int is_right_leg, double target_x, double target_y, double target_z,
                      int grid_density, double* angles, double* error);
```

**接口设计特点**：
- 使用纯C ABI，确保跨语言兼容性
- 指针参数用于输出结果，避免返回值复杂化
- 明确的错误返回码（1=成功，0=失败）

## 4. 性能优化技术

### 4.1 计算优化策略

#### 4.1.1 提前终止
```cpp
// 在网格搜索中设置最大允许误差
if (error <= maxAllowedError) {
    // 找到足够精确的解，提前返回
    bestSolution.valid = true;
    return bestSolution;
}
```

#### 4.1.2 记忆化技术
- 关节限位检查结果缓存
- 常用变换矩阵预计算

### 4.2 内存管理优化
- 使用Eigen库的静态内存分配
- 避免动态内存分配热点
- 结果数组复用机制

## 5. 坐标系处理

### 5.1 坐标系转换
模块内部需要处理多种坐标系：

```cpp
// Unity到DLL坐标系转换
// Unity: 右(X), 上(Y), 前(Z)
// DLL:   前(X), 左(Y), 上(Z)

double target_x_dll = targetPositionUnity.x();      // X不变
double target_y_dll = -targetPositionUnity.z();     // Unity Z → DLL -Y
double target_z_dll = targetPositionUnity.y();      // Unity Y → DLL Z
```

### 5.2 角度规范化
```cpp
auto normalizeAngle = [](double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
};
```

## 6. 测试与验证

### 6.1 验证机制
- **连杆长度验证**：确保正逆运动学一致性
- **关节限位验证**：防止超出物理限制
- **误差统计分析**：监控求解精度变化

### 6.2 性能指标
基于实测数据（M4芯片，gridSize=12）：
- **求解时间**：平均0.13-0.18ms
- **位置误差**：平均0.7-0.8mm
- **成功率**：100% (测试样本内)
- **实时性**：满足1000Hz控制频率需求

## 7. 扩展性与维护性

### 7.1 参数化设计
- 网格密度可配置
- 误差阈值可调整
- 关节限位易修改

### 7.2 模块化接口
- 清晰的API边界
- 独立的测试套件
- 完整的性能分析工具

## 总结

本逆运动学模块通过**混合求解策略**实现了高性能的实时计算，其核心优势在于：
1. **高效性**：亚毫秒级求解时间
2. **精确性**：毫米级位置误差
3. **鲁棒性**：100%求解成功率
4. **可扩展性**：清晰的架构设计

该模块为人形机器人的实时控制提供了可靠的基础，是机器人智能化系统中的关键计算组件。