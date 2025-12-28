#include "ForwardKinematics.h"
#include <iostream>
#include <cassert>
#include <random>
#include <cmath>

// 测试连杆长度恒定性的函数
void testLinkLengthInvariance(bool is_right_leg) {
    std::string leg_name = is_right_leg ? "Right" : "Left";
    std::cout << "=== Testing " << leg_name << " leg link length invariance ===" << std::endl;
    
    ForwardKinematics fk(is_right_leg);
    
    // 设置随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // 获取关节限制
    ForwardKinematics::JointLimits limits = fk.getJointLimits();
    
    int numTests = 100;
    int passedTests = 0;
    
    for (int i = 0; i < numTests; ++i) {
        // 生成随机关节角度（在关节限制范围内）
        std::uniform_real_distribution<> dist1(limits.theta1_lower, limits.theta1_upper);
        std::uniform_real_distribution<> dist2(limits.theta2_lower, limits.theta2_upper);
        std::uniform_real_distribution<> dist3(limits.theta3_lower, limits.theta3_upper);
        std::uniform_real_distribution<> dist4(limits.theta4_lower, limits.theta4_upper);
        
        double theta1 = dist1(gen);
        double theta2 = dist2(gen);
        double theta3 = dist3(gen);
        double theta4 = dist4(gen);
        
        // 验证连杆长度
        bool valid = fk.validateLinkLengths(theta1, theta2, theta3, theta4, 1e-9);
        
        if (valid) {
            passedTests++;
        } else {
            std::cout << "Test " << i+1 << " failed with angles: "
                      << "theta1=" << theta1 << ", theta2=" << theta2 
                      << ", theta3=" << theta3 << ", theta4=" << theta4 << std::endl;
        }
        
        // 每10个测试输出一次进度
        if ((i+1) % 10 == 0) {
            std::cout << "Completed " << i+1 << " tests..." << std::endl;
        }
    }
    
    std::cout << "\nTest results: " << passedTests << "/" << numTests << " passed" << std::endl;
    
    if (passedTests == numTests) {
        std::cout << "✓ All tests passed! Link lengths remain constant for all joint angles." << std::endl;
    } else {
        std::cout << "✗ Some tests failed. Link lengths are not constant." << std::endl;
    }
}

// 测试边界情况
void testBoundaryConditions(bool is_right_leg) {
    std::string leg_name = is_right_leg ? "Right" : "Left";
    std::cout << "\n=== Testing " << leg_name << " leg boundary conditions ===" << std::endl;
    
    ForwardKinematics fk(is_right_leg);
    ForwardKinematics::JointLimits limits = fk.getJointLimits();
    
    // 测试零角度
    std::cout << "Testing zero angles..." << std::endl;
    assert(fk.validateLinkLengths(0.0, 0.0, 0.0, 0.0));
    std::cout << "✓ Zero angles passed" << std::endl;
    
    // 测试关节极限
    std::cout << "Testing joint limits..." << std::endl;
    
    // theta1 极限
    assert(fk.validateLinkLengths(limits.theta1_lower, 0.0, 0.0, 0.0));
    assert(fk.validateLinkLengths(limits.theta1_upper, 0.0, 0.0, 0.0));
    
    // theta2 极限
    assert(fk.validateLinkLengths(0.0, limits.theta2_lower, 0.0, 0.0));
    assert(fk.validateLinkLengths(0.0, limits.theta2_upper, 0.0, 0.0));
    
    // theta3 极限
    assert(fk.validateLinkLengths(0.0, 0.0, limits.theta3_lower, 0.0));
    assert(fk.validateLinkLengths(0.0, 0.0, limits.theta3_upper, 0.0));
    
    // theta4 极限
    assert(fk.validateLinkLengths(0.0, 0.0, 0.0, limits.theta4_lower));
    assert(fk.validateLinkLengths(0.0, 0.0, 0.0, limits.theta4_upper));
    
    std::cout << "✓ All boundary conditions passed" << std::endl;
}

// 测试 createTransform 函数
void testCreateTransform() {
    std::cout << "\n=== Testing createTransform function ===" << std::endl;
    
    // 创建任意ForwardKinematics对象（腿类型不影响此测试）
    ForwardKinematics fk(true);
    
    // 测试零旋转
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    Eigen::Vector3d axis(0.0, 0.0, 1.0);
    double angle = 0.0;
    
    Eigen::Matrix4d T = fk.createTransform(translation, axis, angle);
    
    // 检查平移部分
    assert(std::abs(T(0,3) - 1.0) < 1e-9);
    assert(std::abs(T(1,3) - 2.0) < 1e-9);
    assert(std::abs(T(2,3) - 3.0) < 1e-9);
    
    // 检查旋转部分（应该是单位矩阵）
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    assert((R - I).norm() < 1e-9);
    
    std::cout << "✓ Zero rotation transform test passed" << std::endl;
    
    // 测试绕Z轴旋转90度
    angle = M_PI / 2;
    T = fk.createTransform(translation, axis, angle);
    
    // 检查旋转矩阵（绕Z轴旋转90度）
    // 应该将X轴转到Y轴，Y轴转到-X轴
    Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
    Eigen::Vector3d transformed_x = T.block<3,3>(0,0) * x_axis;
    assert(std::abs(transformed_x.x() - 0.0) < 1e-9);
    assert(std::abs(transformed_x.y() - 1.0) < 1e-9);
    assert(std::abs(transformed_x.z() - 0.0) < 1e-9);
    
    std::cout << "✓ 90-degree rotation transform test passed" << std::endl;
}

// 测试关节角度有效性
void testJointAnglesValidity(bool is_right_leg) {
    std::string leg_name = is_right_leg ? "Right" : "Left";
    std::cout << "\n=== Testing " << leg_name << " leg joint angles validity ===" << std::endl;
    
    ForwardKinematics fk(is_right_leg);
    ForwardKinematics::JointLimits limits = fk.getJointLimits();
    
    // 测试有效角度
    std::cout << "Testing valid joint angles..." << std::endl;
    
    // 测试中间角度
    double theta1_mid = (limits.theta1_lower + limits.theta1_upper) / 2;
    double theta2_mid = (limits.theta2_lower + limits.theta2_upper) / 2;
    double theta3_mid = (limits.theta3_lower + limits.theta3_upper) / 2;
    double theta4_mid = (limits.theta4_lower + limits.theta4_upper) / 2;
    
    auto positions = fk.computeAllJointPositions(theta1_mid, theta2_mid, theta3_mid, theta4_mid);
    assert(positions.size() == 6);
    std::cout << "✓ Middle angles test passed" << std::endl;
    
    // 测试极限角度组合
    std::cout << "Testing limit combinations..." << std::endl;
    assert(fk.validateLinkLengths(limits.theta1_lower, limits.theta2_lower, 
                                  limits.theta3_lower, limits.theta4_lower));
    assert(fk.validateLinkLengths(limits.theta1_upper, limits.theta2_upper, 
                                  limits.theta3_upper, limits.theta4_upper));
    std::cout << "✓ Limit combinations test passed" << std::endl;
}

int main() {
    std::cout << "Running ForwardKinematics unit tests..." << std::endl;
    std::cout << "======================================" << std::endl;
    
    try {
        // 测试基础功能
        testCreateTransform();
        
        // 测试右腿
        std::cout << "\n=== Testing Right Leg ===" << std::endl;
        testBoundaryConditions(true);
        testLinkLengthInvariance(true);
        testJointAnglesValidity(true);
        
        // 测试左腿
        std::cout << "\n=== Testing Left Leg ===" << std::endl;
        testBoundaryConditions(false);
        testLinkLengthInvariance(false);
        testJointAnglesValidity(false);
        
        std::cout << "\n======================================" << std::endl;
        std::cout << "All unit tests completed successfully!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}