#include "ForwardKinematics.h"
#include <iostream>
#include <cassert>
#include <random>
#include <cmath>

// 测试连杆长度恒定性的函数
void testLinkLengthInvariance() {
    std::cout << "=== Testing link length invariance ===" << std::endl;
    
    ForwardKinematics fk;
    
    // 设置随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(-M_PI/2, M_PI/2);  // 关节角度范围
    
    int numTests = 100;
    int passedTests = 0;
    
    for (int i = 0; i < numTests; ++i) {
        // 生成随机关节角度
        double q6 = dist(gen);
        double q7 = dist(gen);
        double q8 = dist(gen);
        double q9 = dist(gen);
        
        // 验证连杆长度
        bool valid = fk.validateLinkLengths(q6, q7, q8, q9, 1e-9);
        
        if (valid) {
            passedTests++;
        } else {
            std::cout << "Test " << i+1 << " failed with angles: "
                      << "q6=" << q6 << ", q7=" << q7 
                      << ", q8=" << q8 << ", q9=" << q9 << std::endl;
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
void testBoundaryConditions() {
    std::cout << "\n=== Testing boundary conditions ===" << std::endl;
    
    ForwardKinematics fk;
    
    // 测试零角度
    std::cout << "Testing zero angles..." << std::endl;
    assert(fk.validateLinkLengths(0.0, 0.0, 0.0, 0.0));
    std::cout << "✓ Zero angles passed" << std::endl;
    
    // 测试关节极限
    std::cout << "Testing joint limits..." << std::endl;
    
    // Servo_6 极限
    assert(fk.validateLinkLengths(-1.5699994666265709, 0.0, 0.0, 0.0));  // lower
    assert(fk.validateLinkLengths(1.5700005333734293, 0.0, 0.0, 0.0));   // upper
    
    // Servo_7 极限
    assert(fk.validateLinkLengths(0.0, -1.049999813158908, 0.0, 0.0));   // lower
    assert(fk.validateLinkLengths(0.0, 2.0900001868410922, 0.0, 0.0));   // upper
    
    // Servo_8 极限
    assert(fk.validateLinkLengths(0.0, 0.0, -1.3299998161116362, 0.0));  // lower
    assert(fk.validateLinkLengths(0.0, 0.0, 1.810000183888364, 0.0));    // upper
    
    // Servo_9 极限
    assert(fk.validateLinkLengths(0.0, 0.0, 0.0, -1.9200004959175843));  // lower
    assert(fk.validateLinkLengths(0.0, 0.0, 0.0, 1.2199995040824159));   // upper
    
    std::cout << "✓ All boundary conditions passed" << std::endl;
}

// 测试 createTransform 函数
void testCreateTransform() {
    std::cout << "\n=== Testing createTransform function ===" << std::endl;
    
    ForwardKinematics fk;
    
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

int main() {
    std::cout << "Running ForwardKinematics unit tests..." << std::endl;
    std::cout << "======================================" << std::endl;
    
    try {
        testCreateTransform();
        testBoundaryConditions();
        testLinkLengthInvariance();
        
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
