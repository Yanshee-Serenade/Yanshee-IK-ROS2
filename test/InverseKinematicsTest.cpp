#include "InverseKinematics.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <random>

using namespace std;
using namespace Eigen;

// 测试θ1解析解
void testTheta1AnalyticalSolution(bool is_right_leg) {
    string leg_name = is_right_leg ? "Right" : "Left";
    cout << "=== Testing " << leg_name << " leg Theta1 Analytical Solution ===" << endl;
    
    ForwardKinematics fk(is_right_leg);
    InverseKinematics ik(fk);
    
    // 测试1：零角度情况
    {
        // 使用程序计算所有角度为0时的末端位置
        vector<Vector3d> positions = fk.computeAllJointPositions(0.0, 0.0, 0.0, 0.0);
        Vector3d targetPos = positions.back();  // 最后一个关节位置
        
        vector<double> theta1Solutions = ik.computeTheta1Solutions(targetPos);
        
        cout << "Test 1 - Zero angles case:" << endl;
        cout << "  Target position: " << targetPos.transpose() << endl;
        cout << "  Number of solutions: " << theta1Solutions.size() << endl;
        
        // 检查是否有解
        assert(theta1Solutions.size() >= 1 && "Should have at least one theta1 solution");
        
        // 检查其中一个解应该接近0
        bool foundZero = false;
        for (double theta1 : theta1Solutions) {
            cout << "  Theta1 solution: " << theta1 << " (" << theta1 * 180.0 / M_PI << "°)" << endl;
            
            // 将角度归一化到[-π, π]范围进行比较
            double normalizedTheta1 = theta1;
            while (normalizedTheta1 > M_PI) normalizedTheta1 -= 2 * M_PI;
            while (normalizedTheta1 < -M_PI) normalizedTheta1 += 2 * M_PI;
            
            if (fabs(normalizedTheta1) < 1e-6) {
                foundZero = true;
                break;
            }
        }
        
        assert(foundZero && "Should find theta1 = 0 solution");
        cout << "  ✓ Test 1 passed" << endl << endl;
    }
    
    // 测试2：随机角度测试
    {
        random_device rd;
        mt19937 gen(rd());
        
        // 使用关节限制来生成随机角度
        auto limits = fk.getJointLimits();
        uniform_real_distribution<> dist_theta1(limits.theta1_lower, limits.theta1_upper);
        uniform_real_distribution<> dist_theta3(limits.theta3_lower, limits.theta3_upper);
        uniform_real_distribution<> dist_theta4_diff(-0.5, 0.5);  // theta2 = theta3 + theta4
        
        int numTests = 10;
        int passedTests = 0;
        
        for (int i = 0; i < numTests; ++i) {
            // 生成随机但有效的关节角度
            double theta1 = dist_theta1(gen);
            double theta3 = dist_theta3(gen);
            double theta4 = dist_theta4_diff(gen);
            double theta2 = theta3 + theta4;
            
            // 检查关节限制
            if (theta2 < limits.theta2_lower || theta2 > limits.theta2_upper ||
                theta4 < limits.theta4_lower || theta4 > limits.theta4_upper) {
                i--;  // 重新生成
                continue;
            }
            
            // 计算末端位置
            vector<Vector3d> positions = fk.computeAllJointPositions(
                theta1, theta2, theta3, theta4);
            Vector3d targetPos = positions.back();
            
            vector<double> theta1Solutions = ik.computeTheta1Solutions(targetPos);
            
            // 检查是否有解
            if (theta1Solutions.empty()) {
                cout << "Warning: No theta1 solution for test " << i+1 << endl;
                continue;
            }
            
            // 检查其中是否有解接近原始theta1（考虑周期性）
            bool foundValidSolution = false;
            for (double solution : theta1Solutions) {
                double diff = fabs(solution - theta1);
                diff = fmod(diff, 2 * M_PI);
                if (diff < 1e-6 || fabs(diff - 2 * M_PI) < 1e-6) {
                    foundValidSolution = true;
                    break;
                }
            }
            
            if (foundValidSolution) {
                passedTests++;
            }
        }
        
        cout << "Test 2 - Random angles:" << endl;
        cout << "  Passed " << passedTests << "/" << numTests << " tests" << endl;
        assert(passedTests >= 5 && "Should find solutions for at least half of random positions");
        cout << "  ✓ Test 2 passed" << endl << endl;
    }
    
    // 测试3：边界情况测试
    {
        Vector3d p1 = fk.getJoint1Position();
        double x_rel = fk.getXRelConstant();
        
        // 测试当目标位置在圆柱面上时应该有解
        for (double angle = -M_PI/2; angle <= M_PI/2; angle += M_PI/4) {
            double radius = fabs(x_rel) + 0.1;  // 确保有解
            double target_x = p1.x() + radius * cos(angle);
            double target_z = p1.z() + radius * sin(angle);
            double target_y = p1.y() + 0.1;  // 任意Y值
            
            Vector3d targetPos(target_x, target_y, target_z);
            vector<double> theta1Solutions = ik.computeTheta1Solutions(targetPos);
            
            cout << "Test 3 - Boundary case (angle=" << angle << "): ";
            cout << "solutions = " << theta1Solutions.size() << endl;
            
            // 应该至少有一个解
            assert(!theta1Solutions.empty() && "Should have solution for valid target position");
        }
        
        cout << "  ✓ Test 3 passed" << endl << endl;
    }
}

// 测试网格搜索
void testGridSearch(bool is_right_leg) {
    string leg_name = is_right_leg ? "Right" : "Left";
    cout << "=== Testing " << leg_name << " leg Grid Search ===" << endl;
    
    ForwardKinematics fk(is_right_leg);
    InverseKinematics ik(fk);
    
    // 测试1：简单已知位置
    {
        // 使用简单关节角度计算末端位置
        double testTheta1 = 0.1;
        double testTheta2 = 0.2;
        double testTheta3 = -0.1;
        double testTheta4 = 0.3;
        
        vector<Vector3d> positions = fk.computeAllJointPositions(
            testTheta1, testTheta2, testTheta3, testTheta4);
        Vector3d targetPos = positions.back();
        
        // 调用逆运动学
        InverseKinematics::IKSolution solution = ik.solve(targetPos, 8, 1e-2);
        
        cout << "Test 1 - Known position case:" << endl;
        cout << "  Target position: " << targetPos.transpose() << endl;
        cout << "  Original angles: " << testTheta1 << ", " << testTheta2 
             << ", " << testTheta3 << ", " << testTheta4 << endl;
        
        // 必须找到有效解
        assert(solution.valid && "Should find valid solution for known position");
        
        cout << "  Found solution: " << solution.theta1 << ", " << solution.theta2
             << ", " << solution.theta3 << ", " << solution.theta4 << endl;
        cout << "  Error: " << solution.error << endl;
        
        // 检查误差是否足够小
        assert(solution.error < 1e-2 && "Solution error should be small");
        
        // 检查约束
        assert(fabs(solution.theta2 - (solution.theta3 + solution.theta4)) < 1e-6 &&
               "Constraint theta2 = theta3 + theta4 should be satisfied");
        assert(fabs(solution.theta5 - (-solution.theta1)) < 1e-6 &&
               "Constraint theta5 = -theta1 should be satisfied");
        
        cout << "  ✓ Test 1 passed" << endl << endl;
    }
    
    // 测试2：多个可达位置
    {
        random_device rd;
        mt19937 gen(rd());
        
        auto limits = fk.getJointLimits();
        uniform_real_distribution<> dist_theta1(limits.theta1_lower, limits.theta1_upper);
        uniform_real_distribution<> dist_theta3(limits.theta3_lower, limits.theta3_upper);
        
        int numTests = 5;
        int passedTests = 0;
        
        for (int i = 0; i < numTests; ++i) {
            // 生成随机但合理的关节角度
            double theta1 = dist_theta1(gen);
            double theta3 = dist_theta3(gen);
            
            // 选择theta4使得theta2在范围内
            double theta4_min = max(limits.theta2_lower - theta3, limits.theta4_lower);
            double theta4_max = min(limits.theta2_upper - theta3, limits.theta4_upper);
            
            if (theta4_min > theta4_max) {
                i--;  // 重新生成
                continue;
            }
            
            uniform_real_distribution<> dist_theta4(theta4_min, theta4_max);
            double theta4 = dist_theta4(gen);
            double theta2 = theta3 + theta4;
            
            // 计算末端位置
            vector<Vector3d> positions = fk.computeAllJointPositions(
                theta1, theta2, theta3, theta4);
            Vector3d targetPos = positions.back();
            
            // 调用逆运动学
            InverseKinematics::IKSolution solution = ik.solve(targetPos, 30, 1e-2);
            
            if (solution.valid && solution.error < 1e-2) {
                passedTests++;
            } else {
                cout << "  Warning: Failed to find solution for test " << i+1 << endl;
            }
        }
        
        cout << "Test 2 - Multiple reachable positions:" << endl;
        cout << "  Passed " << passedTests << "/" << numTests << " tests" << endl;
        assert(passedTests >= 3 && "Should find solutions for most reachable positions");
        cout << "  ✓ Test 2 passed" << endl << endl;
    }
    
    // 测试3：不可达位置
    {
        // 创建一个明显不可达的位置（远超出工作空间）
        Vector3d unreachablePos(10.0, 10.0, 10.0);
        
        InverseKinematics::IKSolution solution = ik.solve(unreachablePos, 10, 1e-2);
        
        cout << "Test 3 - Unreachable position:" << endl;
        cout << "  Target position: " << unreachablePos.transpose() << endl;
        
        // 不应该找到有效解
        assert(!solution.valid && "Should not find solution for unreachable position");
        
        cout << "  ✓ Test 3 passed" << endl << endl;
    }
}

// 测试关节限制检查
void testJointLimits(bool is_right_leg) {
    string leg_name = is_right_leg ? "Right" : "Left";
    cout << "=== Testing " << leg_name << " leg Joint Limits ===" << endl;
    
    ForwardKinematics fk(is_right_leg);
    InverseKinematics ik(fk);
    
    auto limits = fk.getJointLimits();
    
    cout << "Joint limits:" << endl;
    cout << "  Theta1: [" << limits.theta1_lower << ", " << limits.theta1_upper << "]" << endl;
    cout << "  Theta2: [" << limits.theta2_lower << ", " << limits.theta2_upper << "]" << endl;
    cout << "  Theta3: [" << limits.theta3_lower << ", " << limits.theta3_upper << "]" << endl;
    cout << "  Theta4: [" << limits.theta4_lower << ", " << limits.theta4_upper << "]" << endl;
    
    // 测试1：有效角度应该通过检查
    {
        double theta1 = 0.0;
        double theta2 = 0.5;
        double theta3 = 0.2;
        double theta4 = 0.3;
        
        bool withinLimits = ik.checkJointLimits(theta1, theta2, theta3, theta4);
        
        cout << "Test 1 - Valid joint angles:" << endl;
        cout << "  Angles: " << theta1 << ", " << theta2 << ", " << theta3 << ", " << theta4 << endl;
        cout << "  Within limits: " << (withinLimits ? "true" : "false") << endl;
        
        assert(withinLimits && "Valid angles should be within limits");
        cout << "  ✓ Test 1 passed" << endl << endl;
    }
    
    // 测试2：θ1超出限制
    {
        double theta1 = 2.0;  // 超出上限
        double theta2 = 0.5;
        double theta3 = 0.2;
        double theta4 = 0.3;
        
        bool withinLimits = ik.checkJointLimits(theta1, theta2, theta3, theta4);
        
        cout << "Test 2 - Theta1 out of limits:" << endl;
        cout << "  Angles: " << theta1 << ", " << theta2 << ", " << theta3 << ", " << theta4 << endl;
        cout << "  Within limits: " << (withinLimits ? "true" : "false") << endl;
        
        assert(!withinLimits && "Theta1 out of limits should fail check");
        cout << "  ✓ Test 2 passed" << endl << endl;
    }
    
    // 测试3：θ2超出限制
    {
        double theta1 = 0.0;
        double theta2 = 3.0;  // 超出上限
        double theta3 = 1.0;
        double theta4 = 2.0;
        
        bool withinLimits = ik.checkJointLimits(theta1, theta2, theta3, theta4);
        
        cout << "Test 3 - Theta2 out of limits:" << endl;
        cout << "  Angles: " << theta1 << ", " << theta2 << ", " << theta3 << ", " << theta4 << endl;
        cout << "  Within limits: " << (withinLimits ? "true" : "false") << endl;
        
        assert(!withinLimits && "Theta2 out of limits should fail check");
        cout << "  ✓ Test 3 passed" << endl << endl;
    }
    
    // 测试4：θ5约束检查
    {
        // 测试θ1会使得θ5超出限制的情况
        double theta1 = -1.4;  // 接近下限
        double theta2 = 0.5;
        double theta3 = 0.2;
        double theta4 = 0.3;
        
        bool withinLimits = ik.checkJointLimits(theta1, theta2, theta3, theta4);
        
        cout << "Test 4 - Theta5 constraint (theta1 = -1.4):" << endl;
        cout << "  Angles: " << theta1 << ", " << theta2 << ", " << theta3 << ", " << theta4 << endl;
        cout << "  Theta5 = " << -theta1 << endl;
        cout << "  Within limits: " << (withinLimits ? "true" : "false") << endl;
        
        // θ5 = 1.4，应该在限制内
        assert(withinLimits && "Theta5 should be within limits for theta1 = -1.4");
        cout << "  ✓ Test 4 passed" << endl << endl;
    }
}

// 测试网格密度影响
void testGridDensity(bool is_right_leg) {
    string leg_name = is_right_leg ? "Right" : "Left";
    cout << "=== Testing " << leg_name << " leg Grid Density ===" << endl;
    
    ForwardKinematics fk(is_right_leg);
    InverseKinematics ik(fk);
    
    // 使用已知位置
    double testTheta1 = 0.1;
    double testTheta2 = 0.2;
    double testTheta3 = -0.1;
    double testTheta4 = 0.3;
    
    vector<Vector3d> positions = fk.computeAllJointPositions(
        testTheta1, testTheta2, testTheta3, testTheta4);
    Vector3d targetPos = positions.back();
    
    // 测试不同网格密度
    vector<int> gridDensities = {4, 8, 12};
    vector<double> errors;
    vector<bool> validSolutions;
    
    cout << "Test - Grid density effect on solution:" << endl;
    cout << "  Target position: " << targetPos.transpose() << endl;
    
    for (int density : gridDensities) {
        InverseKinematics::IKSolution solution = ik.solve(targetPos, density, 1e-2);
        
        if (solution.valid) {
            cout << "  Density " << density << ": error = " << solution.error;
            cout << ", angles = [" << solution.theta1 << ", " << solution.theta2
                 << ", " << solution.theta3 << ", " << solution.theta4 << "]" << endl;
            errors.push_back(solution.error);
            validSolutions.push_back(true);
        } else {
            cout << "  Density " << density << ": no valid solution" << endl;
            errors.push_back(numeric_limits<double>::max());
            validSolutions.push_back(false);
        }
    }
    
    // 检查至少有一种网格密度能找到解
    bool foundAnySolution = false;
    for (bool valid : validSolutions) {
        if (valid) {
            foundAnySolution = true;
            break;
        }
    }
    
    assert(foundAnySolution && "Should find solution for at least one grid density");
    
    // 如果多个密度都找到解，检查误差是否合理
    int validCount = 0;
    for (size_t i = 0; i < validSolutions.size(); i++) {
        if (validSolutions[i]) {
            assert(errors[i] < 1e-2 && "Valid solution should have small error");
            validCount++;
        }
    }
    
    cout << "  Found " << validCount << "/" << gridDensities.size() << " valid solutions" << endl;
    cout << "  ✓ Test passed" << endl << endl;
}

int main() {
    cout << "=========================================" << endl;
    cout << "  Inverse Kinematics Unit Tests" << endl;
    cout << "=========================================" << endl;
    
    int totalTests = 0;
    int passedTests = 0;
    
    try {
        cout << "\n=== Testing Right Leg ===" << endl;
        testTheta1AnalyticalSolution(true);
        totalTests++; passedTests++;
        cout << "✓ Right leg Theta1 Analytical Solution tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Right leg Theta1 Analytical Solution tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Right leg Theta1 Analytical Solution tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testGridSearch(true);
        totalTests++; passedTests++;
        cout << "✓ Right leg Grid Search tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Right leg Grid Search tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Right leg Grid Search tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testJointLimits(true);
        totalTests++; passedTests++;
        cout << "✓ Right leg Joint Limits tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Right leg Joint Limits tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Right leg Joint Limits tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testGridDensity(true);
        totalTests++; passedTests++;
        cout << "✓ Right leg Grid Density tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Right leg Grid Density tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Right leg Grid Density tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        cout << "\n=== Testing Left Leg ===" << endl;
        testTheta1AnalyticalSolution(false);
        totalTests++; passedTests++;
        cout << "✓ Left leg Theta1 Analytical Solution tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Left leg Theta1 Analytical Solution tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Left leg Theta1 Analytical Solution tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testGridSearch(false);
        totalTests++; passedTests++;
        cout << "✓ Left leg Grid Search tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Left leg Grid Search tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Left leg Grid Search tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testJointLimits(false);
        totalTests++; passedTests++;
        cout << "✓ Left leg Joint Limits tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Left leg Joint Limits tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Left leg Joint Limits tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testGridDensity(false);
        totalTests++; passedTests++;
        cout << "✓ Left leg Grid Density tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Left leg Grid Density tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Left leg Grid Density tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    cout << "\n=========================================" << endl;
    cout << "  Test Summary: " << passedTests << "/" << totalTests << " passed" << endl;
    
    if (passedTests == totalTests) {
        cout << "  All tests completed successfully!" << endl;
        cout << "=========================================" << endl;
        return 0;
    } else {
        cout << "  Some tests failed!" << endl;
        cout << "=========================================" << endl;
        return 1;
    }
}