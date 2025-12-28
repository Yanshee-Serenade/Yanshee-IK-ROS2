#include "InverseKinematics.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <random>

using namespace std;
using namespace Eigen;

// 测试θ6解析解
void testTheta6AnalyticalSolution() {
    cout << "=== Testing Theta6 Analytical Solution ===" << endl;
    
    ForwardKinematics fk;
    InverseKinematics ik(fk);
    
    // 测试1：零角度情况
    {
        // 使用程序计算所有角度为0时的末端位置
        vector<Vector3d> positions = fk.computeAllJointPositions(0.0, 0.0, 0.0, 0.0);
        Vector3d targetPos = positions.back();  // 最后一个关节位置
        
        vector<double> theta6Solutions = ik.computeTheta6Solutions(targetPos);
        
        cout << "Test 1 - Zero angles case:" << endl;
        cout << "  Target position: " << targetPos.transpose() << endl;
        cout << "  Number of solutions: " << theta6Solutions.size() << endl;
        
        // 检查是否有解
        assert(theta6Solutions.size() >= 1 && "Should have at least one theta6 solution");
        
        // 检查其中一个解应该接近0
        bool foundZero = false;
        for (double theta6 : theta6Solutions) {
            cout << "  Theta6 solution: " << theta6 << " (" << theta6 * 180.0 / M_PI << "°)" << endl;
            
            // 将角度归一化到[-π, π]范围进行比较
            double normalizedTheta6 = theta6;
            while (normalizedTheta6 > M_PI) normalizedTheta6 -= 2 * M_PI;
            while (normalizedTheta6 < -M_PI) normalizedTheta6 += 2 * M_PI;
            
            if (fabs(normalizedTheta6) < 1e-6) {
                foundZero = true;
                break;
            }
        }
        
        assert(foundZero && "Should find theta6 = 0 solution");
        cout << "  ✓ Test 1 passed" << endl << endl;
    }
    
    // 测试2：随机角度测试
    {
        random_device rd;
        mt19937 gen(rd());
        
        // 使用关节限制来生成随机角度
        auto limits = fk.getJointLimits();
        uniform_real_distribution<> dist_theta6(limits.theta6_lower, limits.theta6_upper);
        uniform_real_distribution<> dist_theta8(limits.theta8_lower, limits.theta8_upper);
        uniform_real_distribution<> dist_theta9_diff(-0.5, 0.5);  // theta7 = theta8 + theta9
        
        int numTests = 10;
        int passedTests = 0;
        
        for (int i = 0; i < numTests; ++i) {
            // 生成随机但有效的关节角度
            double theta6 = dist_theta6(gen);
            double theta8 = dist_theta8(gen);
            double theta9 = dist_theta9_diff(gen);
            double theta7 = theta8 + theta9;
            
            // 检查关节限制
            if (theta7 < limits.theta7_lower || theta7 > limits.theta7_upper ||
                theta9 < limits.theta9_lower || theta9 > limits.theta9_upper) {
                i--;  // 重新生成
                continue;
            }
            
            // 计算末端位置
            vector<Vector3d> positions = fk.computeAllJointPositions(
                theta6, theta7, theta8, theta9);
            Vector3d targetPos = positions.back();
            
            vector<double> theta6Solutions = ik.computeTheta6Solutions(targetPos);
            
            // 检查是否有解
            if (theta6Solutions.empty()) {
                cout << "Warning: No theta6 solution for test " << i+1 << endl;
                continue;
            }
            
            // 检查其中是否有解接近原始theta6（考虑周期性）
            bool foundValidSolution = false;
            for (double solution : theta6Solutions) {
                double diff = fabs(solution - theta6);
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
        Vector3d p6 = fk.getJoint6Position();
        double x_rel = fk.getXRelConstant();
        
        // 测试当目标位置在圆柱面上时应该有解
        for (double angle = -M_PI/2; angle <= M_PI/2; angle += M_PI/4) {
            double radius = fabs(x_rel) + 0.1;  // 确保有解
            double target_x = p6.x() + radius * cos(angle);
            double target_z = p6.z() + radius * sin(angle);
            double target_y = p6.y() + 0.1;  // 任意Y值
            
            Vector3d targetPos(target_x, target_y, target_z);
            vector<double> theta6Solutions = ik.computeTheta6Solutions(targetPos);
            
            cout << "Test 3 - Boundary case (angle=" << angle << "): ";
            cout << "solutions = " << theta6Solutions.size() << endl;
            
            // 应该至少有一个解
            assert(!theta6Solutions.empty() && "Should have solution for valid target position");
        }
        
        cout << "  ✓ Test 3 passed" << endl << endl;
    }
}

// 测试网格搜索
void testGridSearch() {
    cout << "=== Testing Grid Search ===" << endl;
    
    ForwardKinematics fk;
    InverseKinematics ik(fk);
    
    // 测试1：简单已知位置
    {
        // 使用简单关节角度计算末端位置
        double testTheta6 = 0.1;
        double testTheta7 = 0.2;
        double testTheta8 = -0.1;
        double testTheta9 = 0.3;
        
        vector<Vector3d> positions = fk.computeAllJointPositions(
            testTheta6, testTheta7, testTheta8, testTheta9);
        Vector3d targetPos = positions.back();
        
        // 调用逆运动学
        InverseKinematics::IKSolution solution = ik.solve(targetPos, 20, 1e-2);
        
        cout << "Test 1 - Known position case:" << endl;
        cout << "  Target position: " << targetPos.transpose() << endl;
        cout << "  Original angles: " << testTheta6 << ", " << testTheta7 
             << ", " << testTheta8 << ", " << testTheta9 << endl;
        
        // 必须找到有效解
        assert(solution.valid && "Should find valid solution for known position");
        
        cout << "  Found solution: " << solution.theta6 << ", " << solution.theta7
             << ", " << solution.theta8 << ", " << solution.theta9 << endl;
        cout << "  Error: " << solution.error << endl;
        
        // 检查误差是否足够小
        assert(solution.error < 1e-2 && "Solution error should be small");
        
        // 检查约束
        assert(fabs(solution.theta7 - (solution.theta8 + solution.theta9)) < 1e-6 &&
               "Constraint theta7 = theta8 + theta9 should be satisfied");
        assert(fabs(solution.theta10 - (-solution.theta6)) < 1e-6 &&
               "Constraint theta10 = -theta6 should be satisfied");
        
        cout << "  ✓ Test 1 passed" << endl << endl;
    }
    
    // 测试2：多个可达位置
    {
        random_device rd;
        mt19937 gen(rd());
        
        auto limits = fk.getJointLimits();
        uniform_real_distribution<> dist_theta6(limits.theta6_lower, limits.theta6_upper);
        uniform_real_distribution<> dist_theta8(limits.theta8_lower, limits.theta8_upper);
        
        int numTests = 5;
        int passedTests = 0;
        
        for (int i = 0; i < numTests; ++i) {
            // 生成随机但合理的关节角度
            double theta6 = dist_theta6(gen);
            double theta8 = dist_theta8(gen);
            
            // 选择theta9使得theta7在范围内
            double theta9_min = max(limits.theta7_lower - theta8, limits.theta9_lower);
            double theta9_max = min(limits.theta7_upper - theta8, limits.theta9_upper);
            
            if (theta9_min > theta9_max) {
                i--;  // 重新生成
                continue;
            }
            
            uniform_real_distribution<> dist_theta9(theta9_min, theta9_max);
            double theta9 = dist_theta9(gen);
            double theta7 = theta8 + theta9;
            
            // 计算末端位置
            vector<Vector3d> positions = fk.computeAllJointPositions(
                theta6, theta7, theta8, theta9);
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
void testJointLimits() {
    cout << "=== Testing Joint Limits ===" << endl;
    
    ForwardKinematics fk;
    InverseKinematics ik(fk);
    
    auto limits = fk.getJointLimits();
    
    cout << "Joint limits:" << endl;
    cout << "  Theta6: [" << limits.theta6_lower << ", " << limits.theta6_upper << "]" << endl;
    cout << "  Theta7: [" << limits.theta7_lower << ", " << limits.theta7_upper << "]" << endl;
    cout << "  Theta8: [" << limits.theta8_lower << ", " << limits.theta8_upper << "]" << endl;
    cout << "  Theta9: [" << limits.theta9_lower << ", " << limits.theta9_upper << "]" << endl;
    
    // 测试1：有效角度应该通过检查
    {
        double theta6 = 0.0;
        double theta7 = 0.5;
        double theta8 = 0.2;
        double theta9 = 0.3;
        
        bool withinLimits = ik.checkJointLimits(theta6, theta7, theta8, theta9);
        
        cout << "Test 1 - Valid joint angles:" << endl;
        cout << "  Angles: " << theta6 << ", " << theta7 << ", " << theta8 << ", " << theta9 << endl;
        cout << "  Within limits: " << (withinLimits ? "true" : "false") << endl;
        
        assert(withinLimits && "Valid angles should be within limits");
        cout << "  ✓ Test 1 passed" << endl << endl;
    }
    
    // 测试2：θ6超出限制
    {
        double theta6 = 2.0;  // 超出上限
        double theta7 = 0.5;
        double theta8 = 0.2;
        double theta9 = 0.3;
        
        bool withinLimits = ik.checkJointLimits(theta6, theta7, theta8, theta9);
        
        cout << "Test 2 - Theta6 out of limits:" << endl;
        cout << "  Angles: " << theta6 << ", " << theta7 << ", " << theta8 << ", " << theta9 << endl;
        cout << "  Within limits: " << (withinLimits ? "true" : "false") << endl;
        
        assert(!withinLimits && "Theta6 out of limits should fail check");
        cout << "  ✓ Test 2 passed" << endl << endl;
    }
    
    // 测试3：θ7超出限制
    {
        double theta6 = 0.0;
        double theta7 = 3.0;  // 超出上限
        double theta8 = 1.0;
        double theta9 = 2.0;
        
        bool withinLimits = ik.checkJointLimits(theta6, theta7, theta8, theta9);
        
        cout << "Test 3 - Theta7 out of limits:" << endl;
        cout << "  Angles: " << theta6 << ", " << theta7 << ", " << theta8 << ", " << theta9 << endl;
        cout << "  Within limits: " << (withinLimits ? "true" : "false") << endl;
        
        assert(!withinLimits && "Theta7 out of limits should fail check");
        cout << "  ✓ Test 3 passed" << endl << endl;
    }
    
    // 测试4：θ10约束检查
    {
        // 测试θ6会使得θ10超出限制的情况
        double theta6 = -1.4;  // 接近下限
        double theta7 = 0.5;
        double theta8 = 0.2;
        double theta9 = 0.3;
        
        bool withinLimits = ik.checkJointLimits(theta6, theta7, theta8, theta9);
        
        cout << "Test 4 - Theta10 constraint (theta6 = -1.4):" << endl;
        cout << "  Angles: " << theta6 << ", " << theta7 << ", " << theta8 << ", " << theta9 << endl;
        cout << "  Theta10 = " << -theta6 << endl;
        cout << "  Within limits: " << (withinLimits ? "true" : "false") << endl;
        
        // θ10 = 1.4，应该在限制内
        assert(withinLimits && "Theta10 should be within limits for theta6 = -1.4");
        cout << "  ✓ Test 4 passed" << endl << endl;
    }
}

// 测试网格密度影响
void testGridDensity() {
    cout << "=== Testing Grid Density ===" << endl;
    
    ForwardKinematics fk;
    InverseKinematics ik(fk);
    
    // 使用已知位置
    double testTheta6 = 0.1;
    double testTheta7 = 0.2;
    double testTheta8 = -0.1;
    double testTheta9 = 0.3;
    
    vector<Vector3d> positions = fk.computeAllJointPositions(
        testTheta6, testTheta7, testTheta8, testTheta9);
    Vector3d targetPos = positions.back();
    
    // 测试不同网格密度
    vector<int> gridDensities = {10, 20, 50};
    vector<double> errors;
    vector<bool> validSolutions;
    
    cout << "Test - Grid density effect on solution:" << endl;
    cout << "  Target position: " << targetPos.transpose() << endl;
    
    for (int density : gridDensities) {
        InverseKinematics::IKSolution solution = ik.solve(targetPos, density, 1e-2);
        
        if (solution.valid) {
            cout << "  Density " << density << ": error = " << solution.error;
            cout << ", angles = [" << solution.theta6 << ", " << solution.theta7
                 << ", " << solution.theta8 << ", " << solution.theta9 << "]" << endl;
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
        testTheta6AnalyticalSolution();
        totalTests++; passedTests++;
        cout << "✓ Theta6 Analytical Solution tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Theta6 Analytical Solution tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Theta6 Analytical Solution tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testGridSearch();
        totalTests++; passedTests++;
        cout << "✓ Grid Search tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Grid Search tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Grid Search tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testJointLimits();
        totalTests++; passedTests++;
        cout << "✓ Joint Limits tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Joint Limits tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Joint Limits tests failed with unknown exception" << endl;
        totalTests++;
    }
    
    try {
        testGridDensity();
        totalTests++; passedTests++;
        cout << "✓ Grid Density tests passed" << endl;
    } catch (const exception& e) {
        cerr << "✗ Grid Density tests failed: " << e.what() << endl;
        totalTests++;
    } catch (...) {
        cerr << "✗ Grid Density tests failed with unknown exception" << endl;
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