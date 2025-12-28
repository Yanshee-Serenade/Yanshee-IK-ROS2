#include "InverseKinematics.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace Eigen;
using namespace std;

InverseKinematics::InverseKinematics(const ForwardKinematics& fk_ref) : fk(fk_ref) {}

// 计算θ1的解析解
vector<double> InverseKinematics::computeTheta1Solutions(const Vector3d& targetPos) const {
    vector<double> solutions;
    
    // 获取正向运动学的参数
    Vector3d p1 = fk.getJoint1Position();
    const double x_rel = fk.getXRelConstant();
    
    const double delta_x = targetPos.x() - p1.x();
    const double delta_z = targetPos.z() - p1.z();
    
    // 计算r和φ
    const double r = sqrt(delta_x * delta_x + delta_z * delta_z);
    
    // 检查是否有解
    if (fabs(x_rel) > r) {
        // 无解，返回空向量
        return solutions;
    }
    
    const double phi = atan2(delta_z, delta_x);
    const double alpha = acos(x_rel / r);
    
    // 两个可能的解
    double theta1_1 = -phi + alpha;
    double theta1_2 = -phi - alpha;
    
    // 将角度归一化到[-π, π]范围
    auto normalizeAngle = [](double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };
    
    theta1_1 = normalizeAngle(theta1_1);
    theta1_2 = normalizeAngle(theta1_2);
    
    // 添加到解集
    solutions.push_back(theta1_1);
    
    // 如果两个解不相等（考虑浮点误差），添加第二个解
    if (fabs(theta1_1 - theta1_2) > 1e-10) {
        solutions.push_back(theta1_2);
    }
    
    return solutions;
}

// 逆运动学求解主函数
InverseKinematics::IKSolution InverseKinematics::solve(
    const Vector3d& targetPos, int gridDensity, double maxAllowedError) const {
    
    IKSolution bestSolution;
    bestSolution.error = numeric_limits<double>::max();
    bestSolution.valid = false;
    
    // 步骤1：计算θ1的所有解析解
    vector<double> theta1Solutions = computeTheta1Solutions(targetPos);
    
    if (theta1Solutions.empty()) {
        cout << "Warning: No valid theta1 solution found for target position: " 
             << targetPos.transpose() << endl;
        return bestSolution;
    }
    
    // 步骤2：对每个θ1解进行网格搜索
    for (double theta1 : theta1Solutions) {
        // 检查θ1是否在关节限制内
        auto limits = fk.getJointLimits();
        if (theta1 < limits.theta1_lower || theta1 > limits.theta1_upper) {
            continue;
        }
        
        // 对该θ1解进行网格搜索
        IKSolution solution = gridSearchForTheta3Theta4(theta1, targetPos, 
                                                      gridDensity, maxAllowedError);
        
        // 更新最佳解
        if (solution.valid && solution.error < bestSolution.error) {
            bestSolution = solution;
        }
    }
    
    return bestSolution;
}

// 二级网格搜索θ3和θ4
InverseKinematics::IKSolution InverseKinematics::gridSearchForTheta3Theta4(
    double theta1, const Vector3d& targetPos, int gridDensity, double maxAllowedError) const {
    
    IKSolution bestSolution;
    bestSolution.theta1 = theta1;
    bestSolution.theta5 = -theta1;  // 根据约束
    bestSolution.error = numeric_limits<double>::max();
    bestSolution.valid = false;
    
    auto limits = fk.getJointLimits();
    
    // 第一级：粗网格搜索
    double coarseDensity = gridDensity / 2;  // 粗搜索使用一半的密度
    if (coarseDensity < 5) coarseDensity = 5;  // 最小密度
    
    double theta3_step_coarse = (limits.theta3_upper - limits.theta3_lower) / coarseDensity;
    double theta4_step_coarse = (limits.theta4_upper - limits.theta4_lower) / coarseDensity;
    
    double best_theta3_coarse = 0.0;
    double best_theta4_coarse = 0.0;
    double best_error_coarse = numeric_limits<double>::max();
    bool found_coarse = false;
    
    // 粗搜索
    for (int i = 0; i <= coarseDensity; ++i) {
        double theta3 = limits.theta3_lower + i * theta3_step_coarse;
        
        for (int j = 0; j <= coarseDensity; ++j) {
            double theta4 = limits.theta4_lower + j * theta4_step_coarse;
            
            // 根据约束计算theta2
            double theta2 = theta3 + theta4;
            
            // 检查关节限制
            if (!checkJointLimits(theta1, theta2, theta3, theta4)) {
                continue;
            }
            
            // 计算位置误差
            double error = computePositionError(theta1, theta2, theta3, theta4, targetPos);
            
            // 更新粗搜索最佳解
            if (error < best_error_coarse) {
                best_error_coarse = error;
                best_theta3_coarse = theta3;
                best_theta4_coarse = theta4;
                found_coarse = true;
            }
        }
    }
    
    // 如果没有找到粗搜索解，直接返回
    if (!found_coarse) {
        return bestSolution;
    }
    
    // 第二级：细网格搜索（在粗搜索最佳点周围进行）
    double search_radius = max(theta3_step_coarse, theta4_step_coarse);  // 搜索半径
    
    // 定义细搜索范围（确保不超出关节限制）
    double theta3_min_fine = max(limits.theta3_lower, best_theta3_coarse - search_radius);
    double theta3_max_fine = min(limits.theta3_upper, best_theta3_coarse + search_radius);
    double theta4_min_fine = max(limits.theta4_lower, best_theta4_coarse - search_radius);
    double theta4_max_fine = min(limits.theta4_upper, best_theta4_coarse + search_radius);
    
    // 细搜索使用更高的密度（例如粗密度的2倍）
    int fineDensity = 20;  // 固定细搜索密度
    double theta3_step_fine = (theta3_max_fine - theta3_min_fine) / fineDensity;
    double theta4_step_fine = (theta4_max_fine - theta4_min_fine) / fineDensity;
    
    // 如果步长为0（搜索范围太小），直接使用粗搜索结果
    if (theta3_step_fine <= 0 || theta4_step_fine <= 0) {
        double theta2 = best_theta3_coarse + best_theta4_coarse;
        double error = computePositionError(theta1, theta2, best_theta3_coarse, best_theta4_coarse, targetPos);
        
        if (error <= maxAllowedError) {
            bestSolution.theta2 = theta2;
            bestSolution.theta3 = best_theta3_coarse;
            bestSolution.theta4 = best_theta4_coarse;
            bestSolution.error = error;
            bestSolution.valid = true;
        }
        return bestSolution;
    }
    
    // 细搜索
    for (int i = 0; i <= fineDensity; ++i) {
        double theta3 = theta3_min_fine + i * theta3_step_fine;
        
        for (int j = 0; j <= fineDensity; ++j) {
            double theta4 = theta4_min_fine + j * theta4_step_fine;
            
            // 根据约束计算theta2
            double theta2 = theta3 + theta4;
            
            // 检查关节限制
            if (!checkJointLimits(theta1, theta2, theta3, theta4)) {
                continue;
            }
            
            // 计算位置误差
            double error = computePositionError(theta1, theta2, theta3, theta4, targetPos);
            
            // 更新最佳解
            if (error < bestSolution.error && error <= maxAllowedError) {
                bestSolution.theta2 = theta2;
                bestSolution.theta3 = theta3;
                bestSolution.theta4 = theta4;
                bestSolution.error = error;
                bestSolution.valid = true;
            }
        }
    }
    
    // 如果细搜索没找到解，但粗搜索误差很小，使用粗搜索结果
    if (!bestSolution.valid && best_error_coarse <= maxAllowedError) {
        double theta2 = best_theta3_coarse + best_theta4_coarse;
        bestSolution.theta2 = theta2;
        bestSolution.theta3 = best_theta3_coarse;
        bestSolution.theta4 = best_theta4_coarse;
        bestSolution.error = best_error_coarse;
        bestSolution.valid = true;
    }
    
    return bestSolution;
}

// 计算位置误差
double InverseKinematics::computePositionError(double theta1, double theta2, 
                                             double theta3, double theta4,
                                             const Vector3d& targetPos) const {
    // 计算正运动学得到末端位置
    vector<Vector3d> positions = fk.computeAllJointPositions(theta1, theta2, theta3, theta4);
    Vector3d calculatedPos = positions.back();  // 最后一个关节位置
    
    // 计算欧氏距离误差
    return (calculatedPos - targetPos).norm();
}

// 检查关节角度是否在限制内
bool InverseKinematics::checkJointLimits(double theta1, double theta2, 
                                       double theta3, double theta4) const {
    auto limits = fk.getJointLimits();
    
    // 检查θ1
    if (theta1 < limits.theta1_lower || theta1 > limits.theta1_upper) {
        return false;
    }
    
    // 检查θ2
    if (theta2 < limits.theta2_lower || theta2 > limits.theta2_upper) {
        return false;
    }
    
    // 检查θ3
    if (theta3 < limits.theta3_lower || theta3 > limits.theta3_upper) {
        return false;
    }
    
    // 检查θ4
    if (theta4 < limits.theta4_lower || theta4 > limits.theta4_upper) {
        return false;
    }
    
    // 检查θ5（根据约束θ5 = -θ1）
    double theta5 = -theta1;
    if (theta5 < limits.theta5_lower || theta5 > limits.theta5_upper) {
        return false;
    }
    
    return true;
}