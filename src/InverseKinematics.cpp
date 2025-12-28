#include "InverseKinematics.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace Eigen;
using namespace std;

InverseKinematics::InverseKinematics(const ForwardKinematics& fk_ref) : fk(fk_ref) {}

// 计算θ6的解析解
vector<double> InverseKinematics::computeTheta6Solutions(const Vector3d& targetPos) const {
    vector<double> solutions;
    
    // 获取正向运动学的参数
    Vector3d p6 = fk.getJoint6Position();
    const double x_rel = fk.getXRelConstant();
    
    const double delta_x = targetPos.x() - p6.x();
    const double delta_z = targetPos.z() - p6.z();
    
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
    double theta6_1 = -phi + alpha;
    double theta6_2 = -phi - alpha;
    
    // 将角度归一化到[-π, π]范围
    auto normalizeAngle = [](double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };
    
    theta6_1 = normalizeAngle(theta6_1);
    theta6_2 = normalizeAngle(theta6_2);
    
    // 添加到解集
    solutions.push_back(theta6_1);
    
    // 如果两个解不相等（考虑浮点误差），添加第二个解
    if (fabs(theta6_1 - theta6_2) > 1e-10) {
        solutions.push_back(theta6_2);
    }
    
    return solutions;
}

// 逆运动学求解主函数
InverseKinematics::IKSolution InverseKinematics::solve(
    const Vector3d& targetPos, int gridDensity, double maxAllowedError) const {
    
    IKSolution bestSolution;
    bestSolution.error = numeric_limits<double>::max();
    bestSolution.valid = false;
    
    // 步骤1：计算θ6的所有解析解
    vector<double> theta6Solutions = computeTheta6Solutions(targetPos);
    
    if (theta6Solutions.empty()) {
        cout << "Warning: No valid theta6 solution found for target position: " 
             << targetPos.transpose() << endl;
        return bestSolution;
    }
    
    // 步骤2：对每个θ6解进行网格搜索
    for (double theta6 : theta6Solutions) {
        // 检查θ6是否在关节限制内
        auto limits = fk.getJointLimits();
        if (theta6 < limits.theta6_lower || theta6 > limits.theta6_upper) {
            continue;
        }
        
        // 对该θ6解进行网格搜索
        IKSolution solution = gridSearchForTheta8Theta9(theta6, targetPos, 
                                                      gridDensity, maxAllowedError);
        
        // 更新最佳解
        if (solution.valid && solution.error < bestSolution.error) {
            bestSolution = solution;
        }
    }
    
    return bestSolution;
}

// 网格搜索θ8和θ9
InverseKinematics::IKSolution InverseKinematics::gridSearchForTheta8Theta9(
    double theta6, const Vector3d& targetPos, int gridDensity, double maxAllowedError) const {
    
    IKSolution bestSolution;
    bestSolution.theta6 = theta6;
    bestSolution.theta10 = -theta6;  // 根据约束
    bestSolution.error = numeric_limits<double>::max();
    bestSolution.valid = false;
    
    auto limits = fk.getJointLimits();
    
    // 计算网格步长
    double theta8_step = (limits.theta8_upper - limits.theta8_lower) / gridDensity;
    double theta9_step = (limits.theta9_upper - limits.theta9_lower) / gridDensity;
    
    // 网格搜索
    for (int i = 0; i <= gridDensity; ++i) {
        double theta8 = limits.theta8_lower + i * theta8_step;
        
        for (int j = 0; j <= gridDensity; ++j) {
            double theta9 = limits.theta9_lower + j * theta9_step;
            
            // 根据约束计算theta7
            double theta7 = theta8 + theta9;
            
            // 检查关节限制
            if (!checkJointLimits(theta6, theta7, theta8, theta9)) {
                continue;
            }
            
            // 计算位置误差
            double error = computePositionError(theta6, theta7, theta8, theta9, targetPos);
            
            // 更新最佳解
            if (error < bestSolution.error && error <= maxAllowedError) {
                bestSolution.theta7 = theta7;
                bestSolution.theta8 = theta8;
                bestSolution.theta9 = theta9;
                bestSolution.error = error;
                bestSolution.valid = true;
            }
        }
    }
    
    return bestSolution;
}

// 计算位置误差
double InverseKinematics::computePositionError(double theta6, double theta7, 
                                             double theta8, double theta9,
                                             const Vector3d& targetPos) const {
    // 计算正运动学得到末端位置
    vector<Vector3d> positions = fk.computeAllJointPositions(theta6, theta7, theta8, theta9);
    Vector3d calculatedPos = positions.back();  // 最后一个关节位置
    
    // 计算欧氏距离误差
    return (calculatedPos - targetPos).norm();
}

// 检查关节角度是否在限制内
bool InverseKinematics::checkJointLimits(double theta6, double theta7, 
                                       double theta8, double theta9) const {
    auto limits = fk.getJointLimits();
    
    // 检查θ6
    if (theta6 < limits.theta6_lower || theta6 > limits.theta6_upper) {
        return false;
    }
    
    // 检查θ7
    if (theta7 < limits.theta7_lower || theta7 > limits.theta7_upper) {
        return false;
    }
    
    // 检查θ8
    if (theta8 < limits.theta8_lower || theta8 > limits.theta8_upper) {
        return false;
    }
    
    // 检查θ9
    if (theta9 < limits.theta9_lower || theta9 > limits.theta9_upper) {
        return false;
    }
    
    // 检查θ10（根据约束θ10 = -θ6）
    double theta10 = -theta6;
    if (theta10 < limits.theta10_lower || theta10 > limits.theta10_upper) {
        return false;
    }
    
    return true;
}
