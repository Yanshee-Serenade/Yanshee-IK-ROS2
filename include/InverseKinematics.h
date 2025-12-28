#pragma once

#include "ForwardKinematics.h"
#include <Eigen/Dense>
#include <vector>

class InverseKinematics {
public:
    // 构造函数，接受ForwardKinematics引用
    InverseKinematics(const ForwardKinematics& fk);
    
    // 逆运动学求解结构
    struct IKSolution {
        double theta6;
        double theta7;
        double theta8;
        double theta9;
        double theta10;  // 由约束theta10 = -theta6决定
        double error;    // 位置误差
        bool valid;      // 解是否有效（在关节限制内）
    };
    
    // 求解逆运动学
    // 参数：目标位置[x, y, z]，搜索网格密度，最大允许误差
    IKSolution solve(const Eigen::Vector3d& targetPos, 
                    int gridDensity = 50, 
                    double maxAllowedError = 1e-2) const;
    
    // 计算θ6的解析解（公开用于测试）
    std::vector<double> computeTheta6Solutions(const Eigen::Vector3d& targetPos) const;
    
    // 检查关节角度是否在限制内
    bool checkJointLimits(double theta6, double theta7, 
                         double theta8, double theta9) const;
    
private:
    // 引用到正向运动学对象
    const ForwardKinematics& fk;
    
    // 网格搜索函数
    IKSolution gridSearchForTheta8Theta9(double theta6, const Eigen::Vector3d& targetPos,
                                       int gridDensity, double maxAllowedError) const;
    
    // 计算末端位置误差
    double computePositionError(double theta6, double theta7, double theta8, double theta9,
                              const Eigen::Vector3d& targetPos) const;
};
