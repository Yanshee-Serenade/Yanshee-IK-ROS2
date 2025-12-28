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
        double theta1;   // 关节1角度（对应Servo_6或Servo_11）
        double theta2;   // 关节2角度（对应Servo_7或Servo_12）
        double theta3;   // 关节3角度（对应Servo_8或Servo_13）
        double theta4;   // 关节4角度（对应Servo_9或Servo_14）
        double theta5;   // 关节5角度（对应Servo_10或Servo_15），由约束theta5 = -theta1决定
        double error;    // 位置误差
        bool valid;      // 解是否有效（在关节限制内）
    };
    
    // 求解逆运动学
    // 参数：目标位置[x, y, z]，搜索网格密度，最大允许误差
    IKSolution solve(const Eigen::Vector3d& targetPos, 
                    int gridDensity = 50, 
                    double maxAllowedError = 1e-2) const;
    
    // 计算θ1的解析解（公开用于测试）
    std::vector<double> computeTheta1Solutions(const Eigen::Vector3d& targetPos) const;
    
    // 检查关节角度是否在限制内
    bool checkJointLimits(double theta1, double theta2, 
                         double theta3, double theta4) const;
    
private:
    // 引用到正向运动学对象
    const ForwardKinematics& fk;
    
    // 网格搜索函数
    IKSolution gridSearchForTheta3Theta4(double theta1, const Eigen::Vector3d& targetPos,
                                       int gridDensity, double maxAllowedError) const;
    
    // 计算末端位置误差
    double computePositionError(double theta1, double theta2, double theta3, double theta4,
                              const Eigen::Vector3d& targetPos) const;
};