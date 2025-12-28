#include "ForwardKinematicsInterface.h"
#include "ForwardKinematics.h"
#include <vector>

void compute_last_joint_position(double q6, double q7, double q8, double q9, double* result) {
    // 创建ForwardKinematics对象
    static ForwardKinematics fk;  // static避免重复初始化
    
    // 计算所有关节位置
    std::vector<Eigen::Vector3d> positions = fk.computeAllJointPositions(q6, q7, q8, q9);
    
    // 获取最后一个关节（Servo_10）的位置
    const Eigen::Vector3d& last_joint_position = positions.back();
    
    // 填充结果数组
    result[0] = last_joint_position.x();
    result[1] = last_joint_position.y();
    result[2] = last_joint_position.z();
}
