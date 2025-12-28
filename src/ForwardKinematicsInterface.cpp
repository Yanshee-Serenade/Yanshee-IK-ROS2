#include "ForwardKinematicsInterface.h"
#include "ForwardKinematics.h"
#include <vector>

void compute_last_joint_position(int is_right_leg, double theta1, double theta2, 
                                double theta3, double theta4, double* result) {
    // 创建ForwardKinematics对象
    static ForwardKinematics fk_right(true);
    static ForwardKinematics fk_left(false);
    
    ForwardKinematics& fk = is_right_leg ? fk_right : fk_left;
    
    // 计算所有关节位置
    std::vector<Eigen::Vector3d> positions = fk.computeAllJointPositions(theta1, theta2, 
                                                                         theta3, theta4);
    
    // 获取最后一个关节的位置
    const Eigen::Vector3d& last_joint_position = positions.back();
    
    // 填充结果数组
    result[0] = last_joint_position.x();
    result[1] = last_joint_position.y();
    result[2] = last_joint_position.z();
}