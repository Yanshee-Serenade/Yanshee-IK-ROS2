#include "InverseKinematicsInterface.h"
#include "InverseKinematics.h"
#include "ForwardKinematics.h"
#include <vector>

// 全局对象（注意：这假设在同一线程中调用）
static ForwardKinematics* fk_instance = nullptr;
static InverseKinematics* ik_instance = nullptr;

// 初始化函数（内部使用）
static void initialize_if_needed() {
    if (fk_instance == nullptr) {
        fk_instance = new ForwardKinematics();
    }
    if (ik_instance == nullptr) {
        ik_instance = new InverseKinematics(*fk_instance);
    }
}

// 逆运动学求解
int inverse_kinematics(double target_x, double target_y, double target_z,
                      int grid_density, double* angles, double* error) {
    // 初始化
    initialize_if_needed();
    
    // 创建目标位置向量
    Eigen::Vector3d target_pos(target_x, target_y, target_z);
    
    // 调用逆运动学求解
    InverseKinematics::IKSolution solution = ik_instance->solve(target_pos, grid_density, 1e-2);
    
    // 检查解是否有效
    if (!solution.valid) {
        return 0;  // 失败
    }
    
    // 填充输出数组
    angles[0] = solution.theta6;
    angles[1] = solution.theta7;
    angles[2] = solution.theta8;
    angles[3] = solution.theta9;
    angles[4] = solution.theta10;
    
    // 输出误差
    *error = solution.error;
    
    return 1;  // 成功
}

// 逆运动学求解（使用默认网格密度50）
int inverse_kinematics_default(double target_x, double target_y, double target_z,
                              double* angles, double* error) {
    return inverse_kinematics(target_x, target_y, target_z, 50, angles, error);
}
