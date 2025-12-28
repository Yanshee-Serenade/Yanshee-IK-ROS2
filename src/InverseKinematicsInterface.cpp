#include "InverseKinematicsInterface.h"
#include "InverseKinematics.h"
#include "ForwardKinematics.h"
#include <vector>

// 全局对象（注意：这假设在同一线程中调用）
static ForwardKinematics* fk_right_instance = nullptr;
static ForwardKinematics* fk_left_instance = nullptr;
static InverseKinematics* ik_right_instance = nullptr;
static InverseKinematics* ik_left_instance = nullptr;

// 初始化函数（内部使用）
static void initialize_if_needed() {
    if (fk_right_instance == nullptr) {
        fk_right_instance = new ForwardKinematics(true);  // 右腿
    }
    if (fk_left_instance == nullptr) {
        fk_left_instance = new ForwardKinematics(false);  // 左腿
    }
    if (ik_right_instance == nullptr) {
        ik_right_instance = new InverseKinematics(*fk_right_instance);
    }
    if (ik_left_instance == nullptr) {
        ik_left_instance = new InverseKinematics(*fk_left_instance);
    }
}

// 逆运动学求解
int inverse_kinematics(int is_right_leg, double target_x, double target_y, double target_z,
                      int grid_density, double* angles, double* error) {
    // 初始化
    initialize_if_needed();
    
    // 选择对应的腿
    InverseKinematics* ik = is_right_leg ? ik_right_instance : ik_left_instance;
    
    // 创建目标位置向量
    Eigen::Vector3d target_pos(target_x, target_y, target_z);
    
    // 调用逆运动学求解
    InverseKinematics::IKSolution solution = ik->solve(target_pos, grid_density, 100);
    
    // 检查解是否有效
    if (!solution.valid) {
        return 0;  // 失败
    }
    
    // 填充输出数组
    angles[0] = solution.theta1;
    angles[1] = solution.theta2;
    angles[2] = solution.theta3;
    angles[3] = solution.theta4;
    angles[4] = solution.theta5;
    
    // 输出误差
    *error = solution.error;
    
    return 1;  // 成功
}

// 逆运动学求解（使用默认网格密度8）
int inverse_kinematics_default(int is_right_leg, double target_x, double target_y, double target_z,
                              double* angles, double* error) {
    return inverse_kinematics(is_right_leg, target_x, target_y, target_z, 8, angles, error);
}