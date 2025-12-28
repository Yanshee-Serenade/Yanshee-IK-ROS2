#include "ForwardKinematics.h"
#include <iostream>

using namespace Eigen;

ForwardKinematics::ForwardKinematics() {
    // 初始化URDF中定义的固定变换
    p6 << 0.02855967037834571, -0.00179832597649984, 0.2220230558077417;
    p7 << 6.815502767362758e-08, 0.00032021516317055627, -0.04409385499648252;
    p8 << 0.0010021932636168733, 0.024925799103312565, -0.043364891746383694;
    p9 << -0.0012519008111556082, -0.02166650217195428, -0.055883969289138524;
    p10 << -0.007445250212467634, -0.002130880800689635, -0.03678118697918865;
    
    // 关节轴方向
    axis6 << 0, 1, 0;
    axis7 << -1, 0, 0;
    axis8 << 1, 0, 0;
    axis9 << 1, 0, 0;
}

Vector3d ForwardKinematics::computeForwardKinematics(double q6, double q7, double q8, double q9) {
    // 初始化变换矩阵为单位矩阵
    Matrix4d T_base = Matrix4d::Identity();
    
    // 1. 从base_link到RightHipYaw_3（Servo_6）
    Matrix4d T_6 = createTransform(p6, axis6, q6);
    Matrix4d T_base_to_3 = T_base * T_6;
    
    // 2. 从RightHipYaw_3到RightHipPitch_3（Servo_7）
    Matrix4d T_7 = createTransform(p7, axis7, q7);
    Matrix4d T_base_to_4 = T_base_to_3 * T_7;
    
    // 3. 从RightHipPitch_3到RightKneePitch_3（Servo_8）
    Matrix4d T_8 = createTransform(p8, axis8, q8);
    Matrix4d T_base_to_5 = T_base_to_4 * T_8;
    
    // 4. 从RightKneePitch_3到DEF_RightAnklePitch_2（Servo_9）
    Matrix4d T_9 = createTransform(p9, axis9, q9);
    Matrix4d T_base_to_6 = T_base_to_5 * T_9;
    
    // 5. Servo_10的origin是固定平移
    Vector4d p10_homogeneous;
    p10_homogeneous << p10, 1.0;
    Vector4d p10_in_base = T_base_to_6 * p10_homogeneous;
    
    return p10_in_base.head<3>();
}

Matrix4d ForwardKinematics::computeFullTransform(double q6, double q7, double q8, double q9) {
    Matrix4d T_base = Matrix4d::Identity();
    
    // 链式相乘
    T_base = T_base * createTransform(p6, axis6, q6);
    T_base = T_base * createTransform(p7, axis7, q7);
    T_base = T_base * createTransform(p8, axis8, q8);
    T_base = T_base * createTransform(p9, axis9, q9);
    
    // 添加Servo_10的固定平移
    T_base.block<3,1>(0,3) += T_base.block<3,3>(0,0) * p10;
    
    return T_base;
}

Matrix4d ForwardKinematics::createTransform(const Vector3d& translation, const Vector3d& axis, double angle) {
    Matrix4d T = Matrix4d::Identity();
    
    // 创建旋转矩阵
    AngleAxisd rotation(angle, axis.normalized());
    T.block<3,3>(0,0) = rotation.toRotationMatrix();
    
    // 设置平移
    T.block<3,1>(0,3) = translation;
    
    return T;
}

MatrixXd ForwardKinematics::computeJacobian(double q6, double q7, double q8, double q9) {
    MatrixXd J = MatrixXd::Zero(3, 4);
    
    // 计算每个关节变换后的位置
    Matrix4d T0 = Matrix4d::Identity();
    Matrix4d T1 = T0 * createTransform(p6, axis6, q6);
    Matrix4d T2 = T1 * createTransform(p7, axis7, q7);
    Matrix4d T3 = T2 * createTransform(p8, axis8, q8);
    Matrix4d T4 = T3 * createTransform(p9, axis9, q9);
    
    // 末端执行器位置（Servo_10原点）
    Vector4d p_end_homo = T4 * Vector4d(p10[0], p10[1], p10[2], 1);
    Vector3d p_end = p_end_homo.head<3>();
    
    // 计算每个关节轴在base_link中的方向
    Vector3d z0 = axis6;
    Vector3d z1 = T1.block<3,3>(0,0) * axis7;
    Vector3d z2 = T2.block<3,3>(0,0) * axis8;
    Vector3d z3 = T3.block<3,3>(0,0) * axis9;
    
    // 计算每个关节原点的位置
    Vector3d p0 = Vector3d::Zero();
    Vector3d p1 = T1.block<3,1>(0,3);
    Vector3d p2 = T2.block<3,1>(0,3);
    Vector3d p3 = T3.block<3,1>(0,3);
    
    // 计算雅可比矩阵的每一列
    J.col(0) = z0.cross(p_end - p0);
    J.col(1) = z1.cross(p_end - p1);
    J.col(2) = z2.cross(p_end - p2);
    J.col(3) = z3.cross(p_end - p3);
    
    return J;
}