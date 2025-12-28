#include "ForwardKinematics.h"
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

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
    
    // 计算x_rel常数（从关节6到末端的x方向距离，固定值）
    x_rel_const = p7.x() + p8.x() + p9.x() + p10.x();
    cout << "x_rel constant: " << x_rel_const << endl;
    
    // 设置关节限制（从URDF中获取）
    jointLimits.theta6_lower = -1.5699994666265709;
    jointLimits.theta6_upper = 1.5700005333734293;
    jointLimits.theta7_lower = -1.049999813158908;
    jointLimits.theta7_upper = 2.0900001868410922;
    jointLimits.theta8_lower = -1.3299998161116362;
    jointLimits.theta8_upper = 1.810000183888364;
    jointLimits.theta9_lower = -1.9200004959175843;
    jointLimits.theta9_upper = 1.2199995040824159;
    jointLimits.theta10_lower = -1.5699997046774934;
    jointLimits.theta10_upper = 1.5700002953225067;
    
    // 计算连杆长度
    computeLinkLengths();
}

Matrix4d ForwardKinematics::createTransform(const Vector3d& translation, 
                                          const Vector3d& axis, double angle) const {
    Matrix4d T = Matrix4d::Identity();
    
    // 创建旋转矩阵
    AngleAxisd rotation(angle, axis.normalized());
    T.block<3,3>(0,0) = rotation.toRotationMatrix();
    
    // 设置平移
    T.block<3,1>(0,3) = translation;
    
    return T;
}

vector<Vector3d> ForwardKinematics::computeAllJointPositions(double q6, double q7, 
                                                            double q8, double q9) const {
    vector<Vector3d> positions;
    
    // 1. base_link 位置 (原点)
    positions.push_back(Vector3d::Zero());
    
    // 2. Servo_6 (RightHipYaw_3) 位置
    Matrix4d T_base = Matrix4d::Identity();
    Matrix4d T_6 = createTransform(p6, axis6, q6);
    Matrix4d T_base_to_3 = T_base * T_6;
    Vector4d pos_6_homo = T_base_to_3 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_6_homo.head<3>());
    
    // 3. Servo_7 (RightHipPitch_3) 位置
    Matrix4d T_7 = createTransform(p7, axis7, q7);
    Matrix4d T_base_to_4 = T_base_to_3 * T_7;
    Vector4d pos_7_homo = T_base_to_4 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_7_homo.head<3>());
    
    // 4. Servo_8 (RightKneePitch_3) 位置
    Matrix4d T_8 = createTransform(p8, axis8, q8);
    Matrix4d T_base_to_5 = T_base_to_4 * T_8;
    Vector4d pos_8_homo = T_base_to_5 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_8_homo.head<3>());
    
    // 5. Servo_9 (DEF_RightAnklePitch_2) 位置
    Matrix4d T_9 = createTransform(p9, axis9, q9);
    Matrix4d T_base_to_6 = T_base_to_5 * T_9;
    Vector4d pos_9_homo = T_base_to_6 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_9_homo.head<3>());
    
    // 6. Servo_10 origin 位置
    Vector4d pos_10_homo = T_base_to_6 * Vector4d(p10[0], p10[1], p10[2], 1);
    positions.push_back(pos_10_homo.head<3>());
    
    return positions;
}

ForwardKinematics::JointLimits ForwardKinematics::getJointLimits() const {
    return jointLimits;
}

// 其他函数实现（保持不变）
bool ForwardKinematics::validateLinkLengths(double q6, double q7, double q8, double q9, 
                                          double tolerance) const {
    // 计算所有关节位置
    vector<Vector3d> positions = computeAllJointPositions(q6, q7, q8, q9);
    
    // 验证连杆长度
    bool valid = true;
    
    // 关节6到关节7的距离
    double dist_6_7 = computeDistance(positions[1], positions[2]);
    if (abs(dist_6_7 - linkLengths.length_6_7) > tolerance) {
        cout << "Link 6-7 length mismatch: expected " << linkLengths.length_6_7 
             << ", got " << dist_6_7 << endl;
        valid = false;
    }
    
    // 关节7到关节8的距离
    double dist_7_8 = computeDistance(positions[2], positions[3]);
    if (abs(dist_7_8 - linkLengths.length_7_8) > tolerance) {
        cout << "Link 7-8 length mismatch: expected " << linkLengths.length_7_8 
             << ", got " << dist_7_8 << endl;
        valid = false;
    }
    
    // 关节8到关节9的距离
    double dist_8_9 = computeDistance(positions[3], positions[4]);
    if (abs(dist_8_9 - linkLengths.length_8_9) > tolerance) {
        cout << "Link 8-9 length mismatch: expected " << linkLengths.length_8_9 
             << ", got " << dist_8_9 << endl;
        valid = false;
    }
    
    // 关节9到关节10的距离
    double dist_9_10 = computeDistance(positions[4], positions[5]);
    if (abs(dist_9_10 - linkLengths.length_9_10) > tolerance) {
        cout << "Link 9-10 length mismatch: expected " << linkLengths.length_9_10 
             << ", got " << dist_9_10 << endl;
        valid = false;
    }
    
    return valid;
}

double ForwardKinematics::computeDistance(const Vector3d& p1, const Vector3d& p2) const {
    return (p2 - p1).norm();
}

void ForwardKinematics::computeLinkLengths() {
    // 计算base_link坐标系下的连杆长度
    // 注意：这些是关节原点之间的距离，在关节角度为0时的长度
    
    // 当所有关节角度为0时的变换
    vector<Vector3d> positions = computeAllJointPositions(0.0, 0.0, 0.0, 0.0);
    
    // 计算连杆长度
    linkLengths.length_6_7 = computeDistance(positions[1], positions[2]);
    linkLengths.length_7_8 = computeDistance(positions[2], positions[3]);
    linkLengths.length_8_9 = computeDistance(positions[3], positions[4]);
    linkLengths.length_9_10 = computeDistance(positions[4], positions[5]);
    
    // 输出连杆长度供参考
    cout << "Link lengths (all joints at 0 position):" << endl;
    cout << "  Link 6-7: " << linkLengths.length_6_7 << endl;
    cout << "  Link 7-8: " << linkLengths.length_7_8 << endl;
    cout << "  Link 8-9: " << linkLengths.length_8_9 << endl;
    cout << "  Link 9-10: " << linkLengths.length_9_10 << endl;
    cout << endl;
}