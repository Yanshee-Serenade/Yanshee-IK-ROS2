#include "ForwardKinematics.h"
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

ForwardKinematics::ForwardKinematics(bool is_right_leg) : is_right_leg_(is_right_leg) {
    initializeLegParameters();
    computeLinkLengths();
}

void ForwardKinematics::setLegType(bool is_right_leg) {
    if (is_right_leg_ != is_right_leg) {
        is_right_leg_ = is_right_leg;
        initializeLegParameters();
        computeLinkLengths();
    }
}

void ForwardKinematics::initializeLegParameters() {
    if (is_right_leg_) {
        initializeRightLeg();
    } else {
        initializeLeftLeg();
    }
}

void ForwardKinematics::initializeRightLeg() {
    // 右腿关节参数 (Servo 6-10)
    p1 << 0.02855967037834571, -0.00179832597649984, 0.2220230558077417;    // Servo_6
    p2 << 6.815502767362758e-08, 0.00032021516317055627, -0.04409385499648252;  // Servo_7
    p3 << 0.0010021932636168733, 0.024925799103312565, -0.043364891746383694;   // Servo_8
    p4 << -0.0012519008111556082, -0.02166650217195428, -0.055883969289138524;  // Servo_9
    p5 << -0.007445250212467634, -0.002130880800689635, -0.03678118697918865;   // Servo_10
    
    // 关节轴方向
    axis1 << 0, 1, 0;    // Servo_6: Y轴旋转
    axis2 << -1, 0, 0;   // Servo_7: -X轴旋转
    axis3 << 1, 0, 0;    // Servo_8: X轴旋转
    axis4 << 1, 0, 0;    // Servo_9: X轴旋转
    
    // 计算x_rel常数
    x_rel_const = p2.x() + p3.x() + p4.x() + p5.x();
    
    // 关节限制
    jointLimits.theta1_lower = -1.5699994666265709;
    jointLimits.theta1_upper = 1.5700005333734293;
    jointLimits.theta2_lower = -1.049999813158908;
    jointLimits.theta2_upper = 2.0900001868410922;
    jointLimits.theta3_lower = -1.3299998161116362;
    jointLimits.theta3_upper = 1.810000183888364;
    jointLimits.theta4_lower = -1.9200004959175843;
    jointLimits.theta4_upper = 1.2199995040824159;
    jointLimits.theta5_lower = -1.5699997046774934;
    jointLimits.theta5_upper = 1.5700002953225067;
    
    cout << "Initialized right leg kinematics" << endl;
}

void ForwardKinematics::initializeLeftLeg() {
    // 左腿关节参数 (Servo 11-15)
    p1 << -0.029156270748436554, -0.0018051759973011134, 0.22184016804540255;   // Servo_11
    p2 << -1.4974728158867467e-09, 0.0001417277665730261, -0.043786342379200954; // Servo_12
    p3 << 7.007415266338524e-06, 0.025091327815532742, -0.04291861040000247;    // Servo_13
    p4 << 0.0012483023518837753, -0.021833280584548267, -0.055363702931461034;  // Servo_14
    p5 << 0.0074484320078616155, -0.002323586872961718, -0.03788865133133677;   // Servo_15
    
    // 关节轴方向
    axis1 << 0, 1, 0;    // Servo_11: Y轴旋转
    axis2 << 1, 0, 0;    // Servo_12: X轴旋转
    axis3 << -1, 0, 0;   // Servo_13: -X轴旋转
    axis4 << -1, 0, 0;   // Servo_14: -X轴旋转
    
    // 计算x_rel常数
    x_rel_const = p2.x() + p3.x() + p4.x() + p5.x();
    
    // 关节限制
    jointLimits.theta1_lower = -1.570000321704434;
    jointLimits.theta1_upper = 1.569999678295566;
    jointLimits.theta2_lower = -2.0899997317507197;
    jointLimits.theta2_upper = 1.0500002682492804;
    jointLimits.theta3_lower = -1.8100001680686482;
    jointLimits.theta3_upper = 1.329999831931352;
    jointLimits.theta4_lower = -1.2200005763320116;
    jointLimits.theta4_upper = 1.9199994236679885;
    jointLimits.theta5_lower = -1.5700007032934171;
    jointLimits.theta5_upper = 1.569999296706583;
    
    cout << "Initialized left leg kinematics" << endl;
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

vector<Vector3d> ForwardKinematics::computeAllJointPositions(double theta1, double theta2, 
                                                            double theta3, double theta4) const {
    vector<Vector3d> positions;
    
    // 1. base_link 位置 (原点)
    positions.push_back(Vector3d::Zero());
    
    // 2. 关节1位置
    Matrix4d T_base = Matrix4d::Identity();
    Matrix4d T_1 = createTransform(p1, axis1, theta1);
    Matrix4d T_base_to_1 = T_base * T_1;
    Vector4d pos_1_homo = T_base_to_1 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_1_homo.head<3>());
    
    // 3. 关节2位置
    Matrix4d T_2 = createTransform(p2, axis2, theta2);
    Matrix4d T_base_to_2 = T_base_to_1 * T_2;
    Vector4d pos_2_homo = T_base_to_2 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_2_homo.head<3>());
    
    // 4. 关节3位置
    Matrix4d T_3 = createTransform(p3, axis3, theta3);
    Matrix4d T_base_to_3 = T_base_to_2 * T_3;
    Vector4d pos_3_homo = T_base_to_3 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_3_homo.head<3>());
    
    // 5. 关节4位置
    Matrix4d T_4 = createTransform(p4, axis4, theta4);
    Matrix4d T_base_to_4 = T_base_to_3 * T_4;
    Vector4d pos_4_homo = T_base_to_4 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_4_homo.head<3>());
    
    // 6. 关节5原点位置
    Vector4d pos_5_homo = T_base_to_4 * Vector4d(p5[0], p5[1], p5[2], 1);
    positions.push_back(pos_5_homo.head<3>());
    
    return positions;
}

ForwardKinematics::JointLimits ForwardKinematics::getJointLimits() const {
    return jointLimits;
}

bool ForwardKinematics::validateLinkLengths(double theta1, double theta2, double theta3, double theta4, 
                                          double tolerance) const {
    // 计算所有关节位置
    vector<Vector3d> positions = computeAllJointPositions(theta1, theta2, theta3, theta4);
    
    // 验证连杆长度
    bool valid = true;
    
    // 关节1到关节2的距离
    double dist_1_2 = computeDistance(positions[1], positions[2]);
    if (abs(dist_1_2 - linkLengths.length_1_2) > tolerance) {
        cout << "Link 1-2 length mismatch: expected " << linkLengths.length_1_2 
             << ", got " << dist_1_2 << endl;
        valid = false;
    }
    
    // 关节2到关节3的距离
    double dist_2_3 = computeDistance(positions[2], positions[3]);
    if (abs(dist_2_3 - linkLengths.length_2_3) > tolerance) {
        cout << "Link 2-3 length mismatch: expected " << linkLengths.length_2_3 
             << ", got " << dist_2_3 << endl;
        valid = false;
    }
    
    // 关节3到关节4的距离
    double dist_3_4 = computeDistance(positions[3], positions[4]);
    if (abs(dist_3_4 - linkLengths.length_3_4) > tolerance) {
        cout << "Link 3-4 length mismatch: expected " << linkLengths.length_3_4 
             << ", got " << dist_3_4 << endl;
        valid = false;
    }
    
    // 关节4到关节5的距离
    double dist_4_5 = computeDistance(positions[4], positions[5]);
    if (abs(dist_4_5 - linkLengths.length_4_5) > tolerance) {
        cout << "Link 4-5 length mismatch: expected " << linkLengths.length_4_5 
             << ", got " << dist_4_5 << endl;
        valid = false;
    }
    
    return valid;
}

double ForwardKinematics::computeDistance(const Vector3d& p1, const Vector3d& p2) const {
    return (p2 - p1).norm();
}

void ForwardKinematics::computeLinkLengths() {
    // 当所有关节角度为0时的变换
    vector<Vector3d> positions = computeAllJointPositions(0.0, 0.0, 0.0, 0.0);
    
    // 计算连杆长度
    linkLengths.length_1_2 = computeDistance(positions[1], positions[2]);
    linkLengths.length_2_3 = computeDistance(positions[2], positions[3]);
    linkLengths.length_3_4 = computeDistance(positions[3], positions[4]);
    linkLengths.length_4_5 = computeDistance(positions[4], positions[5]);
    
    // 输出连杆长度
    cout << "Link lengths (all joints at 0 position):" << endl;
    cout << "  Link 1-2: " << linkLengths.length_1_2 << endl;
    cout << "  Link 2-3: " << linkLengths.length_2_3 << endl;
    cout << "  Link 3-4: " << linkLengths.length_3_4 << endl;
    cout << "  Link 4-5: " << linkLengths.length_4_5 << endl;
    cout << endl;
}