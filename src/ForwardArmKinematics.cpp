#include "ForwardArmKinematics.h"
#include <cmath>

using namespace Eigen;
using namespace std;

ForwardArmKinematics::ForwardArmKinematics(bool is_right_arm) : is_right_arm_(is_right_arm) {
    initializeArmParameters();
}

void ForwardArmKinematics::setArmType(bool is_right_arm) {
    if (is_right_arm_ != is_right_arm) {
        is_right_arm_ = is_right_arm;
        initializeArmParameters();
    }
}

void ForwardArmKinematics::initializeArmParameters() {
    if (is_right_arm_) {
        initializeRightArm();
    } else {
        initializeLeftArm();
    }
}

void ForwardArmKinematics::initializeRightArm() {
    // Right Arm Parameters (URDF Data)
    // Servo_0 (RightShoulderPitch)
    p0 << 0.053854213998775824, 0.01054703414366889, 0.302338293948836;
    axis0 << -1, 0, 0; 
    jointLimits.theta0_lower = -1.569999918630965;
    jointLimits.theta0_upper = 1.5700000813690351;

    // Servo_1 (RightShoulderYaw)
    p1 << 0.023040852333471307, -0.0012033914063669885, -0.0027901741159528926;
    axis1 << 0, 1, 0;
    jointLimits.theta1_lower = -2.4400000360131244;
    jointLimits.theta1_upper = 0.6999999639868757;

    // Servo_2 (RightElbowYaw)
    p2 << 0.04113053989807973, -0.00025551854823879956, -0.047104410344261716;
    axis2 << 0, 1, 0;
    jointLimits.theta2_lower = -2.8699999911849536;
    jointLimits.theta2_upper = 0.2700000088150465;

    // End Effector Offset (From Prompt)
    p_ee_offset << -0.039, 0, -0.056;

    cout << "Initialized right arm kinematics" << endl;
}

void ForwardArmKinematics::initializeLeftArm() {
    // Left Arm Parameters (URDF Data)
    // Servo_3 (LeftShoulderPitch)
    p0 << -0.053880482122875974, 0.011467197514028365, 0.301193704611223;
    axis0 << 1, 0, 0;
    jointLimits.theta0_lower = -1.5700000778346614;
    jointLimits.theta0_upper = 1.5699999221653387;

    // Servo_4 (LeftShoulderYaw)
    p1 << -0.022900099225807845, -4.30236108181839e-06, 4.120145493891059e-06;
    axis1 << 0, 1, 0;
    jointLimits.theta1_lower = -0.6999999646794246;
    jointLimits.theta1_upper = 2.4400000353205753;

    // Servo_5 (LeftElbowYaw)
    p2 << -0.03906818108214223, -0.0012568088401258802, -0.04558043171043935;
    axis2 << 0, 1, 0;
    jointLimits.theta2_lower = -0.26200000859256173;
    jointLimits.theta2_upper = 2.8779999914074383;

    // End Effector Offset (From Prompt)
    p_ee_offset << 0.039, 0, -0.056;

    cout << "Initialized left arm kinematics" << endl;
}

Matrix4d ForwardArmKinematics::createTransform(const Vector3d& translation, 
                                             const Vector3d& axis, double angle) const {
    Matrix4d T = Matrix4d::Identity();
    AngleAxisd rotation(angle, axis.normalized());
    T.block<3,3>(0,0) = rotation.toRotationMatrix();
    T.block<3,1>(0,3) = translation;
    return T;
}

vector<Vector3d> ForwardArmKinematics::computeAllJointPositions(double theta0, double theta1, double theta2) const {
    vector<Vector3d> positions;
    
    // 0. Base Link (Origin)
    positions.push_back(Vector3d::Zero()); // Index 0

    // 1. Joint 0 (Shoulder Pitch) Position
    Matrix4d T_base = Matrix4d::Identity();
    Matrix4d T_0 = createTransform(p0, axis0, theta0);
    Matrix4d T_base_to_0 = T_base * T_0;
    Vector4d pos_0_homo = T_base_to_0 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_0_homo.head<3>()); // Index 1

    // 2. Joint 1 (Shoulder Yaw) Position
    Matrix4d T_1 = createTransform(p1, axis1, theta1);
    Matrix4d T_base_to_1 = T_base_to_0 * T_1;
    Vector4d pos_1_homo = T_base_to_1 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_1_homo.head<3>()); // Index 2

    // 3. Joint 2 (Elbow Yaw) Position
    Matrix4d T_2 = createTransform(p2, axis2, theta2);
    Matrix4d T_base_to_2 = T_base_to_1 * T_2;
    Vector4d pos_2_homo = T_base_to_2 * Vector4d(0, 0, 0, 1);
    positions.push_back(pos_2_homo.head<3>()); // Index 3

    // 4. End Effector Position
    // The End Effector offset is a fixed transform after the last joint rotation
    Vector4d pos_ee_homo = T_base_to_2 * Vector4d(p_ee_offset.x(), p_ee_offset.y(), p_ee_offset.z(), 1);
    positions.push_back(pos_ee_homo.head<3>()); // Index 4

    return positions;
}

ForwardArmKinematics::ArmJointLimits ForwardArmKinematics::getJointLimits() const {
    return jointLimits;
}