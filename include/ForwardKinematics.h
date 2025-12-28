#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <array>

class ForwardKinematics {
public:
    // 构造函数，传入true表示右腿，false表示左腿
    explicit ForwardKinematics(bool is_right_leg = true);
    
    // 创建变换矩阵
    Eigen::Matrix4d createTransform(const Eigen::Vector3d& translation, 
                                   const Eigen::Vector3d& axis, 
                                   double angle) const;
    
    // 计算所有关节位置
    std::vector<Eigen::Vector3d> computeAllJointPositions(double theta1, double theta2, 
                                                         double theta3, double theta4) const;
    
    // 验证连杆长度是否恒定
    bool validateLinkLengths(double theta1, double theta2, double theta3, double theta4, 
                           double tolerance = 1e-6) const;
    
    // 获取关节限制
    struct JointLimits {
        double theta1_lower, theta1_upper;
        double theta2_lower, theta2_upper;
        double theta3_lower, theta3_upper;
        double theta4_lower, theta4_upper;
        double theta5_lower, theta5_upper;
    };
    
    JointLimits getJointLimits() const;
    
    // 获取关节参数
    Eigen::Vector3d getJoint1Position() const { return p1; }
    double getXRelConstant() const { return x_rel_const; }
    
    // 设置腿类型
    void setLegType(bool is_right_leg);
    
private:
    // 腿类型：true=右腿，false=左腿
    bool is_right_leg_;
    
    // 连杆长度（关节之间的固定距离）
    struct LinkLengths {
        double length_1_2;
        double length_2_3;
        double length_3_4;
        double length_4_5;
    } linkLengths;
    
    // 关节原点在父坐标系中的位置
    Eigen::Vector3d p1, p2, p3, p4, p5;
    
    // 关节轴方向
    Eigen::Vector3d axis1, axis2, axis3, axis4;
    
    // x_rel常数（从关节1到末端的x方向距离，固定值）
    double x_rel_const;
    
    // 关节限制
    JointLimits jointLimits;
    
    // 计算两点之间的距离
    double computeDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) const;
    
    // 初始化腿参数
    void initializeLegParameters();
    void initializeRightLeg();
    void initializeLeftLeg();
    
    // 计算连杆长度
    void computeLinkLengths();
};