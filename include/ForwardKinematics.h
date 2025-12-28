#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <array>

class ForwardKinematics {
public:
    // 构造函数
    ForwardKinematics();
    
    // 公共成员函数
    Eigen::Matrix4d createTransform(const Eigen::Vector3d& translation, 
                                   const Eigen::Vector3d& axis, 
                                   double angle) const;
    
    // 计算所有关节位置
    std::vector<Eigen::Vector3d> computeAllJointPositions(double q6, double q7, 
                                                         double q8, double q9) const;
    
    // 验证连杆长度是否恒定
    bool validateLinkLengths(double q6, double q7, double q8, double q9, 
                           double tolerance = 1e-6) const;
    
    // 获取关节限制
    struct JointLimits {
        double theta6_lower, theta6_upper;
        double theta7_lower, theta7_upper;
        double theta8_lower, theta8_upper;
        double theta9_lower, theta9_upper;
        double theta10_lower, theta10_upper;
    };
    
    JointLimits getJointLimits() const;
    
    // 获取关节参数（用于逆运动学）
    Eigen::Vector3d getJoint6Position() const { return p6; }
    double getXRelConstant() const { return x_rel_const; }
    
private:
    // 连杆长度（关节之间的固定距离）
    struct LinkLengths {
        double length_6_7;
        double length_7_8;
        double length_8_9;
        double length_9_10;
    } linkLengths;
    
    // 关节原点在父坐标系中的位置
    Eigen::Vector3d p6, p7, p8, p9, p10;
    
    // 关节轴方向
    Eigen::Vector3d axis6, axis7, axis8, axis9;
    
    // x_rel常数（从关节6到末端的x方向距离，固定值）
    double x_rel_const;
    
    // 关节限制
    JointLimits jointLimits;
    
    // 计算两点之间的距离
    double computeDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) const;
    
    // 计算连杆长度
    void computeLinkLengths();
};