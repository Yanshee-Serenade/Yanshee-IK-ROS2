#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

class ForwardKinematics {
public:
    // 构造函数
    ForwardKinematics();
    
    // 公共成员函数
    Eigen::Matrix4d createTransform(const Eigen::Vector3d& translation, 
                                   const Eigen::Vector3d& axis, 
                                   double angle);
    
    // 计算所有关节位置（返回base_link到每个关节变换后的位置向量）
    std::vector<Eigen::Vector3d> computeAllJointPositions(double q6, double q7, 
                                                         double q8, double q9);
    
    // 验证连杆长度是否恒定
    bool validateLinkLengths(double q6, double q7, double q8, double q9, 
                           double tolerance = 1e-6);
    
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
    
    // 计算两点之间的距离
    double computeDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    
    // 计算连杆长度
    void computeLinkLengths();
};