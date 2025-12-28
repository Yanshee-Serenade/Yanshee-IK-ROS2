#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

class ForwardKinematics {
private:
    Eigen::Vector3d p6, p7, p8, p9, p10;
    Eigen::Vector3d axis6, axis7, axis8, axis9;
    
    Eigen::Matrix4d createTransform(const Eigen::Vector3d& translation, 
                                   const Eigen::Vector3d& axis, 
                                   double angle);
    
public:
    ForwardKinematics();
    
    Eigen::Vector3d computeForwardKinematics(double q6, double q7, 
                                            double q8, double q9);
    
    Eigen::Matrix4d computeFullTransform(double q6, double q7, 
                                        double q8, double q9);
    
    Eigen::MatrixXd computeJacobian(double q6, double q7, 
                                   double q8, double q9);
};