#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

class ForwardArmKinematics {
public:
    // Constructor: true for right arm, false for left arm
    explicit ForwardArmKinematics(bool is_right_arm = true);

    // Create transformation matrix
    Eigen::Matrix4d createTransform(const Eigen::Vector3d& translation, 
                                   const Eigen::Vector3d& axis, 
                                   double angle) const;

    // Compute all joint positions (Joint 0, Joint 1, Joint 2, EndEffector)
    // Angles in radians (URDF frame)
    std::vector<Eigen::Vector3d> computeAllJointPositions(double theta0, double theta1, double theta2) const;

    // Joint Limits Structure
    struct ArmJointLimits {
        double theta0_lower, theta0_upper; // Shoulder Pitch
        double theta1_lower, theta1_upper; // Shoulder Yaw
        double theta2_lower, theta2_upper; // Elbow Yaw
    };

    ArmJointLimits getJointLimits() const;
    Eigen::Vector3d getEndEffectorOffset() const { return p_ee_offset; }
    bool isRightArm() const { return is_right_arm_; }

    // Set arm type
    void setArmType(bool is_right_arm);

private:
    bool is_right_arm_;

    // Joint origins in parent frame
    Eigen::Vector3d p0; // Base -> Shoulder Pitch
    Eigen::Vector3d p1; // Shoulder Pitch -> Shoulder Yaw
    Eigen::Vector3d p2; // Shoulder Yaw -> Elbow Yaw
    
    // End Effector offset relative to the last joint
    Eigen::Vector3d p_ee_offset; 

    // Rotation Axes
    Eigen::Vector3d axis0;
    Eigen::Vector3d axis1;
    Eigen::Vector3d axis2;

    // Limits
    ArmJointLimits jointLimits;

    void initializeArmParameters();
    void initializeRightArm();
    void initializeLeftArm();
};