#include "ForwardArmKinematicsInterface.h"
#include "ForwardArmKinematics.h"
#include <vector>

void compute_arm_end_effector_position(int is_right_arm, double theta0, double theta1, 
                                      double theta2, double* result) {
    static ForwardArmKinematics fk_right(true);
    static ForwardArmKinematics fk_left(false);
    
    ForwardArmKinematics& fk = is_right_arm ? fk_right : fk_left;
    ForwardArmKinematics::ArmJointLimits limits = fk.getJointLimits();
    
    // Convert from ROS range (0 based) to URDF range
    std::vector<Eigen::Vector3d> positions = fk.computeAllJointPositions(
        theta0 + limits.theta0_lower,
        theta1 + limits.theta1_lower,
        theta2 + limits.theta2_lower
    );
    
    // Last element is End Effector
    const Eigen::Vector3d& ee_pos = positions.back();
    result[0] = ee_pos.x();
    result[1] = ee_pos.y();
    result[2] = ee_pos.z();
}

void compute_all_arm_joints_positions(int is_right_arm, double theta0, double theta1, 
                                     double theta2, double* result) {
    static ForwardArmKinematics fk_right(true);
    static ForwardArmKinematics fk_left(false);
    
    ForwardArmKinematics& fk = is_right_arm ? fk_right : fk_left;
    ForwardArmKinematics::ArmJointLimits limits = fk.getJointLimits();
    
    std::vector<Eigen::Vector3d> positions = fk.computeAllJointPositions(
        theta0 + limits.theta0_lower,
        theta1 + limits.theta1_lower,
        theta2 + limits.theta2_lower
    );

    // Positions indices: 0:Base, 1:J0, 2:J1, 3:J2, 4:EE
    // Output format requested: usually J0, J1, J2, EE
    int result_idx = 0;
    for (size_t i = 1; i < positions.size(); i++) {
        result[result_idx++] = positions[i].x();
        result[result_idx++] = positions[i].y();
        result[result_idx++] = positions[i].z();
    }
}