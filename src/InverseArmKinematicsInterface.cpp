#include "InverseArmKinematicsInterface.h"
#include "InverseArmKinematics.h"
#include "ForwardArmKinematics.h"

// Global instances singleton pattern
static ForwardArmKinematics* fk_arm_right_ptr = nullptr;
static ForwardArmKinematics* fk_arm_left_ptr = nullptr;
static InverseArmKinematics* ik_arm_right_ptr = nullptr;
static InverseArmKinematics* ik_arm_left_ptr = nullptr;

static void init_arm_kinematics_if_needed() {
    if (!fk_arm_right_ptr) {
        fk_arm_right_ptr = new ForwardArmKinematics(true);
        ik_arm_right_ptr = new InverseArmKinematics(*fk_arm_right_ptr);
    }
    if (!fk_arm_left_ptr) {
        fk_arm_left_ptr = new ForwardArmKinematics(false);
        ik_arm_left_ptr = new InverseArmKinematics(*fk_arm_left_ptr);
    }
}

int arm_inverse_kinematics(int is_right_arm, double target_x, double target_y, double target_z,
                          double* angles, double* error) {
    init_arm_kinematics_if_needed();

    InverseArmKinematics* ik = is_right_arm ? ik_arm_right_ptr : ik_arm_left_ptr;
    ForwardArmKinematics* fk = is_right_arm ? fk_arm_right_ptr : fk_arm_left_ptr;
    
    Eigen::Vector3d target(target_x, target_y, target_z);
    
    // Perform IK
    // Grid density 20 provides reasonable balance for real-time
    InverseArmKinematics::ArmIKSolution solution = ik->solve(target, 20, 100); 

    auto limits = fk->getJointLimits();

    // Convert URDF angles (-PI to PI) back to ROS angles (0 to Range)
    // Formula: ROS_Angle = URDF_Angle - Lower_Limit
    angles[0] = solution.theta0 - limits.theta0_lower;
    angles[1] = solution.theta1 - limits.theta1_lower;
    angles[2] = solution.theta2 - limits.theta2_lower;
    
    *error = solution.error;

    return solution.valid ? 1 : 0;
}