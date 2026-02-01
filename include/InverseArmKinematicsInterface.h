#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Arm Inverse Kinematics
 * @param is_right_arm (1=Right, 0=Left)
 * @param target_x/y/z Target position coordinates
 * @param angles Output array [3] for theta0, theta1, theta2 (normalized 0 to Range)
 * @param error Output position error
 * @return 1 if success (error < limit), 0 otherwise (but still returns best guess)
 */
int arm_inverse_kinematics(int is_right_arm, double target_x, double target_y, double target_z,
                          double* angles, double* error);

#ifdef __cplusplus
}
#endif