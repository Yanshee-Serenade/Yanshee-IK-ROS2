#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// C Interface for Shared Library calls

/**
 * @brief Compute Arm End Effector Position
 * @param is_right_arm (1 = Right, 0 = Left)
 * @param theta0 Shoulder Pitch Angle (radians, normalized 0 to range)
 * @param theta1 Shoulder Yaw Angle (radians, normalized 0 to range)
 * @param theta2 Elbow Yaw Angle (radians, normalized 0 to range)
 * @param result Output array [x, y, z] of the end effector
 */
void compute_arm_end_effector_position(int is_right_arm, double theta0, double theta1, 
                                      double theta2, double* result);

/**
 * @brief Compute all Arm Joint positions
 * @param result Output array [12] -> [x0, y0, z0, x1, y1, z1, x2, y2, z2, xEE, yEE, zEE]
 */
void compute_all_arm_joints_positions(int is_right_arm, double theta0, double theta1, 
                                     double theta2, double* result);

#ifdef __cplusplus
}
#endif