#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// C接口，用于动态库调用

/**
 * @brief 计算最后一个关节（Servo_10/Servo_15）的位置
 * @param is_right_leg 是否为右腿 (1=右腿, 0=左腿)
 * @param theta1 关节1角度（弧度）
 * @param theta2 关节2角度（弧度）
 * @param theta3 关节3角度（弧度）
 * @param theta4 关节4角度（弧度）
 * @param result 输出数组，包含3个元素：[x, y, z]
 */
void compute_last_joint_position(int is_right_leg, double theta1, double theta2, 
                                double theta3, double theta4, double* result);

/**
 * @brief 计算后四个关节（关节2-5）的位置
 * @param is_right_leg 是否为右腿 (1=右腿, 0=左腿)
 * @param theta1 关节1角度（弧度）
 * @param theta2 关节2角度（弧度）
 * @param theta3 关节3角度（弧度）
 * @param theta4 关节4角度（弧度）
 * @param result 输出数组，包含12个元素：[x2, y2, z2, x3, y3, z3, x4, y4, z4, x5, y5, z5]
 *               对应关节2、3、4、5的xyz坐标
 */
void compute_last_four_joints_positions(int is_right_leg, double theta1, double theta2, 
                                       double theta3, double theta4, double* result);

#ifdef __cplusplus
}
#endif