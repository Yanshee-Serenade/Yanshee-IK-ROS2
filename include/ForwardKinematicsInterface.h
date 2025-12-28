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

#ifdef __cplusplus
}
#endif