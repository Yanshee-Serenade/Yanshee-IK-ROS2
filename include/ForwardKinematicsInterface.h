#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// C接口，用于动态库调用
/**
 * @brief 计算最后一个关节（Servo_10）的位置
 * @param q6 Servo_6关节角度（弧度）
 * @param q7 Servo_7关节角度（弧度）
 * @param q8 Servo_8关节角度（弧度）
 * @param q9 Servo_9关节角度（弧度）
 * @param result 输出数组，包含3个元素：[x, y, z]
 */
void compute_last_joint_position(double q6, double q7, double q8, double q9, double* result);

#ifdef __cplusplus
}
#endif
