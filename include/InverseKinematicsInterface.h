#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// C接口，用于动态库调用

/**
 * @brief 逆运动学求解
 * @param is_right_leg 是否为右腿 (1=右腿, 0=左腿)
 * @param target_x 目标位置X坐标
 * @param target_y 目标位置Y坐标
 * @param target_z 目标位置Z坐标
 * @param grid_density 网格搜索密度（可选，默认为50）
 * @param angles 输出数组，包含5个关节角度：[theta1, theta2, theta3, theta4, theta5]
 * @param error 输出误差
 * @return 1表示成功，0表示失败
 */
int inverse_kinematics(int is_right_leg, double target_x, double target_y, double target_z,
                      int grid_density, double* angles, double* error);

/**
 * @brief 逆运动学求解（使用默认网格密度50）
 */
int inverse_kinematics_default(int is_right_leg, double target_x, double target_y, double target_z,
                              double* angles, double* error);

#ifdef __cplusplus
}
#endif