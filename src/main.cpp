#include <iostream>
#include <cmath>

// 声明C接口函数
extern "C" {
    int inverse_kinematics(int is_right_leg, double target_x, double target_y, double target_z,
                          int grid_density, double* angles, double* error);
    int inverse_kinematics_default(int is_right_leg, double target_x, double target_y, double target_z,
                                  double* angles, double* error);
}

// 将弧度转换为角度
double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// 打印关节角度（弧度和角度）
void printJointAngles(const double* angles, const char* leg_name) {
    std::cout << "\n" << leg_name << " leg inverse kinematics solution:" << std::endl;
    std::cout << "  theta1 (HipYaw):   " << angles[0] << " rad  (" << rad2deg(angles[0]) << "°)" << std::endl;
    std::cout << "  theta2 (HipPitch): " << angles[1] << " rad  (" << rad2deg(angles[1]) << "°)" << std::endl;
    std::cout << "  theta3 (KneePitch):" << angles[2] << " rad  (" << rad2deg(angles[2]) << "°)" << std::endl;
    std::cout << "  theta4 (AnklePitch):" << angles[3] << " rad  (" << rad2deg(angles[3]) << "°)" << std::endl;
    std::cout << "  theta5 (AnkleYaw): " << angles[4] << " rad  (" << rad2deg(angles[4]) << "°)" << std::endl;
    
    // 验证约束条件
    std::cout << "\nConstraint verification:" << std::endl;
    std::cout << "  theta2 = theta3 + theta4: " << angles[1] << " ≈ " 
              << angles[2] + angles[3] << " (diff: " << fabs(angles[1] - (angles[2] + angles[3])) << ")" << std::endl;
    std::cout << "  theta5 = -theta1: " << angles[4] << " ≈ " 
              << -angles[0] << " (diff: " << fabs(angles[4] + angles[0]) << ")" << std::endl;
}

// 测试逆运动学函数
void testInverseKinematicsInterface() {
    std::cout << "========================================" << std::endl;
    std::cout << "Testing Inverse Kinematics Interface" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 测试右腿的目标位置 (0.02, 0, 0.08)
    double target_x = 0.02;
    double target_y = 0.0;
    double target_z = 0.08;
    
    std::cout << "\nTesting right leg for target position:" << std::endl;
    std::cout << "  (" << target_x << ", " << target_y << ", " << target_z << ")" << std::endl;
    
    // 使用默认网格密度
    double angles[5];
    double error;
    
    int result = inverse_kinematics_default(1, target_x, target_y, target_z, angles, &error);
    
    if (result == 1) {
        std::cout << "\n✓ Successfully found inverse kinematics solution!" << std::endl;
        std::cout << "  Position error: " << error << " meters" << std::endl;
        
        printJointAngles(angles, "Right");
        
        // 测试不同的网格密度
        std::cout << "\n\nTesting with different grid densities:" << std::endl;
        
        int densities[] = {20, 50, 100};
        for (int i = 0; i < 3; ++i) {
            double angles2[5];
            double error2;
            
            std::cout << "\nGrid density: " << densities[i] << std::endl;
            int result2 = inverse_kinematics(1, target_x, target_y, target_z, densities[i], angles2, &error2);
            
            if (result2 == 1) {
                std::cout << "  Success! Error: " << error2 << std::endl;
                std::cout << "  theta1: " << angles2[0] << " rad" << std::endl;
            } else {
                std::cout << "  Failed to find solution" << std::endl;
            }
        }
    } else {
        std::cout << "\n✗ Failed to find inverse kinematics solution for right leg" << std::endl;
        std::cout << "  Target position may be unreachable" << std::endl;
    }
    
    // 测试左腿的对称位置
    std::cout << "\n\nTesting left leg for symmetric target position:" << std::endl;
    double target_x_left = -0.02;  // 左腿的对称位置
    double target_y_left = 0.0;
    double target_z_left = 0.08;
    
    std::cout << "  (" << target_x_left << ", " << target_y_left << ", " << target_z_left << ")" << std::endl;
    
    double angles_left[5];
    double error_left;
    
    int result_left = inverse_kinematics_default(0, target_x_left, target_y_left, target_z_left, angles_left, &error_left);
    
    if (result_left == 1) {
        std::cout << "\n✓ Successfully found inverse kinematics solution for left leg!" << std::endl;
        std::cout << "  Position error: " << error_left << " meters" << std::endl;
        
        printJointAngles(angles_left, "Left");
        
        // 检查左右腿的对称性
        if (result == 1) {  // 如果右腿也成功求解
            std::cout << "\n\nSymmetry check between right and left legs:" << std::endl;
            std::cout << "  theta1 symmetry: " << angles[0] << " (right) ≈ " << angles_left[0] << " (left)" 
                      << " (should be approximately symmetric)" << std::endl;
            std::cout << "  theta5 symmetry: " << angles[4] << " (right) ≈ " << angles_left[4] << " (left)" 
                      << " (should be approximately symmetric)" << std::endl;
        }
    } else {
        std::cout << "\n✗ Failed to find inverse kinematics solution for left leg" << std::endl;
    }
    
    // 测试不可达位置
    std::cout << "\n\nTesting unreachable position:" << std::endl;
    double unreachable_x = 10.0;
    double unreachable_y = 10.0;
    double unreachable_z = 10.0;
    
    std::cout << "  (" << unreachable_x << ", " << unreachable_y << ", " << unreachable_z << ")" << std::endl;
    
    double angles_unreachable[5];
    double error_unreachable;
    
    int result_unreachable = inverse_kinematics_default(1, unreachable_x, unreachable_y, unreachable_z, 
                                                       angles_unreachable, &error_unreachable);
    
    if (result_unreachable == 0) {
        std::cout << "✓ Correctly identified as unreachable position" << std::endl;
    } else {
        std::cout << "✗ Unexpectedly found a solution for unreachable position" << std::endl;
        std::cout << "  Error: " << error_unreachable << std::endl;
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "Test completed!" << std::endl;
    std::cout << "========================================" << std::endl;
}

int main() {
    std::cout << "Inverse Kinematics Interface Test" << std::endl;
    std::cout << "Testing C interface functions from InverseKinematicsInterface" << std::endl;
    
    try {
        testInverseKinematicsInterface();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
}