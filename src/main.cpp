#include "ForwardKinematics.h"
#include <iostream>

int main() {
    ForwardKinematics fk;
    
    // 示例：设置关节角度（弧度）
    double q6 = 0.1;   // 约5.7度
    double q7 = 0.2;   // 约11.5度
    double q8 = -0.3;  // 约-17.2度
    double q9 = 0.4;   // 约22.9度
    
    // 计算正运动学
    Eigen::Vector3d servo10_position = fk.computeForwardKinematics(q6, q7, q8, q9);
    
    std::cout << "Servo_10 origin position in base_link frame:" << std::endl;
    std::cout << "X: " << servo10_position[0] << std::endl;
    std::cout << "Y: " << servo10_position[1] << std::endl;
    std::cout << "Z: " << servo10_position[2] << std::endl;
    std::cout << std::endl;
    
    return 0;
}