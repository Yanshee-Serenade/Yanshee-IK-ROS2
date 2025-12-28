#include "ForwardKinematics.h"
#include <iostream>

int main() {
    ForwardKinematics fk;
    
    // 示例：设置关节角度（弧度）
    double q6 = 0.1;   // 约5.7度
    double q7 = 0.2;   // 约11.5度
    double q8 = -0.3;  // 约-17.2度
    double q9 = 0.4;   // 约22.9度
    
    // 计算所有关节位置
    std::vector<Eigen::Vector3d> positions = fk.computeAllJointPositions(q6, q7, q8, q9);
    
    std::cout << "All joint positions in base_link frame:" << std::endl;
    std::cout << "Base link (origin): (0, 0, 0)" << std::endl;
    std::cout << "Joint 6: (" << positions[1].x() << ", " << positions[1].y() << ", " << positions[1].z() << ")" << std::endl;
    std::cout << "Joint 7: (" << positions[2].x() << ", " << positions[2].y() << ", " << positions[2].z() << ")" << std::endl;
    std::cout << "Joint 8: (" << positions[3].x() << ", " << positions[3].y() << ", " << positions[3].z() << ")" << std::endl;
    std::cout << "Joint 9: (" << positions[4].x() << ", " << positions[4].y() << ", " << positions[4].z() << ")" << std::endl;
    std::cout << "Joint 10: (" << positions[5].x() << ", " << positions[5].y() << ", " << positions[5].z() << ")" << std::endl;
    std::cout << std::endl;
    
    // 验证连杆长度
    bool valid = fk.validateLinkLengths(q6, q7, q8, q9);
    if (valid) {
        std::cout << "All link lengths are constant (mechanism is valid)." << std::endl;
    } else {
        std::cout << "Link lengths are not constant (mechanism is invalid)." << std::endl;
    }
    
    return 0;
}