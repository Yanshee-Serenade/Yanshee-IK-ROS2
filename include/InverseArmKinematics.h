#pragma once

#include "ForwardArmKinematics.h"
#include <Eigen/Dense>
#include <vector>

class InverseArmKinematics {
public:
    InverseArmKinematics(const ForwardArmKinematics& fk);

    struct ArmIKSolution {
        double theta0; // Shoulder Pitch
        double theta1; // Shoulder Yaw
        double theta2; // Elbow Yaw
        double error;
        bool valid;
    };

    /**
     * @brief Solve IK for 3DOF arm.
     * Uses a Coarse-to-Fine Grid Search approach.
     */
    ArmIKSolution solve(const Eigen::Vector3d& targetPos, 
                       int gridDensity = 20, 
                       double maxAllowedError = 1e-2) const;

private:
    const ForwardArmKinematics& fk;

    // Helper to calculate position error for a specific config
    double computeError(double t0, double t1, double t2, const Eigen::Vector3d& target) const;

    // Helper to clamp angles
    double clamp(double val, double min, double max) const;
};