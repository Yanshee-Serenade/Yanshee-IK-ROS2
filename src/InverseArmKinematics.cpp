#include "InverseArmKinematics.h"
#include <limits>
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace Eigen;
using namespace std;

InverseArmKinematics::InverseArmKinematics(const ForwardArmKinematics& fk_ref) : fk(fk_ref) {}

double InverseArmKinematics::computeError(double t0, double t1, double t2, const Vector3d& target) const {
    vector<Vector3d> pos = fk.computeAllJointPositions(t0, t1, t2);
    return (pos.back() - target).norm();
}

InverseArmKinematics::ArmIKSolution InverseArmKinematics::solve(
    const Vector3d& targetPos, int gridDensity, double maxAllowedError) const {

    ArmIKSolution bestSol;
    bestSol.error = numeric_limits<double>::max();
    bestSol.valid = false;

    auto limits = fk.getJointLimits();

    // Strategy: Coarse Grid Search followed by Fine Grid Search (Local Optimization)
    // 3 DOF allows for relatively fast full grid search compared to high DOF chains.
    
    // --- Phase 1: Coarse Grid Search ---
    int coarse_steps = 15; // 15^3 = 3375 checks
    double step0 = (limits.theta0_upper - limits.theta0_lower) / coarse_steps;
    double step1 = (limits.theta1_upper - limits.theta1_lower) / coarse_steps;
    double step2 = (limits.theta2_upper - limits.theta2_lower) / coarse_steps;

    for(int i = 0; i <= coarse_steps; i++) {
        double t0 = limits.theta0_lower + i * step0;
        for(int j = 0; j <= coarse_steps; j++) {
            double t1 = limits.theta1_lower + j * step1;
            for(int k = 0; k <= coarse_steps; k++) {
                double t2 = limits.theta2_lower + k * step2;
                
                double err = computeError(t0, t1, t2, targetPos);
                if(err < bestSol.error) {
                    bestSol.error = err;
                    bestSol.theta0 = t0;
                    bestSol.theta1 = t1;
                    bestSol.theta2 = t2;
                    bestSol.valid = true;
                }
            }
        }
    }

    // --- Phase 2: Fine Grid Search around best solution ---
    // Search radius is roughly the step size of the previous phase
    double radius0 = step0 * 1.5;
    double radius1 = step1 * 1.5;
    double radius2 = step2 * 1.5;

    int fine_steps = gridDensity; 
    
    double start0 = max(limits.theta0_lower, bestSol.theta0 - radius0);
    double end0   = min(limits.theta0_upper, bestSol.theta0 + radius0);
    double start1 = max(limits.theta1_lower, bestSol.theta1 - radius1);
    double end1   = min(limits.theta1_upper, bestSol.theta1 + radius1);
    double start2 = max(limits.theta2_lower, bestSol.theta2 - radius2);
    double end2   = min(limits.theta2_upper, bestSol.theta2 + radius2);

    double f_step0 = (end0 - start0) / fine_steps;
    double f_step1 = (end1 - start1) / fine_steps;
    double f_step2 = (end2 - start2) / fine_steps;

    // Handle case where range is zero
    if(f_step0 == 0) f_step0 = 1; 
    if(f_step1 == 0) f_step1 = 1;
    if(f_step2 == 0) f_step2 = 1;

    for(int i = 0; i <= fine_steps; i++) {
        double t0 = start0 + i * f_step0;
        for(int j = 0; j <= fine_steps; j++) {
            double t1 = start1 + j * f_step1;
            for(int k = 0; k <= fine_steps; k++) {
                double t2 = start2 + k * f_step2;
                
                double err = computeError(t0, t1, t2, targetPos);
                if(err < bestSol.error) {
                    bestSol.error = err;
                    bestSol.theta0 = t0;
                    bestSol.theta1 = t1;
                    bestSol.theta2 = t2;
                }
            }
        }
    }

    if (bestSol.error > maxAllowedError) {
        // Solution valid flag logic can be adjusted based on strictness
        bestSol.valid = false; 
    }

    return bestSol;
}