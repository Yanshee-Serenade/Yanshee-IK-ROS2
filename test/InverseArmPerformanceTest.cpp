#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>

#include "ForwardArmKinematics.h"
#include "InverseArmKinematics.h"

using namespace std;
using namespace Eigen;

// Configuration
const int NUM_TEST_ITERATIONS = 1000;
const double ERROR_TOLERANCE = 0.015; // 1.5 cm tolerance (matches IK default)
const int GRID_DENSITY = 20;          // Matches IK default

// Random number generator helper
double randomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

void runTest(bool is_right_arm) {
    string side = is_right_arm ? "RIGHT" : "LEFT";
    cout << "========================================" << endl;
    cout << "Testing " << side << " Arm Kinematics" << endl;
    cout << "========================================" << endl;

    // 1. Initialization
    ForwardArmKinematics fk(is_right_arm);
    InverseArmKinematics ik(fk);
    auto limits = fk.getJointLimits();

    // Statistics trackers
    double total_time_ms = 0;
    double max_time_ms = 0;
    double total_pos_error = 0;
    double max_pos_error = 0;
    int success_count = 0;

    cout << fixed << setprecision(4);

    // 2. Loop
    for (int i = 0; i < NUM_TEST_ITERATIONS; ++i) {
        // A. Generate Random Valid Joint Angles
        double t0_truth = randomDouble(limits.theta0_lower, limits.theta0_upper);
        double t1_truth = randomDouble(limits.theta1_lower, limits.theta1_upper);
        double t2_truth = randomDouble(limits.theta2_lower, limits.theta2_upper);

        if (is_right_arm) {
            t0_truth = 90  * 0.0174533 + limits.theta0_lower;
            t1_truth = 165 * 0.0174533 + limits.theta1_lower;
            t2_truth = 100 * 0.0174533 + limits.theta2_lower;
        } else {
            t0_truth = 90 * 0.0174533 + limits.theta0_lower;
            t1_truth = 15 * 0.0174533 + limits.theta1_lower;
            t2_truth = 80 * 0.0174533 + limits.theta2_lower;
        }

        // B. Compute Target Position using FK (The "Ground Truth")
        vector<Vector3d> fk_res = fk.computeAllJointPositions(t0_truth, t1_truth, t2_truth);
        Vector3d target_pos = fk_res.back(); // End Effector

        // C. Solve IK
        auto start = chrono::high_resolution_clock::now();
        
        InverseArmKinematics::ArmIKSolution sol = ik.solve(target_pos, GRID_DENSITY, ERROR_TOLERANCE);
        
        auto end = chrono::high_resolution_clock::now();
        double duration = chrono::duration<double, milli>(end - start).count();

        // D. Verify Solution using FK again
        vector<Vector3d> check_fk = fk.computeAllJointPositions(sol.theta0, sol.theta1, sol.theta2);
        Vector3d actual_pos = check_fk.back();

        double current_error = (actual_pos - target_pos).norm();

        // E. Update Stats
        total_time_ms += duration;
        if (duration > max_time_ms) max_time_ms = duration;
        
        total_pos_error += current_error;
        if (current_error > max_pos_error) max_pos_error = current_error;

        if (current_error <= ERROR_TOLERANCE) {
            success_count++;
        }

        // Print first few iterations detailed
        if (i < 3) {
            cout << "[Test " << i << "]" << endl;
            cout << "  Target: " << target_pos.transpose() << endl;
            cout << "  Truth Angles: " << t0_truth << ", " << t1_truth << ", " << t2_truth << endl;
            cout << "  Solved Angles: " << sol.theta0 << ", " << sol.theta1 << ", " << sol.theta2 << endl;
            cout << "  Actual: " << actual_pos.transpose() << endl;
            cout << "  Error:  " << current_error << " m" << endl;
            cout << "  Time:   " << duration << " ms" << endl;
            cout << "----------------------------------------" << endl;
        }
    }

    // 3. Report Results
    double avg_time = total_time_ms / NUM_TEST_ITERATIONS;
    double avg_error = total_pos_error / NUM_TEST_ITERATIONS;
    double success_rate = (double)success_count / NUM_TEST_ITERATIONS * 100.0;

    cout << "SUMMARY for " << side << " Arm:" << endl;
    cout << "  Iterations: " << NUM_TEST_ITERATIONS << endl;
    cout << "  Grid Density: " << GRID_DENSITY << endl;
    cout << "  Avg Time per Solve: " << avg_time << " ms" << endl;
    cout << "  Max Time per Solve: " << max_time_ms << " ms" << endl;
    cout << "  Avg Position Error: " << avg_error * 1000.0 << " mm" << endl;
    cout << "  Max Position Error: " << max_pos_error * 1000.0 << " mm" << endl;
    cout << "  Success Rate: " << success_rate << "% (Tolerance: " << ERROR_TOLERANCE*1000 << "mm)" << endl;
    cout << endl;
}

int main() {
    // Seed random number generator
    srand(time(NULL));

    // Test Right Arm
    runTest(true);

    // Test Left Arm
    runTest(false);

    return 0;
}