#include "InverseKinematics.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <random>
#include <chrono>
#include <iomanip>
#include <fstream>

using namespace std;
using namespace Eigen;
using namespace chrono;

// 性能测试结构体
struct PerformanceStats {
    double avg_time_ms = 0.0;
    double max_time_ms = 0.0;
    double min_time_ms = 0.0;
    double std_dev_ms = 0.0;
    int success_count = 0;
    int total_count = 0;
    
    double avg_position_error_mm = 0.0;  // 位置误差，单位：毫米
    double max_position_error_mm = 0.0;
    double min_position_error_mm = 0.0;
    double avg_angle_error_deg = 0.0;    // 角度误差，单位：度
    double max_angle_error_deg = 0.0;
};

// 精度测试：计算位置误差（单位：毫米）
double calculatePositionError(const Vector3d& target, const Vector3d& achieved, 
                            bool is_right_leg, ForwardKinematics& fk) {
    Vector3d error_vec = achieved - target;
    
    // 将误差转换为毫米（假设原始单位是米）
    double error_mm = error_vec.norm() * 1000.0;
    
    // 也可以计算各个方向上的误差
    double error_x_mm = fabs(error_vec.x()) * 1000.0;
    double error_y_mm = fabs(error_vec.y()) * 1000.0;
    double error_z_mm = fabs(error_vec.z()) * 1000.0;
    
    return error_mm;
}

// 角度误差计算（单位：度）
double calculateAngleError(const vector<double>& original_angles,
                          const InverseKinematics::IKSolution& solution) {
    double error_sum = 0.0;
    
    // θ1 误差
    error_sum += pow(solution.theta1 - original_angles[0], 2);
    // θ2 误差（需要从θ3和θ4计算）
    error_sum += pow(solution.theta2 - original_angles[1], 2);
    // θ3 误差
    error_sum += pow(solution.theta3 - original_angles[2], 2);
    // θ4 误差
    error_sum += pow(solution.theta4 - original_angles[3], 2);
    
    // 转换为度
    return sqrt(error_sum / 4.0) * 180.0 / M_PI;
}

// 完整的性能测试
PerformanceStats testPerformanceAndAccuracy(bool is_right_leg, int grid_size = 12, 
                                          int num_tests = 100) {
    string leg_name = is_right_leg ? "Right" : "Left";
    cout << "=== " << leg_name << " Leg Performance Test (Grid Size = " << grid_size << ") ===" << endl;
    
    ForwardKinematics fk(is_right_leg);
    InverseKinematics ik(fk);
    auto limits = fk.getJointLimits();
    
    PerformanceStats stats;
    stats.total_count = num_tests;
    
    random_device rd;
    mt19937 gen(rd());
    
    // 使用关节限制生成随机角度
    uniform_real_distribution<> dist_theta1(limits.theta1_lower, limits.theta1_upper);
    uniform_real_distribution<> dist_theta3(limits.theta3_lower, limits.theta3_upper);
    
    vector<double> execution_times_ms;
    vector<double> position_errors_mm;
    vector<double> angle_errors_deg;
    
    int valid_test_count = 0;
    
    for (int i = 0; i < num_tests; ++i) {
        // 生成随机但合理的关节角度
        double theta1 = dist_theta1(gen);
        double theta3 = dist_theta3(gen);
        
        // 选择theta4使得theta2在范围内
        double theta4_min = max(limits.theta2_lower - theta3, limits.theta4_lower);
        double theta4_max = min(limits.theta2_upper - theta3, limits.theta4_upper);
        
        if (theta4_min > theta4_max) {
            i--;  // 重新生成
            continue;
        }
        
        uniform_real_distribution<> dist_theta4(theta4_min, theta4_max);
        double theta4 = dist_theta4(gen);
        double theta2 = theta3 + theta4;
        
        // 保存原始角度用于误差计算
        vector<double> original_angles = {theta1, theta2, theta3, theta4};
        
        // 计算末端位置
        vector<Vector3d> positions = fk.computeAllJointPositions(
            theta1, theta2, theta3, theta4);
        Vector3d targetPos = positions.back();
        
        // 性能测试：计时
        auto start_time = high_resolution_clock::now();
        
        // 调用逆运动学
        InverseKinematics::IKSolution solution = ik.solve(targetPos, grid_size, 1e-2);
        
        auto end_time = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end_time - start_time);
        double time_ms = duration.count() / 1000.0;  // 转换为毫秒
        
        execution_times_ms.push_back(time_ms);
        
        if (solution.valid) {
            // 计算达到的末端位置
            vector<Vector3d> achieved_positions = fk.computeAllJointPositions(
                solution.theta1, solution.theta2, 
                solution.theta3, solution.theta4);
            Vector3d achievedPos = achieved_positions.back();
            
            // 计算误差
            double pos_error_mm = calculatePositionError(targetPos, achievedPos, is_right_leg, fk);
            double ang_error_deg = calculateAngleError(original_angles, solution);
            
            position_errors_mm.push_back(pos_error_mm);
            angle_errors_deg.push_back(ang_error_deg);
            
            stats.success_count++;
            valid_test_count++;
            
            if (i < 5) {  // 打印前5个测试的详细信息
                cout << "\nTest " << (i+1) << ":" << endl;
                cout << "  Original angles: " << fixed << setprecision(4)
                     << theta1 << ", " << theta2 << ", " << theta3 << ", " << theta4 << endl;
                cout << "  Solved angles:   " << solution.theta1 << ", " << solution.theta2
                     << ", " << solution.theta3 << ", " << solution.theta4 << endl;
                cout << "  Position error:  " << pos_error_mm << " mm" << endl;
                cout << "  Angle error:     " << ang_error_deg << " °" << endl;
                cout << "  Execution time:  " << time_ms << " ms" << endl;
            }
        }
    }
    
    // 计算统计信息
    if (!execution_times_ms.empty()) {
        // 时间统计
        double sum_time = 0.0;
        double sum_sq_time = 0.0;
        stats.max_time_ms = execution_times_ms[0];
        stats.min_time_ms = execution_times_ms[0];
        
        for (double time : execution_times_ms) {
            sum_time += time;
            sum_sq_time += time * time;
            stats.max_time_ms = max(stats.max_time_ms, time);
            stats.min_time_ms = min(stats.min_time_ms, time);
        }
        
        stats.avg_time_ms = sum_time / execution_times_ms.size();
        stats.std_dev_ms = sqrt(sum_sq_time / execution_times_ms.size() - 
                               stats.avg_time_ms * stats.avg_time_ms);
        
        // 位置误差统计
        if (!position_errors_mm.empty()) {
            double sum_pos_err = 0.0;
            stats.max_position_error_mm = position_errors_mm[0];
            stats.min_position_error_mm = position_errors_mm[0];
            
            for (double err : position_errors_mm) {
                sum_pos_err += err;
                stats.max_position_error_mm = max(stats.max_position_error_mm, err);
                stats.min_position_error_mm = min(stats.min_position_error_mm, err);
            }
            stats.avg_position_error_mm = sum_pos_err / position_errors_mm.size();
        }
        
        // 角度误差统计
        if (!angle_errors_deg.empty()) {
            double sum_ang_err = 0.0;
            stats.max_angle_error_deg = angle_errors_deg[0];
            
            for (double err : angle_errors_deg) {
                sum_ang_err += err;
                stats.max_angle_error_deg = max(stats.max_angle_error_deg, err);
            }
            stats.avg_angle_error_deg = sum_ang_err / angle_errors_deg.size();
        }
    }
    
    return stats;
}

// 测试不同位置的性能
void testVariousPositions(bool is_right_leg) {
    string leg_name = is_right_leg ? "Right" : "Left";
    cout << "\n=== " << leg_name << " Leg Various Positions Test ===" << endl;
    
    ForwardKinematics fk(is_right_leg);
    InverseKinematics ik(fk);
    
    struct TestCase {
        string name;
        vector<double> angles;  // theta1, theta2, theta3, theta4
    };
    
    // 定义一系列测试位置（典型工作空间位置）
    vector<TestCase> test_cases = {
        {"Zero Position", {0.0, 0.0, 0.0, 0.0}},
        {"Extended", {0.0, 0.5, 0.2, 0.3}},
        {"Bent", {0.3, -0.2, -0.4, 0.2}},
        {"Max Theta1", {1.2, 0.3, 0.1, 0.2}},
        {"Min Theta1", {-1.2, 0.3, 0.1, 0.2}},
        {"Complex 1", {0.5, 0.8, 0.3, 0.5}},
        {"Complex 2", {-0.5, -0.3, -0.2, -0.1}}
    };
    
    cout << setw(20) << left << "Test Case" 
         << setw(12) << "Time(ms)" 
         << setw(12) << "PosErr(mm)" 
         << setw(12) << "AngErr(°)" 
         << setw(10) << "Valid" 
         << endl;
    cout << string(66, '-') << endl;
    
    double total_time = 0.0;
    int success_count = 0;
    
    for (const auto& test_case : test_cases) {
        // 计算目标位置
        vector<Vector3d> positions = fk.computeAllJointPositions(
            test_case.angles[0], test_case.angles[1],
            test_case.angles[2], test_case.angles[3]);
        Vector3d targetPos = positions.back();
        
        // 性能测试
        auto start_time = high_resolution_clock::now();
        InverseKinematics::IKSolution solution = ik.solve(targetPos, 12, 1e-2);
        auto end_time = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(end_time - start_time);
        double time_ms = duration.count() / 1000.0;
        
        if (solution.valid) {
            // 计算误差
            vector<Vector3d> achieved_positions = fk.computeAllJointPositions(
                solution.theta1, solution.theta2, 
                solution.theta3, solution.theta4);
            Vector3d achievedPos = achieved_positions.back();
            
            double pos_error_mm = calculatePositionError(targetPos, achievedPos, is_right_leg, fk);
            double ang_error_deg = calculateAngleError(test_case.angles, solution);
            
            total_time += time_ms;
            success_count++;
            
            cout << setw(20) << left << test_case.name
                 << setw(12) << fixed << setprecision(2) << time_ms
                 << setw(12) << setprecision(3) << pos_error_mm
                 << setw(12) << setprecision(3) << ang_error_deg
                 << setw(10) << "✓" << endl;
        } else {
            cout << setw(20) << left << test_case.name
                 << setw(12) << fixed << setprecision(2) << time_ms
                 << setw(12) << "N/A"
                 << setw(12) << "N/A"
                 << setw(10) << "✗" << endl;
        }
    }
    
    cout << string(66, '-') << endl;
    if (success_count > 0) {
        cout << "Average time: " << fixed << setprecision(2) << total_time / success_count << " ms" << endl;
        cout << "Success rate: " << success_count << "/" << test_cases.size() << " (" 
             << (success_count * 100.0 / test_cases.size()) << "%)" << endl;
    }
}

// 生成性能报告
void generatePerformanceReport(const PerformanceStats& right_stats, 
                             const PerformanceStats& left_stats) {
    cout << "\n================================================" << endl;
    cout << "           PERFORMANCE TEST REPORT" << endl;
    cout << "          (Grid Size = 12, 100 tests)" << endl;
    cout << "================================================" << endl;
    
    // 输出表格标题
    cout << "\n" << setw(25) << left << "Metric"
         << setw(15) << "Right Leg"
         << setw(15) << "Left Leg"
         << setw(15) << "Unit" << endl;
    cout << string(70, '-') << endl;
    
    // 执行时间统计
    cout << setw(25) << left << "Avg Execution Time"
         << setw(15) << fixed << setprecision(2) << right_stats.avg_time_ms
         << setw(15) << left_stats.avg_time_ms
         << setw(15) << "ms" << endl;
    
    cout << setw(25) << "Min Execution Time"
         << setw(15) << right_stats.min_time_ms
         << setw(15) << left_stats.min_time_ms
         << setw(15) << "ms" << endl;
    
    cout << setw(25) << "Max Execution Time"
         << setw(15) << right_stats.max_time_ms
         << setw(15) << left_stats.max_time_ms
         << setw(15) << "ms" << endl;
    
    cout << setw(25) << "Time Std Dev"
         << setw(15) << right_stats.std_dev_ms
         << setw(15) << left_stats.std_dev_ms
         << setw(15) << "ms" << endl;
    
    cout << string(70, '-') << endl;
    
    // 位置误差统计
    cout << setw(25) << left << "Avg Position Error"
         << setw(15) << setprecision(3) << right_stats.avg_position_error_mm
         << setw(15) << left_stats.avg_position_error_mm
         << setw(15) << "mm" << endl;
    
    cout << setw(25) << "Min Position Error"
         << setw(15) << right_stats.min_position_error_mm
         << setw(15) << left_stats.min_position_error_mm
         << setw(15) << "mm" << endl;
    
    cout << setw(25) << "Max Position Error"
         << setw(15) << right_stats.max_position_error_mm
         << setw(15) << left_stats.max_position_error_mm
         << setw(15) << "mm" << endl;
    
    cout << string(70, '-') << endl;
    
    // 角度误差统计
    cout << setw(25) << left << "Avg Angle Error"
         << setw(15) << setprecision(3) << right_stats.avg_angle_error_deg
         << setw(15) << left_stats.avg_angle_error_deg
         << setw(15) << "°" << endl;
    
    cout << setw(25) << "Max Angle Error"
         << setw(15) << right_stats.max_angle_error_deg
         << setw(15) << left_stats.max_angle_error_deg
         << setw(15) << "°" << endl;
    
    cout << string(70, '-') << endl;
    
    // 成功率统计
    cout << setw(25) << left << "Success Count"
         << setw(15) << right_stats.success_count
         << setw(15) << left_stats.success_count
         << setw(15) << "" << endl;
    
    cout << setw(25) << "Success Rate"
         << setw(15) << fixed << setprecision(1) 
         << (right_stats.success_count * 100.0 / right_stats.total_count) << "%"
         << setw(15) << (left_stats.success_count * 100.0 / left_stats.total_count) << "%"
         << setw(15) << "" << endl;
    
    cout << string(70, '-') << endl;
    
    // 性能评估
    cout << "\nPERFORMANCE EVALUATION:" << endl;
    cout << "1. Real-time Capability:" << endl;
    if (right_stats.avg_time_ms < 10.0 && left_stats.avg_time_ms < 10.0) {
        cout << "   ✓ Suitable for real-time control (< 10ms)" << endl;
    } else if (right_stats.avg_time_ms < 50.0 && left_stats.avg_time_ms < 50.0) {
        cout << "   ○ Acceptable for near real-time (< 50ms)" << endl;
    } else {
        cout << "   ✗ May not be suitable for real-time applications" << endl;
    }
    
    cout << "\n2. Accuracy Assessment:" << endl;
    if (right_stats.avg_position_error_mm < 1.0 && left_stats.avg_position_error_mm < 1.0) {
        cout << "   ✓ High accuracy (< 1mm avg error)" << endl;
    } else if (right_stats.avg_position_error_mm < 5.0 && left_stats.avg_position_error_mm < 5.0) {
        cout << "   ○ Medium accuracy (< 5mm avg error)" << endl;
    } else {
        cout << "   ✗ Low accuracy, consider improving algorithm" << endl;
    }
    
    cout << "\n3. Reliability:" << endl;
    double right_success_rate = right_stats.success_count * 100.0 / right_stats.total_count;
    double left_success_rate = left_stats.success_count * 100.0 / left_stats.total_count;
    if (right_success_rate > 90.0 && left_success_rate > 90.0) {
        cout << "   ✓ High reliability (> 90% success rate)" << endl;
    } else if (right_success_rate > 70.0 && left_success_rate > 70.0) {
        cout << "   ○ Moderate reliability (> 70% success rate)" << endl;
    } else {
        cout << "   ✗ Low reliability, needs improvement" << endl;
    }
}

// 保存测试结果到CSV文件
void saveResultsToCSV(const PerformanceStats& stats, const string& filename, 
                     bool is_right_leg) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << " for writing" << endl;
        return;
    }
    
    string leg_name = is_right_leg ? "Right" : "Left";
    
    file << leg_name << " Leg Performance Test Results (Grid Size = 12)" << endl;
    file << "Metric,Value,Unit" << endl;
    file << "Avg Execution Time," << stats.avg_time_ms << ",ms" << endl;
    file << "Min Execution Time," << stats.min_time_ms << ",ms" << endl;
    file << "Max Execution Time," << stats.max_time_ms << ",ms" << endl;
    file << "Time Std Dev," << stats.std_dev_ms << ",ms" << endl;
    file << "Avg Position Error," << stats.avg_position_error_mm << ",mm" << endl;
    file << "Min Position Error," << stats.min_position_error_mm << ",mm" << endl;
    file << "Max Position Error," << stats.max_position_error_mm << ",mm" << endl;
    file << "Avg Angle Error," << stats.avg_angle_error_deg << ",deg" << endl;
    file << "Max Angle Error," << stats.max_angle_error_deg << ",deg" << endl;
    file << "Success Count," << stats.success_count << "," << endl;
    file << "Total Count," << stats.total_count << "," << endl;
    file << "Success Rate," << (stats.success_count * 100.0 / stats.total_count) << ",%" << endl;
    
    file.close();
    cout << "Results saved to " << filename << endl;
}

int main() {
    cout << "=========================================" << endl;
    cout << "  Inverse Kinematics Performance Tests" << endl;
    cout << "       Grid Size = 12, 100 tests" << endl;
    cout << "=========================================" << endl;
    
    try {
        // 测试右腿性能
        cout << "\n";
        PerformanceStats right_stats = testPerformanceAndAccuracy(true, 12, 100);
        testVariousPositions(true);
        
        // 测试左腿性能
        cout << "\n";
        PerformanceStats left_stats = testPerformanceAndAccuracy(false, 12, 100);
        testVariousPositions(false);
        
        // 生成报告
        generatePerformanceReport(right_stats, left_stats);
        
        // 保存结果
        saveResultsToCSV(right_stats, "right_leg_performance.csv", true);
        saveResultsToCSV(left_stats, "left_leg_performance.csv", false);
        
        cout << "\n=========================================" << endl;
        cout << "  Performance tests completed successfully!" << endl;
        cout << "=========================================" << endl;
        
        return 0;
        
    } catch (const exception& e) {
        cerr << "\nError during performance testing: " << e.what() << endl;
        return 1;
    } catch (...) {
        cerr << "\nUnknown error during performance testing" << endl;
        return 1;
    }
}