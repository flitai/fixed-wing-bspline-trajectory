// example_usage.cpp
#include "fixed_wing_bspline_trajectory.h"
#include "trajectory_optimizer.h"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace fixed_wing_planning;

// 创建一个简单的测试地图
void createTestMap(GridMap& map) {
    // 设置一些障碍物
    // 障碍物1：矩形
    for (int x = 100; x < 150; ++x) {
        for (int y = 80; y < 120; ++y) {
            map.setOccupied(x, y, true);
        }
    }
    
    // 障碍物2：圆形
    int cx = 250, cy = 150, radius = 30;
    for (int x = cx - radius; x <= cx + radius; ++x) {
        for (int y = cy - radius; y <= cy + radius; ++y) {
            if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= radius * radius) {
                map.setOccupied(x, y, true);
            }
        }
    }
    
    // 障碍物3：L形
    for (int x = 350; x < 400; ++x) {
        for (int y = 100; y < 150; ++y) {
            map.setOccupied(x, y, true);
        }
    }
    for (int x = 350; x < 450; ++x) {
        for (int y = 100; y < 130; ++y) {
            map.setOccupied(x, y, true);
        }
    }
    
    // 计算距离场
    std::cout << "计算距离场..." << std::endl;
    map.computeDistanceField();
}

// 保存轨迹到文件
void saveTrajectory(const std::vector<TrajectoryPoint>& trajectory,
                   const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << filename << std::endl;
        return;
    }
    
    file << "# time, x, y, vx, vy, ax, ay, speed, accel" << std::endl;
    for (const auto& point : trajectory) {
        double speed = point.velocity.norm();
        double accel = point.acceleration.norm();
        
        file << std::fixed << std::setprecision(6)
             << point.time << ", "
             << point.position.x << ", " << point.position.y << ", "
             << point.velocity.x << ", " << point.velocity.y << ", "
             << point.acceleration.x << ", " << point.acceleration.y << ", "
             << speed << ", " << accel << std::endl;
    }
    
    file.close();
    std::cout << "轨迹已保存到：" << filename << std::endl;
}

// 保存控制点到文件
void saveControlPoints(const std::vector<Point2D>& control_points,
                      const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << filename << std::endl;
        return;
    }
    
    file << "# x, y" << std::endl;
    for (const auto& point : control_points) {
        file << std::fixed << std::setprecision(6)
             << point.x << ", " << point.y << std::endl;
    }
    
    file.close();
    std::cout << "控制点已保存到：" << filename << std::endl;
}

// 主函数示例
int main() {
    std::cout << "=== 固定翼无人机B样条轨迹生成示例 ===" << std::endl;
    
    // 1. 创建栅格地图（500x300，分辨率0.5米）
    GridMap map(500, 300, 0.5);
    createTestMap(map);
    
    // 2. 设置固定翼动力学参数
    FixedWingDynamics dynamics;
    dynamics.v_min = 15.0;      // 最小速度 15 m/s
    dynamics.v_max = 30.0;      // 最大速度 30 m/s
    dynamics.a_max = 5.0;       // 最大加速度 5 m/s^2
    dynamics.r_min = 50.0;      // 最小转弯半径 50 m
    
    // 3. 设置B样条参数
    BSplineParams bspline_params;
    bspline_params.degree = 3;
    bspline_params.delta_t = 0.5;  // 时间间隔 0.5 秒
    
    // 4. 设置优化参数
    OptimizationParams opt_params;
    opt_params.lambda_smooth = 1.0;
    opt_params.lambda_velocity = 100.0;
    opt_params.lambda_accel = 50.0;
    opt_params.lambda_curvature = 100.0;
    opt_params.lambda_collision = 1000.0;
    opt_params.max_iterations = 100;
    opt_params.tolerance = 1e-4;
    
    // 5. 创建初始路径点（模拟从路径搜索算法得到的结果）
    std::vector<PathPoint> waypoints;
    
    // 起点
    waypoints.emplace_back(Point2D(25, 50), Point2D(20, 0));  
    
    // 中间路径点（绕过障碍物）
    waypoints.emplace_back(Point2D(75, 50), Point2D(20, 0));
    waypoints.emplace_back(Point2D(125, 30), Point2D(20, -5));  // 绕过第一个障碍物
    waypoints.emplace_back(Point2D(175, 50), Point2D(20, 5));
    waypoints.emplace_back(Point2D(225, 100), Point2D(15, 15));
    waypoints.emplace_back(Point2D(250, 200), Point2D(0, 20));  // 绕过第二个障碍物
    waypoints.emplace_back(Point2D(300, 225), Point2D(15, 0));
    waypoints.emplace_back(Point2D(350, 200), Point2D(20, -10));
    waypoints.emplace_back(Point2D(425, 175), Point2D(20, -5));  // 绕过第三个障碍物
    
    // 终点
    waypoints.emplace_back(Point2D(475, 150), Point2D(20, 0));
    
    std::cout << "\n原始路径点数量: " << waypoints.size() << std::endl;
    
    // 6. 创建B样条轨迹生成器
    FixedWingBSplineTrajectory trajectory_generator(
        map, dynamics, bspline_params, opt_params);
    
    // 7. 生成轨迹
    std::vector<TrajectoryPoint> trajectory;
    std::cout << "\n开始生成B样条轨迹..." << std::endl;
    
    bool success = trajectory_generator.generateTrajectory(waypoints, trajectory);
    
    if (success) {
        std::cout << "\n轨迹生成成功！" << std::endl;
        std::cout << "控制点数量: " << trajectory_generator.getControlPoints().size() << std::endl;
        std::cout << "轨迹点数量: " << trajectory.size() << std::endl;
        
        // 8. 保存结果
        saveTrajectory(trajectory, "trajectory_output.csv");
        saveControlPoints(trajectory_generator.getControlPoints(), "control_points.csv");
        
        // 9. 输出轨迹统计信息
        double min_speed = std::numeric_limits<double>::max();
        double max_speed = 0.0;
        double max_accel = 0.0;
        double max_curvature = 0.0;
        
        for (size_t i = 0; i < trajectory.size(); ++i) {
            double speed = trajectory[i].velocity.norm();
            double accel = trajectory[i].acceleration.norm();
            
            min_speed = std::min(min_speed, speed);
            max_speed = std::max(max_speed, speed);
            max_accel = std::max(max_accel, accel);
            
            if (i > 0 && i < trajectory.size() - 1) {
                double curvature = utils::computeCurvature(
                    trajectory[i-1].position,
                    trajectory[i].position,
                    trajectory[i+1].position
                );
                max_curvature = std::max(max_curvature, curvature);
            }
        }
        
        std::cout << "\n轨迹统计信息：" << std::endl;
        std::cout << "  最小速度: " << min_speed << " m/s" << std::endl;
        std::cout << "  最大速度: " << max_speed << " m/s" << std::endl;
        std::cout << "  最大加速度: " << max_accel << " m/s^2" << std::endl;
        std::cout << "  最大曲率: " << max_curvature << " 1/m" << std::endl;
        std::cout << "  对应最小转弯半径: " << 1.0 / max_curvature << " m" << std::endl;
        
        // 10. 检查约束满足情况
        std::cout << "\n约束检查：" << std::endl;
        bool constraints_satisfied = true;
        
        if (min_speed < dynamics.v_min) {
            std::cout << "  [警告] 最小速度违反: " << min_speed 
                      << " < " << dynamics.v_min << std::endl;
            constraints_satisfied = false;
        }
        if (max_speed > dynamics.v_max) {
            std::cout << "  [警告] 最大速度违反: " << max_speed 
                      << " > " << dynamics.v_max << std::endl;
            constraints_satisfied = false;
        }
        if (max_accel > dynamics.a_max) {
            std::cout << "  [警告] 最大加速度违反: " << max_accel 
                      << " > " << dynamics.a_max << std::endl;
            constraints_satisfied = false;
        }
        if (max_curvature > dynamics.kappa_max) {
            std::cout << "  [警告] 最大曲率违反: " << max_curvature 
                      << " > " << dynamics.kappa_max << std::endl;
            constraints_satisfied = false;
        }
        
        if (constraints_satisfied) {
            std::cout << "  [通过] 所有约束均满足" << std::endl;
        }
        
    } else {
        std::cerr << "\n轨迹生成失败！" << std::endl;
    }
    
    return 0;
}