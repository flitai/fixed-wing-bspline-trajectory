// trajectory_optimizer.cpp
#include "trajectory_optimizer.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

namespace fixed_wing_planning {

// ===== SimpleTrajectoryOptimizer 实现 =====
SimpleTrajectoryOptimizer::SimpleTrajectoryOptimizer(
    const GridMap& map,
    const FixedWingDynamics& dynamics,
    const BSplineParams& bspline_params,
    const OptimizationParams& opt_params)
    : map_(map), dynamics_(dynamics), 
      bspline_params_(bspline_params), opt_params_(opt_params) {
}

bool SimpleTrajectoryOptimizer::optimize(std::vector<Point2D>& control_points) {
    const int n = control_points.size();
    const int fixed_start = 3;
    const int fixed_end = 3;
    
    if (n <= fixed_start + fixed_end) {
        // 控制点太少，无需优化
        return true;
    }
    
    // 使用L-BFGS或梯度下降进行优化
    double prev_cost = std::numeric_limits<double>::max();
    const double min_cost_decrease = 1e-6;
    
    for (int iter = 0; iter < opt_params_.max_iterations; ++iter) {
        // 计算当前成本
        CostComponents costs = computeCost(control_points);
        
        if (iter % 10 == 0) {
            std::cout << "优化迭代 " << iter << ":" << std::endl;
            std::cout << "  总成本: " << costs.total_cost << std::endl;
            std::cout << "  平滑成本: " << costs.smooth_cost << std::endl;
            std::cout << "  速度成本: " << costs.velocity_cost << std::endl;
            std::cout << "  加速度成本: " << costs.acceleration_cost << std::endl;
            std::cout << "  曲率成本: " << costs.curvature_cost << std::endl;
            std::cout << "  碰撞成本: " << costs.collision_cost << std::endl;
        }
        
        // 检查收敛
        if (std::abs(prev_cost - costs.total_cost) < min_cost_decrease) {
            std::cout << "优化收敛，迭代次数: " << iter << std::endl;
            break;
        }
        prev_cost = costs.total_cost;
        
        // 计算梯度
        std::vector<Point2D> gradients = computeGradient(control_points);
        
        // 自适应学习率
        double learning_rate = 0.1 / (1.0 + iter * 0.01);
        
        // 更新控制点（跳过固定的首尾控制点）
        for (int i = fixed_start; i < n - fixed_end; ++i) {
            control_points[i].x -= learning_rate * gradients[i].x;
            control_points[i].y -= learning_rate * gradients[i].y;
        }
        
        // 投影到可行域
        projectToFeasible(control_points);
    }
    
    // 最终检查约束
    return checkConstraints(control_points);
}

SimpleTrajectoryOptimizer::CostComponents 
SimpleTrajectoryOptimizer::computeCost(const std::vector<Point2D>& points) {
    CostComponents costs;
    const int n = points.size();
    
    // 1. 平滑成本 - 最小化三阶导数（加加速度）
    for (int i = 0; i < n - 3; ++i) {
        Point2D jerk = points[i+3] - points[i+2] * 3.0 + points[i+1] * 3.0 - points[i];
        double jerk_squared = jerk.x * jerk.x + jerk.y * jerk.y;
        costs.smooth_cost += jerk_squared;
    }
    
    // 2. 速度约束成本
    for (int i = 0; i < n - 1; ++i) {
        double dist = utils::distance(points[i], points[i+1]);
        double velocity = dist / bspline_params_.delta_t;
        
        if (velocity > dynamics_.v_max) {
            double excess = velocity - dynamics_.v_max;
            costs.velocity_cost += excess * excess;
        } else if (velocity < dynamics_.v_min) {
            double deficit = dynamics_.v_min - velocity;
            costs.velocity_cost += deficit * deficit;
        }
    }
    
    // 3. 加速度约束成本
    for (int i = 0; i < n - 2; ++i) {
        Point2D accel = (points[i+2] - points[i+1] * 2.0 + points[i]) * 
                        (1.0 / (bspline_params_.delta_t * bspline_params_.delta_t));
        double accel_mag = accel.norm();
        
        if (accel_mag > dynamics_.a_max) {
            double excess = accel_mag - dynamics_.a_max;
            costs.acceleration_cost += excess * excess;
        }
    }
    
    // 4. 曲率约束成本
    for (int i = 0; i < n - 2; ++i) {
        double curvature = utils::computeCurvature(points[i], points[i+1], points[i+2]);
        
        if (curvature > dynamics_.kappa_max) {
            double excess = curvature - dynamics_.kappa_max;
            costs.curvature_cost += excess * excess;
        }
    }
    
    // 5. 碰撞避免成本
    const double safe_distance = 5.0; // 安全距离阈值（米）
    
    // 检查每个控制点
    for (int i = 0; i < n; ++i) {
        double dist = map_.getDistance(points[i].x, points[i].y);
        if (dist < safe_distance) {
            double penalty = safe_distance - dist;
            costs.collision_cost += penalty * penalty;
        }
    }
    
    // 检查四点凸包
    for (int i = 0; i < n - 3; ++i) {
        // 计算四点的最小障碍距离
        double min_dist = std::numeric_limits<double>::max();
        for (int j = 0; j < 4; ++j) {
            min_dist = std::min(min_dist, 
                               map_.getDistance(points[i+j].x, points[i+j].y));
        }
        
        // 计算相邻点间距总和
        double total_spacing = 0;
        for (int j = 0; j < 3; ++j) {
            total_spacing += utils::distance(points[i+j], points[i+j+1]);
        }
        
        // 凸包安全性惩罚
        if (min_dist < total_spacing + safe_distance) {
            double penalty = total_spacing + safe_distance - min_dist;
            costs.collision_cost += penalty * penalty * 0.5; // 权重稍低
        }
    }
    
    // 计算总成本
    costs.total_cost = opt_params_.lambda_smooth * costs.smooth_cost +
                       opt_params_.lambda_velocity * costs.velocity_cost +
                       opt_params_.lambda_accel * costs.acceleration_cost +
                       opt_params_.lambda_curvature * costs.curvature_cost +
                       opt_params_.lambda_collision * costs.collision_cost;
    
    return costs;
}

std::vector<Point2D> SimpleTrajectoryOptimizer::computeGradient(
    const std::vector<Point2D>& points) {
    
    const int n = points.size();
    std::vector<Point2D> gradients(n, Point2D(0, 0));
    const double eps = 1e-6; // 数值微分步长
    
    // 固定首尾控制点
    const int fixed_start = 3;
    const int fixed_end = 3;
    
    // 对每个自由控制点计算梯度
    for (int i = fixed_start; i < n - fixed_end; ++i) {
        // 计算关于x的偏导数
        std::vector<Point2D> points_plus_x = points;
        points_plus_x[i].x += eps;
        double cost_plus_x = computeCost(points_plus_x).total_cost;
        
        std::vector<Point2D> points_minus_x = points;
        points_minus_x[i].x -= eps;
        double cost_minus_x = computeCost(points_minus_x).total_cost;
        
        gradients[i].x = (cost_plus_x - cost_minus_x) / (2.0 * eps);
        
        // 计算关于y的偏导数
        std::vector<Point2D> points_plus_y = points;
        points_plus_y[i].y += eps;
        double cost_plus_y = computeCost(points_plus_y).total_cost;
        
        std::vector<Point2D> points_minus_y = points;
        points_minus_y[i].y -= eps;
        double cost_minus_y = computeCost(points_minus_y).total_cost;
        
        gradients[i].y = (cost_plus_y - cost_minus_y) / (2.0 * eps);
    }
    
    return gradients;
}

bool SimpleTrajectoryOptimizer::checkConstraints(const std::vector<Point2D>& points) {
    const int n = points.size();
    bool all_satisfied = true;
    
    // 检查速度约束
    for (int i = 0; i < n - 1; ++i) {
        double velocity = utils::distance(points[i], points[i+1]) / bspline_params_.delta_t;
        if (velocity < dynamics_.v_min || velocity > dynamics_.v_max) {
            std::cerr << "速度约束违反：段 " << i << ", 速度 = " << velocity << " m/s" << std::endl;
            all_satisfied = false;
        }
    }
    
    // 检查加速度约束
    for (int i = 0; i < n - 2; ++i) {
        Point2D accel = (points[i+2] - points[i+1] * 2.0 + points[i]) * 
                        (1.0 / (bspline_params_.delta_t * bspline_params_.delta_t));
        double accel_mag = accel.norm();
        
        if (accel_mag > dynamics_.a_max) {
            std::cerr << "加速度约束违反：段 " << i << ", 加速度 = " << accel_mag << " m/s^2" << std::endl;
            all_satisfied = false;
        }
    }
    
    // 检查曲率约束
    for (int i = 0; i < n - 2; ++i) {
        double curvature = utils::computeCurvature(points[i], points[i+1], points[i+2]);
        if (curvature > dynamics_.kappa_max) {
            std::cerr << "曲率约束违反：段 " << i << ", 曲率 = " << curvature << " 1/m" << std::endl;
            all_satisfied = false;
        }
    }
    
    // 检查碰撞
    for (int i = 0; i < n; ++i) {
        int gx, gy;
        map_.worldToGrid(points[i].x, points[i].y, gx, gy);
        if (map_.isOccupied(gx, gy)) {
            std::cerr << "碰撞检测失败：控制点 " << i << " 在障碍物内" << std::endl;
            all_satisfied = false;
        }
    }
    
    return all_satisfied;
}

void SimpleTrajectoryOptimizer::projectToFeasible(std::vector<Point2D>& points) {
    const int n = points.size();
    const int fixed_start = 3;
    const int fixed_end = 3;
    
    // 速度约束投影
    for (int i = fixed_start; i < n - fixed_end - 1; ++i) {
        double dist = utils::distance(points[i], points[i+1]);
        double velocity = dist / bspline_params_.delta_t;
        
        if (velocity > dynamics_.v_max) {
            // 缩短距离
            double scale = (dynamics_.v_max * bspline_params_.delta_t) / dist;
            Point2D mid = (points[i] + points[i+1]) * 0.5;
            points[i] = mid + (points[i] - mid) * scale;
            points[i+1] = mid + (points[i+1] - mid) * scale;
        } else if (velocity < dynamics_.v_min && dist > 0) {
            // 增加距离
            double scale = (dynamics_.v_min * bspline_params_.delta_t) / dist;
            Point2D mid = (points[i] + points[i+1]) * 0.5;
            points[i] = mid + (points[i] - mid) * scale;
            points[i+1] = mid + (points[i+1] - mid) * scale;
        }
    }
    
    // 碰撞约束投影
    for (int i = fixed_start; i < n - fixed_end; ++i) {
        double dist = map_.getDistance(points[i].x, points[i].y);
        const double min_clearance = 2.0; // 最小安全距离
        
        if (dist < min_clearance) {
            // 计算梯度方向（远离障碍物）
            const double eps = 0.1;
            double dist_x_plus = map_.getDistance(points[i].x + eps, points[i].y);
            double dist_x_minus = map_.getDistance(points[i].x - eps, points[i].y);
            double dist_y_plus = map_.getDistance(points[i].x, points[i].y + eps);
            double dist_y_minus = map_.getDistance(points[i].x, points[i].y - eps);
            
            double grad_x = (dist_x_plus - dist_x_minus) / (2.0 * eps);
            double grad_y = (dist_y_plus - dist_y_minus) / (2.0 * eps);
            
            double grad_norm = std::sqrt(grad_x * grad_x + grad_y * grad_y);
            if (grad_norm > 0) {
                // 沿梯度方向移动
                double move_dist = min_clearance - dist;
                points[i].x += (grad_x / grad_norm) * move_dist;
                points[i].y += (grad_y / grad_norm) * move_dist;
            }
        }
    }
}

} // namespace fixed_wing_planning