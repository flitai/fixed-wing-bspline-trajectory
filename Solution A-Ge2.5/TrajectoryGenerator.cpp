#include "TrajectoryGenerator.h"
#include "TrajectoryOptimizer.h"
#include <iostream>
#include <algorithm>

TrajectoryGenerator::TrajectoryGenerator(const UAVParams& uav_params, 
                                       const BsplineParams& bspline_params, 
                                       const OptimizerWeights& weights,
                                       double map_resolution)
    : uav_params_(uav_params), 
      bspline_params_(bspline_params),
      weights_(weights),
      map_resolution_(map_resolution) {}

bool TrajectoryGenerator::generateTrajectory(const std::vector<InitialPathPoint>& initial_path,
                                             const DistanceField& df,
                                             std::vector<TrajectoryPoint>& final_trajectory,
                                             std::vector<ControlPoint>& optimal_control_points) {
    if (initial_path.size() < 2) {
        std::cerr << "初始路径点过少，无法生成轨迹。" << std::endl;
        return false;
    }

    // 步骤 1 & 2: 插值生成初始控制点
    std::cout << "步骤 1 & 2: 插值生成初始控制点..." << std::endl;
    std::vector<ControlPoint> initial_q = interpolateControlPoints(initial_path, df);
    std::cout << "插值后控制点数量: " << initial_q.size() << std::endl;
    refineControlPoints(initial_q);
    std::cout << "合并过近点后数量: " << initial_q.size() << std::endl;

    if (initial_q.size() < 7) {
         std::cerr << "前端处理后控制点数量不足7个，无法进行优化。" << std::endl;
        return false;
    }

    // 步骤 3 & 4: 后端优化
    std::cout << "步骤 3 & 4: 后端优化..." << std::endl;
    TrajectoryOptimizer optimizer(initial_q, uav_params_, bspline_params_, weights_, df, map_resolution_);
    optimal_control_points = optimizer.optimize();

    // 步骤 5: 构造最终B样条并离散化
    std::cout << "步骤 5: 离散化最终轨迹..." << std::endl;
    Bspline final_spline(bspline_params_.degree, optimal_control_points, bspline_params_.delta_t);
    final_trajectory = discretizeTrajectory(final_spline);
    
    // 最终检查
    std::cout << "执行最终安全检查..." << std::endl;
    if (!finalCheck(final_trajectory, df)) {
        std::cerr << "警告: 生成的轨迹未能通过最终安全检查！" << std::endl;
        return false;
    }

    std::cout << "轨迹生成成功！" << std::endl;
    return true;
}

std::vector<ControlPoint> TrajectoryGenerator::interpolateControlPoints(
    const std::vector<InitialPathPoint>& initial_path, const DistanceField& df) const {
    
    std::vector<ControlPoint> q;
    q.push_back(initial_path.front().pos);

    for (size_t i = 0; i < initial_path.size() - 1; ++i) {
        const auto& p_start = initial_path[i].pos;
        const auto& p_end = initial_path[i+1].pos;

        double d_c = std::min(getDistanceAt(df, p_start), getDistanceAt(df, p_end));
        if (d_c <= 0) {
            std::cerr << "错误: 初始路径点在障碍物内！" << std::endl;
            return {}; // 返回空表示失败
        }
        
        double r_max = std::min(d_c / 3.0, uav_params_.v_max * bspline_params_.delta_t);
        double l = (p_end - p_start).magnitude();

        if (l > r_max) {
            int n_ins = static_cast<int>(std::ceil(l / r_max)) - 1;
            if (n_ins > 0) {
                for (int k = 1; k <= n_ins; ++k) {
                    double alpha = static_cast<double>(k) / (n_ins + 1);
                    ControlPoint new_pt = p_start + (p_end - p_start) * alpha;
                    q.push_back(new_pt);
                }
            }
        }
        q.push_back(p_end);
    }
    return q;
}

void TrajectoryGenerator::refineControlPoints(std::vector<ControlPoint>& q) const {
    bool adjusted = true;
    double min_dist = uav_params_.v_min * bspline_params_.delta_t;

    while(adjusted){
        adjusted = false;
        if (q.size() < 2) break;

        for (auto it = q.begin(); it != q.end() - 1; ) {
            double dist = (*(it + 1) - *it).magnitude();
            if (dist < min_dist) {
                // 合并相邻过近的点（这里简单地移除后一个点）
                it = q.erase(it + 1);
                adjusted = true;
            } else {
                ++it;
            }
        }
    }
}

std::vector<TrajectoryPoint> TrajectoryGenerator::discretizeTrajectory(const Bspline& bspline) const {
    std::vector<TrajectoryPoint> trajectory;
    double total_time = bspline.getTotalTime();

    for (double t = 0; t <= total_time; t += bspline_params_.delta_t) {
        TrajectoryPoint p;
        p.timestamp = t;
        p.pos = bspline.evaluate(t);
        p.vel = bspline.evaluateDerivative(t);
        p.acc = bspline.evaluateSecondDerivative(t);
        trajectory.push_back(p);
    }
    return trajectory;
}

bool TrajectoryGenerator::finalCheck(const std::vector<TrajectoryPoint>& trajectory, const DistanceField& df) const {
    for (const auto& p : trajectory) {
        // 1. 碰撞检查
        if (getDistanceAt(df, p.pos) < map_resolution_) { // 留一个栅格宽度的安全裕量
            std::cerr << "最终检查失败: 轨迹点 (" << p.pos.x << ", " << p.pos.y << ") 与障碍物碰撞。" << std::endl;
            return false;
        }
        // 2. 动力学检查
        double v = p.vel.magnitude();
        if (v < uav_params_.v_min * 0.95 || v > uav_params_.v_max * 1.05) { // 允许5%的容忍度
             std::cerr << "最终检查失败: 速度 " << v << " 超出范围 [" << uav_params_.v_min << ", " << uav_params_.v_max << "]" << std::endl;
            return false;
        }
        double a = p.acc.magnitude();
        if (a > uav_params_.a_max * 1.05) {
             std::cerr << "最终检查失败: 加速度 " << a << " 超出最大值 " << uav_params_.a_max << std::endl;
            return false;
        }
        // 3. 曲率检查
        double kappa_num = std::abs(p.vel.x * p.acc.y - p.vel.y * p.acc.x);
        double kappa_den = std::pow(v, 3);
        if (kappa_den > 1e-6) {
            double kappa = kappa_num / kappa_den;
            if (kappa > uav_params_.kappa_max * 1.05) {
                std::cerr << "最终检查失败: 曲率 " << kappa << " 超出最大值 " << uav_params_.kappa_max << std::endl;
                return false;
            }
        }
    }
    return true;
}

double TrajectoryGenerator::getDistanceAt(const DistanceField& df, const Vector2D& world_pos) const {
     if (df.empty() || df[0].empty()) return std::numeric_limits<double>::max();
    int x_idx = static_cast<int>(world_pos.x / map_resolution_);
    int y_idx = static_cast<int>(world_pos.y / map_resolution_);
    x_idx = std::max(0, std::min((int)df[0].size() - 1, x_idx));
    y_idx = std::max(0, std::min((int)df.size() - 1, y_idx));
    return df[y_idx][x_idx];
}