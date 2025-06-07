// fixed_wing_bspline_trajectory.cpp
#include "fixed_wing_bspline_trajectory.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <iostream>

namespace fixed_wing_planning {

// ===== GridMap 实现 =====
GridMap::GridMap(int width, int height, double resolution)
    : width_(width), height_(height), resolution_(resolution) {
    occupancy_.resize(height_, std::vector<bool>(width_, false));
    distance_field_.resize(height_, std::vector<double>(width_, 0.0));
}

void GridMap::setOccupied(int x, int y, bool occupied) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        occupancy_[y][x] = occupied;
    }
}

bool GridMap::isOccupied(int x, int y) const {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        return occupancy_[y][x];
    }
    return true; // 边界外视为占用
}

void GridMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor(wx / resolution_));
    gy = static_cast<int>(std::floor(wy / resolution_));
}

void GridMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = (gx + 0.5) * resolution_;
    wy = (gy + 0.5) * resolution_;
}

void GridMap::computeDistanceField() {
    computeEDT();
}

void GridMap::computeEDT() {
    // 使用简化的欧氏距离变换算法
    const double INF = 1e9;
    
    // 初始化距离场
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            distance_field_[y][x] = occupancy_[y][x] ? 0.0 : INF;
        }
    }
    
    // 使用BFS计算距离场（简化版本，实际应用中可以使用更高效的算法）
    std::queue<std::pair<int, int>> q;
    std::vector<std::vector<bool>> visited(height_, std::vector<bool>(width_, false));
    
    // 将所有障碍物点加入队列
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (occupancy_[y][x]) {
                q.push({x, y});
                visited[y][x] = true;
            }
        }
    }
    
    // 8邻域偏移
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const double dist[] = {std::sqrt(2), 1, std::sqrt(2), 1, 1, std::sqrt(2), 1, std::sqrt(2)};
    
    while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();
        
        for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && !visited[ny][nx]) {
                distance_field_[ny][nx] = distance_field_[cy][cx] + dist[i] * resolution_;
                visited[ny][nx] = true;
                q.push({nx, ny});
            }
        }
    }
}

double GridMap::getDistance(double x, double y) const {
    int gx, gy;
    worldToGrid(x, y, gx, gy);
    return getDistanceGrid(gx, gy);
}

double GridMap::getDistanceGrid(int x, int y) const {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        return distance_field_[y][x];
    }
    return 0.0; // 边界外返回0
}

// ===== BSplineCurve 实现 =====
BSplineCurve::BSplineCurve(int degree) : degree_(degree) {}

void BSplineCurve::setControlPoints(const std::vector<Point2D>& control_points) {
    control_points_ = control_points;
}

void BSplineCurve::setUniformKnots(double delta_t) {
    int n = control_points_.size() - 1;
    int m = n + degree_ + 1;
    knots_.resize(m + 1);
    
    // 首尾打结
    for (int i = 0; i <= degree_; ++i) {
        knots_[i] = 0.0;
    }
    
    // 中间均匀节点
    for (int i = degree_ + 1; i <= n; ++i) {
        knots_[i] = knots_[i-1] + delta_t;
    }
    
    // 末尾打结
    double t_max = knots_[n];
    for (int i = n + 1; i <= m; ++i) {
        knots_[i] = t_max;
    }
}

double BSplineCurve::basisFunction(int i, int p, double t) const {
    return basisFunctionRecursive(i, p, t);
}

double BSplineCurve::basisFunctionRecursive(int i, int p, double t) const {
    if (p == 0) {
        return (t >= knots_[i] && t < knots_[i+1]) ? 1.0 : 0.0;
    }
    
    double left = 0.0, right = 0.0;
    
    if (knots_[i+p] != knots_[i]) {
        left = (t - knots_[i]) / (knots_[i+p] - knots_[i]) * 
               basisFunctionRecursive(i, p-1, t);
    }
    
    if (knots_[i+p+1] != knots_[i+1]) {
        right = (knots_[i+p+1] - t) / (knots_[i+p+1] - knots_[i+1]) * 
                basisFunctionRecursive(i+1, p-1, t);
    }
    
    return left + right;
}

Point2D BSplineCurve::evaluate(double t) const {
    Point2D result(0, 0);
    int n = control_points_.size() - 1;
    
    for (int i = 0; i <= n; ++i) {
        double basis = basisFunction(i, degree_, t);
        result = result + control_points_[i] * basis;
    }
    
    return result;
}

Point2D BSplineCurve::evaluateDerivative(double t, int order) const {
    if (order == 0) {
        return evaluate(t);
    }
    
    // 使用数值微分（简化实现）
    const double h = 1e-6;
    if (order == 1) {
        Point2D p1 = evaluate(t + h);
        Point2D p0 = evaluate(t - h);
        return (p1 - p0) * (1.0 / (2.0 * h));
    } else if (order == 2) {
        Point2D p1 = evaluate(t + h);
        Point2D p0 = evaluate(t);
        Point2D p_1 = evaluate(t - h);
        return (p1 - p0 * 2.0 + p_1) * (1.0 / (h * h));
    }
    
    return Point2D(0, 0);
}

double BSplineCurve::getMinParameter() const {
    return knots_[degree_];
}

double BSplineCurve::getMaxParameter() const {
    return knots_[knots_.size() - degree_ - 1];
}

// ===== 工具函数实现 =====
namespace utils {

double computeCurvature(const Point2D& p1, const Point2D& p2, const Point2D& p3) {
    // 使用三点计算离散曲率
    Point2D v1 = p2 - p1;
    Point2D v2 = p3 - p2;
    
    double cross = v1.x * v2.y - v1.y * v2.x;
    double d1 = v1.norm();
    double d2 = v2.norm();
    double d3 = (p3 - p1).norm();
    
    if (d1 * d2 * d3 < 1e-10) {
        return 0.0;
    }
    
    return 2.0 * std::abs(cross) / (d1 * d2 * d3);
}

} // namespace utils

// ===== FixedWingBSplineTrajectory 实现 =====
FixedWingBSplineTrajectory::FixedWingBSplineTrajectory(
    const GridMap& map,
    const FixedWingDynamics& dynamics,
    const BSplineParams& bspline_params,
    const OptimizationParams& opt_params)
    : map_(map), dynamics_(dynamics), 
      bspline_params_(bspline_params), opt_params_(opt_params),
      bspline_curve_(bspline_params.degree) {
    dynamics_.kappa_max = 1.0 / dynamics_.r_min;
}

bool FixedWingBSplineTrajectory::generateTrajectory(
    const std::vector<PathPoint>& waypoints,
    std::vector<TrajectoryPoint>& trajectory) {
    
    if (waypoints.size() < 2) {
        std::cerr << "错误：路径点数量不足" << std::endl;
        return false;
    }
    
    // 步骤1：计算每段到障碍的最小距离
    std::vector<double> segment_distances = computeSegmentDistances(waypoints);
    
    // 步骤2：等距插值生成初始控制点
    if (!interpolateControlPoints(waypoints, segment_distances)) {
        std::cerr << "错误：插值失败" << std::endl;
        return false;
    }
    
    // 步骤3：构造B样条
    constructBSpline();
    
    // 步骤4：后端优化
    if (!optimizeTrajectory()) {
        std::cerr << "错误：优化失败" << std::endl;
        return false;
    }
    
    // 步骤5：离散化输出
    discretizeTrajectory(trajectory);
    
    return true;
}

std::vector<double> FixedWingBSplineTrajectory::computeSegmentDistances(
    const std::vector<PathPoint>& waypoints) {
    
    std::vector<double> distances;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const Point2D& p1 = waypoints[i].position;
        const Point2D& p2 = waypoints[i + 1].position;
        
        // 查询两端点的距离场值
        double d1 = map_.getDistance(p1.x, p1.y);
        double d2 = map_.getDistance(p2.x, p2.y);
        
        // 取最小值作为该段的障碍距离
        double min_dist = std::min(d1, d2);
        
        // 可以在线段上采样更多点以获得更准确的距离
        const int num_samples = 5;
        for (int j = 1; j < num_samples; ++j) {
            double t = static_cast<double>(j) / num_samples;
            Point2D p = utils::lerp(p1, p2, t);
            min_dist = std::min(min_dist, map_.getDistance(p.x, p.y));
        }
        
        distances.push_back(min_dist);
    }
    
    return distances;
}

double FixedWingBSplineTrajectory::computeMaxSpacing(double obstacle_distance) const {
    // 根据算法：r_max = min(d_c/3, v_max * delta_t)
    return std::min(obstacle_distance / 3.0, dynamics_.v_max * bspline_params_.delta_t);
}

bool FixedWingBSplineTrajectory::interpolateControlPoints(
    const std::vector<PathPoint>& waypoints,
    const std::vector<double>& segment_distances) {
    
    control_points_.clear();
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const Point2D& p1 = waypoints[i].position;
        const Point2D& p2 = waypoints[i + 1].position;
        
        // 添加起点
        if (i == 0) {
            control_points_.push_back(p1);
        }
        
        double segment_length = utils::distance(p1, p2);
        double max_spacing = computeMaxSpacing(segment_distances[i]);
        
        if (segment_length <= max_spacing) {
            // 不需要插值
            control_points_.push_back(p2);
        } else {
            // 需要插值
            int num_insertions = static_cast<int>(std::ceil(segment_length / max_spacing)) - 1;
            
            for (int k = 1; k <= num_insertions; ++k) {
                double t = static_cast<double>(k) / (num_insertions + 1);
                Point2D interpolated = utils::lerp(p1, p2, t);
                control_points_.push_back(interpolated);
            }
            
            control_points_.push_back(p2);
        }
    }
    
    // 检查相邻控制点间距是否满足速度约束
    return checkVelocityConstraints(control_points_);
}

bool FixedWingBSplineTrajectory::checkVelocityConstraints(
    const std::vector<Point2D>& points) const {
    
    for (size_t i = 0; i < points.size() - 1; ++i) {
        double dist = utils::distance(points[i], points[i + 1]);
        double implied_velocity = dist / bspline_params_.delta_t;
        
        if (implied_velocity < dynamics_.v_min || implied_velocity > dynamics_.v_max) {
            // 需要调整
            std::cerr << "警告：第" << i << "段速度不满足约束: " 
                      << implied_velocity << " m/s" << std::endl;
            // 这里可以实现合并或进一步插值的逻辑
        }
    }
    
    return true;
}

void FixedWingBSplineTrajectory::constructBSpline() {
    bspline_curve_.setControlPoints(control_points_);
    bspline_curve_.setUniformKnots(bspline_params_.delta_t);
}

bool FixedWingBSplineTrajectory::optimizeTrajectory() {
    // 这里实现简化版本的优化
    // 实际应用中应该使用IPOPT等专业优化库
    
    // 固定首尾各3个控制点
    const int fixed_start = 3;
    const int fixed_end = 3;
    const int n = control_points_.size();
    
    if (n <= fixed_start + fixed_end) {
        // 控制点太少，无需优化
        return true;
    }
    
    // 简单的梯度下降优化示例
    std::vector<Point2D> optimized_points = control_points_;
    const double learning_rate = 0.01;
    
    for (int iter = 0; iter < opt_params_.max_iterations; ++iter) {
        std::vector<Point2D> gradients(n, Point2D(0, 0));
        double total_cost = 0.0;
        
        // 计算各项成本和梯度
        // 1. 平滑性成本（简化：最小化相邻控制点距离变化）
        for (int i = fixed_start + 1; i < n - fixed_end - 1; ++i) {
            Point2D d1 = optimized_points[i] - optimized_points[i-1];
            Point2D d2 = optimized_points[i+1] - optimized_points[i];
            Point2D diff = d2 - d1;
            
            gradients[i] = gradients[i] + diff * (2.0 * opt_params_.lambda_smooth);
            total_cost += opt_params_.lambda_smooth * (diff.x * diff.x + diff.y * diff.y);
        }
        
        // 2. 碰撞避免成本（简化版本）
        for (int i = fixed_start; i < n - fixed_end; ++i) {
            double dist = map_.getDistance(optimized_points[i].x, optimized_points[i].y);
            double safe_dist = 5.0; // 安全距离阈值
            
            if (dist < safe_dist) {
                // 计算数值梯度
                const double eps = 0.1;
                double dist_x_plus = map_.getDistance(optimized_points[i].x + eps, 
                                                      optimized_points[i].y);
                double dist_x_minus = map_.getDistance(optimized_points[i].x - eps, 
                                                       optimized_points[i].y);
                double dist_y_plus = map_.getDistance(optimized_points[i].x, 
                                                      optimized_points[i].y + eps);
                double dist_y_minus = map_.getDistance(optimized_points[i].x, 
                                                       optimized_points[i].y - eps);
                
                double grad_x = (dist_x_plus - dist_x_minus) / (2.0 * eps);
                double grad_y = (dist_y_plus - dist_y_minus) / (2.0 * eps);
                
                double penalty = opt_params_.lambda_collision * (safe_dist - dist) * (safe_dist - dist);
                gradients[i].x += -2.0 * opt_params_.lambda_collision * (safe_dist - dist) * grad_x;
                gradients[i].y += -2.0 * opt_params_.lambda_collision * (safe_dist - dist) * grad_y;
                
                total_cost += penalty;
            }
        }
        
        // 更新控制点
        for (int i = fixed_start; i < n - fixed_end; ++i) {
            optimized_points[i].x -= learning_rate * gradients[i].x;
            optimized_points[i].y -= learning_rate * gradients[i].y;
        }
        
        // 检查收敛
        if (iter % 10 == 0) {
            std::cout << "迭代 " << iter << ", 成本: " << total_cost << std::endl;
        }
        
        if (total_cost < opt_params_.tolerance) {
            break;
        }
    }
    
    control_points_ = optimized_points;
    bspline_curve_.setControlPoints(control_points_);
    
    return checkCollisionFree(control_points_);
}

bool FixedWingBSplineTrajectory::checkCollisionFree(
    const std::vector<Point2D>& points) const {
    
    // 检查每个控制点是否在自由空间
    for (const auto& point : points) {
        int gx, gy;
        map_.worldToGrid(point.x, point.y, gx, gy);
        if (map_.isOccupied(gx, gy)) {
            return false;
        }
    }
    
    // 检查四点凸包的安全性
    for (size_t i = 0; i + 3 < points.size(); ++i) {
        // 获取四个连续控制点
        const Point2D& p0 = points[i];
        const Point2D& p1 = points[i + 1];
        const Point2D& p2 = points[i + 2];
        const Point2D& p3 = points[i + 3];
        
        // 计算四点的最小障碍距离
        double d0 = map_.getDistance(p0.x, p0.y);
        double d1 = map_.getDistance(p1.x, p1.y);
        double d2 = map_.getDistance(p2.x, p2.y);
        double d3 = map_.getDistance(p3.x, p3.y);
        double min_dist = std::min({d0, d1, d2, d3});
        
        // 计算相邻点间距总和
        double r01 = utils::distance(p0, p1);
        double r12 = utils::distance(p1, p2);
        double r23 = utils::distance(p2, p3);
        double total_spacing = r01 + r12 + r23;
        
        // 凸包安全性检查
        if (min_dist <= total_spacing) {
            std::cerr << "警告：四点凸包可能与障碍物碰撞" << std::endl;
            // 在实际应用中，这里应该返回false或进行更精确的凸包碰撞检测
        }
    }
    
    return true;
}

void FixedWingBSplineTrajectory::discretizeTrajectory(
    std::vector<TrajectoryPoint>& trajectory) {
    
    trajectory.clear();
    
    double t_min = bspline_curve_.getMinParameter();
    double t_max = bspline_curve_.getMaxParameter();
    
    // 计算离散点数量
    int num_points = static_cast<int>((t_max - t_min) / bspline_params_.delta_t) + 1;
    
    for (int i = 0; i < num_points; ++i) {
        double t = t_min + i * bspline_params_.delta_t;
        if (t > t_max) {
            t = t_max;
        }
        
        TrajectoryPoint traj_point;
        traj_point.time = t;
        traj_point.position = bspline_curve_.evaluate(t);
        traj_point.velocity = bspline_curve_.evaluateDerivative(t, 1);
        traj_point.acceleration = bspline_curve_.evaluateDerivative(t, 2);
        
        // 检查动力学约束
        double speed = traj_point.velocity.norm();
        if (speed < dynamics_.v_min || speed > dynamics_.v_max) {
            std::cerr << "警告：时刻 " << t << " 的速度不满足约束: " 
                      << speed << " m/s" << std::endl;
        }
        
        double accel = traj_point.acceleration.norm();
        if (accel > dynamics_.a_max) {
            std::cerr << "警告：时刻 " << t << " 的加速度超过限制: " 
                      << accel << " m/s^2" << std::endl;
        }
        
        trajectory.push_back(traj_point);
    }
    
    // 计算并检查曲率
    for (size_t i = 1; i < trajectory.size() - 1; ++i) {
        double curvature = utils::computeCurvature(
            trajectory[i-1].position,
            trajectory[i].position,
            trajectory[i+1].position
        );
        
        if (curvature > dynamics_.kappa_max) {
            std::cerr << "警告：时刻 " << trajectory[i].time 
                      << " 的曲率超过限制: " << curvature << " 1/m" << std::endl;
        }
    }
}

} // namespace fixed_wing_planning