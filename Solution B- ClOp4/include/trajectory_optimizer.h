// trajectory_optimizer.h
#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#include "fixed_wing_bspline_trajectory.h"
#include <vector>
#include <memory>

// 如果使用IPOPT，需要包含IPOPT头文件
// #include "IpIpoptApplication.hpp"
// #include "IpTNLP.hpp"

namespace fixed_wing_planning {

// 轨迹优化器类 - 使用IPOPT进行非线性优化
class TrajectoryOptimizer {
public:
    TrajectoryOptimizer(
        const GridMap& map,
        const FixedWingDynamics& dynamics,
        const BSplineParams& bspline_params,
        const OptimizationParams& opt_params);
    
    // 执行优化
    bool optimize(std::vector<Point2D>& control_points);
    
private:
    const GridMap& map_;
    const FixedWingDynamics& dynamics_;
    const BSplineParams& bspline_params_;
    const OptimizationParams& opt_params_;
    
    // 内部优化问题定义
    class OptimizationProblem; // : public Ipopt::TNLP
    std::unique_ptr<OptimizationProblem> problem_;
    
    // 辅助函数
    void setupProblem(const std::vector<Point2D>& initial_points);
    void extractSolution(std::vector<Point2D>& optimized_points);
};

// 如果不使用IPOPT，这里提供一个简化的优化器实现
class SimpleTrajectoryOptimizer {
public:
    SimpleTrajectoryOptimizer(
        const GridMap& map,
        const FixedWingDynamics& dynamics,
        const BSplineParams& bspline_params,
        const OptimizationParams& opt_params);
    
    bool optimize(std::vector<Point2D>& control_points);
    
private:
    const GridMap& map_;
    const FixedWingDynamics& dynamics_;
    const BSplineParams& bspline_params_;
    const OptimizationParams& opt_params_;
    
    // 成本函数计算
    struct CostComponents {
        double smooth_cost;
        double velocity_cost;
        double acceleration_cost;
        double curvature_cost;
        double collision_cost;
        double total_cost;
        
        CostComponents() : smooth_cost(0), velocity_cost(0), 
                           acceleration_cost(0), curvature_cost(0), 
                           collision_cost(0), total_cost(0) {}
    };
    
    CostComponents computeCost(const std::vector<Point2D>& points);
    
    // 梯度计算
    std::vector<Point2D> computeGradient(const std::vector<Point2D>& points);
    
    // 约束检查
    bool checkConstraints(const std::vector<Point2D>& points);
    
    // 投影到可行域
    void projectToFeasible(std::vector<Point2D>& points);
    
    // L-BFGS相关
    class LBFGSData;
    std::unique_ptr<LBFGSData> lbfgs_data_;
};

} // namespace fixed_wing_planning

#endif // TRAJECTORY_OPTIMIZER_H