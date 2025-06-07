#pragma once
#include "DataTypes.h"
#include <vector>

// 前向声明，避免循环引用
class GridUtils;

// 轨迹优化器类
// 依赖外部非线性优化库（如NLopt），此类负责定义成本函数并调用优化器
class TrajectoryOptimizer {
public:
    // 构造函数
    TrajectoryOptimizer(std::vector<ControlPoint> initial_control_points,
                        const UAVParams& uav_params,
                        const BsplineParams& bspline_params,
                        const OptimizerWeights& weights,
                        const DistanceField& df,
                        double map_resolution);

    // 执行优化，返回优化后的控制点序列
    std::vector<ControlPoint> optimize();

private:
    // 供NLopt使用的静态目标函数接口
    static double objectiveFunction(const std::vector<double> &x, std::vector<double> &grad, void *data);

    // 计算总成本
    double computeTotalCost(const std::vector<ControlPoint>& q, std::vector<double>* grad_vec = nullptr) const;

    // 各项成本函数及其梯度
    double costSmoothness(const std::vector<ControlPoint>& q, std::vector<Vector2D>& grad) const;
    double costVelocity(const std::vector<ControlPoint>& q, std::vector<Vector2D>& grad) const;
    double costAcceleration(const std::vector<ControlPoint>& q, std::vector<Vector2D>& grad) const;
    double costCurvature(const std::vector<ControlPoint>& q, std::vector<Vector2D>& grad) const;
    double costCollision(const std::vector<ControlPoint>& q, std::vector<Vector2D>& grad) const;
    
    // 将优化变量向量(1D)转换为控制点列表(2D)
    std::vector<ControlPoint> unflatten(const std::vector<double>& x) const;

    // 根据世界坐标查询距离场的值
    double getDistanceAt(const Vector2D& world_pos) const;
    // 数值法获取距离场梯度
    Vector2D getDistanceGradientAt(const Vector2D& world_pos) const;


    // 成员变量
    std::vector<ControlPoint> Q_initial_; // 优化前的完整控制点序列
    std::vector<ControlPoint> Q_fixed_start_; // 固定的起始控制点（前3个）
    std::vector<ControlPoint> Q_fixed_end_;   // 固定的结束控制点（后3个）
    
    UAVParams uav_params_;
    BsplineParams bspline_params_;
    OptimizerWeights weights_;
    const DistanceField& df_;
    double map_resolution_;

    int M_;                     // 总控制点数 - 1
    int free_vars_count_;       // 自由变量数量 (自由控制点个数 * 2)
};