#pragma once
#include "DataTypes.h"
#include "Bspline.h"

// 轨迹生成器主类，封装整个算法流程
class TrajectoryGenerator {
public:
    // 构造函数
    TrajectoryGenerator(const UAVParams& uav_params, 
                        const BsplineParams& bspline_params, 
                        const OptimizerWeights& weights,
                        double map_resolution);

    // 主函数：生成轨迹
    bool generateTrajectory(const std::vector<InitialPathPoint>& initial_path,
                            const DistanceField& df,
                            // 输出
                            std::vector<TrajectoryPoint>& final_trajectory,
                            std::vector<ControlPoint>& optimal_control_points);
private:
    // 步骤1 & 2: 插值生成初始控制点
    std::vector<ControlPoint> interpolateControlPoints(
        const std::vector<InitialPathPoint>& initial_path,
        const DistanceField& df) const;

    // 步骤2的细化：合并过近的点
    void refineControlPoints(std::vector<ControlPoint>& control_points) const;

    // 步骤5：离散化最终轨迹
    std::vector<TrajectoryPoint> discretizeTrajectory(const Bspline& bspline) const;

    // 步骤5的最终检查
    bool finalCheck(const std::vector<TrajectoryPoint>& trajectory, const DistanceField& df) const;
    
    // 根据世界坐标查询距离场的值
    double getDistanceAt(const DistanceField& df, const Vector2D& world_pos) const;

    // 成员变量
    UAVParams uav_params_;
    BsplineParams bspline_params_;
    OptimizerWeights weights_;
    double map_resolution_;
};