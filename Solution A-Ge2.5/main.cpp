#include "TrajectoryGenerator.h"
#include <iostream>
#include <fstream>

// 辅助函数：创建一个简单的距离场
DistanceField create_simple_distance_field(int width, int height, double resolution, int obs_x, int obs_y, int obs_r) {
    DistanceField df(height, std::vector<double>(width));
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            double dist = std::sqrt(std::pow(c - obs_x, 2) + std::pow(r - obs_y, 2)) - obs_r;
            df[r][c] = dist * resolution;
        }
    }
    return df;
}

// 辅助函数：将轨迹数据保存到文件，以便可视化
void save_trajectory(const std::string& filename, const std::vector<TrajectoryPoint>& trajectory) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件 " << filename << std::endl;
        return;
    }
    file << "timestamp,x,y,vx,vy,ax,ay\n";
    for (const auto& p : trajectory) {
        file << p.timestamp << "," << p.pos.x << "," << p.pos.y << ","
             << p.vel.x << "," << p.vel.y << "," << p.acc.x << "," << p.acc.y << "\n";
    }
    file.close();
    std::cout << "轨迹已保存到 " << filename << std::endl;
}

int main() {
    // 1. 设置无人机和算法参数
    UAVParams uav_params = {
        .v_min = 2.0,       // 最小速度
        .v_max = 10.0,      // 最大速度
        .a_max = 3.0,       // 最大加速度
        .kappa_max = 0.5    // 最大曲率 (R_min = 2.0m)
    };
    BsplineParams bspline_params = {
        .degree = 3,
        .delta_t = 0.2     // 时间间隔
    };
    OptimizerWeights weights = {
        .smooth = 1.0,      // 平滑权重
        .v = 10.0,          // 速度权重
        .a = 5.0,           // 加速度权重
        .kappa = 20.0,      // 曲率权重
        .col = 50.0         // 碰撞权重
    };
    double map_resolution = 0.1; // 地图分辨率 0.1m/pixel

    // 2. 创建一个简单的初始路径 (例如，从A*算法获得)
    std::vector<InitialPathPoint> initial_path = {
        {{1.0, 1.0}},
        {{5.0, 9.0}},
        {{15.0, 3.0}},
        {{19.0, 10.0}}
    };

    // 3. 创建一个虚拟环境（一个圆形障碍物）
    int map_width = 200, map_height = 120; // 20m x 12m 地图
    DistanceField df = create_simple_distance_field(map_width, map_height, map_resolution, 100, 60, 20); // 障碍物在(10,6)，半径2m

    // 4. 创建轨迹生成器实例
    TrajectoryGenerator generator(uav_params, bspline_params, weights, map_resolution);

    // 5. 调用API生成轨迹
    std::vector<TrajectoryPoint> final_trajectory;
    std::vector<ControlPoint> optimal_control_points;
    bool success = generator.generateTrajectory(initial_path, df, final_trajectory, optimal_control_points);

    // 6. 处理结果
    if (success) {
        std::cout << "成功生成轨迹，共 " << final_trajectory.size() << " 个点。" << std::endl;
        save_trajectory("final_trajectory.csv", final_trajectory);
    } else {
        std::cerr << "轨迹生成失败。" << std::endl;
    }

    return 0;
}