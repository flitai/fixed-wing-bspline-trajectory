#pragma once

#include <vector>
#include <cmath>
#include <iostream>

// 2D向量，用于表示位置、速度等
struct Vector2D {
    double x = 0.0, y = 0.0;

    Vector2D operator+(const Vector2D& other) const { return {x + other.x, y + other.y}; }
    Vector2D operator-(const Vector2D& other) const { return {x - other.x, y - other.y}; }
    Vector2D operator*(double scalar) const { return {x * scalar, y * scalar}; }
    double dot(const Vector2D& other) const { return x * other.x + y * other.y; }
    double magnitude() const { return std::sqrt(x * x + y * y); }
    double cross(const Vector2D& other) const { return x * other.y - y * other.x; }
};

// 初始离散路径点
struct InitialPathPoint {
    Vector2D pos;
};

// B样条控制点
using ControlPoint = Vector2D;

// 最终生成的轨迹点状态
struct TrajectoryPoint {
    double timestamp;
    Vector2D pos;
    Vector2D vel;
    Vector2D acc;
};

// 固定翼无人机动力学参数
struct UAVParams {
    double v_min;      // 最小飞行速度 (m/s)
    double v_max;      // 最大飞行速度 (m/s)
    double a_max;      // 最大加速度 (m/s^2)
    double kappa_max;  // 最大曲率 (1 / R_min)
};

// B样条算法参数
struct BsplineParams {
    int degree = 3;     // B样条次数 (p)，文档指定为3
    double delta_t;   // 时间间隔 (s)
};

// 后端优化器使用的各项成本权重
struct OptimizerWeights {
    double smooth; // 平滑度成本权重
    double v;      // 速度约束成本权重
    double a;      // 加速度约束成本权重
    double kappa;  // 曲率约束成本权重
    double col;    // 碰撞约束成本权重
};

// 二维栅格地图（0表示空闲，1表示障碍）
using GridMap = std::vector<std::vector<int>>;
// 二维距离场（存储每个栅格到最近障碍物的欧氏距离）
using DistanceField = std::vector<std::vector<double>>;