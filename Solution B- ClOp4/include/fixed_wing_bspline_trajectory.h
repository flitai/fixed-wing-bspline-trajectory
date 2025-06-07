// fixed_wing_bspline_trajectory.h
#ifndef FIXED_WING_BSPLINE_TRAJECTORY_H
#define FIXED_WING_BSPLINE_TRAJECTORY_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace fixed_wing_planning {

// 基本数据结构定义
struct Point2D {
    double x, y;
    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
    
    Point2D operator+(const Point2D& other) const {
        return Point2D(x + other.x, y + other.y);
    }
    
    Point2D operator-(const Point2D& other) const {
        return Point2D(x - other.x, y - other.y);
    }
    
    Point2D operator*(double scalar) const {
        return Point2D(x * scalar, y * scalar);
    }
    
    double norm() const {
        return std::sqrt(x * x + y * y);
    }
};

// 路径点结构（包含位置、速度、加速度）
struct PathPoint {
    Point2D position;
    Point2D velocity;
    Point2D acceleration;
    
    PathPoint() = default;
    PathPoint(const Point2D& pos, const Point2D& vel = Point2D(), 
              const Point2D& acc = Point2D()) 
        : position(pos), velocity(vel), acceleration(acc) {}
};

// 离散时序状态点
struct TrajectoryPoint {
    double time;
    Point2D position;
    Point2D velocity;
    Point2D acceleration;
    
    TrajectoryPoint() : time(0) {}
};

// 固定翼动力学参数
struct FixedWingDynamics {
    double v_min;          // 最小允许飞行速度 (m/s)
    double v_max;          // 最大允许飞行速度 (m/s)
    double a_max;          // 最大允许加速度 (m/s^2)
    double r_min;          // 最小回转半径 (m)
    double kappa_max;      // 最大曲率 (1/m)
    
    FixedWingDynamics() : v_min(10.0), v_max(30.0), a_max(5.0), 
                          r_min(50.0), kappa_max(0.02) {}
};

// B样条参数
struct BSplineParams {
    int degree;            // 样条次数（通常为3）
    double delta_t;        // 时间间隔
    
    BSplineParams() : degree(3), delta_t(0.1) {}
};

// 优化参数
struct OptimizationParams {
    // 权重系数
    double lambda_smooth;    // 平滑性权重
    double lambda_velocity;  // 速度约束权重
    double lambda_accel;     // 加速度约束权重
    double lambda_curvature; // 曲率约束权重
    double lambda_collision; // 碰撞避免权重
    
    // 优化器参数
    int max_iterations;      // 最大迭代次数
    double tolerance;        // 收敛容差
    
    OptimizationParams() : 
        lambda_smooth(1.0), lambda_velocity(10.0), lambda_accel(10.0),
        lambda_curvature(10.0), lambda_collision(100.0),
        max_iterations(100), tolerance(1e-6) {}
};

// 栅格地图类
class GridMap {
public:
    GridMap(int width, int height, double resolution);
    
    // 设置障碍物
    void setOccupied(int x, int y, bool occupied = true);
    bool isOccupied(int x, int y) const;
    
    // 坐标转换
    void worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;
    
    // 距离场计算
    void computeDistanceField();
    double getDistance(double x, double y) const;
    double getDistanceGrid(int x, int y) const;
    
    // 获取地图参数
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    
private:
    int width_, height_;
    double resolution_;
    std::vector<std::vector<bool>> occupancy_;
    std::vector<std::vector<double>> distance_field_;
    
    void computeEDT(); // 欧氏距离变换
};

// B样条曲线类
class BSplineCurve {
public:
    BSplineCurve(int degree = 3);
    
    // 设置控制点
    void setControlPoints(const std::vector<Point2D>& control_points);
    
    // 设置节点向量（均匀打结）
    void setUniformKnots(double delta_t);
    
    // 计算B样条基函数
    double basisFunction(int i, int p, double t) const;
    
    // 计算曲线上的点
    Point2D evaluate(double t) const;
    Point2D evaluateDerivative(double t, int order = 1) const;
    
    // 获取参数
    int getDegree() const { return degree_; }
    int getNumControlPoints() const { return control_points_.size(); }
    const std::vector<Point2D>& getControlPoints() const { return control_points_; }
    const std::vector<double>& getKnots() const { return knots_; }
    
    // 获取有效参数范围
    double getMinParameter() const;
    double getMaxParameter() const;
    
private:
    int degree_;
    std::vector<Point2D> control_points_;
    std::vector<double> knots_;
    
    // 递归计算B样条基函数
    double basisFunctionRecursive(int i, int p, double t) const;
};

// B样条轨迹生成器主类
class FixedWingBSplineTrajectory {
public:
    FixedWingBSplineTrajectory(
        const GridMap& map,
        const FixedWingDynamics& dynamics,
        const BSplineParams& bspline_params,
        const OptimizationParams& opt_params);
    
    // 主要接口：生成B样条轨迹
    bool generateTrajectory(
        const std::vector<PathPoint>& waypoints,
        std::vector<TrajectoryPoint>& trajectory);
    
    // 获取优化后的B样条曲线
    const BSplineCurve& getBSplineCurve() const { return bspline_curve_; }
    
    // 获取插值后的控制点
    const std::vector<Point2D>& getControlPoints() const { 
        return control_points_; 
    }
    
private:
    const GridMap& map_;
    FixedWingDynamics dynamics_;
    BSplineParams bspline_params_;
    OptimizationParams opt_params_;
    
    std::vector<Point2D> control_points_;
    BSplineCurve bspline_curve_;
    
    // 步骤1：计算路径段到障碍的最小距离
    std::vector<double> computeSegmentDistances(
        const std::vector<PathPoint>& waypoints);
    
    // 步骤2：等距插值生成初始控制点
    bool interpolateControlPoints(
        const std::vector<PathPoint>& waypoints,
        const std::vector<double>& segment_distances);
    
    // 步骤3：构造B样条
    void constructBSpline();
    
    // 步骤4：后端优化
    bool optimizeTrajectory();
    
    // 步骤5：离散化输出
    void discretizeTrajectory(std::vector<TrajectoryPoint>& trajectory);
    
    // 辅助函数
    double computeMaxSpacing(double obstacle_distance) const;
    bool checkVelocityConstraints(const std::vector<Point2D>& points) const;
    bool checkCollisionFree(const std::vector<Point2D>& points) const;
    
    // 优化相关
    class TrajectoryOptimizer;
    std::unique_ptr<TrajectoryOptimizer> optimizer_;
};

// 工具函数
namespace utils {
    // 计算两点间的欧氏距离
    inline double distance(const Point2D& p1, const Point2D& p2) {
        return (p2 - p1).norm();
    }
    
    // 线性插值
    inline Point2D lerp(const Point2D& p1, const Point2D& p2, double t) {
        return p1 + (p2 - p1) * t;
    }
    
    // 计算三点确定的曲率
    double computeCurvature(const Point2D& p1, const Point2D& p2, const Point2D& p3);
    
    // 计算速度大小
    inline double computeSpeed(const Point2D& velocity) {
        return velocity.norm();
    }
}

} // namespace fixed_wing_planning

#endif // FIXED_WING_BSPLINE_TRAJECTORY_H