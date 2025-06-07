#pragma once
#include "DataTypes.h"
#include <vector>
#include <numeric>

// B样条曲线类，封装了B样条的评估和导数计算
class Bspline {
public:
    // 构造函数：需要次数、控制点和时间间隔
    Bspline(int degree, std::vector<ControlPoint> control_points, double delta_t);

    // 获取B样条曲线的总时长
    double getTotalTime() const;

    // 在参数t处评估样条曲线位置 (S(t))
    Vector2D evaluate(double t) const;

    // 评估一阶导数（速度, S'(t))
    Vector2D evaluateDerivative(double t) const;

    // 评估二阶导数（加速度, S''(t))
    Vector2D evaluateSecondDerivative(double t) const;

    // 获取节点向量
    const std::vector<double>& getKnotVector() const { return T_; }
    // 获取控制点
    const std::vector<ControlPoint>& getControlPoints() const { return Q_; }

private:
    // Cox-de Boor 递归算法计算基函数 N_{j,p}(t)
    double basisFunction(int j, int p, double t) const;

    // 生成均匀钳位节点向量
    void generateKnotVector();

    int p_;                                // B样条次数
    double dt_;                            // 时间间隔
    std::vector<ControlPoint> Q_;          // 控制点序列 {Q_0, ..., Q_M}
    std::vector<double> T_;                // 节点向量 {t_0, ..., t_{M+p+1}}
    int M_;                                // 控制点数量 - 1
};