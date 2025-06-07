#include "Bspline.h"
#include <stdexcept>
#include <algorithm>

Bspline::Bspline(int degree, std::vector<ControlPoint> control_points, double delta_t)
    : p_(degree), Q_(std::move(control_points)), dt_(delta_t) {
    M_ = Q_.size() - 1;
    if (M_ < p_) {
        throw std::invalid_argument("控制点数量必须大于等于B样条次数。");
    }
    generateKnotVector();
}

void Bspline::generateKnotVector() {
    int knot_size = M_ + p_ + 2;
    T_.resize(knot_size);

    // 根据文档的三次B样条均匀打结方法（Clamped B-spline）
    // 首尾各有 p+1 个重复节点
    double current_knot_val = 0.0;
    for (int i = 0; i <= p_; ++i) {
        T_[i] = current_knot_val;
    }

    for (int i = p_ + 1; i <= M_; ++i) {
        current_knot_val += dt_;
        T_[i] = current_knot_val;
    }

    for (int i = M_ + 1; i < knot_size; ++i) {
        T_[i] = current_knot_val;
    }
}

double Bspline::getTotalTime() const {
    // 曲线的有效参数范围是 [t_p, t_{M+1}]
    // 对于我们生成的均匀节点，即 [T_[p_], T_[M_+1]]
    return T_[M_ + 1];
}

double Bspline::basisFunction(int j, int p, double t) const {
    if (p == 0) {
        return (t >= T_[j] && t < T_[j + 1]) ? 1.0 : 0.0;
    }

    double term1 = 0.0, term2 = 0.0;
    double den1 = T_[j + p] - T_[j];
    if (den1 > 1e-9) { // 避免除以零
        term1 = (t - T_[j]) / den1 * basisFunction(j, p - 1, t);
    }

    double den2 = T_[j + p + 1] - T_[j + 1];
    if (den2 > 1e-9) {
        term2 = (T_[j + p + 1] - t) / den2 * basisFunction(j + 1, p - 1, t);
    }

    return term1 + term2;
}

Vector2D Bspline::evaluate(double t) const {
    // 确保t在有效范围内，特别是对于钳位B样条，t=T_max时也能正确评估
    t = std::max(T_[p_], std::min(t, T_[M_ + 1]));
    if (t >= T_[M_ + 1]) t = T_[M_ + 1] - 1e-9; // 处理边界情况

    Vector2D point;
    for (int j = 0; j <= M_; ++j) {
        double basis_val = basisFunction(j, p_, t);
        if (basis_val > 0) {
            point = point + Q_[j] * basis_val;
        }
    }
    return point;
}

Vector2D Bspline::evaluateDerivative(double t) const {
    if (p_ == 0) return {0, 0};
    t = std::max(T_[p_], std::min(t, T_[M_ + 1]));
    if (t >= T_[M_ + 1]) t = T_[M_ + 1] - 1e-9;

    // 导数是一条 p-1 次的B样条，其控制点为 Q'_j = p * (Q_{j+1} - Q_j) / (t_{j+p+1} - t_{j+1})
    Vector2D vel;
    for (int j = 0; j < M_; ++j) {
        double den = T_[j + p_ + 1] - T_[j + 1];
        if (den > 1e-9) {
            ControlPoint q_prime = (Q_[j + 1] - Q_[j]) * (static_cast<double>(p_) / den);
            vel = vel + q_prime * basisFunction(j + 1, p_ - 1, t);
        }
    }
    return vel;
}

Vector2D Bspline::evaluateSecondDerivative(double t) const {
    if (p_ < 2) return {0, 0};
    t = std::max(T_[p_], std::min(t, T_[M_ + 1]));
    if (t >= T_[M_ + 1]) t = T_[M_ + 1] - 1e-9;
    
    // 计算一阶导数的控制点 Q'
    std::vector<ControlPoint> Q_prime(M_);
    for(int j = 0; j < M_; ++j) {
        double den = T_[j + p_ + 1] - T_[j + 1];
        if (den > 1e-9) {
             Q_prime[j] = (Q_[j + 1] - Q_[j]) * (static_cast<double>(p_) / den);
        }
    }

    // 计算二阶导数的控制点 Q''
    Vector2D acc;
    for (int j = 0; j < M_ - 1; ++j) {
        // 注意这里的节点向量对应 p-1 阶B样条
        double den = T_[j + p_ + 1] - T_[j + 2];
        if (den > 1e-9) {
            ControlPoint q_double_prime = (Q_prime[j + 1] - Q_prime[j]) * (static_cast<double>(p_ - 1) / den);
            acc = acc + q_double_prime * basisFunction(j + 2, p_ - 2, t);
        }
    }
    return acc;
}