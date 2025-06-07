# 固定翼无人机B样条避障轨迹生成算法

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

## 概述

本项目实现了一个适用于固定翼无人机的B样条避障轨迹生成算法。该算法能够在给定栅格地图和动力学约束的情况下，将稀疏的离散路径点优化为满足固定翼飞行器动力学特性的平滑B样条轨迹。

### 主要特性

- **固定翼动力学约束**：考虑最小/最大飞行速度、最大加速度、最小转弯半径
- **基于栅格地图的避障**：使用距离场进行高效的障碍物检测
-  **三次B样条轨迹**：生成C²连续的平滑轨迹
-  **凸包安全性保证**：确保B样条曲线段的凸包不与障碍物相交
-  **模块化设计**：易于集成到现有的无人机仿真或控制系统
-  **后端优化**：支持多种优化目标（平滑性、动力学约束、避障等）

## 算法原理

算法主要包含以下5个步骤：

1. **计算原始路径段的障碍距离**：评估每段路径到最近障碍物的距离
2. **等距插值生成控制点**：根据障碍距离和速度约束进行自适应插值
3. **构造B样条曲线**：使用均匀打结的三次B样条
4. **后端优化**：优化控制点位置以满足所有约束
5. **离散化输出**：生成可供飞控系统跟踪的时序轨迹点

详细算法说明请参考 [算法设计文档](https://hill68.github.io/uas/trajectory_planning/-b-spline/%E9%9A%9C%E7%A2%8D%E7%8E%AF%E5%A2%83%E4%B8%8B%E5%9B%BA%E5%AE%9A%E7%BF%BC%E6%97%A0%E4%BA%BA%E6%9C%BAb-%E6%A0%B7%E6%9D%A1%E9%81%BF%E9%9A%9C%E8%BD%A8%E8%BF%B9%E7%94%9F%E6%88%90%E7%AE%97%E6%B3%95/index.html)。

## 系统要求

- C++17 或更高版本
- CMake 3.10+
- Eigen3
- Python 3.x + matplotlib（可选，用于可视化）

### 可选依赖

- IPOPT（推荐，用于高性能非线性优化）
- OpenMP（用于并行加速）

## 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/yourusername/fixed-wing-bspline-trajectory.git
cd fixed-wing-bspline-trajectory
```

### 2. 编译

```bash
mkdir build && cd build
cmake ..
make -j4
```

### 3. 运行示例

```bash
./trajectory_example
```

这将生成两个输出文件：
- `trajectory_output.csv`：离散化的轨迹点
- `control_points.csv`：B样条控制点

### 4. 可视化结果（可选）

```bash
cd ..
python scripts/visualize_trajectory.py
```

## 使用方法

### 基本使用

```cpp
#include "fixed_wing_bspline_trajectory.h"

using namespace fixed_wing_planning;

// 1. 创建栅格地图
GridMap map(500, 300, 0.5);  // 500x300格子，分辨率0.5m
// 设置障碍物...
map.computeDistanceField();

// 2. 设置动力学参数
FixedWingDynamics dynamics;
dynamics.v_min = 15.0;      // 最小速度 15 m/s
dynamics.v_max = 30.0;      // 最大速度 30 m/s
dynamics.a_max = 5.0;       // 最大加速度 5 m/s²
dynamics.r_min = 50.0;      // 最小转弯半径 50 m

// 3. 设置B样条参数
BSplineParams bspline_params;
bspline_params.degree = 3;        // 三次B样条
bspline_params.delta_t = 0.5;     // 时间间隔

// 4. 设置优化参数
OptimizationParams opt_params;
opt_params.lambda_smooth = 1.0;
opt_params.lambda_collision = 100.0;
// ... 其他参数

// 5. 创建轨迹生成器
FixedWingBSplineTrajectory generator(map, dynamics, bspline_params, opt_params);

// 6. 生成轨迹
std::vector<PathPoint> waypoints = /* 从路径搜索算法获得 */;
std::vector<TrajectoryPoint> trajectory;
bool success = generator.generateTrajectory(waypoints, trajectory);
```

### 高级配置

#### 使用IPOPT优化器

如果安装了IPOPT，可以在CMake中启用：

```bash
cmake -DUSE_IPOPT=ON ..
```

#### 自定义优化权重

```cpp
OptimizationParams opt_params;
opt_params.lambda_smooth = 1.0;      // 平滑性权重
opt_params.lambda_velocity = 10.0;   // 速度约束权重
opt_params.lambda_accel = 10.0;      // 加速度约束权重
opt_params.lambda_curvature = 10.0;  // 曲率约束权重
opt_params.lambda_collision = 100.0; // 碰撞避免权重
```

## 项目结构

```
fixed-wing-bspline-trajectory/
├── include/                          # 头文件
│   ├── fixed_wing_bspline_trajectory.h
│   └── trajectory_optimizer.h
├── src/                             # 源文件
│   ├── fixed_wing_bspline_trajectory.cpp
│   └── trajectory_optimizer.cpp
├── examples/                        # 示例代码
│   └── example_usage.cpp
├── scripts/                         # 辅助脚本
│   └── visualize_trajectory.py
├── tests/                          # 单元测试
├── docs/                           # 文档
│   └── algorithm_design.md
├── CMakeLists.txt
└── README.md
```

## API文档

### 主要类

#### `FixedWingBSplineTrajectory`

轨迹生成器的主类。

```cpp
class FixedWingBSplineTrajectory {
public:
    // 构造函数
    FixedWingBSplineTrajectory(const GridMap& map,
                               const FixedWingDynamics& dynamics,
                               const BSplineParams& bspline_params,
                               const OptimizationParams& opt_params);
    
    // 生成轨迹
    bool generateTrajectory(const std::vector<PathPoint>& waypoints,
                           std::vector<TrajectoryPoint>& trajectory);
    
    // 获取结果
    const BSplineCurve& getBSplineCurve() const;
    const std::vector<Point2D>& getControlPoints() const;
};
```

#### `GridMap`

栅格地图管理类，支持障碍物表示和距离场计算。

```cpp
class GridMap {
public:
    GridMap(int width, int height, double resolution);
    
    void setOccupied(int x, int y, bool occupied = true);
    void computeDistanceField();
    double getDistance(double x, double y) const;
};
```

详细API文档请参考 [API Reference](docs/api_reference.md)。

## 性能优化

### 计算性能

- 距离场计算：O(n²)，其中n是栅格数量
- B样条评估：O(p)，其中p是样条次数
- 优化迭代：取决于控制点数量和优化方法

### 优化建议

1. **预计算距离场**：在地图不变的情况下，只需计算一次
2. **减少控制点数量**：通过合理的插值策略减少优化变量
3. **使用稀疏优化器**：如IPOPT，利用问题的稀疏结构
4. **并行化**：距离查询和梯度计算可以并行

## 示例结果

![轨迹生成结果](docs/images/trajectory_example.png)

上图展示了算法生成的轨迹：
- 红色点线：B样条控制点
- 蓝色实线：生成的平滑轨迹
- 灰色区域：障碍物
- 橙色箭头：速度向量

## 贡献指南

欢迎贡献代码、报告问题或提出建议！

1. Fork 本仓库
2. 创建您的特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交您的更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启一个 Pull Request

### 代码规范

- 遵循 C++ Core Guidelines
- 使用 clang-format 格式化代码
- 添加适当的单元测试
- 更新相关文档

## 常见问题

### Q: 如何处理动态障碍物？

A: 当前实现假设静态环境。对于动态障碍物，可以：
1. 定期更新距离场
2. 在优化过程中增加时间维度
3. 使用预测的障碍物轨迹

### Q: 能否扩展到三维空间？

A: 可以。需要：
1. 将 `Point2D` 扩展为 `Point3D`
2. 使用三维栅格地图
3. 添加高度相关的动力学约束

### Q: 如何调整轨迹的平滑度？

A: 调整优化参数中的 `lambda_smooth` 权重。值越大，轨迹越平滑，但可能牺牲其他性能。

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

## 引用

如果您在研究中使用了本项目，请引用：

```bibtex
@software{fixed_wing_bspline_trajectory,
  title = {Fixed-Wing B-Spline Trajectory Generator},
  author = {Your Name},
  year = {2024},
  url = {https://github.com/yourusername/fixed-wing-bspline-trajectory}
}
```

## 相关项目

- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) - 开源飞控系统
- [OMPL](https://ompl.kavrakilab.org/) - 开源运动规划库
- [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) - 四旋翼快速轨迹规划

## 联系方式

- 项目维护者：[Your Name](mailto:your.email@example.com)
- 问题反馈：[GitHub Issues](https://github.com/yourusername/fixed-wing-bspline-trajectory/issues)
- 技术讨论：[Discussions](https://github.com/yourusername/fixed-wing-bspline-trajectory/discussions)

---

⭐ 如果这个项目对您有帮助，请给个星标支持一下！