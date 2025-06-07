#!/usr/bin/env python3
"""
可视化固定翼无人机B样条轨迹
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.patches import Rectangle, Circle
import matplotlib.animation as animation

def load_trajectory(filename):
    """加载轨迹数据"""
    df = pd.read_csv(filename, comment='#', header=None)
    df.columns = ['time', 'x', 'y', 'vx', 'vy', 'ax', 'ay', 'speed', 'accel']
    return df

def load_control_points(filename):
    """加载控制点数据"""
    df = pd.read_csv(filename, comment='#', header=None)
    df.columns = ['x', 'y']
    return df

def create_obstacles():
    """创建障碍物（与C++代码中的障碍物对应）"""
    obstacles = []
    
    # 障碍物1：矩形
    obstacles.append(Rectangle((50, 40), 25, 20, facecolor='gray'))
    
    # 障碍物2：圆形
    obstacles.append(Circle((125, 75), 15, facecolor='gray'))
    
    # 障碍物3：L形（用两个矩形表示）
    obstacles.append(Rectangle((175, 50), 25, 25, facecolor='gray'))
    obstacles.append(Rectangle((175, 50), 50, 15, facecolor='gray'))
    
    return obstacles

def plot_static_trajectory(trajectory_df, control_points_df):
    """绘制静态轨迹图"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # 上图：轨迹和障碍物
    ax1.set_title('固定翼无人机B样条轨迹', fontsize=14)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    
    # 添加障碍物
    for obstacle in create_obstacles():
        ax1.add_patch(obstacle)
    
    # 绘制控制点
    ax1.plot(control_points_df['x'], control_points_df['y'], 
             'ro-', label='控制点', markersize=4, alpha=0.5)
    
    # 绘制轨迹
    ax1.plot(trajectory_df['x'], trajectory_df['y'], 
             'b-', label='B样条轨迹', linewidth=2)
    
    # 标记起点和终点
    ax1.plot(trajectory_df['x'].iloc[0], trajectory_df['y'].iloc[0], 
             'go', markersize=10, label='起点')
    ax1.plot(trajectory_df['x'].iloc[-1], trajectory_df['y'].iloc[-1], 
             'rs', markersize=10, label='终点')
    
    # 绘制速度矢量（每隔一定点数）
    skip = len(trajectory_df) // 20
    for i in range(0, len(trajectory_df), skip):
        ax1.arrow(trajectory_df['x'].iloc[i], trajectory_df['y'].iloc[i],
                  trajectory_df['vx'].iloc[i]*0.5, trajectory_df['vy'].iloc[i]*0.5,
                  head_width=2, head_length=1, fc='orange', ec='orange', alpha=0.6)
    
    ax1.legend()
    ax1.set_xlim(-10, 250)
    ax1.set_ylim(-10, 130)
    
    # 下图：速度和加速度曲线
    ax2.set_title('速度和加速度随时间变化', fontsize=14)
    ax2.set_xlabel('时间 (s)')
    
    ax2_twin = ax2.twinx()
    
    # 绘制速度
    line1 = ax2.plot(trajectory_df['time'], trajectory_df['speed'], 
                     'b-', label='速度', linewidth=2)
    ax2.set_ylabel('速度 (m/s)', color='b')
    ax2.tick_params(axis='y', labelcolor='b')
    
    # 添加速度限制线
    ax2.axhline(y=15, color='b', linestyle='--', alpha=0.5, label='最小速度')
    ax2.axhline(y=30, color='b', linestyle='--', alpha=0.5, label='最大速度')
    
    # 绘制加速度
    line2 = ax2_twin.plot(trajectory_df['time'], trajectory_df['accel'], 
                          'r-', label='加速度', linewidth=2)
    ax2_twin.set_ylabel('加速度 (m/s²)', color='r')
    ax2_twin.tick_params(axis='y', labelcolor='r')
    
    # 添加加速度限制线
    ax2_twin.axhline(y=5, color='r', linestyle='--', alpha=0.5, label='最大加速度')
    
    # 合并图例
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax2.legend(lines, labels, loc='upper right')
    
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def create_animation(trajectory_df, control_points_df):
    """创建动画"""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title('固定翼无人机轨迹动画', fontsize=14)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    # 添加障碍物
    for obstacle in create_obstacles():
        ax.add_patch(obstacle)
    
    # 绘制控制点
    ax.plot(control_points_df['x'], control_points_df['y'], 
            'ro-', label='控制点', markersize=4, alpha=0.3)
    
    # 初始化轨迹线和飞机位置
    trajectory_line, = ax.plot([], [], 'b-', label='轨迹', linewidth=2)
    airplane_marker, = ax.plot([], [], 'go', markersize=10)
    velocity_arrow = ax.arrow(0, 0, 0, 0, head_width=2, head_length=1, 
                             fc='orange', ec='orange')
    
    ax.set_xlim(-10, 250)
    ax.set_ylim(-10, 130)
    ax.legend()
    
    def init():
        trajectory_line.set_data([], [])
        airplane_marker.set_data([], [])
        return trajectory_line, airplane_marker, velocity_arrow
    
    def animate(frame):
        # 更新轨迹
        trajectory_line.set_data(trajectory_df['x'][:frame+1], 
                               trajectory_df['y'][:frame+1])
        
        # 更新飞机位置
        airplane_marker.set_data([trajectory_df['x'].iloc[frame]], 
                               [trajectory_df['y'].iloc[frame]])
        
        # 更新速度箭头
        global velocity_arrow
        velocity_arrow.remove()
        velocity_arrow = ax.arrow(trajectory_df['x'].iloc[frame], 
                                trajectory_df['y'].iloc[frame],
                                trajectory_df['vx'].iloc[frame]*0.5, 
                                trajectory_df['vy'].iloc[frame]*0.5,
                                head_width=2, head_length=1, 
                                fc='orange', ec='orange')
        
        return trajectory_line, airplane_marker, velocity_arrow
    
    ani = animation.FuncAnimation(fig, animate, init_func=init,
                                frames=len(trajectory_df), interval=50,
                                blit=True, repeat=True)
    
    return fig, ani

def compute_curvature(trajectory_df):
    """计算轨迹曲率"""
    curvatures = []
    
    for i in range(1, len(trajectory_df) - 1):
        p1 = np.array([trajectory_df['x'].iloc[i-1], trajectory_df['y'].iloc[i-1]])
        p2 = np.array([trajectory_df['x'].iloc[i], trajectory_df['y'].iloc[i]])
        p3 = np.array([trajectory_df['x'].iloc[i+1], trajectory_df['y'].iloc[i+1]])
        
        v1 = p2 - p1
        v2 = p3 - p2
        
        cross = np.cross(v1, v2)
        d1 = np.linalg.norm(v1)
        d2 = np.linalg.norm(v2)
        d3 = np.linalg.norm(p3 - p1)
        
        if d1 * d2 * d3 > 1e-10:
            curvature = 2 * abs(cross) / (d1 * d2 * d3)
        else:
            curvature = 0
        
        curvatures.append(curvature)
    
    # 首尾补零
    curvatures = [0] + curvatures + [0]
    return curvatures

def plot_dynamics_analysis(trajectory_df):
    """绘制动力学分析图"""
    fig, axes = plt.subplots(3, 1, figsize=(10, 12))
    
    # 计算曲率
    curvatures = compute_curvature(trajectory_df)
    trajectory_df['curvature'] = curvatures
    trajectory_df['turning_radius'] = 1.0 / (trajectory_df['curvature'] + 1e-10)
    
    # 速度分析
    ax = axes[0]
    ax.set_title('速度分析', fontsize=12)
    ax.plot(trajectory_df['time'], trajectory_df['speed'], 'b-', linewidth=2)
    ax.axhline(y=15, color='r', linestyle='--', label='最小速度限制')
    ax.axhline(y=30, color='r', linestyle='--', label='最大速度限制')
    ax.fill_between(trajectory_df['time'], 15, 30, alpha=0.2, color='green')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('速度 (m/s)')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 加速度分析
    ax = axes[1]
    ax.set_title('加速度分析', fontsize=12)
    ax.plot(trajectory_df['time'], trajectory_df['accel'], 'g-', linewidth=2)
    ax.axhline(y=5, color='r', linestyle='--', label='最大加速度限制')
    ax.fill_between(trajectory_df['time'], 0, 5, alpha=0.2, color='green')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('加速度 (m/s²)')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 曲率/转弯半径分析
    ax = axes[2]
    ax.set_title('转弯半径分析', fontsize=12)
    # 限制显示范围，避免无穷大值
    turning_radius_clipped = np.clip(trajectory_df['turning_radius'], 0, 500)
    ax.plot(trajectory_df['time'], turning_radius_clipped, 'm-', linewidth=2)
    ax.axhline(y=50, color='r', linestyle='--', label='最小转弯半径限制')
    ax.fill_between(trajectory_df['time'], 50, 500, alpha=0.2, color='green')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('转弯半径 (m)')
    ax.set_ylim(0, 500)
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    plt.tight_layout()
    return fig

def main():
    # 加载数据
    trajectory_df = load_trajectory('trajectory_output.csv')
    control_points_df = load_control_points('control_points.csv')
    
    print(f"轨迹点数: {len(trajectory_df)}")
    print(f"控制点数: {len(control_points_df)}")
    print(f"总时间: {trajectory_df['time'].iloc[-1]:.2f} 秒")
    
    # 绘制静态图
    fig1 = plot_static_trajectory(trajectory_df, control_points_df)
    plt.savefig('trajectory_static.png', dpi=300, bbox_inches='tight')
    
    # 绘制动力学分析
    fig2 = plot_dynamics_analysis(trajectory_df)
    plt.savefig('dynamics_analysis.png', dpi=300, bbox_inches='tight')
    
    # 创建动画（可选）
    # fig3, ani = create_animation(trajectory_df, control_points_df)
    # ani.save('trajectory_animation.gif', writer='pillow', fps=20)
    
    plt.show()

if __name__ == '__main__':
    main()