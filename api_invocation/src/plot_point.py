import matplotlib.pyplot as plt
import numpy as np

# 坐标点数据 (x, y, angle)
points = [
    (0.0,0.0,0),(0.314,0.309,17.66),(0.628,0.588,30.96),(0.942,0.809,38.17),(1.257,0.951,40.36),(1.571,1.0,38.17),(1.885,0.951,30.96),(2.199,0.809,17.66),(2.513,0.588,0),(2.827,0.309,-17.66),(3.142,0.0,-30.96),(3.456,-0.309,-38.17),(3.770,-0.588,-40.36),(4.084,-0.809,-38.17),(4.398,-0.951,-30.96),(4.712,-1.0,-17.66),(5.027,-0.951,0),(5.341,-0.809,17.66),(5.655,-0.588,30.96),(5.969,-0.309,38.17),(6.283,0.0,40.36)
]

# 提取x和y坐标
x = [p[0] for p in points]
y = [p[1] for p in points]
angles = [p[2] for p in points]

# 创建图形
plt.figure(figsize=(10, 10))

# 绘制轨迹线
plt.plot(x, y, 'b-', linewidth=2, label='轨迹')

# 绘制点
plt.scatter(x, y, c='red', s=50, zorder=5)

# 标注起点
plt.scatter(x[0], y[0], c='green', s=200, marker='*', zorder=6, label='起点')

# 在每个点上标注角度
for i, (xi, yi, angle) in enumerate(points):
    plt.annotate(f'{i}\n({angle}°)', 
                xy=(xi, yi), 
                xytext=(5, 5), 
                textcoords='offset points',
                fontsize=8,
                bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.5))

# 绘制方向箭头
arrow_length = 0.08
for i in range(len(points) - 1):
    angle_rad = np.radians(points[i][2])
    dx = arrow_length * np.cos(angle_rad)
    dy = arrow_length * np.sin(angle_rad)
    plt.arrow(x[i], y[i], dx, dy, 
             head_width=0.03, head_length=0.02, 
             fc='orange', ec='orange', alpha=0.6)

# 设置坐标轴
plt.axhline(y=0, color='k', linewidth=0.5, linestyle='--', alpha=0.3)
plt.axvline(x=0, color='k', linewidth=0.5, linestyle='--', alpha=0.3)
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot sinx 曲线轨迹图')
plt.legend()

# 显示图形
plt.tight_layout()
plt.savefig('robot_trajectory.png', dpi=300, bbox_inches='tight')
plt.show()

print("轨迹图已保存为 robot_trajectory.png")
print(f"总共 {len(points)} 个点")