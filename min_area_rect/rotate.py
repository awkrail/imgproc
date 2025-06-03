import numpy as np
import matplotlib.pyplot as plt
import math

# 凸包の点（反時計回り）
points = np.array([
    [0, 0],
    [4, 1],
    [5, 4],
    [3, 6],
    [1, 5],
    [-1, 3]
])

def min_area_rect(points):
    n = len(points)
    min_area = float('inf')
    best_rect = None

    for i in range(n):
        p1 = points[i]
        p2 = points[(i + 1) % n]
        edge = p2 - p1
        angle = -math.atan2(edge[1], edge[0])

        # 回転行列
        rotation = np.array([
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle),  math.cos(angle)]
        ])
        # 全点を回転
        rotated = np.dot(points, rotation.T)

        min_x, max_x = rotated[:, 0].min(), rotated[:, 0].max()
        min_y, max_y = rotated[:, 1].min(), rotated[:, 1].max()
        area = (max_x - min_x) * (max_y - min_y)

        if area < min_area:
            min_area = area
            best_rect = {
                "angle": angle,
                "area": area,
                "width": max_x - min_x,
                "height": max_y - min_y,
                "center": np.dot([[ (min_x + max_x) / 2, (min_y + max_y) / 2 ]], rotation)[0],
                "rotation": rotation,
                "rect_pts": np.dot(np.array([
                    [min_x, min_y],
                    [max_x, min_y],
                    [max_x, max_y],
                    [min_x, max_y]
                ]), rotation)
            }

    return best_rect

# 最小矩形を計算
rect = min_area_rect(points)

# 描画
plt.figure(figsize=(6,6))
# 元の点（凸包）
hull = np.vstack([points, points[0]])  # 閉じる
plt.plot(hull[:,0], hull[:,1], 'b--', label='Convex Hull')
plt.scatter(points[:,0], points[:,1], color='blue')

# 最小外接矩形
rect_pts = np.vstack([rect["rect_pts"], rect["rect_pts"][0]])  # 閉じる
import ipdb; ipdb.set_trace()
plt.plot(rect_pts[:,0], rect_pts[:,1], 'r-', label='Min Area Rect')

# センターをプロット
plt.plot(rect["center"][0], rect["center"][1], 'ro', label='Rect Center')

# 軸設定
plt.gca().set_aspect('equal')
plt.title(f'MinAreaRect: area={rect["area"]:.2f}, angle={math.degrees(-rect["angle"]):.2f}°')
plt.legend()
plt.grid(True)
plt.savefig('test.png')
