import numpy as np

# 相机内参
fx, fy = 611.82, 610.94
cx, cy = 324.55, 231.57

# 期望像素点（顺序：右下、右上、左上、左下）
pixels = np.array([[380., 300.],
                   [380., 180.],
                   [260., 180.],
                   [260., 300.]])

# 转换成归一化坐标 (x*, y*)
norm = np.zeros_like(pixels)
norm[:, 0] = (pixels[:, 0] - cx) / fx
norm[:, 1] = (pixels[:, 1] - cy) / fy

# Tag 尺寸 (2 cm)
s = 0.02
half = s / 2

# Tag 在物体坐标系下的角点 (X, Y, 0)
tag_points = np.array([
    [ half, -half],  # 右下
    [ half,  half],  # 右上
    [-half,  half],  # 左上
    [-half, -half],  # 左下
])

# 构造线性方程 A t = b
rows = []
b = []

for i in range(4):
    X, Y = tag_points[i]
    x_star, y_star = norm[i]

    # X = x* tz - tx
    rows.append([1, 0, -x_star])
    b.append(X)

    # -Y = y* tz - ty
    rows.append([0, 1, -y_star])
    b.append(-Y)

A = np.array(rows)
b = np.array(b)

# 最小二乘解
t, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)
tx, ty, tz = t

print("解得平移向量 (相机坐标系下):")
print(f"tx = {tx:.4f} m, ty = {ty:.4f} m, tz = {tz:.4f} m")

print("\n期望角点深度 (Z*):")
for i in range(4):
    print(f"Point {i+1}: Z* = {tz:.4f} m")
