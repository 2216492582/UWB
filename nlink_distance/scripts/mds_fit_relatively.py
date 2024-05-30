import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform
from sklearn.manifold import MDS

# 假设我们有8个模块的原始坐标，以1号模块为原点
original_coordinates = np.array([
    [0, 0],  # 模块1
    [2, 3],  # 模块2
    [3, 7],  # 模块3
    [5, 1],  # 模块4
    [6, 4],  # 模块5
    [8, 5],  # 模块6
    [7, 8],  # 模块7
    [9, 2]   # 模块8
])

# 计算距离矩阵
distance_matrix = squareform(pdist(original_coordinates))

# 使用MDS进行多维尺度分析
mds = MDS(n_components=2, dissimilarity='precomputed', random_state=42)
mds_coordinates = mds.fit_transform(distance_matrix)

# 绘制原始坐标和MDS坐标的对比图
plt.figure(figsize=(14, 7))

# 原始坐标图
plt.subplot(1, 2, 1)
plt.scatter(original_coordinates[:, 0], original_coordinates[:, 1], color='red', label='Original')
for i in range(original_coordinates.shape[0]):
    plt.text(original_coordinates[i, 0], original_coordinates[i, 1], f'Module {i+1}', fontsize=9, ha='right')
plt.title('Original Coordinates')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)

# MDS坐标图
plt.subplot(1, 2, 2)
plt.scatter(mds_coordinates[:, 0], mds_coordinates[:, 1], color='blue', label='MDS')
for i in range(mds_coordinates.shape[0]):
    plt.text(mds_coordinates[i, 0], mds_coordinates[i, 1], f'Module {i+1}', fontsize=9, ha='right')
plt.title('MDS Coordinates')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
