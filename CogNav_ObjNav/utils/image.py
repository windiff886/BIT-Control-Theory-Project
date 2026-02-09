import numpy as np
import scipy.ndimage as ndimage
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
import cv2
def getObjectCenter(obstacle_map):
    binary_map = np.zeros_like(obstacle_map)
    object_index= np.where(obstacle_map > 1) 
    binary_map[object_index[0],object_index[1]]=1
    binary_map = cv2.medianBlur(binary_map, 5)
    labeled_image, num_features = ndimage.label(binary_map)
    cv2.imwrite('binary_map.png',binary_map*255)
    # 输出所有连通区域的标签
    # print(f"找到的连通区域数量: {num_features}")
    # print("标记后的二值图（连通区域编号）：")
    # print(labeled_image)

    # 计算每个连通区域的中心（质心）
    centers = ndimage.center_of_mass(binary_map, labeled_image, range(1, num_features + 1))

    # 输出每个连通区域的中心坐标
    # for i, center in enumerate(centers):
    #     print(f"连通区域 {i+1} 的中心坐标: {center}")
    centers = np.array(centers)
    db = DBSCAN(eps=10, min_samples=1).fit(centers)

    # 获取聚类结果的标签
    labels = db.labels_

    # 计算每个簇的质心
    unique_labels = set(labels)
    centroids = np.array([centers[labels == label].mean(axis=0) for label in unique_labels])

    # 输出每个聚类的质心（合并后的点）
    # print("合并后的点（质心）：")
    # print(centroids)

    # 可视化聚类结果
    plt.scatter(centers[:, 0], centers[:, 1], c=labels, s=100, cmap='viridis', label='Original Points')
    plt.scatter(centroids[:, 0], centroids[:, 1], c='red', s=200, marker='x', label='Centroids')
    plt.legend()
    plt.title('DBSCAN Clustering and Centroids')
    plt.savefig("dbscan.png")
    plt.clf()
    return centroids