# @brief:  通过保存出来的点云，对场景进行点云重建，然后将其中识别出来的物体通过不同颜色的方式标出来。
import os, sys
import open3d as o3d
from utils import *
import numpy as np

path = '/home/island/Desktop/navigation/LLM-SG/outputs/save_intermediate_element_for_SGG'
begin, end = -1, 2

intrinsic = np.array([[388.19104, 0.0, 319.5],
                      [0.0, 388.19104, 239.5],
                      [0.0, 0.0, 1.0]])

if __name__ == '__main__':
    pose, pcd, img = [], [], []
    for i in range(begin, end + 1, 1):
        pkl_dict = read_pkl(os.path.join(path, 'save_dict_{}.pkl'.format(i)))
        pose.append(pkl_dict['pose'])
        pcd.append(pkl_dict['pcd_frame'])
        img.append(pkl_dict['rgb'])
        # value_to_rgb_cv(pkl_dict['depth'])
        # draw_pcd_according_depth(pkl_dict, intrinsic)
    draw_pcd_with_objs(pose, pcd, img, pkl_dict['fg_detection_list'])
