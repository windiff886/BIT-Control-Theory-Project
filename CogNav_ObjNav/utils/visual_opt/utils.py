import os, sys
import open3d as o3d
import numpy as np
import pickle as pkl
import torch
import torch.nn.functional as F

from detectron2.data import MetadataCatalog
import matplotlib.colors as mcolors

metadata = MetadataCatalog.get('coco_2017_train_panoptic')
css4_colors = mcolors.XKCD_COLORS
color_proposals = [list(mcolors.hex2color(color)) for color in css4_colors.values()]

def read_pkl(path_file):
    output = open(path_file, 'rb')
    res = pkl.load(output)

    for obj in res['objects']:
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(obj['pcd']['points'])
        point_cloud.colors = o3d.utility.Vector3dVector(obj['pcd']['colors'])
        obj['pcd'] = point_cloud

        if 'min_bound' in obj['bbox']:
            obj['bbox'] = o3d.geometry.AxisAlignedBoundingBox(
                min_bound = obj['bbox']['min_bound'],
                max_bound = obj['bbox']['max_bound']
            )
        elif 'center' in obj['bbox']:
            obj['bbox'] = o3d.geometry.OrientedBoundingBox(
                center = obj['bbox']['center'],
                R = obj['bbox']['R'],
                extent = obj['bbox']['extent']
            )

    for obj in res['bg_objects']:
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(res['bg_objects'][obj]['pcd']['points'])
        point_cloud.colors = o3d.utility.Vector3dVector(res['bg_objects'][obj]['pcd']['colors'])
        res['bg_objects'][obj]['pcd'] = point_cloud

        if 'min_bound' in res['bg_objects'][obj]['bbox']:
            res['bg_objects'][obj]['bbox'] = o3d.geometry.AxisAlignedBoundingBox(
                min_bound = res['bg_objects'][obj]['bbox']['min_bound'],
                max_bound = res['bg_objects'][obj]['bbox']['max_bound']
            )
        elif 'center' in res['bg_objects'][obj]['bbox']:
            res['bg_objects'][obj]['bbox'] = o3d.geometry.OrientedBoundingBox(
                center = res['bg_objects'][obj]['bbox']['center'],
                R = res['bg_objects'][obj]['bbox']['R'],
                extent = res['bg_objects'][obj]['bbox']['extent']
            )

    for obj in res['fg_detection_list']:
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(obj['pcd']['points'])
        point_cloud.colors = o3d.utility.Vector3dVector(obj['pcd']['colors'])
        obj['pcd'] = point_cloud

        if 'min_bound' in obj['bbox']:
            obj['bbox'] = o3d.geometry.AxisAlignedBoundingBox(
                min_bound = obj['bbox']['min_bound'],
                max_bound = obj['bbox']['max_bound']
            )
        elif 'center' in obj['bbox']:
            obj['bbox'] = o3d.geometry.OrientedBoundingBox(
                center = obj['bbox']['center'],
                R = obj['bbox']['R'],
                extent = obj['bbox']['extent']
            )

    for obj in res['bg_detection_list']:
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(obj['pcd']['points'])
        point_cloud.colors = o3d.utility.Vector3dVector(obj['pcd']['colors'])
        obj['pcd'] = point_cloud

        if 'min_bound' in obj['bbox']:
            obj['bbox'] = o3d.geometry.AxisAlignedBoundingBox(
                min_bound = obj['bbox']['min_bound'],
                max_bound = obj['bbox']['max_bound']
            )
        elif 'center' in obj['bbox']:
            obj['bbox'] = o3d.geometry.OrientedBoundingBox(
                center = obj['bbox']['center'],
                R = obj['bbox']['R'],
                extent = obj['bbox']['extent']
            )

    return res

def draw_pcd_with_objs(pose, pcd, img, objects):
    ln = len(pose)
    ans_pcd = o3d.geometry.PointCloud()
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="test_poses_with_pcd")
    for i, obj in enumerate(objects):
        tmp_pcd = o3d.geometry.PointCloud()
        tmp_pcd.points = obj['pcd'].points
        tmp_pcd.colors = o3d.utility.Vector3dVector(np.tile(np.array(color_proposals[i]), (np.array(obj['pcd'].points).shape[0], 1)))
        # print('3333333')
        ans_pcd += tmp_pcd


    # for i in range(ln):
    #     print('i:   {}'.format(i))
    #     tmp_pcd = o3d.geometry.PointCloud()
    #     tmp_pcd.points = o3d.utility.Vector3dVector(pcd[i]['points'].reshape(-1, 3))
    #     tmp_pcd.colors = o3d.utility.Vector3dVector(img[i].cpu().numpy().reshape(-1, 3) / 255.)
    #     # tmp_pcd.transform(pose[i].cpu().numpy())
    #     ans_pcd += tmp_pcd
    # print('visual similarities:  {}'.format(_compute_visual_similarities(objects[12], objects[13])))
    # print('overlap similarities:  {}'.format(_compute_overlap_matrix(objects[12], objects[13])))

    vis.add_geometry(ans_pcd)
    vis.run()
    vis.destroy_window()
    print('draw_pcd')

def draw_pcd_according_depth(pkl_dict, intrinsic):
    '''
    深度图转点云数据
    :param depth_img: 深度图
    :return: point_cloud  np.array(N, 3)
    '''

    depth_img = pkl_dict['depth'].cpu().numpy()
    H, W = depth_img.shape
    h, w = np.mgrid[0:H, 0:W]

    fx, fy, cx, cy = intrinsic[0, 0], intrinsic[1, 1], intrinsic[0, 2], intrinsic[1, 2]
    z = depth_img
    x = (w - cx) * z / fx
    y = (h - cy) * z / fy

    point_cloud = np.stack([x, y, z], -1).reshape(-1, 3)
    index = (point_cloud[:, -1] == 0)
    ori_point_cloud = np.stack([x, y, z], -1)

    ans_pcd = o3d.geometry.PointCloud()
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="draw_pcd_according_depth")
    ans_pcd.points = o3d.utility.Vector3dVector(point_cloud)
    ans_pcd.colors = o3d.utility.Vector3dVector(pkl_dict['rgb'].cpu().numpy().reshape(-1, 3) / 255.)

    vis.add_geometry(ans_pcd)
    vis.run()
    vis.destroy_window()
    # return np.delete(point_cloud, index, axis=0), ori_point_cloud

# @brief: 将一个数值数组转成JET格式的RGB值;
# @param value_array: 待转换成RGB的原值数组, EagerTensor(n, );
#-@return: 每个value的伪RGB值 [0., 1.], ndarray(n1, ), dtype=float.
def value_to_rgb_cv(value_array, RGB_MAX=255.):
    import cv2
    value_array = value_array.cpu().numpy()
    min_value = np.min(value_array)
    max_value = np.max(value_array)

    # entropy [min, max] --> grayscale [0, 255]
    grayscale_values =  ( ( ( value_array - min_value ) / ( max_value - min_value ) ) * 255. ).astype(np.uint8)  # entropy值越高, 则灰度值越高, ndarray(n1, ), dtype=uint8

    # grayscale [0, 255] --> fake RGB [0., 1.]
    heatmap_values = cv2.applyColorMap(grayscale_values, cv2.COLORMAP_PLASMA)  # ndarray(n1, 1, 3), dtype=uint8
    values = heatmap_values[:, 0, :].astype(np.float32) / RGB_MAX  # 每个value的伪BGR值, 位于[0, 1], ndarray(n1, 3), dtype=float
    values[:, [2, 0]] = values[:, [0, 2]]   # 第0列的第2列交换, BGR --> RGB (openCV默认是BGR，但是open3d默认是RGB)

    return values, heatmap_values

def compute_iou_batch(bbox1: torch.Tensor, bbox2: torch.Tensor) -> torch.Tensor:
    '''
    Compute IoU between two sets of axis-aligned 3D bounding boxes.

    bbox1: (M, V, D), e.g. (M, 8, 3)
    bbox2: (N, V, D), e.g. (N, 8, 3)

    returns: (M, N)
    '''
    # Compute min and max for each box
    bbox1_min, _ = bbox1.min(dim=1)  # Shape: (M, 3)
    bbox1_max, _ = bbox1.max(dim=1)  # Shape: (M, 3)
    bbox2_min, _ = bbox2.min(dim=1)  # Shape: (N, 3)
    bbox2_max, _ = bbox2.max(dim=1)  # Shape: (N, 3)

    # Expand dimensions for broadcasting
    bbox1_min = bbox1_min.unsqueeze(1)  # Shape: (M, 1, 3)
    bbox1_max = bbox1_max.unsqueeze(1)  # Shape: (M, 1, 3)
    bbox2_min = bbox2_min.unsqueeze(0)  # Shape: (1, N, 3)
    bbox2_max = bbox2_max.unsqueeze(0)  # Shape: (1, N, 3)

    # Compute max of min values and min of max values
    # to obtain the coordinates of intersection box.
    inter_min = torch.max(bbox1_min, bbox2_min)  # Shape: (M, N, 3)
    inter_max = torch.min(bbox1_max, bbox2_max)  # Shape: (M, N, 3)

    # Compute volume of intersection box
    inter_vol = torch.prod(torch.clamp(inter_max - inter_min, min=0), dim=2)  # Shape: (M, N)

    # Compute volumes of the two sets of boxes
    bbox1_vol = torch.prod(bbox1_max - bbox1_min, dim=2)  # Shape: (M, 1)
    bbox2_vol = torch.prod(bbox2_max - bbox2_min, dim=2)  # Shape: (1, N)

    # Compute IoU, handling the special case where there is no intersection
    # by setting the intersection volume to 0.
    iou = inter_vol / (bbox1_vol + bbox2_vol - inter_vol + 1e-10)

    return iou

def _compute_overlap_matrix(objects_i, objects_j):
    bbox_map, bbox_new = [], []
    bbox_map.append(np.asarray(objects_i['bbox'].get_box_points()))
    bbox_new.append(np.asarray(objects_j['bbox'].get_box_points()))
    # iou = compute_iou_batch(torch.from_numpy(bbox_map[0]).unsqueeze(0), torch.from_numpy(bbox_new[0]).unsqueeze(0))
    import faiss
    points_map = [np.asarray(objects_i['pcd'].points, dtype=np.float32)] # m arrays
    indices = [faiss.IndexFlatL2(arr.shape[1]) for arr in points_map] # m indices

    points_new = [np.asarray(objects_j['pcd'].points, dtype=np.float32)]
    D, I = indices[0].search(points_new[0], 1)
    overlap = (D < 0.025 ** 2).sum()
    overlap_matrix = np.zeros((1, 1))
    overlap_matrix[0, 0] = overlap / len(points_new[0])

    return overlap_matrix

def _compute_visual_similarities(objects_i, objects_j):
    det_fts = objects_i['clip_ft'].unsqueeze(-1).unsqueeze(0)
    obj_fts = objects_j['clip_ft'].unsqueeze(0).unsqueeze(-1)
    return F.cosine_similarity(det_fts, obj_fts, dim=1) # (M, N)