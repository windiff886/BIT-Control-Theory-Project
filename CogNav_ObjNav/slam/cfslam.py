'''
The script is used to model Grounded SAM detections in 3D, it assumes the tag2text classes are avaialable. It also assumes the dataset has Clip features saved for each object/mask.
'''

# Standard library imports
import copy
from pathlib import Path
import time
# Related third party imports
import pickle as pkl
import gzip
from PIL import Image
import numpy as np
import torch
from tqdm import tqdm, trange
from utils.draw_pcd import draw_voxel_objects,draw_pcd_objects
import open3d as o3d
# import matplotlib.colors as mcolors
# css4_colors = mcolors.CSS4_COLORS
# color_proposals = [list(mcolors.hex2color(color)) for color in css4_colors.values()]
# Local application/library specific imports
from utils.ious import (
    compute_2d_box_contained_batch
)
from slam.slam_classes import MapObjectList,DetectionList
from slam.utils import (
    create_or_load_colors,
    merge_obj2_into_obj1, 
    merge_obj2_into_obj1_2, 
    denoise_objects,
    filter_objects,
    merge_objects, 
    remove_outlier,
    gobs_to_detection_list,
    gobs_to_detection_list_Replica,
    compute_overlap_matrix
)
from slam.mapping import (
    compute_spatial_similarities,
    compute_visual_similarities,
    aggregate_similarities,
    merge_detections_to_objects,
    compute_voxel_similarities,
    comput_voxel_overlap,
    VoxelMergeStrategy,
    updateVoxelandObject,
    removalOverlap
)



# Disable torch gradient computation
torch.set_grad_enabled(False)

def compute_match_batch(cfg, spatial_sim: torch.Tensor, visual_sim: torch.Tensor) -> torch.Tensor:
    '''
    Compute object association based on spatial and visual similarities
    
    Args:
        spatial_sim: a MxN tensor of spatial similarities
        visual_sim: a MxN tensor of visual similarities
    Returns:
        A MxN tensor of binary values, indicating whether a detection is associate with an object. 
        Each row has at most one 1, indicating one detection can be associated with at most one existing object.
        One existing object can receive multiple new detections
    '''
    assign_mat = torch.zeros_like(spatial_sim)
    if cfg.match_method == "sim_sum":
        sims = (1 + cfg.phys_bias) * spatial_sim + (1 - cfg.phys_bias) * visual_sim # (M, N)
        row_max, row_argmax = torch.max(sims, dim=1) # (M,), (M,)
        for i in row_max.argsort(descending=True):
            if row_max[i] > cfg.sim_threshold:
                assign_mat[i, row_argmax[i]] = 1
            else:
                break
    else:
        raise ValueError(f"Unknown matching method: {cfg.match_method}")
    
    return assign_mat

def prepare_objects_save_vis(objects: MapObjectList, downsample_size: float=0.025):
    objects_to_save = copy.deepcopy(objects)
            
    # Downsample the point cloud
    for i in range(len(objects_to_save)):
        objects_to_save[i]['pcd'] = objects_to_save[i]['pcd']

    # Remove unnecessary keys
    for i in range(len(objects_to_save)):
        for k in list(objects_to_save[i].keys()):
            if k not in [
                'pcd', 'bbox', 'clip_ft', 'text_ft', 'class_id', 'num_detections', 'inst_color'
            ]:
                del objects_to_save[i][k]
                
    return objects_to_save.to_serializable()
def cfvoxelReplica(cfg,color_path,data_one,gobs,pose,idx,length,classes,BG_CLASSES,objects,bg_objects,changed,pcd_path,som_path):
    color_tensor, depth_tensor, intrinsics, *_ = data_one
    # Get the RGB image
    color_np = color_tensor.cpu().numpy() # (H, W, 3)
    image_rgb = (color_np).astype(np.uint8) # (H, W, 3)
    assert image_rgb.max() > 1, "Image is not in range [0, 255]"
    
    # Get the depth image
    depth_tensor = depth_tensor[..., 0]
    depth_array = depth_tensor.cpu().numpy()

    # Get the intrinsics matrix
    cam_K = intrinsics.cpu().numpy()[:3, :3]
    unt_pose = pose.cpu().numpy()
        
    # Don't apply any transformation otherwise
    adjusted_pose = unt_pose
    fg_detection_list, bg_detection_list = gobs_to_detection_list_Replica(
            cfg = cfg,
            image = image_rgb,
            depth_array = depth_array,
            cam_K = cam_K,
            idx = idx,
            gobs = gobs,
            trans_pose = adjusted_pose,
            class_names = classes,
            BG_CLASSES = BG_CLASSES,
            color_path = color_path,
        )
    bg_detection_list = remove_outlier(cfg,bg_detection_list)
    fg_detection_list = remove_outlier(cfg,fg_detection_list)
    if len(bg_detection_list) > 0:
        changed_bg=[]
        for detected_object in bg_detection_list:
            class_name = detected_object['class_name'][0]
            if bg_objects[class_name] is None:
                bg_objects[class_name] = detected_object
                changed_bg.append(class_name)
            else:
                matched_obj = bg_objects[class_name]
                matched_det = detected_object
                bg_objects[class_name] = merge_obj2_into_obj1(cfg, matched_obj, matched_det, run_dbscan=False)
                changed_bg.append(class_name)
    if len(objects) == 0:
        # Add all detections to the map
        # pcd = o3d.geometry.PointCloud()
        for i in range(len(fg_detection_list)):
            objects.append(fg_detection_list[i])
            changed.append(i)

    else :
        if len(fg_detection_list) > 0 :
            overlap_sim = comput_voxel_overlap(cfg,fg_detection_list,objects)
            visual_sim = compute_visual_similarities(cfg, fg_detection_list, objects)
            objects,changed = VoxelMergeStrategy(cfg,fg_detection_list,objects,overlap_sim,visual_sim,image_rgb,pose,cam_K,som_path,idx,changed)
    # end=time.time()
    # print("objects time :",end-start)
    if idx > 1 and (idx % 10 == 0 or idx == length-1):
        ###delete overlap part between two objects
        # start =time.time()
        objects,bg_objects,changed,mapindex = removalOverlap(cfg,objects,bg_objects,changed)
        # end=time.time()
        # print("removalOverlap time:",end-start)
    if idx > 1 and (idx % 20 == 0 or idx == length-1) :   
        draw_voxel_objects(objects,bg_objects,idx,pcd_path)
    if idx == length -1 :
        draw_pcd_objects(objects,bg_objects,idx,pcd_path)

    return objects,bg_objects,changed,mapindex

def cfslam(cfg,color_path,data_one,gobs,pose,idx,classes,BG_CLASSES,objects,bg_objects):
    
    
    color_tensor, depth_tensor, intrinsics, *_ = data_one
    # Get the RGB image
    color_np = color_tensor.cpu().numpy() # (H, W, 3)
    image_rgb = (color_np).astype(np.uint8) # (H, W, 3)
    assert image_rgb.max() > 1, "Image is not in range [0, 255]"
    
    # Get the depth image
    depth_tensor = depth_tensor[..., 0]
    depth_array = depth_tensor.cpu().numpy()

    # Get the intrinsics matrix
    cam_K = intrinsics.cpu().numpy()[:3, :3]
    unt_pose = pose.cpu().numpy()
        
    # Don't apply any transformation otherwise
    adjusted_pose = unt_pose
    fg_detection_list, bg_detection_list = gobs_to_detection_list_Replica(
            cfg = cfg,
            image = image_rgb,
            depth_array = depth_array,
            cam_K = cam_K,
            idx = idx,
            gobs = gobs,
            trans_pose = adjusted_pose,
            class_names = classes,
            BG_CLASSES = BG_CLASSES,
            color_path = color_path,
        )
    
    if len(bg_detection_list) > 0:
        for detected_object in bg_detection_list:
            class_name = detected_object['class_name'][0]
            if bg_objects[class_name] is None:
                bg_objects[class_name] = detected_object
            else:
                matched_obj = bg_objects[class_name]
                matched_det = detected_object
                bg_objects[class_name] = merge_obj2_into_obj1_2(cfg, matched_obj, matched_det, run_dbscan=False)
    if cfg.use_contain_number:
        xyxy = fg_detection_list.get_stacked_values_torch('xyxy', 0)
        contain_numbers = compute_2d_box_contained_batch(xyxy, cfg.contain_area_thresh)
        for i in range(len(fg_detection_list)):
            fg_detection_list[i]['contain_number'] = [contain_numbers[i]]
    if len(objects) == 0:
        # Add all detections to the map
        changed=[]
        for i in range(len(fg_detection_list)):
            objects.append(fg_detection_list[i])
            changed.append(i)
    else :
        spatial_sim = compute_spatial_similarities(cfg, fg_detection_list, objects)
        visual_sim = compute_visual_similarities(cfg, fg_detection_list, objects)
        agg_sim = aggregate_similarities(cfg, spatial_sim, visual_sim)
         # Compute the contain numbers for each detection
        if cfg.use_contain_number:
            # Get the contain numbers for all objects
            contain_numbers_objects = torch.Tensor([obj['contain_number'][0] for obj in objects])
            detection_contained = contain_numbers > 0 # (M,)
            object_contained = contain_numbers_objects > 0 # (N,)
            detection_contained = detection_contained.unsqueeze(1) # (M, 1)
            object_contained = object_contained.unsqueeze(0) # (1, N)                

            # Get the non-matching entries, penalize their similarities
            xor = detection_contained ^ object_contained
            agg_sim[xor] = agg_sim[xor] - cfg.contain_mismatch_penalty
        
        # Threshold sims according to cfg. Set to negative infinity if below threshold
        agg_sim[agg_sim < cfg.sim_threshold] = float('-inf')
        
        objects,changed = merge_detections_to_objects(cfg, fg_detection_list, objects, agg_sim)
        # Perform post-processing periodically if told so
        if cfg.denoise_interval > 0 and (idx+1) % cfg.denoise_interval == 0:
            objects = denoise_objects(cfg, objects)
        if cfg.filter_interval > 0 and (idx+1) % cfg.filter_interval == 0:
            objects = filter_objects(cfg, objects)
        if cfg.merge_interval > 0 and (idx+1) % cfg.merge_interval == 0:
            objects = merge_objects(cfg, objects)
    return objects,bg_objects,changed,fg_detection_list



def cfvoxel(cfg,res,gobs,objects,idx,step,classes,bg_objects,img_name,pcd_path,som_path,changed):
    mapindex=None
    image_rgb, pcd_array, cam_K,pose = res[0].cpu().numpy(),res[1].cpu().numpy(),res[2][:3,:3],res[3].cpu().numpy()
    fg_detection_list, bg_detection_list = gobs_to_detection_list(
            cfg = cfg,
            image = image_rgb,
            pcd_array = pcd_array,
            idx = idx,
            gobs = gobs,
            trans_pose = None,
            class_names = classes,
            BG_CLASSES = bg_objects,
            color_path = img_name
        )

    bg_detection_list = remove_outlier(cfg,bg_detection_list)
    fg_detection_list = remove_outlier(cfg,fg_detection_list)

    if len(bg_detection_list) > 0:
        changed_bg=[]
        for detected_object in bg_detection_list:
            class_name = detected_object['class_name'][0]
            if bg_objects[class_name] is None:
                bg_objects[class_name] = detected_object
                changed_bg.append(class_name)
            else:
                matched_obj = bg_objects[class_name]
                matched_det = detected_object
                bg_objects[class_name] = merge_obj2_into_obj1(cfg, matched_obj, matched_det, run_dbscan=False)
                changed_bg.append(class_name)
    if len(objects) == 0:
        for i in range(len(fg_detection_list)):
            objects.append(fg_detection_list[i])
            changed.append(i)
    else :
        if len(fg_detection_list) > 0 :
            overlap_sim = comput_voxel_overlap(cfg,fg_detection_list,objects)
            visual_sim = compute_visual_similarities(cfg, fg_detection_list, objects)
            objects,changed = VoxelMergeStrategy(cfg,fg_detection_list,objects,overlap_sim,visual_sim,changed)
        
    # if True :
    #     objects,bg_objects,changed,mapindex = removalOverlap(cfg,objects,bg_objects,changed)
    if step > 1 and step % 10 == 0:     
        # draw_voxel_objects(objects,bg_objects,step,pcd_path)
        objects,bg_objects,changed = removalOverlap(cfg,objects,bg_objects,changed)
        # print(changed)
        # draw_pcd_objects(objects,bg_objects,step,pcd_path)
    return objects,bg_objects,changed

def cfrgbd(cfg,fg_detection_list,objects,voxel_map_dict,voxel_map,merge=False):
    fg_detection_list = remove_outlier(cfg,fg_detection_list)
    if len(objects) == 0:
        changed=[]
        for i in range(len(fg_detection_list)):
            objects.append(fg_detection_list[i])
            changed.append(i)
    else :
        start=time.time()
        spatial_sim,spatial_sim_fg = compute_spatial_similarities(cfg, fg_detection_list, objects)

        end=time.time()
        # print("spatial_sim time:",end-start)
        visual_sim = compute_visual_similarities(cfg, fg_detection_list, objects)
        agg_sim = aggregate_similarities(cfg, spatial_sim, visual_sim)
        agg_sim2 = aggregate_similarities(cfg, spatial_sim_fg, visual_sim)
         # Compute the contain numbers for each detection
        # for i in range(len(objects)):
        #     print(i,objects[i]['class_name'])
        # for i in range(len(fg_detection_list)):
        #     print(i,fg_detection_list[i]['class_name'])
        # # print(spatial_sim)
        # # print(visual_sim)
        # print(agg_sim)
        # Threshold sims according to cfg. Set to negative infinity if below threshold
        agg_sim[agg_sim < cfg.sim_threshold] = float('-inf')
        agg_sim2[agg_sim2 < cfg.sim_threshold] = float('-inf')
        # print(agg_sim)
        objects,changed = merge_detections_to_objects(cfg, fg_detection_list, objects, agg_sim,agg_sim2)
        
        # Perform post-processing periodically if told so
        
        # if cfg.filter_interval > 0 and (idx+1) % cfg.filter_interval == 0:
        #     objects = filter_objects(cfg, objects)
        # import open3d as o3d
        # import numpy as np
        # pcd=[]
        # i=0
        # for obj in objects:
        #     pcd_now=obj['pcd']
        #     pcd_now.colors=o3d.utility.Vector3dVector(np.ones_like(np.asarray(pcd_now.colors))*color_proposals[i])
        #     pcd.append(pcd_now)
        #     i+=1
        # o3d.visualization.draw_geometries(pcd)
        if merge == True :
            start=time.time()
            objects = denoise_objects(cfg, objects)
            objects = merge_objects(cfg, objects)
            end=time.time()
            # print("denoise and merge time:",end-start)
            # import open3d as o3d
            # import numpy as np
            # pcd=[]
            # i=0
            # for obj in objects:
            #     pcd_now=obj['pcd']
            #     pcd_now.colors=o3d.utility.Vector3dVector(np.ones_like(np.asarray(pcd_now.colors))*color_proposals[i])
            #     pcd.append(pcd_now)
            #     i+=1
            # o3d.visualization.draw_geometries(pcd)
    return objects,changed

