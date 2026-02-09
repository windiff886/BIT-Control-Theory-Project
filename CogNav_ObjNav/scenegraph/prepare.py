from slam.utils import compute_relationship_matrix,compute_relationship_matrixreplica
import numpy as np
from utils.voronoi import merge_close_nodes
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from utils.image import getObjectCenter
import math
from collections import Counter
import string
def prepareRelationForGPT(cfg,objects):
    bbox_overlaps = compute_relationship_matrixreplica(objects)
    indices = np.argwhere(bbox_overlaps == 1)
    relation_dict={}
    
    for i in range(len(bbox_overlaps)) :
        index = indices[np.where(indices[:,0]==i)[0],1].tolist()
        rr={}
        for j in index :
            rr[j]='dd'
        relation_dict[i] = rr
        # relation_dict[i]=indices[np.where(indices[:,0]==i)[0],1].tolist()
    return relation_dict
def prepareRelationForCOG(cfg,objects,changed,som_pth,idx):
    bbox_overlaps = compute_relationship_matrix(objects)
    indices = np.argwhere(bbox_overlaps == 1)
    relation_list=[]
    image_list =[]
    image_objects_dict={}
    image_pair_dict={}
    check_relation={}
    for i in range(len(objects)) :
        relation_candidate=indices[np.where(indices[:,0]==i)[0],1].tolist()
        rr={}
        for j in relation_candidate :
            if j <=i :
                break
            rr[int(j)]="dd"
            for image in list(set(objects[i]['color_path'])&set(objects[j]['color_path'])):
                relation_list.append([i,j])
                image_new = som_pth + image.split('/')[-1].split('.')[0]+".png"
                image_list.append(image_new)
                if image_new in image_objects_dict.keys() :
                    image_objects_dict[image_new].append(i)
                    image_objects_dict[image_new].append(j)
                    image_pair_dict[image_new].append([i,j])
                    image_objects_dict[image_new] = list(set(image_objects_dict[image_new]))
                else :
                    image_objects_dict[image_new]=[i,j]
                    image_pair_dict[image_new]=[[i,j]]
            check_relation[int(i)] = rr
    return relation_list,image_list,image_objects_dict,image_pair_dict,check_relation

def prepareForQueryNode(save_path,step,graph_fordraw):
    # obj_centers = getObjectCenter(obstacle_map)
    pos = nx.get_node_attributes(graph_fordraw, 'pos')
    # #### generate graph node for LLM####################################
    # uppercase_letters = list(string.ascii_uppercase)
    nx.draw_networkx_nodes(graph_fordraw, pos, node_size=250, node_color='lightblue', edgecolors='black')
    # 绘制边
    nx.draw_networkx_edges(graph_fordraw, pos)
    # 绘制节点标签
    nx.draw_networkx_labels(graph_fordraw, pos, font_size=8, font_color='black')
    # wall_index = np.where(np.rint(obstacle_map) == 1)
    # plt.scatter(wall_index[1],wall_index[0],color='sienna',s=20,zorder=1)
    # plt.scatter(np.where(ex_map!=0)[1],np.where(ex_map!=0)[0],color='lightgrey',s=10,zorder=0)
    # for i,center in enumerate(obj_centers) :
    #     lower_left_corner = (int(center[1]) - 4, int(center[0]) - 4)
    #     square = patches.Rectangle(lower_left_corner, 8, 8, color='green', fill=False, lw=2)
    #     plt.gca().add_patch(square)
    #     plt.text(int(center[1]), int(center[0]), uppercase_letters[i], fontsize=10, color='black', ha='center', va='center')
    # 显示图形
    plt.gca().invert_yaxis()  # 保持坐标轴比例
    plt.axis('off')
    plt.savefig(save_path+"map/graph_"+str(step)+".jpg")
    plt.clf()
    # return obj_centers

def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

# 获取指定距离范围内字符串最多的一个
def get_most_frequent_string_within_distance(coordinates, target_coord, max_distance):
    # 筛选在最大距离范围内的字符串
    within_distance_strings = [
        label for coord, label in coordinates.items() 
        if euclidean_distance(coord, target_coord) <= max_distance
    ]
    
    if not within_distance_strings:
        return None  # 如果没有符合条件的字符串，则返回None
    
    # 统计字符串出现的次数
    string_count = Counter(within_distance_strings)
    
    # 返回出现次数最多的字符串
    most_frequent_string = string_count.most_common(1)[0][0]
    return most_frequent_string


def projectHistoryToGraph(graph,current_pos,room_message,room_threshold):
    pos = nx.get_node_attributes(graph, 'pos')
    node_rooms={}
    for node,position in pos.items() :
        node_rooms[node] = get_most_frequent_string_within_distance(room_message, position, room_threshold)
    return node_rooms