from scipy.spatial import  Voronoi,voronoi_plot_2d
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from scipy.spatial import distance
import open3d as o3d
from collections import deque
import os
from scenegraph.visSceneGraph import drawScenegraph,drawScenegraph2,draw_Voronoi
import matplotlib.colors as mcolors
import cv2
from scipy.ndimage import binary_erosion
from scipy.spatial._qhull import QhullError
from envs.utils.fmm_planner import FMMPlanner
from skimage.draw import line
from tqdm import tqdm
import time
import math
from networkx.exception import NetworkXNoPath
import copy
css4_colors = mcolors.TABLEAU_COLORS
color_proposals = [list(mcolors.hex2color(color)) for color in css4_colors.values()]
def VorRemoveOut(vor,map_2d,obs_map):
    vertices=vor.vertices
    relation = vor.ridge_vertices
    b_f,b_u = np.min(vor.points,axis=0),np.max(vor.points,axis=0)
    index1 = np.where(((vertices >= b_f) & (vertices < b_u))[:,0] & ((vertices >= b_f) & (vertices < b_u))[:,1])[0]
    verticesfloor= np.round(vertices[index1]).astype(np.int32)
    index_final = np.where(map_2d[verticesfloor[:,1],verticesfloor[:,0]]==1)[0]
    index_remain = index1[index_final]
    v_remain = vertices[index_remain]
    points =np.vstack((np.where(obs_map==1)[1],np.where(obs_map==1)[0])).T
    # print(points.shape)
    distance = np.linalg.norm(v_remain[:,None,:] - points[None,:,:],axis=-1)
    index_remain= list(set(index_remain)-set(index_remain[np.where(distance < 4)[0]]))
    vor.vertices = vertices[index_remain]
    relation_new=[]
    for i in range(len(relation)):
        if relation[i][0] in index_remain and relation[i][1] in index_remain and relation[i][0] >=0 and relation[i][1] >=0:
            relation_new.append((index_remain.index(relation[i][0]),index_remain.index(relation[i][1])))
    vor.ridge_vertices = relation_new
    vor.vertices= vertices[index_remain]
    return vor
def visual2Dgraph(graph,step,n,save_path):
    # 获取节点位置
    pos = nx.get_node_attributes(graph, 'pos')
    # 绘制原图
    plt.figure(figsize=(10, 8))
    # nx.draw(G, pos, with_labels=True, node_color='lightblue', edge_color='gray')
    # 绘制最大连通分量
    nx.draw(graph, pos, with_labels=True, node_color='orange', edge_color='red')
    plt.title('Graph with Largest Connected Component Highlighted')
    plt.savefig(save_path+str(step)+"_"+str(n)+".png")
    plt.close() 
def remove_isolated_nodes(G):
    isolated_nodes = [node for node, degree in G.degree() if degree == 0]
    G.remove_nodes_from(isolated_nodes)
    return G
def judgePassObstacle(pos1,pos2,obstacle_map):
    rr, cc = line(int(pos1[1]), int(pos1[0]), int(pos2[1]), int(pos2[0]))
    passed_occupied = np.any(obstacle_map[rr, cc] == 1)
    if passed_occupied:
        return False
    else :
        return True
def calculate_angle_between_vectors(v1, v2):
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    cos_theta = dot_product / (norm_v1 * norm_v2)
    angle_rad = np.arccos(np.clip(cos_theta, -1.0, 1.0))  # 防止浮点数精度问题
    angle_deg = np.degrees(angle_rad)  # 转换为角度
    return angle_deg
def simplify_graph(G):
    # 创建一个副本以避免在迭代时修改图
    H = G.copy()
    # 查找所有度数为2的中间节点
    nodes_to_remove = [node for node in H.nodes() if H.degree(node) == 2]
    
    for node in nodes_to_remove:
       while H.degree(node) == 2:  # 确保处理后的节点度数仍然为2
            neighbors = list(H.neighbors(node))
            if len(neighbors) == 2:
                # 添加一条边连接中间节点的邻居
                H.add_edge(neighbors[0], neighbors[1])
                # 移除节点的所有边
                H.remove_edge(node, neighbors[0])
                H.remove_edge(node, neighbors[1])
        # 移除中间节点
        # H.remove_node(node)
    
    # # 移除冗余边（可能会在某些情况下出现）
    # H = nx.Graph(H)  # 确保图是无向图并去除重复边
    H=remove_isolated_nodes(H)
    return H
def simplify_graph2(G):
    # 创建一个副本以避免在迭代时修改图
    H = G.copy()
    # pos = nx.get_node_attributes(H, 'pos')            
    # for node in list(pos.keys()) :
    #     if node in H.nodes() and node not in leaf_values.keys():
    #         neighbors = list(H.neighbors(node))
    #         for neighbor in neighbors :
    #             if neighbor in H.nodes() and neighbor not in leaf_values.keys():
    #                 dist = compute_euclidean_distance(pos[node],pos[neighbor])
    #                 if dist < threshold :
    #                     if H.degree(node) > H.degree(neighbor) :
    #                         H = nx.contracted_nodes(H , node, neighbor, self_loops=False)
    #                     else :
    #                         H = nx.contracted_nodes(H , neighbor, node, self_loops=False)
    #                         break
    pos = nx.get_node_attributes(H, 'pos')
    nodes_to_remove = [node for node in H.nodes() if H.degree(node) == 2]
    for node in nodes_to_remove:
       while H.degree(node) == 2:  # 确保处理后的节点度数仍然为2
            neighbors = list(H.neighbors(node))
            if len(neighbors) == 2:
                # 添加一条边连接中间节点的邻居
                vector1 = np.array(pos[neighbors[0]]) - np.array(pos[node])
                vector2 = np.array(pos[neighbors[1]]) - np.array(pos[node])
                # 计算夹角
                angle = calculate_angle_between_vectors(vector1, vector2)
                if angle > 150 :
                # if judgePassObstacle(pos[neighbors[0]],pos[neighbors[1]],obs_map):
                    H.add_edge(neighbors[0], neighbors[1])
                    # 移除节点的所有边
                    H.remove_edge(node, neighbors[0])
                    H.remove_edge(node, neighbors[1])
                # if judgePassObstacle(pos[neighbors[0]],pos[neighbors[1]],obs_map) == False:
                if angle <= 150 :
                    break                        
    H=remove_isolated_nodes(H)
    
    return H
def find_nearest_non_zero_bfs(map, start):
    rows, cols = map.shape
    visited = set()
    queue = deque([start])
    
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上下左右四个方向

    while queue:
        x, y = queue.popleft()
        if (x, y) in visited:
            continue
        visited.add((x, y))
        
        if map[x, y] != 0:
            return [x, y]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and (nx, ny) not in visited:
                queue.append((nx, ny))
    
    return None  # 如果没有找到非零值
def getAngle(G,node1,node2):
    neighbor1 = list(G.neighbors(node1))[0]
    neighbor2 = list(G.neighbors(node2))[0]
    a1 = np.array([G.nodes()[node1]['pos'][0]-G.nodes()[neighbor1]['pos'][0],G.nodes()[node1]['pos'][1]-G.nodes()[neighbor1]['pos'][1]])
    a2 = np.array([G.nodes()[node2]['pos'][0]-G.nodes()[neighbor2]['pos'][0],G.nodes()[node2]['pos'][1]-G.nodes()[neighbor2]['pos'][1]])
    if np.dot(a1,a2)/(np.linalg.norm(a1)*np.linalg.norm(a2)) > np.cos(np.radians(45)) :
        return True
    else :
        return False
def find_merge_leaf2(G, leaf_nodes, start_leaf,leaf_value,distance_dict,threshold = 20):
    # 计算从start_leaf出发到所有其他叶子节点的最短路径长度
    nearest_leaf = []
    # shortest_path_lengths = dict(nx.all_pairs_shortest_path_length(G))
    for leaf in leaf_nodes:
        if leaf > start_leaf:
            query = (start_leaf,leaf)
            if  distance_dict.get(query,100) <=threshold and getAngle(G,start_leaf,leaf):
                    nearest_leaf.append(leaf)
    return nearest_leaf
def find_merge_leaf(G, leaf_nodes, start_leaf,leaf_value,distance_threshold=20,threshold = 5):
    # 计算从start_leaf出发到所有其他叶子节点的最短路径长度
    nearest_leaf = []
    pos = nx.get_node_attributes(G, 'pos') 
    shortest_path_lengths = dict(nx.all_pairs_shortest_path_length(G))
    for leaf in leaf_nodes:
        if start_leaf in shortest_path_lengths.keys() and leaf in shortest_path_lengths[start_leaf].keys() :
            if leaf != start_leaf and leaf_value[start_leaf] == leaf_value[leaf] and shortest_path_lengths[start_leaf][leaf] <=threshold and euclidean_distance(pos[leaf],pos[start_leaf]) <= distance_threshold  and getAngle(G,start_leaf,leaf):
                nearest_leaf.append(leaf)
            # elif leaf != start_leaf and euclidean_distance(pos[leaf],pos[start_leaf]) <= 15 and getAngle(G,start_leaf,leaf):
            #     nearest_leaf.append(leaf)
    return nearest_leaf
def calculate_Distance(G,leaf_nodes):
    pos = nx.get_node_attributes(G, 'pos')
    distances_pair={}
    for i in range(len(leaf_nodes)-1):
        for j in range(i+1,len(leaf_nodes)):
            try:
                path = nx.shortest_path(G, source=leaf_nodes[i], target=leaf_nodes[j])

                distance = compute_euclidean_distance(pos[leaf_nodes[i]],pos[path[0]])
                distance += compute_euclidean_distance(pos[leaf_nodes[j]],pos[path[-1]])
                for nidx in range(len(path)-1) :
                    distance += compute_euclidean_distance(pos[path[nidx]],pos[path[nidx+1]])
                distances_pair[(leaf_nodes[i],leaf_nodes[j])] = distance
            except NetworkXNoPath:
                # 如果没有路径，捕获异常并提示
                pass
    return distances_pair
def getLeafValue(map_2d,graph):
    leaf_nodes = [node for node in graph.nodes if graph.degree[node] == 1 and 'pos' in graph.nodes[node].keys()]
    node = []
    points_2d=[]
    direction = []
    for i in leaf_nodes:
        points_2d.append([graph.nodes()[i]['pos'][0],graph.nodes()[i]['pos'][1]])
        neighbor =  list(graph.neighbors(i))[0]
        direction.append([graph.nodes()[i]['pos'][0]-graph.nodes()[neighbor]['pos'][0],graph.nodes()[i]['pos'][1]-graph.nodes()[neighbor]['pos'][1]])
        node.append(i)
    
    direction = np.array(direction)
    points_2d = np.array(points_2d)
    direction = direction/np.linalg.norm(direction,axis=-1,keepdims=True)
    scales = np.arange(1,11).reshape(10,1)
    direction = direction[:, np.newaxis, :] * scales
    points_index = direction + points_2d[:,np.newaxis,:]
    indices = np.floor(np.array(points_index)).astype(np.int16)
    indices = np.clip(indices,np.array([0,0]),np.array([map_2d.shape[1]-1,map_2d.shape[0]-1]))
    value = map_2d[indices.reshape(-1,2)[:,1],indices.reshape(-1,2)[:,0]].reshape(-1,10)
    non_zero_one_mask = ((value != 0) * (value != 1))
    first_non_zero_one_indices = np.argmax(non_zero_one_mask, axis=1)
    first_non_zero_one_indices[np.all(~non_zero_one_mask, axis=1)] = -1
    first_non_zero_values = value[np.arange(value.shape[0]), first_non_zero_one_indices]
    # first_non_zero_values[first_non_zero_one_indices == -1] = 0
    leaf_value = first_non_zero_values
    value_dict={}
    value_position={}
    # remove_ele = []
    for node,value in zip(leaf_nodes,leaf_value):
        # if value !=0 :
            value_dict[node] = value
            value_position[node] = graph.nodes()[node]['pos']
    return value_dict,value_position
def checkleafnew(graph,leaf_nodes):
    isolated_nodes = [node for node, degree in graph.degree() if degree == 1 and 'pos' in graph.nodes[node].keys()]
    deleted=list(set(isolated_nodes)-set(leaf_nodes))
    while deleted :
        for delete in deleted :
            graph.remove_node(delete)
        isolated_nodes = [node for node, degree in graph.degree() if degree == 1 and 'pos' in graph.nodes[node].keys()]
        deleted=list(set(isolated_nodes)-set(leaf_nodes))
    return graph
def mergeGraphByObjects(graph,value_dict,leaf_position):
    # start=time.time()
    # # distance_dict=calculate_Distance(graph,list(value_dict.keys()))
    # end=time.time()
    # print("distance calculation time :",end-start)
    # merged=[]
    nearest_leaf_nodes={}
    # start = time.time()
    values = np.array(list(value_dict.values()))
    # print("leaf_values:",values)
    keys = np.array(list(value_dict.keys()))
    value_matrix = (values[:, None] == values[None, :]).astype(int)
    np.fill_diagonal(value_matrix, 0)
    positions= np.array(list(leaf_position.values()))
    distance = np.linalg.norm(positions[None,:,:]-positions[:,None,:],axis=-1)
    # print(value_matrix.shape,distance.shape)
    judge = ((distance <= 20) *(value_matrix==1))
    judge[np.tril_indices_from(judge, k=-1)] = 0
    shortest_path_lengths = dict(nx.all_pairs_shortest_path_length(graph))
    for node in range(len(judge)):
        nearest_leaf = []
        related_node = keys[np.where(judge[node])[0]]
        # print(keys,related_node)
        for node_r in related_node :
            if keys[node] in shortest_path_lengths.keys() and node_r in shortest_path_lengths[keys[node]].keys() and shortest_path_lengths[keys[node]][node_r] <= 5 :
                if getAngle(graph,keys[node],node_r):
                    nearest_leaf.append(node_r)
        # print(keys[node],nearest_leaf)
        nearest_leaf_nodes[keys[node]]=nearest_leaf
    # end = time.time()
    # print("calculate1:",end-start)
    # start = time.time()
    # # pos_leaf = 
    # for leaf in value_dict.keys():
    #     if leaf not in merged :
    #         group = find_merge_leaf(graph, value_dict.keys(), leaf,value_dict)
    #         print(leaf,group)
    #         merged = list(set(merged + group))
    #         nearest_leaf_nodes[leaf]=group
    # end = time.time()
    # print("calculate2:",end-start)
    # nearest_leaf_nodes = {leaf: find_merge_leaf(graph, value_dict.keys(), leaf,value_dict) for leaf in value_dict.keys()}
    group_dict = {}
    group = []
    for key,values in nearest_leaf_nodes.items():
        if len(values) == 0 :
            length=len(group)
            group.append([key])
            group_dict[key]=length
        else :
            for value in values :
                if key in group_dict.keys():
                    if value not in group_dict.keys() :
                        group_dict[value] = group_dict[key]
                        group[group_dict[key]].append(value)
                else :
                    if value in group_dict.keys():
                        group_dict[key] = group_dict[value]
                        group[group_dict[value]].append(key)
                    else :
                        length=len(group)
                        group.append([key,value])
                        group_dict[key]=length
                        group_dict[value]=length
    return group
def remove_degree_2_nodes(graph):
    # 找到所有度为2的节点
    degree_2_nodes = [node for node in graph.nodes() if graph.degree(node) == 2]
    
    while degree_2_nodes:
        for node in degree_2_nodes:
            neighbors = list(graph.neighbors(node))
            if len(neighbors) == 2:
                # 获取两个邻居节点
                u, v = neighbors
                # 如果 u 和 v 之间还没有边，则添加一条边
                if not graph.has_edge(u, v):
                    graph.add_edge(u, v)
                # 删除度为2的节点
                graph.remove_node(node)
        
        # 重新计算所有度为2的节点
        degree_2_nodes = [node for node in graph.nodes() if graph.degree(node) == 2]
    
    return graph
def CoarseGraph(graph,group,leaf_nodes):
    # print(group)
    G = graph.copy()
    pos = nx.get_node_attributes(G, 'pos')
    for grp in group :
        if len(grp) == 2 :
            G.remove_node(grp[1])
            del leaf_nodes[grp[1]]
        elif len(grp) > 2 :
            node_ave=np.array([0,0],dtype=np.float64)
            nodes_array=[]
            neighbors=[]
            for i in grp :
                node_ave += np.array(pos[i])
                nodes_array.append(list(pos[i]))
                neighbors.append(list(graph.neighbors(i))[0])
            node_ave /= len(grp)
            merged_to_node = np.argmin(np.linalg.norm(node_ave-np.array(nodes_array),axis=-1))
            for i,n in zip(grp,neighbors) :
                if i != grp[merged_to_node] :
                    # if i in leaf_nodes :
                        del leaf_nodes[i]
                        G.remove_node(i)
    return G,leaf_nodes
def FinalCoarseGraph(graph,group,leaf_nodes):
    G = graph.copy()
    pos = nx.get_node_attributes(G, 'pos')
    nodes_to_merge = []
    for grp in group :
        if len(grp) == 2 :
           nodes_to_merge.append((grp[0],grp[1]))
           if list(G.neighbors(grp[0]))[0] != list(G.neighbors(grp[1]))[0]:
                nodes_to_merge.append((list(G.neighbors(grp[0]))[0],list(G.neighbors(grp[1]))[0]))
                del leaf_nodes[grp[1]]
        else :
            node_ave=np.array([0,0],dtype=np.float64)
            nodes_array=[]
            neighbors=[]
            for i in grp :
                node_ave += np.array(pos[i])
                nodes_array.append(list(pos[i]))
                neighbors.append(list(graph.neighbors(i))[0])
            node_ave /= len(grp)
            merged_to_node = np.argmin(np.linalg.norm(node_ave-np.array(nodes_array),axis=-1))
            for i,n in zip(grp,neighbors) :
                if i != grp[merged_to_node] :
                    del leaf_nodes[i]
                    nodes_to_merge.append((grp[merged_to_node],i))
                    if n != list(graph.neighbors(grp[merged_to_node]))[0] :
                        nodes_to_merge.append((list(graph.neighbors(grp[merged_to_node]))[0],n))
    # 合并节点
    for node1, node2 in nodes_to_merge:
        if G.has_node(node1) and G.has_node(node2):
            G = nx.contracted_nodes(G, node1, node2, self_loops=False)
    for node in leaf_nodes.keys() :
        if node in G.nodes() and len(list(G.neighbors(node))) > 1 :
            remain_node = None
            nmax=0
            for neighbor in list(G.neighbors(node)) :
                if len(list(G.neighbors(neighbor))) > nmax :
                    remain_node = neighbor
                    nmax = len(list(G.neighbors(neighbor)))
            for neighbor in list(G.neighbors(node)) :
                if neighbor != remain_node :
                    G.remove_edge(node,neighbor)
    nodes_to_remove = []
    for node in G.nodes() :
        if G.degree[node] == 1 and 'pos' in G.nodes[node].keys() and node not in leaf_nodes.keys() :
            nodes_to_remove.append(node)
    while nodes_to_remove :
        for node in nodes_to_remove :
            G.remove_node(node)
        nodes_to_remove = []
        for node in G.nodes() :
            if G.degree[node] == 1 and 'pos' in G.nodes[node].keys() and node not in leaf_nodes.keys() :
                nodes_to_remove.append(node)
    # G = remove_degree_2_nodes(G)
    # G = merge_closest_non_leaf_nodes(G)
    # G = remove_isolated_nodes(G)
    return G,leaf_nodes
def visualizationVoro(objects,bg_objects,floor_pcd,voronoi,bbox):
    points_2d=(voronoi.points+np.array([bbox[0]-1,bbox[1]-1])+0.5)*5
    vertices_2d = (voronoi.vertices+np.array([bbox[0]-1,bbox[1]-1])+0.5)*5
    relation=voronoi.ridge_vertices
    points_3d = np.append(points_2d,np.ones((points_2d.shape[0],1)),1)
    vertices_3d = np.append(vertices_2d,np.ones((vertices_2d.shape[0],1)),1)
    colors_p = np.ones_like(points_3d)*[1,0,0]
    pcd=o3d.geometry.PointCloud()
    pcd.points=o3d.utility.Vector3dVector(points_3d)
    pcd.normals=o3d.utility.Vector3dVector(points_3d)
    pcd.colors=o3d.utility.Vector3dVector(colors_p)
    colors = [[0, 0, 1] for i in range(len(relation))]

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(vertices_3d),
        lines=o3d.utility.Vector2iVector(np.array(relation)),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([pcd,line_set])
    draw_Voronoi(objects,bg_objects,floor_pcd,pcd,line_set)
def create_sphere_at_point(point, color, radius=3):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.paint_uniform_color(color)
    sphere.translate(point)
    return sphere
def visualizationGraph(objects,bg_objects,voronoi,graph,group):
    points_2d=(voronoi.points+0.5)*5
    points_3d = np.append(points_2d,np.ones((points_2d.shape[0],1)),1)
    colors_p = np.ones_like(points_3d)*[1,0,0]
    pcd=o3d.geometry.PointCloud()
    pcd.points=o3d.utility.Vector3dVector(points_3d)
    pcd.normals=o3d.utility.Vector3dVector(points_3d)
    pcd.colors=o3d.utility.Vector3dVector(colors_p)

    all_edges_with_data = graph.edges(data=False)
    pos = nx.get_node_attributes(graph, 'pos')
    # 打印所有边及其属性
    node = []
    points_2d=[]
    relation=[]
    for i,pose in pos.items():
        points_2d.append([pose[0],pose[1]])
        node.append(i)
    for edge in all_edges_with_data:
        relation.append([node.index(edge[0]),node.index(edge[1])])
    # points_2d = (points_2d+np.array([bbox[0],bbox[1]])+0.5)*5
    colors = [[0, 0, 1] for i in range(len(relation))]
    vertices_3d = np.append(np.array(points_2d),np.ones((np.array(points_2d).shape[0],1)),1)
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(vertices_3d),
        lines=o3d.utility.Vector2iVector(np.array(relation)),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    points=[]
    colors=[]
    for i in range(len(group)):
        for g in group[i] :
            if g in pos.keys():
                points.append([graph.nodes()[g]['pos'][0],graph.nodes()[g]['pos'][1],1])
                colors.append(color_proposals[i%len(color_proposals)])
    # for i in range(len(leaf_nodes)):
    #     points.append([(graph.nodes()[leaf_nodes[i]]['pos'][0]+bbox[0]-0.5)*5,(graph.nodes()[leaf_nodes[i]]['pos'][1]+bbox[1]-0.5)*5,1])
    #     if leaf_values[i] == -1 :
    #         colors.append([1,1,0])
    #     else :
    #         colors.append([0,1,1])
    spheres = [create_sphere_at_point(point, color) for point, color in zip(points, colors)]
    # drawScenegraph2(objects,bg_objects,pcd,spheres,line_set)
def find_non_leaf_nodes(graph):
    non_leaf_nodes = [node for node in graph.nodes() if len(list(graph.neighbors(node))) > 1]
    return non_leaf_nodes

def euclidean_distance(pos, node1, node2):
    x1, y1 = pos[node1]
    x2, y2 = pos[node2]
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def merge_closest_non_leaf_nodes(graph,min_distance = 5):
    non_leaf_nodes = list(nx.get_node_attributes(graph, 'pos').keys())
    lengths = dict(nx.all_pairs_shortest_path_length(graph))
    # while len(non_leaf_nodes) > 1:
    closest_pair = []
    pos = nx.get_node_attributes(graph, 'pos')
    for i, node1 in enumerate(non_leaf_nodes):
        
        for node2 in non_leaf_nodes[i + 1:]:
            distance = euclidean_distance(pos[node1], pos[node2])
            if distance < min_distance and node1 in lengths.keys() and node2 in lengths[node1].keys() and lengths[node1][node2] < 3:
                closest_pair.append((node1, node2)) 
        if closest_pair:
            for pair in closest_pair:
                node1, node2 = pair
                if graph.has_node(node1) and graph.has_node(node2) :
                    # 计算子节点数量
                    num_subnodes1 = graph.degree(node1)

                    # 计算子节点数量
                    num_subnodes2 = graph.degree(node2)
                    if num_subnodes1 >= num_subnodes2 :
                        graph = nx.contracted_nodes(graph, node1, node2, self_loops=False)
                    else :
                        graph = nx.contracted_nodes(graph, node2, node1, self_loops=False)

    return graph
def generate_local_graph(voronoi,obstacle_map):
    G = nx.Graph()
    for i, point in enumerate(voronoi.vertices):
        G.add_node(i, pos=(point[0],point[1]))
    G.add_edges_from(voronoi.ridge_vertices)
    G = sparsify_graph(G)
    G = remove_degree_2_nodes(G)
    largest_cc = max(nx.connected_components(G), key=len)
    subgraph = G.subgraph(largest_cc)
    extractG,leaf_values = getLeafValue(obstacle_map,subgraph)
    
    ### generate by agent pose and choose the nearest path 
    # extractG,group,node_values = mergeGraphByObjects(G,leaf_values)
    # graph_coarse = FinalCoarseGraph(extractG,group,leafs)
    return extractG,leaf_values

#### merge to global  
def find_non_leaf_nodes(G):
    return [n for n, d in G.degree() if d > 1]

def compute_euclidean_distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))
def judgeDirection(G1,G2,node1,node2):
    neighbor1 = list(G1.neighbors(node1))[0]
    neighbor2 = list(G2.neighbors(node2))[0]
    a1 = np.array([G1.nodes()[node1]['pos'][0]-G1.nodes()[neighbor1]['pos'][0],G1.nodes()[node1]['pos'][1]-G1.nodes()[neighbor1]['pos'][1]])
    a2 = np.array([G2.nodes()[node2]['pos'][0]-G2.nodes()[neighbor2]['pos'][0],G2.nodes()[node2]['pos'][1]-G2.nodes()[neighbor2]['pos'][1]])

    dot_product = np.dot(a1, -a2)
    
    # 计算向量的模长
    norm_v1 = np.linalg.norm(a1)
    norm_v2 = np.linalg.norm(a2)
    
    # 计算夹角的余弦值
    cos_theta = dot_product / (norm_v1 * norm_v2)
    
    # 计算cos(30度)
    cos_30 = np.cos(np.radians(30))
    
    # 判断夹角是否小于30度
    return cos_theta > cos_30
def find_closest_non_leaf_nodes(G1, G2):
    non_leaf_nodes_G1 = find_non_leaf_nodes(G1)
    non_leaf_nodes_G2 = find_non_leaf_nodes(G2)
    pos1 = nx.get_node_attributes(G1, 'pos')
    pos2 = nx.get_node_attributes(G2, 'pos')
    min_distance = float('inf')
    closest_nodes = (None, None)
    
    for n1 in non_leaf_nodes_G1:
        for n2 in non_leaf_nodes_G2:
            dist = compute_euclidean_distance(pos1[n1], pos2[n2])
            if dist < min_distance:
                min_distance = dist
                closest_nodes = (n1, n2)
    return closest_nodes
def merge_graphs_at_closest_non_leaf_nodes1(G1, G2,global_node_values,local_node_values,remove_global):
    add_edges=None
    pos1 = nx.get_node_attributes(G1, 'pos')
    pos2 = nx.get_node_attributes(G2, 'pos')
    remove_local=np.array(list(local_node_values.keys()))[np.where(np.array(list(local_node_values.values()))==-2)[0]]
    remove_local,remove_global = remove_local.tolist(),remove_global.tolist()

    min_distance=float('inf')
    edge1,edge2 = None,None
    for n2 in remove_local:
        for n1 in remove_global:
            dist = compute_euclidean_distance(pos1[list(G1.neighbors(n1))[0]], pos2[list(G2.neighbors(n2))[0]])
            if dist < min_distance  and judgeDirection(G1,G2,n1,n2):
                min_distance = dist
                edge1 = n1
                edge2 = n2
    if edge1 != None and edge2 != None :
        add_edges=(list(G1.neighbors(edge1))[0],list(G2.neighbors(edge2))[0])
            
   
    for node in remove_global :
        neighbor = list(G1.neighbors(node))[0]
        G1.remove_node(node)
        if G1.degree(neighbor) == 1 and add_edges != None and  neighbor != add_edges[0]:
            G1.remove_node(neighbor)
    for node in remove_local :
        neighbor = list(G2.neighbors(node))[0]
        G2.remove_node(node)
        if G2.degree(neighbor) == 1 and add_edges != None and neighbor != add_edges[1]:
            G2.remove_node(neighbor)
    global_new ={}
    # Merge graphs by creating a new graph
    mapping_G1 = {node: i for i, node in enumerate(G1.nodes())}
    G1 = nx.relabel_nodes(G1, mapping_G1)
    for key,value in global_node_values.items():
        global_new[mapping_G1[key]] = value
    offset = len(G1.nodes())
    mapping_G2 = {node: i + offset for i, node in enumerate(G2.nodes())}
    G2 = nx.relabel_nodes(G2, mapping_G2)
    for key,value in local_node_values.items():
        if key not in remove_local:
            global_new[mapping_G2[key]] = value
    merged_graph = nx.compose(G1, G2)
    if add_edges != None :
        merged_graph.add_edge(mapping_G1[add_edges[0]],mapping_G2[add_edges[1]])
    # merged_graph.add_edge(n1, n2)
    # pos1 = nx.get_node_attributes(G1, 'pos')
    # pos2 = nx.get_node_attributes(G2, 'pos')
    # pos = nx.get_node_attributes(merged_graph, 'pos')

    del G1,G2,global_node_values,local_node_values
    merged_graph = remove_degree_2_nodes(merged_graph)
    leaf_nodes = [node for node in merged_graph.nodes if merged_graph.degree[node] == 1 and 'pos' in merged_graph.nodes[node].keys()]
    deleted2 = list(set(leaf_nodes)-set(global_new.keys()))
    for d in deleted2 :
        neighbor = list(merged_graph.neighbors(d))[0]
        merged_graph.remove_node(d)
        while merged_graph.degree()[neighbor] ==1 :
            neighbor = list(merged_graph.neighbors(neighbor))[0]
            merged_graph.remove_node(neighbor)
    leaf_nodes = [node for node in merged_graph.nodes if merged_graph.degree[node] == 1 and 'pos' in merged_graph.nodes[node].keys()]
    deleted = list(set(global_new.keys())-set(leaf_nodes))
    if deleted :
        for d in deleted :
            del global_new[d]
    return merged_graph,global_new
def merge_graphs_at_closest_non_leaf_nodes(G1, G2,global_node_values,local_node_values,remove_global):
    add_edges=[]
    add_node_list1={}
    add_node_list2={}
    pos1 = nx.get_node_attributes(G1, 'pos')
    pos2 = nx.get_node_attributes(G2, 'pos')
    remove_local=np.array(list(local_node_values.keys()))[np.where(np.array(list(local_node_values.values()))==-2)[0]]
    remove_local,remove_global = remove_local.tolist(),remove_global.tolist()
    if len(remove_local) < len(remove_global) :
        remove_global1 = remove_global.copy()
        for n2 in remove_local:
            min_distance=float('inf')
            n1 = None
            for n in remove_global1:
                dist = compute_euclidean_distance(pos1[n], pos2[n2])
                if dist < min_distance :
                    min_distance = dist
                    n1 = n
            if n1 is not None :
                if list(G1.neighbors(n1))[0] not in add_node_list1.keys() and list(G2.neighbors(n2))[0] not in add_node_list2.keys():
                    add_edges.append((list(G1.neighbors(n1))[0],list(G2.neighbors(n2))[0],min_distance))
                    add_node_list1[list(G1.neighbors(n1))[0]] = len(add_edges)-1
                    add_node_list2[list(G2.neighbors(n2))[0]] = len(add_edges)-1
                elif list(G1.neighbors(n1))[0] in add_node_list1.keys():
                    if add_edges[add_node_list1[list(G1.neighbors(n1))[0]]][2] > min_distance :
                        del add_edges[add_node_list1[list(G1.neighbors(n1))[0]]]
                        del add_node_list1[list(G1.neighbors(n1))[0]]
                        if list(G2.neighbors(n2))[0] in add_node_list2.keys():
                            del add_node_list2[list(G2.neighbors(n2))[0]]
                        add_edges.append((list(G1.neighbors(n1))[0],list(G2.neighbors(n2))[0],min_distance))
                        add_node_list1[list(G1.neighbors(n1))[0]] = len(add_edges)-1
                        add_node_list2[list(G2.neighbors(n2))[0]] = len(add_edges)-1
                else :
                    if add_edges[add_node_list2[list(G2.neighbors(n2))[0]]][2] > min_distance :
                        del add_edges[add_node_list2[list(G2.neighbors(n2))[0]]]
                        del add_node_list2[list(G2.neighbors(n2))[0]]
                        add_edges.append((list(G1.neighbors(n1))[0],list(G2.neighbors(n2))[0],min_distance))
                        add_node_list1[list(G1.neighbors(n1))[0]] = len(add_edges)-1
                        add_node_list2[list(G2.neighbors(n2))[0]] = len(add_edges)-1
                remove_global1.remove(n1)
    else :
        remove_local1 = remove_local.copy()
        for n1 in remove_global:
            min_distance=float('inf')
            n2 = None
            for n in remove_local1:
                dist = compute_euclidean_distance(pos1[n1], pos2[n])
                if dist < min_distance :
                    min_distance = dist
                    n2 = n
            if n2 is not None :
                if list(G1.neighbors(n1))[0] not in add_node_list1.keys() and list(G2.neighbors(n2))[0] not in add_node_list2.keys():
                    add_edges.append((list(G1.neighbors(n1))[0],list(G2.neighbors(n2))[0],min_distance))
                    add_node_list1[list(G1.neighbors(n1))[0]] = len(add_edges)-1
                    add_node_list2[list(G2.neighbors(n2))[0]] = len(add_edges)-1
                elif list(G1.neighbors(n1))[0] in add_node_list1.keys():
                    if add_edges[add_node_list1[list(G1.neighbors(n1))[0]]][2] > min_distance :
                        del add_edges[add_node_list1[list(G1.neighbors(n1))[0]]]
                        del add_node_list1[list(G1.neighbors(n1))[0]]
                        if list(G2.neighbors(n2))[0] in add_node_list2.keys():
                            del add_node_list2[list(G2.neighbors(n2))[0]]
                        add_edges.append((list(G1.neighbors(n1))[0],list(G2.neighbors(n2))[0],min_distance))
                        add_node_list1[list(G1.neighbors(n1))[0]] = len(add_edges)-1
                        add_node_list2[list(G2.neighbors(n2))[0]] = len(add_edges)-1
                else :
                    if add_edges[add_node_list2[list(G2.neighbors(n2))[0]]][2] > min_distance :
                        del add_edges[add_node_list2[list(G2.neighbors(n2))[0]]]
                        del add_node_list2[list(G2.neighbors(n2))[0]]
                        add_edges.append((list(G1.neighbors(n1))[0],list(G2.neighbors(n2))[0],min_distance))
                        add_node_list1[list(G1.neighbors(n1))[0]] = len(add_edges)-1
                        add_node_list2[list(G2.neighbors(n2))[0]] = len(add_edges)-1
                remove_local1.remove(n2)
    for node in remove_global :
        G1.remove_node(node)
    for node in remove_local :
        G2.remove_node(node)
    global_new ={}
    # Merge graphs by creating a new graph
    mapping_G1 = {node: i for i, node in enumerate(G1.nodes())}
    G1 = nx.relabel_nodes(G1, mapping_G1)
    for key,value in global_node_values.items():
        global_new[mapping_G1[key]] = value
    offset = len(G1.nodes())
    mapping_G2 = {node: i + offset for i, node in enumerate(G2.nodes())}
    G2 = nx.relabel_nodes(G2, mapping_G2)
    for key,value in local_node_values.items():
        if key not in remove_local:
            global_new[mapping_G2[key]] = value
    merged_graph = nx.compose(G1, G2)
    if add_edges:
        for edge in add_edges :
            merged_graph.add_edge(mapping_G1[edge[0]],mapping_G2[edge[1]])
    # merged_graph.add_edge(n1, n2)
    # pos1 = nx.get_node_attributes(G1, 'pos')
    # pos2 = nx.get_node_attributes(G2, 'pos')
    # pos = nx.get_node_attributes(merged_graph, 'pos')

    del G1,G2,global_node_values,local_node_values
    merged_graph = remove_degree_2_nodes(merged_graph)
    leaf_nodes = [node for node in merged_graph.nodes if merged_graph.degree[node] == 1 and 'pos' in merged_graph.nodes[node].keys()]
    deleted = list(set(global_new.keys())-set(leaf_nodes))
    if deleted :
        for d in deleted :
            del global_new[d]
    return merged_graph,global_new

def checkOutlierNodes(graph,node_values,obstacle_map,bbox) :
    leaf_nodes = list(node_values.keys())
    node = []
    points_2d=[]
    direction = []
    for i in leaf_nodes:
        points_2d.append([graph.nodes()[i]['pos'][0]/5-0.5-bbox[0],graph.nodes()[i]['pos'][1]/5-0.5-bbox[1]])
        neighbor =  list(graph.neighbors(i))[0]
        direction.append([graph.nodes()[i]['pos'][0]/5-graph.nodes()[neighbor]['pos'][0]/5,graph.nodes()[i]['pos'][1]/5-graph.nodes()[neighbor]['pos'][1]/5])
        node.append(i)
    direction = np.array(direction)
    points_2d = np.array(points_2d)
    direction = direction/np.linalg.norm(direction,axis=-1,keepdims=True)
    scales = np.arange(1,11).reshape(10,1)
    direction = direction[:, np.newaxis, :] * scales
    points_index = direction + points_2d[:,np.newaxis,:]
    indices = np.floor(np.array(points_index)).astype(np.int16)
    indices = np.clip(indices,np.array([0,0]),np.array([obstacle_map.shape[0]-1,obstacle_map.shape[1]-1]))
    value = obstacle_map[indices.reshape(-1,2)[:,0],indices.reshape(-1,2)[:,1]].reshape(-1,10)
    non_zero_one_mask = (value != 0) & (value != 1) & (value != -2)
    first_non_zero_one_indices = np.argmax(non_zero_one_mask, axis=1)
    first_non_zero_one_indices[np.all(~non_zero_one_mask, axis=1)] = -1
    first_non_zero_values = value[np.arange(value.shape[0]), first_non_zero_one_indices]
    first_non_zero_values[first_non_zero_one_indices == -1] = 0
    leaf_value = first_non_zero_values
    remove_nodes = np.array(leaf_nodes)[np.where((np.array(leaf_value)==0) & (np.array(list(node_values.values()))==-1))[0]]
    node_value_new ={}
    for i in range(len(leaf_nodes)) :
        if leaf_nodes[i] not in remove_nodes :
            node_value_new[leaf_nodes[i]] = leaf_value[i]
    del node_values
    return node_value_new,remove_nodes


def mapPosToGlobalGraph(graph,bbox):
    for node in graph.nodes():
        if 'pos' in graph.nodes[node].keys():
            graph.nodes()[node]['pos'] = ((graph.nodes()[node]['pos'][0]+bbox[0]+0.5)*5,(graph.nodes()[node]['pos'][1]+bbox[1]+0.5)*5)
            # graph.nodes()[node]['pos'][1] = (graph.nodes()[node]['pos'][1]+bbox[1]+0.5)*5
    return graph
def sparsify_graph(floor_graph: nx.Graph, resampling_dist: float = 10):
        """
        Sparsify a topology graph by removing nodes with degree 2.
        This algorithm first starts at degree-one nodes (dead ends) and
        removes all degree-two nodes until confluence nodes are found.
        Next, we find close pairs of higher-order degree nodes and
        delete all nodes if the shortest path between two nodes consists
        only of degree-two nodes.
        Args:
            floor_graph (nx.Graph): graph to sparsify
        Returns:
            nx.Graph: sparsified graph
        """
        graph = copy.deepcopy(floor_graph)

        if len(graph.nodes) < 10:
            return graph
        # all nodes with degree 1 or 3+
        new_node_candidates = [
            node for node in list(graph.nodes) if (graph.degree(node) != 2)
        ]

        new_graph = nx.Graph()
        for i, node in enumerate(new_node_candidates):
            new_graph.add_node(
                node,
                pos=graph.nodes[node]["pos"]
            )
        new_nodes = set(new_graph.nodes)
        new_nodes_list = list(new_graph.nodes)

        print(
            f"Getting paths between all nodes. Node number: {len(new_node_candidates)}/{len(graph.nodes)}"
        )

        st = time.time()
        all_path_dense_graph = dict(nx.all_pairs_dijkstra_path(graph, weight="dist"))
        ed = time.time()
        print("time for computing all pairs shortest path: ", ed - st, " seconds")
        sampled_edges_to_add = list()
        pbar = tqdm(range(len(new_graph.nodes)), desc="Sparsifying graph")
        for i in pbar:
            inner_pbar = tqdm(
                range(len(new_graph.nodes)), desc="Sparsifying graph", leave=False
            )
            for j in inner_pbar:
                if i >= j:
                    continue
                # Go through all edges along path and extract dist
                node1 = new_nodes_list[i]
                node2 = new_nodes_list[j]
                try:
                    path = all_path_dense_graph[node1][node2]
                    for node in path[1:-1]:
                        if graph.degree(node) > 2:
                            break
                    else:
                        sampled_edges_to_add.append(
                            (
                                path[0],
                                path[-1],
                                np.linalg.norm(np.array(path[0]) - np.array(path[-1])),
                            )
                        )
                        dist = [
                            graph.edges[path[k], path[k + 1]]["dist"]
                            for k in range(len(path) - 1)
                        ]
                        mov_agg_dist = 0
                        predecessor = path[0]
                        # connect the nodes if there is a path between them that does not go through any other of the new nodes
                        if (
                            len(path)
                            and len(set(path[1:-1]).intersection(new_nodes)) == 0
                        ):
                            for cand_idx, cand_node in enumerate(path[1:-1]):
                                mov_agg_dist += dist[cand_idx]
                                print(mov_agg_dist)
                                if mov_agg_dist > resampling_dist:
                                    sampled_edges_to_add.append(
                                        (
                                            predecessor,
                                            cand_node,
                                            np.linalg.norm(
                                                np.array(predecessor)
                                                - np.array(cand_node)
                                            ),
                                        )
                                    )
                                    predecessor = cand_node
                                    mov_agg_dist = 0
                                else:
                                    continue
                            sampled_edges_to_add.append(
                                (
                                    predecessor,
                                    path[-1],
                                    np.linalg.norm(
                                        np.array(predecessor) - np.array(path[-1])
                                    ),
                                )
                            )
                except:
                    continue

        for edge_param in sampled_edges_to_add:
            k, l, dist = edge_param
            if k not in new_graph.nodes:
                new_graph.add_node(
                    k, pos=graph.nodes[k]["pos"]
                )
            if l not in new_graph.nodes:
                new_graph.add_node(
                    l, pos=graph.nodes[l]["pos"]
                )
            new_graph.add_edge(k, l, dist=dist)

        return new_graph
def euclidean_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

def merge_close_nodes(G,threshold,obs_map):
    nodes_to_merge = []
    pos = nx.get_node_attributes(G, 'pos')
    for node1 in list(G.nodes()):
        for node2 in list(G.nodes()):
            if node1 != node2 and euclidean_distance(pos[node1], pos[node2]) < threshold :
                nodes_to_merge.append((node1, node2))
    for node1, node2 in nodes_to_merge:
        if node1 in G.nodes() and node2 in G.nodes() :
            if G.degree[node1] >= G.degree[node2]:
                node_to_keep = node1
                node_to_merge = node2
            else:
                node_to_keep = node2
                node_to_merge = node1
            G = nx.contracted_nodes(G,node_to_keep,node_to_merge,self_loops=False)
    return G
def DealGraph(graph,leaf_values,threshold):
    graph_new = graph.copy()
    leaf_nodes_map={}
    for node,value in leaf_values.items():
        neighbor = list(graph_new.neighbors(node))[0]
        graph_new = nx.contracted_nodes(graph_new,neighbor,node,self_loops=False)
        leaf_nodes_map[node]=neighbor
    pos = nx.get_node_attributes(graph_new, 'pos')
    graph_new,leaf_nodes_map = merge_close_nodes(graph_new,pos,threshold,leaf_nodes_map)
    nodes_show={}
    for value in leaf_nodes_map.values():
        nodes_show[value]=pos[value]
    # for node in graph_new.nodes() :
    #     if node in pos.keys() and len(list(graph_new.neighbors(node))) >2 :
    #         if node not in nodes_show.keys() :
    #             nodes_show[node]=pos[node]
    return graph_new,leaf_nodes_map,nodes_show
def get_largest_region(binary_map: np.ndarray) -> np.ndarray:
    """Get the largest disconnected island region in the binary map.

    Args:
        binary_map (np.ndarray): The binary map.

    Returns:
        np.ndarray: the largest region in the binary map.
    """
    # Threshold it so it becomes binary
    # ret, thresh = cv2.threshold(binary_map, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    input = (binary_map > 0).astype(np.uint8)
    output = cv2.connectedComponentsWithStats(input, 8, cv2.CV_8UC1)
    areas = output[2][:, -1]
    # TODO: the top region is 0 region, so we need to sort the areas and get the second largest
    # but I am not sure if the largest region is always the background
    id = np.argsort(areas)[::-1][1]
    return output[1] == id
# def getFrontierNodes(map,full_w,full_h):
#     kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
#     _local_ob_map = map[0][0].cpu().numpy()
#     local_ex_map = map[0][1].cpu().numpy()
#     local_ob_map = cv2.dilate(_local_ob_map, kernel)

#     show_ex = cv2.inRange(map[0][1].cpu().numpy(), 0.1, 1)

#     kernel = np.ones((5, 5), dtype=np.uint8)
#     free_map = cv2.morphologyEx(show_ex, cv2.MORPH_CLOSE, kernel)

#     contour, _ = cv2.findContours(free_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
#     if len(contour) > 0:
#         contour = max(contour, key=cv2.contourArea)
#         cv2.drawContours(local_ex_map, contour, -1, 1, 1) #  会原地修改输入图像

#     # clear the boundary
#     local_ex_map[0:2, 0:full_h] = 0.0
#     local_ex_map[full_w-2:full_w, 0:full_h-1] = 0.0
#     local_ex_map[0:full_w, 0:2] = 0.0
#     local_ex_map[0:full_w, full_h-2:full_h] = 0.0

#     # target_edge = np.zeros((local_w, local_h))
#     target_edge = local_ex_map - local_ob_map
#     from skimage import measure

#     img_label, num = measure.label(target_edge, connectivity=2, return_num=True)
#     regions = measure.regionprops(img_label)
def add_new_nodes_with_condition2(G, new_nodes):
    # 遍历新节点
    pos = nx.get_node_attributes(G, 'pos') 
    explored_nodes=[]
    n=len(pos)
    for new_coord in new_nodes:
        # 计算新节点与图中所有现有节点的最小距离
        min_distance = float('inf')
        closest_node = None
        
        for existing_node in G.nodes(data=True):
            existing_coord = existing_node[1]['pos']
            distance = euclidean_distance(new_coord, existing_coord)
            
            if distance < min_distance:
                min_distance = distance
                closest_node = existing_node[0]
        
        # 如果最小距离大于10，添加新节点并连接
        if min_distance > 20:
            G.add_node(n, pos=new_coord)  # 添加新节点到图中
            if G.degree(closest_node) == 1 :
                closest_node = list(G.neighbors(closest_node))[0]
            G.add_edge(n, closest_node)  # 连接到距离最小的节点
            explored_nodes.append(n)
            n=n+1
        else :
            explored_nodes.append(closest_node)
    return G,list(set(explored_nodes))
def add_new_nodes_with_condition(G, new_nodes):
    # 遍历新节点
    pos = nx.get_node_attributes(G, 'pos') 
    explored_nodes=[]
    n=len(pos)
    for new_coord in new_nodes:
        # 计算新节点与图中所有现有节点的最小距离
        min_distance = float('inf')
        closest_node = None
        
        for existing_node in G.nodes(data=True):
            existing_coord = existing_node[1]['pos']
            distance = euclidean_distance(new_coord, existing_coord)
            
            if distance < min_distance:
                min_distance = distance
                closest_node = existing_node[0]
        
        # # 如果最小距离大于10，添加新节点并连接
        # if min_distance > 20:
        #     G.add_node(n, pos=new_coord)  # 添加新节点到图中
        #     if G.degree(closest_node) == 1 :
        #         closest_node = list(G.neighbors(closest_node))[0]
        #     G.add_edge(n, closest_node)  # 连接到距离最小的节点
        #     explored_nodes.append(n)
        #     n=n+1
        # else :
        explored_nodes.append(closest_node)
    return G,list(set(explored_nodes))
def getFrontierNode(map,full_w,full_h,save_path,step):
    ex = np.zeros((full_w, full_h))
    kernel = np.ones((5, 5), dtype=np.uint8)
    _local_ob_map = map[0][0].cpu().numpy()
    local = cv2.dilate(_local_ob_map, kernel)

    show_ex = cv2.inRange(map[0][1].cpu().numpy(), 0.1, 1)

    
    free_map = cv2.morphologyEx(show_ex, cv2.MORPH_CLOSE, kernel)

    contour, _ = cv2.findContours(free_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contour) > 0:
            contour = max(contour, key=cv2.contourArea)
            cv2.drawContours(ex, contour, -1, 1, 1) #  会原地修改输入图像
    target_edge = ex-local
    target_edge[target_edge>0.8] = 1.0
    target_edge[target_edge!=1.0] = 0.0
    target_edge = target_edge *255
    frontiers=[]
    target_edge = cv2.convertScaleAbs(target_edge)
    target_edge = cv2.cvtColor(target_edge, cv2.COLOR_GRAY2BGR)
    from skimage import measure
    img_label, num = measure.label(target_edge, connectivity=2, return_num=True) # 输出二值图像中所有连通域
    props = measure.regionprops(img_label) # 输出连通域的属性，包括面积等
    for i in props:
        if i.area > 12 :
            cv2.circle(target_edge,(int(i.centroid[1]),int(i.centroid[0])),10,(0,255,0),2)
            frontiers.append((int(i.centroid[1]),int(i.centroid[0])))
    cv2.imwrite(save_path+"frontier"+str(step)+".png",target_edge)
    return frontiers
def generateVoronoi(map,history_node,full_w,full_h,step,save_path):
    save_map_path = os.path.join(save_path,"map/")
    if not os.path.exists(save_map_path):
        os.makedirs(save_map_path)
    frontier_nodes_2D = getFrontierNode(map,full_w,full_h,save_map_path,step)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    ex_map = map[0,1].cpu().numpy()
    # ex_map = cv2.dilate(ex_map, kernel)
    node = np.vstack(history_node)
    ex_map[np.vstack([node,node-1,node+1,node+2])[:,1],np.vstack([node,node-1,node+1,node+2])[:,0]]=1
    # ex_map = cv2.medianBlur(ex_map, 5)
    free_map = cv2.morphologyEx(ex_map, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    cv2.imwrite(save_map_path+"free_map1_"+str(step)+".png",free_map*255)
    # contours,_ = cv2.findContours(free_map,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    # for i in range(len(contours)):
    #     # `cv2.RETR_CCOMP` 会将外部和内部轮廓分开，所以可以填充内部孔洞
    #     cv2.drawContours(free_map, contours, i, 255, thickness=cv2.FILLED)
    # cv2.imwrite(save_map_path+"free_map2_"+str(step)+".png",free_map)
    obs_map = map[0,0].cpu().numpy()
    obstacle_map = map[0,4].cpu().numpy()
    # obs_map[np.where(obstacle_map > 0)[0],np.where(obstacle_map > 0)[1]]=1
    obs_map = cv2.medianBlur(obs_map, 3)
    obs_map = cv2.dilate(obs_map, kernel)
    free_map[0:2, 0:full_h] = 0.0
    free_map[full_w-2:full_w, 0:full_h-1] = 0.0
    free_map[0:full_w, 0:2] = 0.0
    free_map[0:full_w, full_h-2:full_h] = 0.0
    
    cv2.imwrite(save_map_path+"obs_map_"+str(step)+".png",obs_map*255)
    # path_map = get_largest_region(free_map/255 - obs_map)
    # path_map = get_largest_region(free_map - obs_map)
    path_map = free_map - obs_map
    cv2.imwrite(save_map_path+"path_map_"+str(step)+".png",path_map*255)
    boundary_map = binary_erosion(path_map, iterations=1).astype(np.uint8)
    boundary_map = path_map - boundary_map
    rows, cols = np.where(boundary_map == 1)
    boundaries = np.array(list(zip(cols, rows)))
    try:
        vor = Voronoi(boundaries) 
    except QhullError as e :
        return None,None,ex_map,obs_map,None,None
    vor=VorRemoveOut(vor,path_map,obs_map)
    if len(vor.ridge_vertices) < 1 :
        return None,None,ex_map,obs_map,None,None
    G = nx.Graph()
    for i, point in enumerate(vor.vertices):
        G.add_node(i, pos=(point[0],point[1]))
    G.add_edges_from(vor.ridge_vertices)
    all_edges_with_data = G.edges()
    pos = nx.get_node_attributes(G, 'pos') 
    remove_edge = []
    for edge in all_edges_with_data:
        if judgePassObstacle(pos[edge[0]],pos[edge[1]],obs_map) == False:
            remove_edge.append(edge)
    for edge in remove_edge :
        G.remove_edge(edge[0],edge[1])
    visual2Dgraph(G,step,0,save_map_path)
    # largest_cc = max(nx.connected_components(G), key=len)
    # subgraph = G.subgraph(largest_cc)
    subgraph = simplify_graph(G)
    visual2Dgraph(subgraph,step,1,save_map_path)
    if len(subgraph.nodes()) < 1 :
        return None,None,ex_map,obs_map,None,None
    start = time.time()
    leaf_values,leaf_position = getLeafValue(obstacle_map,subgraph)
    end = time.time()
    # print("find leaf time :",end-start)
    start = time.time()
    group = mergeGraphByObjects(subgraph,leaf_values,leaf_position)
    end = time.time()
    # print("merge time :",end-start)
    start = time.time()
    subgraph,leaf_values = CoarseGraph(subgraph,group,leaf_values)
    end = time.time()
    # print("CoarseGraph time :",end-start)
    start = time.time()
    subgraph = simplify_graph2(subgraph)
    end = time.time()
    # print("simplify_graph2 time :",end-start)
    visual2Dgraph(subgraph,step,2,save_map_path)
    
    pos = nx.get_node_attributes(subgraph, 'pos') 

    deleted=[]
    for node in subgraph.nodes() :
        # if node in leaf_values.keys() and euclidean_distance(pos[node],pos[list(subgraph.neighbors(node))[0]]) <= 10:
        # if node in leaf_values.keys():
        if node in leaf_values.keys() and leaf_values[node] != -1 and euclidean_distance(pos[node],pos[list(subgraph.neighbors(node))[0]]) <= 15:
            deleted.append(node)
    for d in deleted :
        subgraph.remove_node(d)
    
   
    subgraph = merge_closest_non_leaf_nodes(subgraph,10)
    visual2Dgraph(subgraph,step,3,save_map_path)
    if len(subgraph.nodes()) < 1 :
        return None,None,ex_map,obs_map,None,None
    
    subgraph=remove_isolated_nodes(subgraph)
    # largest_cc = max(nx.connected_components(subgraph), key=len)
    # subgraph = subgraph.subgraph(largest_cc)
    mapping = {old_label: new_label for new_label, old_label in enumerate(subgraph.nodes())}
    subgraph = nx.relabel_nodes(subgraph, mapping)
    visual2Dgraph(subgraph,step,4,save_map_path)
    subgraph,explored_nodes = add_new_nodes_with_condition(subgraph,history_node)
    subgraph,frontier_nodes = add_new_nodes_with_condition(subgraph,frontier_nodes_2D)
    pos = nx.get_node_attributes(subgraph, 'pos') 
    if len(list(pos.values())) < 1 :
        return None,None,ex_map,obs_map,None,None
    return subgraph,leaf_values,ex_map,obs_map,explored_nodes,frontier_nodes
def projectCurrentAgentLoc(position,G):
    # 遍历新节点
    pos = nx.get_node_attributes(G, 'pos') 
    # 计算新节点与图中所有现有节点的最小距离
    min_distance = float('inf')
    closest_node = None
    for existing_node in G.nodes(data=True):
        existing_coord = existing_node[1]['pos']
        distance = euclidean_distance(position, existing_coord)
        
        if distance < min_distance:
            min_distance = distance
            closest_node = existing_node[0]
    
    # 如果最小距离大于10，添加新节点并连接
    if min_distance > 10:
        G.add_node(len(pos), pos=position)  # 添加新节点到图中
        G.add_edge(len(pos), closest_node)  # 连接到距离最小的节点
        return G,len(pos)
    else :
        return G,closest_node
def direction_to_number(normal):
    directions = np.array([
    [1, 0],    # 0°
    [1, 1],    # 45°
    [0, 1],    # 90°
    [-1, 1],   # 135°
    [-1, 0],   # 180°
    [-1, -1],  # 225°
    [0, -1],   # 270°
    [1, -1]    # 315°
    ])
    mapping={
        0:0,
        1:1,
        2:2,
        3:3,
        4:4,
        5:-3,
        6:-2,
        7:-1
    }
    dot_products = np.dot(normal[None,:], directions.T)[0]/np.linalg.norm(normal)/np.linalg.norm(directions)
    direction = np.argmax(dot_products)
    return mapping[direction]
def find_another_view(voxels_center,graph,now_position):
    pos = nx.get_node_attributes(graph, 'pos')
    nodes = np.array(list(pos.keys()))
    poses = np.array(list(pos.values()))
    distance = np.linalg.norm(poses[:,None,:]-voxels_center[None,None,:],axis=-1)
    idx = np.where(distance < 60)[0]
    if len(idx) == 0 :
        node_select = np.argmin(distance)
        print("no other view")
    else :
        select_poses = poses[idx]
        iddx=np.argmax(np.max(np.linalg.norm(select_poses[:,None,:]-now_position[None,:,:],axis=-1),axis=-1))
        node_select = nodes[idx][iddx]
        # direction = direction_to_number(voxels_center - select_poses[iddx])
    return node_select
