import networkx as nx
from collections import Counter
import numpy as np
from slam.utils import extractVoxelIndex3
import time
import string
from networkx.exception import NetworkXNoPath
def euclidean_distance(pos, node1, node2):
    x1, y1 = pos[node1]
    x2, y2 = pos[node2]
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
def QueryNextSubGoal(goal_name,llm,objects,scene_graph,graph,leaf_nodes,agent_location,distance_threshold=30):
    pos = nx.get_node_attributes(graph, 'pos')
    objects_around={}
    for node,value in leaf_nodes.items() :
        if node in pos.keys():
            for i,obj in enumerate(objects) :
                coordinates = extractVoxelIndex3(obj['voxel_index'])
                min_distance=np.min(np.linalg.norm(coordinates[:,:2]-np.array([pos[node][1],pos[node][0]]),axis=-1))
                if min_distance < distance_threshold :
                    if node in objects_around.keys() :
                        objects_around[node].append(i)
                    else :
                        objects_around[node]=[i]
    USER=f"Goal: {goal_name}\n"
    USER = USER + f"Agent Location: ({str(agent_location[0])}, {str(agent_location[1])}).\n"
    for node,value in leaf_nodes.items() :
        if node in pos.keys():
            txt=f"-Description {str(node)}: "
            if value == 0 or value == 1 :
                break
            if value == -1 :
                txt += f"This node is directed to the unexplored space, located at ({str(graph.nodes()[node]['pos'][1])}, {str(graph.nodes()[node]['pos'][0])})."
            else :
                counter = Counter(objects[int(value-2)]['class_name'])
                most_common_element, count = counter.most_common(1)[0]
                txt =txt + f"This node is directed to a {most_common_element}, located at ({str(graph.nodes()[node]['pos'][1])}, {str(graph.nodes()[node]['pos'][0])})."
            if node in objects_around.keys() :
                txt += " The area contains "
                objs_around=objects_around[node]
                for obj_index in objs_around :
                    counter = Counter(objects[obj_index]['class_name'])
                    most_common_element, count = counter.most_common(1)[0]
                    txt = txt + f"a {most_common_element}"

                    # if obj_index in scene_graph.keys() :
                    #     txt += ", which is "
                    #     for obj2,relation in scene_graph[obj_index].items() :
                    #         counter = Counter(relation)
                    #         most_common_element, count = counter.most_common(1)[0]
                    #         if most_common_element == 'on':
                    #             most_common_element = 'under'
                    #         elif most_common_element == 'under':
                    #             most_common_element = 'on'
                    #         cc = Counter(objects[obj2]['class_name'])
                    #         mm, _ = cc.most_common(1)[0]
                    #         txt = txt + most_common_element + " a " + mm + ", "
                    #     txt = txt[:-2] + "; "
                    # else :
                    txt +="; "
            if txt[-1] =='.':
                txt+=f"\n"
            else :
                txt = f"{txt[:-2]}.\n"
            USER += txt
    print(f"=========> Query:\n{USER}")
    
    while True:
        try:
            answer, reply = llm.choose_frontier(USER)
            break
        except Exception as ex: # rate limit
            print(f"[ERROR] in LLM inference =====> {ex}, sleep for 20s...")
            time.sleep(20)
            continue
    print(f"=========> LLM output:\n{reply}")
    selection = np.array([int(graph.nodes()[answer]['pos'][0]),int(graph.nodes()[answer]['pos'][1])])
    return selection
def getObjectName(obj):
    counter = Counter(obj['class_name'])
    most_common_element, count = counter.most_common(1)[0]
    return most_common_element
def calculate_detailed_direction(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    dx = x2 - x1
    dy = y2 - y1

    if x1 == x2 and y1 == y2:
        return "Close neighbor"
    elif x1 == x2:
        if y2 > y1:
            return "Directly above"
        else:
            return "Directly below"
    elif y1 == y2:
        if x2 > x1:
            return "Directly to the right"
        else:
            return "Directly to the left"
    else:
        if dx > 0 and dy > 0:
            if abs(dx) > abs(dy):
                return "Up-right leaning right"
            else:
                return "Up-right leaning up"
        elif dx > 0 and dy < 0:
            if abs(dx) > abs(dy):
                return "Down-right leaning right"
            else:
                return "Down-right leaning down"
        elif dx < 0 and dy > 0:
            if abs(dx) > abs(dy):
                return "Up-left leaning left"
            else:
                return "Up-left leaning up"
        elif dx < 0 and dy < 0:
            if abs(dx) > abs(dy):
                return "Down-left leaning left"
            else:
                return "Down-left leaning down"
def landmark_prompt(goal_name,agent_node,obstacle_map,objects,bg_objects,scene_graph,voronoi_graph,node_rooms,explored_nodes,frontier_nodes,leaf_nodes):
    USER=f"Goal: {goal_name}\n"
    USER+=f"Agent Now : Node {agent_node}\n"
    objects_around=getObjectNearby(objects,bg_objects,voronoi_graph,obstacle_map)
    ###describe nodes ###
    pos = nx.get_node_attributes(voronoi_graph, 'pos')
    
    Node_descp=f"Node description : \n"
    for node in pos.keys():
        Node_descp+=f"Node {node} : \n"

        if node in frontier_nodes :
            Node_descp+=f"  Frontier node: True\n"
        else :
            Node_descp+=f"  Frontier node: False\n"
        ##########
        if node != agent_node :
            Node_descp+=f"  Location:\n"
            direction = calculate_detailed_direction(pos[agent_node],pos[node])
            try:
                path = nx.shortest_path(voronoi_graph, source=agent_node, target=node)
                distance= euclidean_distance(pos,agent_node,path[0])
                distance += euclidean_distance(pos,path[-1],node)
                for nidx in range(len(path)-1):
                    distance+=euclidean_distance(pos,path[nidx],path[nidx+1])
                Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
                Node_descp+=f"    Path from {agent_node} to {node}: {agent_node} ->"
                for n in path[1:-1] :
                    Node_descp+=f"{n} ->"
                Node_descp+=f"{node}\n"
                Node_descp+=f"    Distance from {agent_node} to {node}: {distance}\n"
            except NetworkXNoPath:
                Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
        if node in objects_around.keys():
            Node_descp+=f"  Surrounding objects:\n"
            objs_around=objects_around[node]
            for obj_message in objs_around :
                Node_descp+=f"    There is {obj_message['obj']} is in direction: {obj_message['direction']}, distance: {obj_message['distance']} relativate to this node.\n"
        if node_rooms[node] != "None" and node_rooms[node] is not None :
            Node_descp+=f"  Room: {node_rooms[node]}.\n"
        else :
            Node_descp+=f"  Room: Unknown.\n"
        if node in explored_nodes :
            Node_descp+=f"  Explored: True\n"
        else :
            Node_descp+=f"  Explored: False\n"
        Node_descp+=f"\n"
    candidate = list(pos.keys())
    candidate.remove(agent_node)
    nodes=f"The nodes candidate you select: {candidate}"
    USER = USER +Node_descp+nodes
    return USER,candidate
def landmark_BS(goal_name,agent_node,obstacle_map,objects,bg_objects,scene_graph,voronoi_graph,node_rooms,explored_nodes,frontier_nodes,leaf_nodes):
    USER=f"Goal: {goal_name}\n"
    USER+=f"Agent Now : Node {agent_node}\n"
    objects_around=getObjectNearby(objects,bg_objects,voronoi_graph,obstacle_map)
    ###describe nodes ###
    pos = nx.get_node_attributes(voronoi_graph, 'pos')
    Node_descp=f"Node description : \n"
    for node in pos.keys():
        Node_descp+=f"Node {node} : \n"
        if node in frontier_nodes :
            Node_descp+=f"  Frontier node: True\n"
            if node != agent_node :
                Node_descp+=f"  Location:\n"
                direction = calculate_detailed_direction(pos[agent_node],pos[node])
                try:
                    path = nx.shortest_path(voronoi_graph, source=agent_node, target=node)
                    distance= euclidean_distance(pos,agent_node,path[0])
                    distance += euclidean_distance(pos,path[-1],node)
                    for nidx in range(len(path)-1):
                        distance+=euclidean_distance(pos,path[nidx],path[nidx+1])
                    Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
                    Node_descp+=f"    Path from {agent_node} to {node}: {agent_node} ->"
                    for n in path[1:-1] :
                        Node_descp+=f"{n} ->"
                    Node_descp+=f"{node}\n"
                    Node_descp+=f"    Distance from {agent_node} to {node}: {distance}\n"
                except NetworkXNoPath:
                    Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
            if node in objects_around.keys():
                Node_descp+=f"  Surrounding objects:\n"
                objs_around=objects_around[node]
                for obj_message in objs_around :
                    Node_descp+=f"    There is {obj_message['obj']} is in direction: {obj_message['direction']}, distance: {obj_message['distance']} relativate to this node, "
                    if 'id' in obj_message.keys() and obj_message['id'] in scene_graph.keys():
                        for key,value in scene_graph[obj_message['id']].items():
                            if value is not None or value != "none":
                                Node_descp+=f"{value} the {getObjectName(objects[key])}, "
                        Node_descp = Node_descp[:-2]+".\n"
            if node_rooms[node] != "None" and node_rooms[node] is not None :
                Node_descp+=f"  Room: {node_rooms[node]}.\n"
            else :
                Node_descp+=f"  Room: Unknown.\n"
            if node in explored_nodes :
                Node_descp+=f"  Explored: True\n"
            else :
                Node_descp+=f"  Explored: False\n"
            Node_descp+=f"\n"
    nodes=f"The nodes candidate you select: {frontier_nodes}"
    USER = USER +Node_descp+nodes
    return USER,frontier_nodes
def landmark_CS(goal_name,agent_node,obstacle_map,objects,bg_objects,scene_graph,voronoi_graph,node_rooms,explored_nodes,frontier_nodes,relative,param):
    USER=f"Goal: {goal_name}\n"
    USER+=f"Agent Now : Node {agent_node}\n"
    objects_around=getObjectNearby(objects,bg_objects,voronoi_graph,obstacle_map)
    rooms = ["living room", "bedroom", "kitchen", "dining room", "bathroom", "hallway"]
    candidate = []
    if relative.lower() in rooms:
    ###describe nodes ###
        pos = nx.get_node_attributes(voronoi_graph, 'pos')
        Node_descp=f"Node description : \n"
        for node in pos.keys():
            if node_rooms[node] != "None" and node_rooms[node] is not None and node_rooms[node] == relative.lower() :
                candidate.append(node)
                Node_descp+=f"Node {node} : \n"
                if node in frontier_nodes :
                    Node_descp+=f"  Frontier node: True\n"
                    if node != agent_node :
                        Node_descp+=f"  Location:\n"
                        direction = calculate_detailed_direction(pos[agent_node],pos[node])
                        try:
                            path = nx.shortest_path(voronoi_graph, source=agent_node, target=node)
                            distance= euclidean_distance(pos,agent_node,path[0])
                            distance += euclidean_distance(pos,path[-1],node)
                            for nidx in range(len(path)-1):
                                distance+=euclidean_distance(pos,path[nidx],path[nidx+1])
                            Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
                            Node_descp+=f"    Path from {agent_node} to {node}: {agent_node} ->"
                            for n in path[1:-1] :
                                Node_descp+=f"{n} ->"
                            Node_descp+=f"{node}\n"
                            Node_descp+=f"    Distance from {agent_node} to {node}: {distance}\n"
                        except NetworkXNoPath:
                            Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
                    if node in objects_around.keys():
                        Node_descp+=f"  Surrounding objects:\n"
                        objs_around=objects_around[node]
                        for obj_message in objs_around :
                            Node_descp+=f"    There is {obj_message['obj']} is in direction: {obj_message['direction']}, distance: {obj_message['distance']} relativate to this node, "
                            if 'id' in obj_message.keys() and obj_message['id'] in scene_graph.keys():
                                for key,value in scene_graph[obj_message['id']].items():
                                    if value is not None or value != "none":
                                        Node_descp+=f"{value} the {getObjectName(objects[key])}, "
                                Node_descp = Node_descp[:-2]+".\n"
                    if node_rooms[node] != "None" and node_rooms[node] is not None :
                        Node_descp+=f"  Room: {node_rooms[node]}.\n"
                    else :
                        Node_descp+=f"  Room: Unknown.\n"
                    if node in explored_nodes :
                        Node_descp+=f"  Explored: True\n"
                    else :
                        Node_descp+=f"  Explored: False\n"
                    Node_descp+=f"\n"
    else:
        Node_descp=f"Node description : \n"
        relative_objs = []
        for obj in objects:
            if relative in obj['class_name']:
                relative_objs.append(obj)
        pos = nx.get_node_attributes(voronoi_graph, 'pos')
        array = np.array(list(pos.values()))
        array = array.reshape(-1, 2)
        for obj in relative_objs:
            indices_3=np.array(np.unravel_index(obj['voxel_index'], (param[0],param[1],param[2]),order='C')).T
            # 跳过空体素的对象
            if indices_3.size == 0 or len(indices_3) == 0:
                continue
            # 确保 array 也不为空
            if array.size == 0:
                continue
            distance=np.min(np.linalg.norm(array[:,None,:]-indices_3[None,:,:2],axis=-1),axis=-1)
            iddx = np.where(distance <= 30)[0]
            nodes = np.array(list(pos.keys()))[iddx].tolist()
            candidate.extend(nodes)
        for node in candidate:
            Node_descp+=f"Node {node} : \n"
            if node in frontier_nodes :
                Node_descp+=f"  Frontier node: True\n"
                if node != agent_node :
                    Node_descp+=f"  Location:\n"
                    direction = calculate_detailed_direction(pos[agent_node],pos[node])
                    try:
                        path = nx.shortest_path(voronoi_graph, source=agent_node, target=node)
                        distance= euclidean_distance(pos,agent_node,path[0])
                        distance += euclidean_distance(pos,path[-1],node)
                        for nidx in range(len(path)-1):
                            distance+=euclidean_distance(pos,path[nidx],path[nidx+1])
                        Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
                        Node_descp+=f"    Path from {agent_node} to {node}: {agent_node} ->"
                        for n in path[1:-1] :
                            Node_descp+=f"{n} ->"
                        Node_descp+=f"{node}\n"
                        Node_descp+=f"    Distance from {agent_node} to {node}: {distance}\n"
                    except NetworkXNoPath:
                        Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
                if node in objects_around.keys():
                    Node_descp+=f"  Surrounding objects:\n"
                    objs_around=objects_around[node]
                    for obj_message in objs_around :
                        Node_descp+=f"    There is {obj_message['obj']} is in direction: {obj_message['direction']}, distance: {obj_message['distance']} relativate to this node, "
                        if 'id' in obj_message.keys() and obj_message['id'] in scene_graph.keys():
                            for key,value in scene_graph[obj_message['id']].items():
                                if value is not None or value != "none":
                                    Node_descp+=f"{value} the {getObjectName(objects[key])}, "
                            Node_descp = Node_descp[:-2]+".\n"
                if node_rooms[node] != "None" and node_rooms[node] is not None :
                    Node_descp+=f"  Room: {node_rooms[node]}.\n"
                else :
                    Node_descp+=f"  Room: Unknown.\n"
                if node in explored_nodes :
                    Node_descp+=f"  Explored: True\n"
                else :
                    Node_descp+=f"  Explored: False\n"
                Node_descp+=f"\n"
    nodes=f"The nodes candidate you select: {candidate}"
    USER = USER +Node_descp+nodes
    return USER,candidate
def landmark_OT(goal_name,agent_node,obstacle_map,objects,bg_objects,scene_graph,voronoi_graph,node_rooms,explored_nodes,frontier_nodes,target,param):
    USER=f"Goal: {goal_name}\n"
    USER+=f"Agent Now : Node {agent_node}\n"
    objects_around=getObjectNearby(objects,bg_objects,voronoi_graph,obstacle_map)
    candidate = []
    target_obj = None
    for obj in objects:
        if target in obj['class_name']:
            target_obj = obj
            break

    # 如果没有找到目标物体，返回空结果
    if target_obj is None:
        return "", []

    # 检查目标物体是否有有效的体素索引
    if target_obj.get('voxel_index') is None or len(target_obj['voxel_index']) == 0:
        return "", []

    pos = nx.get_node_attributes(voronoi_graph, 'pos')
    array = np.array(list(pos.values()))
    array = array.reshape(-1, 2)
    indices_3=np.array(np.unravel_index(target_obj['voxel_index'], (param[0],param[1],param[2]),order='C')).T
    distance=np.min(np.linalg.norm(array[:,None,:]-indices_3[None,:,:2],axis=-1),axis=-1)
    iddx = np.where(distance <= 30)[0]
    nodes = np.array(list(pos.keys()))[iddx].tolist()
    candidate.extend(nodes)
    Node_descp=f"Node description : \n"
    for node in candidate:
        Node_descp+=f"Node {node} : \n"
        if node in frontier_nodes :
            Node_descp+=f"  Frontier node: True\n"
            if node != agent_node :
                Node_descp+=f"  Location:\n"
                direction = calculate_detailed_direction(pos[agent_node],pos[node])
                try:
                    path = nx.shortest_path(voronoi_graph, source=agent_node, target=node)
                    distance= euclidean_distance(pos,agent_node,path[0])
                    distance += euclidean_distance(pos,path[-1],node)
                    for nidx in range(len(path)-1):
                        distance+=euclidean_distance(pos,path[nidx],path[nidx+1])
                    Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
                    Node_descp+=f"    Path from {agent_node} to {node}: {agent_node} ->"
                    for n in path[1:-1] :
                        Node_descp+=f"{n} ->"
                    Node_descp+=f"{node}\n"
                    Node_descp+=f"    Distance from {agent_node} to {node}: {distance}\n"
                except NetworkXNoPath:
                    Node_descp+=f"    Direction: {direction} relative to {agent_node}\n"
            if node in objects_around.keys():
                Node_descp+=f"  Surrounding objects:\n"
                objs_around=objects_around[node]
                for obj_message in objs_around :
                    Node_descp+=f"    There is {obj_message['obj']} is in direction: {obj_message['direction']}, distance: {obj_message['distance']} relativate to this node, "
                    if 'id' in obj_message.keys() and obj_message['id'] in scene_graph.keys():
                        for key,value in scene_graph[obj_message['id']].items():
                            if value is not None or value != "none":
                                Node_descp+=f"{value} the {getObjectName(objects[key])}, "
                        Node_descp = Node_descp[:-2]+".\n"
            if node_rooms[node] != "None" and node_rooms[node] is not None :
                Node_descp+=f"  Room: {node_rooms[node]}.\n"
            else :
                Node_descp+=f"  Room: Unknown.\n"
            if node in explored_nodes :
                Node_descp+=f"  Explored: True\n"
            else :
                Node_descp+=f"  Explored: False\n"
            Node_descp+=f"\n"
    nodes=f"The nodes candidate you select: {candidate}"
    USER = USER +Node_descp+nodes
    return USER,candidate
def state_prompt(goal_name,obstacle_map,objects,bg_objects,voronoi_graph,node_rooms,frontier_nodes,state,target_level=None):
    USER=f"Goal: {goal_name}\n"
    objects_around=getObjectNearby(objects,bg_objects,voronoi_graph,obstacle_map)
    ###describe nodes ###
    pos = nx.get_node_attributes(voronoi_graph, 'pos')
    USER += f"Current State: {state}\n"
    Node_descp=f"Node description : \n"
    for node in pos.keys():
        Node_descp+=f"Node {node} : \n"
        if node in frontier_nodes :
            Node_descp+=f"  Frontier node: True\n"
        else :
            Node_descp+=f"  Frontier node: False\n"
        if node in objects_around.keys():
            Node_descp+=f"  Surrounding objects:\n"
            objs_around=objects_around[node]
            for obj_message in objs_around :
                Node_descp+=f"    There is {obj_message['obj']} is in direction: {obj_message['direction']}, distance: {obj_message['distance']} relativate to this node.\n"
        if node_rooms[node] != "None" and node_rooms[node] is not None :
            Node_descp+=f"  Room: {node_rooms[node]}.\n"
        else :
            Node_descp+=f"  Room: Unknown.\n"
        Node_descp+=f"\n"
    USER = USER +Node_descp
    if target_level is not None :
        USER += f"Confirmation Level:{target_level}."
    return USER
def getObjectNearby(objects,bg_objects,graph,obstacle_map,distance_threshold=40):
    pos = nx.get_node_attributes(graph, 'pos')
    objects_around={}
    position=np.array(list(pos.values()))
    pos_id = list(pos.keys())
    edge_nodes=[]
    # 打印所有边及其属性
    obstacle_map = (obstacle_map.cpu().numpy() * 255).astype(np.uint8)
    all_edges_with_data = graph.edges(data=False)
    import cv2
    color_image1 = cv2.cvtColor(obstacle_map, cv2.COLOR_GRAY2BGR)
    node = []
    points_2d=[]
    relation=[]
    for i,pose in pos.items():
            points_2d.append([pose[0],pose[1]])
            point_int = (int(pose[0]),int(pose[1]))
            cv2.circle(color_image1, point_int, 1, (0, 0, 255), -1)  # red
            node.append(i)
    
    for edge in all_edges_with_data:
    
        relation.append([node.index(edge[0]),node.index(edge[1])])

    edge_nodes = list(set(edge_nodes))
    for edge in relation:
            pt1 = (int(points_2d[edge[0]][0]),int(points_2d[edge[0]][1]))  # 转换为整数
            pt2 = (int(points_2d[edge[1]][0]),int(points_2d[edge[1]][1]))   # 转换为整数
            cv2.line(color_image1, pt1, pt2, (255, 0, 0), 1)  # blue色线
    for j,obj in enumerate(objects):
        obj_name = getObjectName(obj)

        coordinates = extractVoxelIndex3(obj['voxel_index'])
        # 跳过空坐标的对象
        if coordinates.size == 0 or len(coordinates) == 0:
            continue
        # coordinates[:,0],coordinates[:,1] = coordinates[:,1],coordinates[:,0]
        center = np.mean(coordinates[:,:2],axis=0)
        if np.any(np.isnan(center)):
            continue
        point_int = (int(center[0]),int(center[1]))  # 转换为整数
        cv2.circle(color_image1, point_int, 1, (0, 0, 255), -1)  # red
        min_distance=np.min(np.linalg.norm(coordinates[None,:,:2]-position[:,None,:],axis=-1),axis=-1)
        index = np.where(min_distance < distance_threshold)[0]
        select_id = [pos_id[i] for i in index]
        for i,id in zip(index,select_id) :
            direction = calculate_detailed_direction(position[i],center)
            distance = min_distance[i]
            if id in objects_around.keys():
                objects_around[id].append({'obj':obj_name,'id':j,'direction':direction,'distance':distance})
            else :
                objects_around[id]=[{'obj':obj_name,'id':j,'direction':direction,'distance':distance}]
    cv2.imwrite('color_image_with_circle_and_line.png', color_image1)
    # import matplotlib.pyplot as plt
    # plt.imshow(cv2.cvtColor(color_image1, cv2.COLOR_BGR2RGB))
    # plt.axis('off')
    # # 保存图像到文件 
    # plt.show()
    for name,bg in bg_objects.items() :
        if (name == "wall" or name == "wall-wood") and bg is not None :
            coordinates = extractVoxelIndex3(bg['voxel_index'])
            # 跳过空坐标
            if coordinates.size == 0 or len(coordinates) == 0:
                continue
            min_id=np.argmin(np.linalg.norm(coordinates[None,:,:2]-position[:,None,:],axis=-1),axis=-1)
            min_distance=np.min(np.linalg.norm(coordinates[None,:,:2]-position[:,None,:],axis=-1),axis=-1)
            index = np.where(min_distance < distance_threshold)[0]
            select_id = [pos_id[i] for i in index]
            for i,id in zip(index,select_id) :
                direction = calculate_detailed_direction(position[i],coordinates[min_id[i],:2])
                distance = min_distance[i]
                if id in objects_around.keys():
                    objects_around[id].append({'obj':'wall','direction':direction,'distance':distance})
                else :
                    objects_around[id]=[{'obj':'wall','direction':direction,'distance':distance}]
    return objects_around
def QueryByGraph(goal_name,agent_node,obstacle_map,centers,objects,bg_objects,scene_graph,voronoi_graph,node_rooms,explored_nodes,leaf_nodes):
    USER=f"Goal: {goal_name}\n"
    USER+=f"Agent Now : Node {agent_node}\n"
    uppercase_letters = list(string.ascii_uppercase)
    # objects_descp=f"Green Holl Square Area description: \n"
    # area_objects={}
    # object_area={}
    # for i,center in enumerate(centers) :
    #     obj_id = int(obstacle_map[int(center[0]),int(center[1])]-2)
    #     obj_desc = f"\t{uppercase_letters[i]} contains objects: [{getObjectName(objects[obj_id])}"
    #     area_objects[i]=[obj_id]
    #     object_area[obj_id] = i
    #     if obj_id in scene_graph.keys() :
    #         surd_objs = scene_graph[obj_id]
    #         for obj_id in surd_objs :
    #             obj_desc += f", {getObjectName(objects[obj_id])}"
    #             area_objects[i].append(obj_id)
    #             object_area[obj_id] = i
    #     obj_desc +="]."
    #     objects_descp+=f"{obj_desc}\n"
    ###describe nodes ###
    objects_around=getObjectNearby(objects,bg_objects,voronoi_graph,obstacle_map)
    
    # objects_around={}
    # for node in pos.keys() :
    #     if node in pos.keys():
    #         for i,obj in enumerate(objects) :
    #             coordinates = extractVoxelIndex3(obj['voxel_index'])
    #             min_distance=np.min(np.linalg.norm(coordinates[:,:2]-np.array([pos[node][1],pos[node][0]]),axis=-1))
    #             if min_distance < distance_threshold :
    #                 if node in objects_around.keys() :
    #                     objects_around[node].append(i)
    #                 else :
    #                     objects_around[node]=[i]
    pos = nx.get_node_attributes(voronoi_graph, 'pos')
    Node_descp=f"Node description : \n"
    for node in pos.keys():
        Node_descp+=f"Node {node} : \n"
        if node in leaf_nodes.keys() :
            if leaf_nodes[node] == -1 :
                # Node_descp+=f" a frontier node that can navigate to an unexplored space.\t"
                Node_descp+=f"  Frontier node: True\n"
            else :
                Node_descp+=f"  Frontier node: False\n"
        Node_descp+=f"  Frontier node: False\n"
        if node in objects_around.keys():
            # Node_descp += " Near this node, there are "
            Node_descp+=f"  Surrounding objects:\n"
            objs_around=objects_around[node]
            for obj_message in objs_around :
                Node_descp+=f"    There is {obj_message['obj']} is in direction: {obj_message['direction']}, distance: {obj_message['distance']} relativate to this node.\n"
                # if obj_id in object_area.keys() :
                #     area_id = object_area[obj_id]
                #     Node_descp+=f"\t\tobject {obj_name} in area {uppercase_letters[area_id]};\n"
                # else :
                #     Node_descp+=f"\t\tobject {obj_name};\n"
        if node_rooms[node] != "None" and node_rooms[node] is not None :
            Node_descp+=f"  Room: {node_rooms[node]}.\n"
        else :
            Node_descp+=f"  Room: Unknown.\n"
        if node in explored_nodes :
            Node_descp+=f"  Explored: True\n"
        else :
            Node_descp+=f"  Explored: False\n"
        Node_descp+=f"\n"
    candidate = list(pos.keys())
    candidate.remove(agent_node)
    nodes=f"The nodes candidate you select: {candidate}"
    USER = USER +Node_descp+nodes
    return USER

