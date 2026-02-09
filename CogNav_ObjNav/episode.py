import os
import numpy as np
import skimage.morphology
import torch
import torch.nn as nn
from collections import deque, defaultdict
from envs import make_vec_envs
import matplotlib.pyplot as plt
from scenegraph.cogvlm2 import judgeRoom
import re
import cv2
import networkx as nx
from scipy.spatial import  Voronoi,voronoi_plot_2d
from PIL import Image
from model.Semantic_Mapping_SG import Semantic_Mapping_SG
from utils.constants import category_to_id, categories_mapping_to_openseed,hm3d_category
from scenegraph.querypath import state_prompt,landmark_BS,landmark_CS,landmark_OT
import time
from collections import Counter
import torch.nn.functional as F
# from scenegraph.llm import LLM
from scenegraph.qwen3 import LLM
import random
from scenegraph.prepare import prepareForQueryNode,projectHistoryToGraph
from utils.common import _object_query_constructor, save_ori_obs_rgbd
from utils.voronoi import generateVoronoi,projectCurrentAgentLoc,find_another_view
import json
import logging
import sys
class Episode:
    def __init__(self, args):
        self.args = args
        self.device = args.device
        self.dump_location = args.dump_location
        self.exp_name = args.exp_name
        
        # Logging and loss variables
        self.num_scenes = args.num_processes  # how many training processes to use
        self.num_episodes = int(args.num_eval_episodes)  # number of test episodes per scene

        self.map_size_cm = args.map_size_cm
        self.global_downscaling = args.global_downscaling
        self.load = args.load
        
        self.visualize = args.visualize
        self.print_images = args.print_images

        self.num_training_frames = args.num_training_frames
        self.num_local_steps = args.num_local_steps
        self.num_global_steps = args.num_global_steps

        self.eval = args.eval  # 这里有点奇怪，只有eval?
        self.map_resolution = args.map_resolution
        self.num_sem_categories = args.num_sem_categories
        self.max_height = int(200 / self.map_resolution)
        self.min_height = int(-40 / self.map_resolution)
        self.task_config = args.task_config
        self.not_target=[]
        self.log_interval = args.log_interval
        self.split = args.split
        self.path_object_norm_inv_perplexity = args.path_object_norm_inv_perplexity
        self.object_norm_inv_perplexity = torch.tensor(
            np.load(self.path_object_norm_inv_perplexity)).to(self.device)

        self.sem_map_module = Semantic_Mapping_SG(self.args).to(self.device)
        self.sem_map_module.eval()
        self.llm=None
        self.global_position=None
        self.global_position_history=[]
        self.move_threshold=[]
        self.distance = []
        self.room_message={}
        self.graph=None
        # self.embedder = self.configure_lm('RoBERTa-large')
        # self.ff_net = self.load_feed_forward_net()
        self.last_done = 0
        self.skip_times = args.skip_times
        self.start_found = -1
        self.get_hm3d_semantic_map_index('data/matterport_category_mappings.tsv')

        self.log_dir = "{}/{}/{}/".format(args.dump_location, args.scenes[0],args.skip_times)
        self.dump_dir = "{}/{}/{}/".format(args.dump_location, args.scenes[0],args.skip_times)

        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        if not os.path.exists(self.dump_dir):
            os.makedirs(self.dump_dir)

        logging.basicConfig(filename=self.log_dir + 'train.log', level=logging.INFO)
        print("Dumping at {}".format(self.log_dir))
        print(args)
        logging.info(args)


    def start(self):
        init_data_var_dict, init_data_map_dict, finished, wait_env, path,save_i = self.init_episode()

        start = time.time()
        for step in range(self.num_training_frames // self.num_scenes + 1):
            if finished.sum() == self.num_scenes:
                sys.exit()
            init_data_var_dict, init_data_map_dict, finished, wait_env,save_i =\
                self.forward(step, init_data_var_dict, init_data_map_dict, finished, wait_env,path,save_i)
            if self.skip_times <=0 :
                log = self.logging(step, init_data_var_dict, start)
        self.logging_final(init_data_var_dict, init_data_map_dict, log)

    def forward(self, step, init_data_var_dict, init_data_map_dict, finished, wait_env,path,save_i):
        
        def init_map_and_pose_for_env(e):
            full_map[e].fill_(0.)
            full_pose[e].fill_(0.)
            # full_ob_map = np.zeros((self.num_scenes, full_w, full_h))
            # full_ex_map = np.zeros((self.num_scenes, full_w, full_h))
            # target_edge_map = np.zeros((self.num_scenes, full_w, full_h))
            # target_point_map = np.zeros((self.num_scenes, full_w, full_h))

            step_masks[e] = 0
            stair_flag[e] = 0
            clear_flag[e] = 0

            full_pose[e, :2] = self.map_size_cm / 100.0 / 2.0
            locs = full_pose[e].cpu().numpy()
            planner_pose_inputs[e, :3] = locs
            r, c = locs[1], locs[0]
            loc_r, loc_c = [int(r * 100.0 / self.map_resolution),
                            int(c * 100.0 / self.map_resolution)]

            full_map[e, 2:4, loc_r - 1:loc_r + 2, loc_c - 1:loc_c + 2] = 1.0

            # lmb[e] = self.get_local_map_boundaries((loc_r, loc_c),
            #                                        (local_w, local_h),
            #                                        (full_w, full_h))
            lmb[e] = [0,full_w,0,full_h]
            planner_pose_inputs[e, 3:] = lmb[e]
            origins[e] = [lmb[e][2] * self.map_resolution / 100.0,
                          lmb[e][0] * self.map_resolution / 100.0, 0.]
            self.sem_map_module.reset()
            self.global_position=None
            self.global_position_history=[]
            self.last_done = step 
            self.graph = None
            target_reach = True
            self.start_found = -1
            gpt_target_result=[]
            needed_rotation = []
            self.distance = {}
            # local_map[e] = full_map[e, :, lmb[e, 0]:lmb[e, 1], lmb[e, 2]:lmb[e, 3]]
            # local_pose[e] = full_pose[e] - torch.from_numpy(origins[e]).to(self.device).float()
            return target_reach,gpt_target_result,needed_rotation
        # g_step = (step // self.num_local_steps) % self.num_global_steps
        obs, infos, done = init_data_var_dict['obs'], init_data_var_dict['infos'], init_data_var_dict['done']
        episode_success, episode_spl, episode_dist =\
            init_data_var_dict['episode_success'], init_data_var_dict['episode_spl'], init_data_var_dict['episode_dist']
        full_pose, eve_angle = init_data_var_dict['full_pose'], init_data_var_dict['eve_angle']
        state = init_data_var_dict['state']
        gpt_target_result,target_reach=init_data_var_dict['gpt_target_result'],init_data_var_dict['target_reach']
        origins, planner_pose_inputs =\
            init_data_var_dict['origins'], init_data_var_dict['planner_pose_inputs']
        (full_w, full_h), stair_flag = init_data_var_dict['full_wh'], init_data_var_dict['stair_flag']
        step_masks, clear_flag = init_data_var_dict['step_masks'], init_data_var_dict['clear_flag']
        g_masks, g_process_rewards, g_sum_rewards = init_data_var_dict['g_masks'], init_data_var_dict['g_process_rewards'], init_data_var_dict['g_sum_rewards']
        full_map,full_map_new, lmb = init_data_map_dict['full_map'],init_data_map_dict['full_map_new'], init_data_map_dict['lmb']
        full_ob_map, full_ex_map = init_data_map_dict['full_ob_map'], init_data_map_dict['full_ex_map']
        kernel, tv_kernel = init_data_map_dict['kernel'], init_data_map_dict['tv_kernel']
        target_edge_map, target_point_map = init_data_map_dict['target_edge_map'], init_data_map_dict['target_point_map']
        frontier_score_list = init_data_map_dict['frontier_score_list']
        spl_per_category, success_per_category = init_data_map_dict['spl_per_category'], init_data_map_dict['success_per_category']
        global_goals = init_data_map_dict['global_goals']
        # l_step = step % self.num_local_steps

        # Reinitialize variables when episode ends
        l_masks = torch.FloatTensor([0 if x else 1 for x in done]).to(self.device)
        g_masks *= l_masks
        save_path = path + str(save_i)+"/"
        # @brief: save opt !!! save original observation
        # save_ori_obs_rgbd(infos[0]['ori_obs'], step)
        
        for e, x in enumerate(done):
            if x or self.skip_times > 0:
                spl = infos[e]['spl']
                success = infos[e]['success']
                dist = infos[e]['distance_to_goal']
                spl_per_category[infos[e]['goal_name']].append(spl)
                success_per_category[infos[e]['goal_name']].append(success)
                print("scene",self.args.scenes[0],"episode:",self.args.skip_times,"success:",success,"dist:",dist,"spl:",spl)
                save_i += 1
                save_path = path + str(save_i) + "/"
                if self.eval and self.skip_times <= 0 :
                    episode_success[e].append(success)
                    episode_spl[e].append(spl)
                    episode_dist[e].append(dist)
                    sys.exit()
                    if len(episode_success[e]) == self.num_episodes:
                        finished[e] = 1
                    # import pdb
                    # pdb.set_trace()
                wait_env[e] = 1.
                target_reach,gpt_target_result,needed_rotation = init_map_and_pose_for_env(e)
                
                if self.skip_times > 0 :
                    self.skip_times = self.skip_times -1
        # Semantic Mapping Module
        poses = torch.from_numpy(np.asarray(
            [infos[env_idx]['sensor_pose'] for env_idx in range(self.num_scenes)]
        )).float().to(self.device)

        eve_angle = np.asarray(
            [infos[env_idx]['eve_angle'] for env_idx in range(self.num_scenes)]
        )

        # obs: Tensor(1, m, 120, 160)
        # poses: Tensor(1, 3)  这一次action之后pose的变化
        # local_map: Tensor(1, 20, 480, 480)
        # local_pose: Tensor(1, 3)  # action之前的pose
        # eve_angle: Tensor(1,)
        # self.llm=LLM(infos[e]['goal_name'],'deterministic')
        self.llm = LLM()
        full_map, local_map_stair, full_pose= \
            self.sem_map_module(obs, infos[0]['ori_obs'], poses, full_map, full_pose, eve_angle, origins, idx=step-self.last_done,save_path=save_path)
        
        locs = full_pose.cpu().numpy()
        planner_pose_inputs[:, :3] = locs
        full_map[:, 2, :, :].fill_(0.) # Resetting current location channel
        for e in range(self.num_scenes):
            r, c = locs[e, 1], locs[e, 0]
            loc_r, loc_c = [int(r * 100.0 / self.map_resolution),
                            int(c * 100.0 / self.map_resolution)]
            full_map[e, 2:4, loc_r - 2:loc_r + 3, loc_c - 2:loc_c + 3] = 1.
            # ------------------------------------------------------------------
            if self.eval:
                if loc_r > full_w: loc_r = full_w - 1
                if loc_c > full_h: loc_c = full_h - 1
        pos_map = np.array([loc_c,loc_r])
        if self.global_position is not None:
            shape = self.global_position.shape
            if len(shape) == 1:
                self.move_threshold.append(np.linalg.norm(pos_map-self.global_position))
            else :
                self.move_threshold.append(np.linalg.norm(pos_map-np.mean(self.global_position,axis=1)))
        goal_name =infos[e]['goal_name']
        found_obj_id=-1
        found_obj_rate = 0
        found_goal = [0 for _ in range(self.num_scenes)]
        goal_maps = [np.zeros((full_w, full_h)) for _ in range(self.num_scenes)]
        for i,obj in enumerate(self.sem_map_module.objects) :
            if len(obj['class_id']) != 0 :
                # print(max(set(obj['class_name']), key=obj['class_name'].count))
                if i+2 not in self.not_target:
                    if set(obj['class_id']) & set(categories_mapping_to_openseed[infos[e]['goal_cat_id']]) :
                        counter = Counter(obj['class_id'])
                        most_common_count = counter.most_common(1)[0][1]
                        if most_common_count/len(obj['class_id']) > found_obj_rate:
                            found_obj_id = i+2
                            found_obj_rate = most_common_count/len(obj['class_id'])
                            if self.start_found == -1 :
                                self.start_found = step-self.last_done
        if (step-self.last_done) % 10 ==0 and (step-self.last_done) != 0 :
            start=time.time()
            while True:
                try:
                    agent_room = self.llm.get_room(os.path.join(save_path, 'image',str(step-self.last_done)+'.jpg'))
                    self.room_message[tuple(pos_map)]=agent_room
                    break
                except Exception as ex: # rate limit
                    print(f"[ERROR] in LLM inference =====> {ex}, sleep for 20s...")
                    time.sleep(20)
                    continue
            end=time.time()
            print(agent_room,"query time:",end-start)
        self.global_position_history.append(pos_map)
        ### change this by action corresponds to move 
        if len(self.move_threshold) >= 20 :
            cha = max(self.move_threshold[-20:])-min(self.move_threshold[-20:])
            if cha < 15 or len(self.move_threshold) > 80 :
                print("replan by ourself")
                infos[0]['replan']=True
        if infos[0]['replan'] or self.graph is None or (found_obj_id !=-1 and target_reach and self.start_found !=-1):
            ################################
            target_reach = True
            self.move_threshold=[]
            for e in range(self.num_scenes):
                step_masks[e] += 1
                if wait_env[e] == 1: # New episode
                    wait_env[e] = 0.
                locs = full_pose[e].cpu().numpy()
                r, c = locs[1], locs[0]
                loc_r, loc_c = [int(r * 100.0 / self.map_resolution),
                                int(c * 100.0 / self.map_resolution)]
                lmb[e] = [0,full_w,0,full_h]
                planner_pose_inputs[e, 3:] = lmb[e]

                if infos[e]['clear_flag']:
                    clear_flag[e] = 1

                if clear_flag[e]:
                    full_map[e].fill_(0.)
                    clear_flag[e] = 0
            objects,bg_objects,scene_graph,obstacle_map,wall = self.sem_map_module.updateSceneGraph(save_path=save_path,idx=step-self.last_done,exp_map=full_map[0,1])
            found_obj_id=-1
            found_obj_rate = 0
            for i,obj in enumerate(self.sem_map_module.objects) :
                if i+2 not in self.not_target:
                    if len(obj['class_id']) != 0 :
                        if set(obj['class_id']) & set(categories_mapping_to_openseed[infos[e]['goal_cat_id']]) :
                            counter = Counter(obj['class_id'])
                            most_common_count = counter.most_common(1)[0][1]
                            if most_common_count/len(obj['class_id']) > found_obj_rate:
                                found_obj_id = i+2
                                found_obj_rate = most_common_count/len(obj['class_id'])             
            full_map[0,4]=obstacle_map
            self.graph,leaf_values,full_ex_map[0],full_ob_map[0],explored_nodes,frontier_nodes = generateVoronoi(full_map,self.global_position_history,full_w=full_w,full_h=full_h,step=step-self.last_done,save_path=save_path)
            ##### state transition and landmark selection
            if self.graph is None:
                node_rooms = None
            else:
                node_rooms = projectHistoryToGraph(self.graph,pos_map,self.room_message,50)
            if self.graph is not None :
                answer = None
                if found_obj_id !=-1:
                    if len(gpt_target_result) < 5 and gpt_target_result.count(1) < 2:
                        select_img = self.sem_map_module.find_Maximum_View(found_obj_id-2,self.start_found,step-self.last_done)
                        print("select_img:",select_img,"start_found:",self.start_found,"end:",step-self.last_done)
                        while True:
                            try:
                                answer = self.llm.query_target_obj(os.path.join(save_path, 'image',str(select_img)+'.jpg'),goal_name)
                                break
                            except Exception as ex: # rate limit
                                print(f"[ERROR] in LLM inference =====> {ex}, sleep for 20s...")
                                time.sleep(20)
                                continue
                user = state_prompt(goal_name,obstacle_map,objects,bg_objects,self.graph,node_rooms,frontier_nodes,state,target_level = answer)
                # print(user)
                state_new,relative = self.llm.query_state_transition(user)
                print("current state:",state_new,"relative object or room:",relative)
                if found_obj_id != -1:
                    if state_new == "Broad Search" or state_new == "Contextual Search":
                        self.not_target.append(found_obj_id)
                        found_obj_id = -1
                state = state_new
            else :
                state = "Broad Search"
            if self.graph is not None :
                pos = nx.get_node_attributes(self.graph, 'pos')
            if state == "Broad Search":
                if self.graph is not None :
                    self.graph,agent_location_node = projectCurrentAgentLoc(tuple(pos_map),self.graph)
                    node_rooms = projectHistoryToGraph(self.graph,pos_map,self.room_message,50)
                    user,node_candidate = landmark_BS(goal_name,agent_location_node,obstacle_map,objects,bg_objects,scene_graph,self.graph,node_rooms,explored_nodes,frontier_nodes,leaf_values)
                    start=time.time()
                    while True:
                        try:
                            # response,node = self.llm.query_node_graph_o1("map/graph_"+str(step)+".jpg",user)
                            response,node = self.llm.query_node_txt(user)
                            break
                        except Exception as ex: # rate limit
                            print(f"[ERROR] in LLM inference =====> {ex}, sleep for 20s...")
                            time.sleep(20)
                            continue
                    found_elements=[node]
                    end=time.time()
                    print(response,f"time:{end-start}")
                    if len(found_elements) != 0 :
                        if found_elements[0] in pos.keys() and np.linalg.norm(np.array(pos[found_elements[0]])-pos_map) < 20 :
                            ss = set(node_candidate)-set(explored_nodes)
                            ss = list(ss)
                            if len(ss) > 0 :
                                self.global_position=np.array([int(pos[ss[0]][0]),int(pos[ss[0]][1])])
                            elif len(node_candidate) > 0 :
                                self.global_position=np.array([int(pos[node_candidate[0]][0]),int(pos[node_candidate[0]][1])])
                            else :
                                array = np.array(list(pos.values()))
                                array = array.reshape(-1, 2)
                                iddx=np.argmax(np.max(np.linalg.norm(array[:,None,:]-pos_map[None,None,:],axis=-1),axis=-1))
                                self.global_position=array[iddx].astype(int)
                        # print("found_elements",found_elements[0],"find node time:",end-start)
                        else :
                            self.global_position=np.array([int(pos[found_elements[0]][0]),int(pos[found_elements[0]][1])])
                    else :
                        array = np.array(list(pos.values()))
                        array = array.reshape(-1, 2)
                        iddx=np.argmax(np.max(np.linalg.norm(array[:,None,:]-pos_map[None,None,:],axis=-1),axis=-1))
                        self.global_position=array[iddx].astype(int)
                    self.global_position_history.append(self.global_position)
                
                else:
                    actions = torch.randn(self.num_scenes, 2)*6
                    cpu_actions = nn.Sigmoid()(actions).numpy()
                    global_goals = [[int(action[0] * full_w), int(action[1] * full_h)]
                                    for action in cpu_actions]
                    global_goals = [[min(x, int(full_w - 1)), min(y, int(full_h - 1))]
                                    for x, y in global_goals]

                    g_masks = torch.ones(self.num_scenes).float().to(self.device)
            elif state == "Contextual Search":
                self.graph,agent_location_node = projectCurrentAgentLoc(tuple(pos_map),self.graph)
                node_rooms = projectHistoryToGraph(self.graph,pos_map,self.room_message,50)
                user,node_candidate = landmark_CS(goal_name,agent_location_node,obstacle_map,objects,bg_objects,scene_graph,self.graph,node_rooms,explored_nodes,frontier_nodes,relative,[self.map_size_cm//self.map_resolution,self.map_size_cm//self.map_resolution,self.max_height-self.min_height])
                start=time.time()
                while True:
                    try:
                        # response,node = self.llm.query_node_graph_o1("map/graph_"+str(step)+".jpg",user)
                        response,node = self.llm.query_node_txt(user)
                        break
                    except Exception as ex: # rate limit
                        print(f"[ERROR] in LLM inference =====> {ex}, sleep for 20s...")
                        time.sleep(20)
                        continue
                found_elements=[node]
                end=time.time()
                print(response,f"time:{end-start}")
                if len(found_elements) != 0 :
                    if found_elements[0] in pos.keys() and np.linalg.norm(np.array(pos[found_elements[0]])-pos_map) < 20 :
                        ss = set(node_candidate)-set(explored_nodes)
                        ss = list(ss)
                        if len(ss) > 0 :
                            self.global_position=np.array([int(pos[ss[0]][0]),int(pos[ss[0]][1])])
                        elif len(node_candidate) > 0 :
                            self.global_position=np.array([int(pos[node_candidate[0]][0]),int(pos[node_candidate[0]][1])])
                        else :
                            array = np.array(list(pos.values()))
                            array = array.reshape(-1, 2)
                            iddx=np.argmax(np.max(np.linalg.norm(array[:,None,:]-pos_map[None,None,:],axis=-1),axis=-1))
                            self.global_position=array[iddx].astype(int)
                    # print("found_elements",found_elements[0],"find node time:",end-start)
                    else :
                        self.global_position=np.array([int(pos[found_elements[0]][0]),int(pos[found_elements[0]][1])])
                else :
                    array = np.array(list(pos.values()))
                    array = array.reshape(-1, 2)
                    iddx=np.argmax(np.max(np.linalg.norm(array[:,None,:]-pos_map[None,None,:],axis=-1),axis=-1))
                    self.global_position=array[iddx].astype(int)
                self.global_position_history.append(self.global_position)
            elif state == "Observe Target":
                self.graph,agent_location_node = projectCurrentAgentLoc(tuple(pos_map),self.graph)
                node_rooms = projectHistoryToGraph(self.graph,pos_map,self.room_message,50)
                user,node_candidate = landmark_OT(goal_name,agent_location_node,obstacle_map,objects,bg_objects,scene_graph,self.graph,node_rooms,explored_nodes,frontier_nodes,goal_name,[self.map_size_cm//self.map_resolution,self.map_size_cm//self.map_resolution,self.max_height-self.min_height])
                start=time.time()
                if user == "" or len(node_candidate) == 0:
                    print(f"Warning: landmark_OT returned empty result, skipping LLM query")
                    response = {'result': 0}
                    node = 0
                else:
                    while True:
                        try:
                            # response,node = self.llm.query_node_graph_o1("map/graph_"+str(step)+".jpg",user)
                            response,node = self.llm.query_node_txt(user)
                            break
                        except Exception as ex: # rate limit
                            print(f"[ERROR] in LLM inference =====> {ex}, sleep for 20s...")
                            time.sleep(20)
                            continue
                found_elements=[node]
                end=time.time()
                print(response,f"time:{end-start}")
                if len(found_elements) != 0 :
                    if found_elements[0] in pos.keys() and np.linalg.norm(np.array(pos[found_elements[0]])-pos_map) < 20 :
                        ss = set(node_candidate)-set(explored_nodes)
                        ss = list(ss)
                        if len(ss) > 0 :
                            self.global_position=np.array([int(pos[ss[0]][0]),int(pos[ss[0]][1])])
                        elif len(node_candidate) > 0 :
                            self.global_position=np.array([int(pos[node_candidate[0]][0]),int(pos[node_candidate[0]][1])])
                        else :
                            array = np.array(list(pos.values()))
                            array = array.reshape(-1, 2)
                            iddx=np.argmax(np.max(np.linalg.norm(array[:,None,:]-pos_map[None,None,:],axis=-1),axis=-1))
                            self.global_position=array[iddx].astype(int)
                    # print("found_elements",found_elements[0],"find node time:",end-start)
                    else :
                        self.global_position=np.array([int(pos[found_elements[0]][0]),int(pos[found_elements[0]][1])])
                else :
                    array = np.array(list(pos.values()))
                    array = array.reshape(-1, 2)
                    iddx=np.argmax(np.max(np.linalg.norm(array[:,None,:]-pos_map[None,None,:],axis=-1),axis=-1))
                    self.global_position=array[iddx].astype(int)
                self.global_position_history.append(self.global_position)
                target_reach = False
            elif state == "Candidate Verification":
                indices_3=np.array(np.unravel_index(obj['voxel_index'], (self.map_size_cm//self.map_resolution,self.map_size_cm//self.map_resolution,self.max_height-self.min_height),order='C')).T
                center_2d = np.mean(indices_3[:,:2],axis=0)
                node_new = find_another_view(center_2d,self.graph,np.array(self.global_position_history))
                self.global_position=np.array([int(pos[node_new][0]),int(pos[node_new][1])])
                target_reach = False
            elif state == "Target Confirmation":
                indices_3=np.array(np.unravel_index(self.sem_map_module.objects[found_obj_id-2]['voxel_index'], (self.map_size_cm//self.map_resolution,self.map_size_cm//self.map_resolution,self.max_height-self.min_height),order='C')).T
                array = np.array(list(pos.values()))
                array = array.reshape(-1, 2)
                iddx=np.argmin(np.min(np.linalg.norm(array[:,None,:]-indices_3[None,:,:2],axis=-1),axis=-1))
                self.global_position=array[iddx].astype(int)
                target_reach = False
        ## update cognitive map every 10 times
        elif (step-self.last_done) % 10 ==0 :
            objects,bg_objects,scene_graph,obstacle_map,wall = self.sem_map_module.updateSceneGraph(save_path=save_path,idx=step-self.last_done,exp_map=full_map[0,1])
            full_map[0,4]=obstacle_map
            ### select the frontier edge # 找到目前整個地圖的邊緣
            self.graph,leaf_values,full_ex_map[0],full_ob_map[0],explored_nodes,frontier_nodes = generateVoronoi(full_map,self.global_position_history,full_w=full_w,full_h=full_h,step=step-self.last_done,save_path=save_path)
            if self.graph is not None and self.global_position is not None :
                self.graph,agent_location_node = projectCurrentAgentLoc(tuple(self.global_position),self.graph)
                pos = nx.get_node_attributes(self.graph, 'pos')
                self.global_position=np.array([int(pos[agent_location_node][0]),int(pos[agent_location_node][1])])
        # ------------------------------------------------------------------
        for e in range(self.num_scenes):
            if self.global_position is not None :
                shape = self.global_position.shape
                if len(shape) == 1:
                    goal_maps[e][self.global_position[1],self.global_position[0]]=1
                else :
                    goal_maps[e][self.global_position[:,1],self.global_position[:,0]]=1
                g_sum_rewards += 1
            else:
                goal_maps[e][global_goals[e][0], global_goals[e][1]] = 1
                print("idx:",step-self.last_done," random goal:",[global_goals[e][0], global_goals[e][1]])
        if state == "Target Confirmation":
            indices_3=np.array(np.unravel_index(self.sem_map_module.objects[found_obj_id-2]['voxel_index'], (self.map_size_cm//self.map_resolution,self.map_size_cm//self.map_resolution,self.max_height-self.min_height),order='C')).T
            idx = np.argmin(np.linalg.norm(pos_map-indices_3[:,:2],axis=1))
            if np.linalg.norm(pos_map-indices_3[idx,:2]) < 30 :
                print("achieve the target")
                goal_maps[0][indices_3[:,1], indices_3[:,0]] = 1
                found_goal[0]=1
        # Take action and get next observation
        planner_inputs = [{} for e in range(self.num_scenes)]
        if self.skip_times > 0 :
            done = 1
        else :
            done = 0
        for e, p_input in enumerate(planner_inputs):
            # planner_pose_inputs[e, 3:] = [0, local_w, 0, local_h]
            p_input['map_pred'] = full_map[e, 0, :, :].cpu().numpy()
            p_input['exp_pred'] = full_map[e, 1, :, :].cpu().numpy()
            p_input['pose_pred'] = planner_pose_inputs[e]
            p_input['goal'] = goal_maps[e]  # global_goals[e]
            p_input['map_target'] = target_point_map[e]  # global_goals[e]
            p_input['done']=done
            p_input['new_goal'] = step == self.num_local_steps - 1
            p_input['found_goal'] = found_goal[e]
            p_input['wait'] = wait_env[e] or finished[e]
            p_input['step'] = step-self.last_done
            if self.visualize or self.print_images:
                # p_input['map_edge'] = target_edge_map[e]
                # full_map[e, -1, :, :] = 1e-5
                p_input['sem_map_pred'] = full_map[e,4,:,:].cpu().numpy()
                p_input['graph'] = self.graph
                p_input['semantic_path'] = os.path.join(save_path,"visual/",str(step-self.last_done)+".png")
                # p_input['sem_objects'] = full_map[e,4,:,:].cpu().numpy()
        obs, fail_case, done, infos = self.envs.plan_act_and_preprocess(planner_inputs)

        init_data_var_dict = {
            'obs': obs,
            'done': done,
            'infos': infos,
            'g_masks': g_masks,
            'fail_case': fail_case,
            'episode_success': episode_success,
            'episode_spl': episode_spl,
            'episode_dist': episode_dist,
            'gpt_target_result':gpt_target_result,
            'target_reach':target_reach,
            'direction_action':None,
            'state': state,
            'full_pose': full_pose,
            'eve_angle': eve_angle,
            'origins': origins,
            'planner_pose_inputs': planner_pose_inputs,
            'full_wh': (full_w, full_h),
            'stair_flag': stair_flag,
            'step_masks': step_masks,
            'clear_flag': clear_flag,
            'g_process_rewards': g_process_rewards,
            'g_sum_global': init_data_var_dict['g_sum_global'],
            'g_sum_rewards': g_sum_rewards,
        }

        init_data_map_dict = {
            # 'local_map': local_map,
            'full_map': full_map,
            'full_map_new':full_map_new,
            'global_goals': global_goals,
            'full_ob_map': full_ob_map,
            'full_ex_map': full_ex_map,
            'lmb': lmb,
            'kernel': kernel,
            'tv_kernel': tv_kernel,
            'target_edge_map': target_edge_map,
            'target_point_map': target_point_map,
            'frontier_score_list': frontier_score_list,
            'spl_per_category': spl_per_category,
            'success_per_category': success_per_category,
        }

        return init_data_var_dict, init_data_map_dict, finished, wait_env,save_i

    def init_episode(self):
        g_masks = torch.ones(self.num_scenes).float().to(self.device)
        step_masks = torch.zeros(self.num_scenes).float().to(self.device)

        # if self.eval:
        episode_success, episode_spl, episode_dist = [], [], []
        for _ in range(self.num_scenes):  # num_scenes 目前只跑一个场景
            episode_success.append(deque(maxlen=self.num_episodes)) # self.num_episodes: 2000
            episode_spl.append(deque(maxlen=self.num_episodes))
            episode_dist.append(deque(maxlen=self.num_episodes))

        episode_sem_frontier, episode_sem_goal, episode_loc_frontier = [], [], []
        for _ in range(self.num_scenes):
            episode_sem_frontier.append([])
            episode_sem_goal.append([])
            episode_loc_frontier.append([])
        finished = np.zeros(self.num_scenes)
        wait_env = np.zeros(self.num_scenes)

        g_process_rewards = 0
        g_sum_rewards = 1
        g_sum_global = 1
        stair_flag = np.zeros(self.num_scenes)
        clear_flag = np.zeros(self.num_scenes)

        self.envs,scenes = make_vec_envs(self.args)
        # obs: Tensor: (1, 20, 120, 160)
        # infos: {'distance_to_goal': None, 'success': None, 'time': 0,
        # 'sensor_pose': [0.0, 0.0, 0.0]. 'goal_cat_id': 3,
        # 'goal_name':'bed', 'eve_angle': 0}
        obs, infos = self.envs.reset()
        # Initialize map variables:
        # Full map consists of multiple channels containing the following:
        # 1. Obstacle Map
        # 2. Exploread Area
        # 3. Current Agent Location
        # 4. Past Agent Locations
        # 5,6,7,.. : Semantic Categories
        nc =  5  # num channels  16 + 4 [1 - 4]

        # Calculating full and local map sizes
        map_size = self.map_size_cm // self.map_resolution
        full_w, full_h = map_size, map_size  # 4800 / 5 = 960
        # local_w = int(full_w / self.global_downscaling) # 960 / 2 = 480
        # local_h = int(full_h / self.global_downscaling)

        # Initializing full and local map
        full_map = torch.zeros(self.num_scenes, nc, full_w, full_h).float().to(self.device)
        # local_map = torch.zeros(self.num_scenes, nc, local_w, local_h).float().to(self.device)
        # full_map_new = torch.zeros(self.num_scenes, nc, full_w, full_h).float().to(self.device)
        full_ob_map = np.zeros((self.num_scenes, full_w, full_h))
        full_ex_map = np.zeros((self.num_scenes, full_w, full_h))

        target_edge_map = np.zeros((self.num_scenes, full_w, full_h))
        target_point_map = np.zeros((self.num_scenes, full_w, full_h))

        # dialate for target map
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        tv_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

        # Initial full and local pose
        full_pose = torch.zeros(self.num_scenes, 3).float().to(self.device)
        current_pose = torch.zeros(self.num_scenes, 3).float().to(self.device)

        # Origin of local map
        origins = np.zeros((self.num_scenes, 3))

        # Global Map Boundaries
        lmb = np.zeros((self.num_scenes, 4)).astype(int)

        # Planner pose inputs has 7 dimensions
        # 0-3 store continuous global agent location
        # 4-6 store local map boundaries
        planner_pose_inputs = np.zeros((self.num_scenes, 7))

        frontier_score_list = []
        for _ in range(self.num_scenes):
            frontier_score_list.append(deque(maxlen=10))

        self.init_map_and_pose(full_map, full_pose, planner_pose_inputs, lmb,
                               #origins,local_map, local_pose, (local_w, local_h), 
                               (full_w, full_h))
        current_pose = full_pose
        full_map_new = full_map.clone().detach()
        # Predict semantic map from frame 1
        poses = torch.from_numpy(np.asarray(
            [infos[env_idx]['sensor_pose'] for env_idx in range(self.num_scenes)])
        ).float().to(self.device)

        eve_angle = np.asarray(
            [infos[env_idx]['eve_angle'] for env_idx in range(self.num_scenes)]
        )

        # obs: Tensor(1,  m, 120, 160)
        # poses: Tensor(1, 3)  当前的pose
        # local_map: Tensor(1, 20, 480, 480)
        # local_pose: Tensor(1, 3)  # 要变化的pose
        # eve_angle: Tensor(1,)
        save_i=0
        path=self.args.dump_location+ scenes[0] +"/"
        full_map, local_map_stair, current_pose= \
            self.sem_map_module(obs, infos[0]['ori_obs'], poses, full_map, current_pose, eve_angle, origins, idx=-1,save_path=path+str(save_i)+"/")
        # local_map[:, 0, :, :][local_map[:, 13, :, :] > 0] = 0

        actions = torch.randn(self.num_scenes, 2) * 6
        cpu_actions = nn.Sigmoid()(actions).cpu().numpy()
        global_goals = [[int(action[0] * full_w), int(action[1] * full_h)] for action in cpu_actions]
        global_goals = [[min(x, int(full_w - 1)), min(y, int(full_h - 1))] for x, y in global_goals]

        goal_maps = [np.zeros((full_w, full_h)) for _ in range(self.num_scenes)]

        for e in range(self.num_scenes):
            goal_maps[e][global_goals[e][0], global_goals[e][1]] = 1

        planner_inputs = [{} for e in range(self.num_scenes)]
        for e, p_input in enumerate(planner_inputs):
            p_input['map_pred'] = full_map[e, 0, :, :].cpu().numpy()
            p_input['exp_pred'] = full_map[e, 1, :, :].cpu().numpy()
            p_input['pose_pred'] = planner_pose_inputs[e]
            p_input['goal'] = goal_maps[e]
            p_input['done'] = 0
            p_input['map_target'] = target_point_map[e]
            p_input['new_goal'] = 1
            p_input['found_goal'] = 0
            p_input['wait'] = wait_env[e] or finished[e]
            p_input['step'] = -1
            if self.visualize or self.print_images:
                # p_input['map_edge'] = target_edge_map[e]
                ### full_map or global_map
                # full_map[e, -1, :, :] = 1e-5
                p_input['sem_map_pred'] = full_map[e,4,:,:].cpu().numpy()
                # p_input['sem_objects'] = full_map[e,4,:,:].cpu().numpy()
                p_input['graph'] = None
                p_input['semantic_path'] = os.path.join(path+str(save_i)+"/","visual/","-1.png")
        obs, _, done, infos = self.envs.plan_act_and_preprocess(planner_inputs)
        # import pdb
        # pdb.set_trace()
        init_data_var_dict = {
            'obs': obs,
            'done': done,
            'infos': infos,
            'g_masks': g_masks,
            'episode_success': episode_success,
            'episode_spl': episode_spl,
            'episode_dist': episode_dist,
            'full_pose': full_pose,
            'gpt_target_result':[],
            'needed_rotation': [],
            'target_reach':True,
            'state':"Broad Search",
            'eve_angle': eve_angle,
            'origins': origins,

            'planner_pose_inputs': planner_pose_inputs,
            # 'local_wh': (local_w, local_h),
            'full_wh': (full_w, full_h),
            'stair_flag': stair_flag,
            'step_masks': step_masks,
            'clear_flag': clear_flag,
            'g_process_rewards': g_process_rewards,
            'g_sum_global': g_sum_global,
            'g_sum_rewards': g_sum_rewards,
        }

        init_data_map_dict = {
            # 'local_map': local_map,
            'full_map': full_map,
            "full_map_new":full_map_new,
            'global_goals': global_goals,
            'full_ob_map': full_ob_map,
            'full_ex_map': full_ex_map,
            'lmb': lmb,
            'kernel': kernel,
            'tv_kernel': tv_kernel,
            'target_edge_map': target_edge_map,
            'target_point_map': target_point_map,
            'frontier_score_list': frontier_score_list,
            'spl_per_category': defaultdict(list),
            'success_per_category': defaultdict(list),
        }
        return init_data_var_dict, init_data_map_dict, finished, wait_env,path,save_i

    def init_map_and_pose(self,
                          full_map,
                          full_pose,
                          planner_pose_inputs,
                          lmb,
                        #   origins,
                        #   local_map,
                        #   local_pose,
                        #   local_wh,
                          full_wh):
        full_map.fill_(0.)
        full_pose.fill_(0.)
        full_pose[:, :2] = self.map_size_cm / 100.0 / 2.0
        locs = full_pose.cpu().numpy()
        planner_pose_inputs[:, :3] = locs

        # (local_w, local_h) = local_wh
        (full_w, full_h) = full_wh

        for e in range(self.num_scenes):
            r, c = locs[e, 1], locs[e, 0]
            loc_r, loc_c = [int(r * 100.0 / self.map_resolution),
                            int(c * 100.0 / self.map_resolution)]

            full_map[e, 2:4, loc_r-1:loc_r+2, loc_c-1:loc_c+2] = 1.0
            # lmb[e] = self.get_local_map_boundaries((loc_r, loc_c),
            #                                        (local_w, local_h),
            #                                        (full_w, full_h))
            lmb[e] = [0,full_w,0,full_h]
            planner_pose_inputs[e, 3:] = lmb[e]
            # origins[e] = [lmb[e][2] * self.map_resolution / 100.0,
            #               lmb[e][0] * self.map_resolution / 100.0, 0.]

        # for e in range(self.num_scenes):
        #     local_map[e] = full_map[e, :, lmb[e, 0]:lmb[e, 1], lmb[e, 2]:lmb[e, 3]]
        #     local_pose[e] = full_pose[e] - torch.from_numpy(origins[e]).to(self.device).float()

    def logging(self, step, init_data_var_dict, start):
        g_process_rewards, g_sum_rewards, g_sum_global =\
            init_data_var_dict['g_process_rewards'], init_data_var_dict['g_sum_rewards'], init_data_var_dict['g_sum_global']
        episode_success, episode_spl, episode_dist =\
            init_data_var_dict['episode_success'], init_data_var_dict['episode_spl'], init_data_var_dict['episode_dist']

        fail_case = init_data_var_dict['fail_case']
        log = None
        if step % self.log_interval == 0:
            end = time.time()
            time_elapsed = time.gmtime(end - start)
            log = " ".join([
                "Time: {0:0=2d}d".format(time_elapsed.tm_mday - 1),
                "{},".format(time.strftime("%Hh %Mm %Ss", time_elapsed)),
                "num timesteps {},".format(step * self.num_scenes),
                "FPS {},".format(int(step * self.num_scenes / (end - start)))
            ])

            log += "\n\tLLM Rewards: " + str(g_process_rewards /g_sum_rewards)
            log += "\n\tLLM use rate: " + str(g_sum_rewards /g_sum_global)
            if self.eval:

                total_success = []
                total_spl = []
                total_dist = []
                for e in range(self.num_scenes):
                    for acc in episode_success[e]:
                        total_success.append(acc)
                    for dist in episode_dist[e]:
                        total_dist.append(dist)
                    for spl in episode_spl[e]:
                        total_spl.append(spl)
                # print("succ:",total_success,"spl:",total_spl,"dtg:",total_dist)
                if len(total_spl) > 0:
                    log += " ObjectNav succ/spl/dtg:"
                    log += " {:.3f}/{:.3f}/{:.3f}({:.0f}),".format(
                        np.mean(total_success),
                        np.mean(total_spl),
                        np.mean(total_dist),
                        len(total_spl))
                    log += "\n\tsuccess: " + str(total_success)
                    log += "\n\tspl: " + str(total_spl)
                    log += "\n\tdtg: " + str(total_dist)
                total_collision = []
                total_exploration = []
                total_detection = []
                total_success = []
                for e in range(self.num_scenes):
                    total_collision.append(fail_case[e]['collision'])
                    total_exploration.append(fail_case[e]['exploration'])
                    total_detection.append(fail_case[e]['detection'])
                    total_success.append(fail_case[e]['success'])

                if len(total_spl) > 0:
                    log += " Fail Case: collision/exploration/detection/success:"
                    log += " {:.0f}/{:.0f}/{:.0f}/{:.0f}({:.0f}),".format(
                        np.sum(total_collision),
                        np.sum(total_exploration),
                        np.sum(total_detection),
                        np.sum(total_success),
                        len(total_spl))

            print(log)
            logging.info(log)
        return log

    def logging_final(self, init_data_var_dict, init_data_map_dict, log):
        g_process_rewards, g_sum_rewards, g_sum_global =\
            init_data_var_dict['g_process_rewards'], init_data_var_dict['g_sum_rewards'], init_data_var_dict['g_sum_global']
        episode_success, episode_spl, episode_dist =\
            init_data_var_dict['episode_success'], init_data_var_dict['episode_spl'], init_data_var_dict['episode_dist']
        fail_case = init_data_var_dict['fail_case']

        spl_per_category, success_per_category =\
            init_data_map_dict['spl_per_category'], init_data_map_dict['success_per_category']

        if self.eval:
            print("Dumping eval details...")

            log += "\n\tLLM Rewards: " + str(g_process_rewards / g_sum_rewards)
            log += "\n\tLLM use rate: " + str(g_sum_rewards / g_sum_global)

            total_success = []
            total_spl = []
            total_dist = []
            for e in range(self.num_scenes):
                for acc in episode_success[e]:
                    total_success.append(acc)
                for dist in episode_dist[e]:
                    total_dist.append(dist)
                for spl in episode_spl[e]:
                    total_spl.append(spl)

            if len(total_spl) > 0:
                log = "Final ObjectNav succ/spl/dtg:"
                log += " {:.3f}/{:.3f}/{:.3f}({:.0f}),".format(
                    np.mean(total_success),
                    np.mean(total_spl),
                    np.mean(total_dist),
                    len(total_spl))

            print(log)
            logging.info(log)

            # Save the spl per category
            log = "Success | SPL per category\n"
            for key in success_per_category:
                log += "{}: {} | {}\n".format(key,
                                              sum(success_per_category[key]) /
                                              len(success_per_category[key]),
                                              sum(spl_per_category[key]) /
                                              len(spl_per_category[key]))

            print(log)
            logging.info(log)

            with open('{}/{}_spl_per_cat_pred_thr.json'.format(self.dump_dir, self.split), 'w') as f:
                json.dump(spl_per_category, f)

            with open('{}/{}_success_per_cat_pred_thr.json'.format(self.dump_dir, self.split), 'w') as f:
                json.dump(success_per_category, f)

    # def get_local_map_boundaries(self,
    #                              frontier_loc,
    #                              frontier_sizes,
    #                              map_sizes):
    #     loc_r, loc_c = frontier_loc
    #     local_w, local_h = frontier_sizes
    #     full_w, full_h = map_sizes

    #     if self.global_downscaling > 1:
    #         gx1, gy1 = loc_r - local_w // 2, loc_c - local_h // 2
    #         gx2, gy2 = gx1 + local_w, gy1 + local_h
    #         if gx1 < 0:
    #             gx1, gx2 = 0, local_w
    #         if gx2 > full_w:
    #             gx1, gx2 = full_w - local_w, full_w

    #         if gy1 < 0:
    #             gy1, gy2 = 0, local_h
    #         if gy2 > full_h:
    #             gy1, gy2 = full_h - local_h, full_h
    #     else:
    #         gx1, gx2, gy1, gy2 = 0, full_w, 0, full_h

    #     return [gx1, gx2, gy1, gy2]

    def get_frontier_boundaries(self, frontier_loc, frontier_sizes, map_sizes):
        loc_r, loc_c = frontier_loc
        local_w, local_h = frontier_sizes
        full_w, full_h = map_sizes

        gx1, gy1 = loc_r - local_w // 2, loc_c - local_h // 2
        gx2, gy2 = gx1 + local_w, gy1 + local_h
        if gx1 < 0:
            gx1, gx2 = 0, local_w
        if gx2 > full_w:
            gx1, gx2 = full_w - local_w, full_w

        if gy1 < 0:
            gy1, gy2 = 0, local_h
        if gy2 > full_h:
            gy1, gy2 = full_h - local_h, full_h

        return [int(gx1), int(gx2), int(gy1), int(gy2)]

   

    def get_hm3d_semantic_map_index(self, fileName):
        # fileName = 'data/matterport_category_mappings.tsv'

        text = ''
        lines = []
        items = []
        self.hm3d_semantic_mapping = {}
        self.hm3d_semantic_index = {}
        self.hm3d_semantic_index_inv = {}

        with open(fileName, 'r') as f:
            text = f.read()
        lines = text.split('\n')[1:]

        for l in lines:
            items.append(l.split('    '))

        for i in items:
            if len(i) > 3:
                self.hm3d_semantic_mapping[i[2]] = i[-1]
                self.hm3d_semantic_index[i[-1]] = int(i[-2])
                self.hm3d_semantic_index_inv[int(i[-2])] = i[-1]

    
