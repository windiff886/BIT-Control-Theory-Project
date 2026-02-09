# CogNav 代码执行流程详解

> 本文档详细说明执行 `python main.py --scenes TEEsavR23oF` 后代码的完整执行流程。

## 目录

1. [概述](#1-概述)
2. [入口点: main.py](#2-入口点-mainpy)
3. [参数解析](#3-参数解析)
4. [Episode 初始化](#4-episode-初始化)
5. [主循环: forward()](#5-主循环-forward)
6. [语义建图与目标检测](#6-语义建图与目标检测)
7. [场景图构建](#7-场景图构建)
8. [状态机与决策](#8-状态机与决策)
9. [LLM 集成](#9-llm-集成)
10. [导航规划与动作执行](#10-导航规划与动作执行)
11. [完整时间步流程](#11-完整时间步流程)
12. [执行流程图](#12-执行流程图)

---

## 1. 概述

CogNav 是一个**认知导航系统**，结合了：

| 模块 | 技术 | 作用 |
|------|------|------|
| 视觉 | OpenSeeD | 全景分割，检测物体 |
| 建图 | 3D 体素 + 2D 语义地图 | 空间表示 |
| 规划 | Voronoi 图 + FMM | 路径规划 |
| 推理 | Qwen LLM | 状态决策、地标选择 |
| 场景理解 | 场景图 | 物体关系建模 |

**核心创新**：使用 LLM 作为**决策引擎**，理解空间关系、房间语境和物体语义，进行战略性探索而非纯反应式导航。

---

## 2. 入口点: main.py

**文件**: `main.py`

```python
# 1. 加载语义类别映射
# 从 data/matterport_category_mappings.tsv 加载 HM3D 物体类别

# 2. 配置随机种子
np.random.seed(args.seed)
torch.manual_seed(args.seed)

# 3. CUDA 设置
torch.set_grad_enabled(False)  # 仅推理，不训练

# 4. 创建 Episode 对象
navigation_episodes = Episode(args)

# 5. 开始导航
navigation_episodes.start()
```

---

## 3. 参数解析

**文件**: `utils/arguments.py`

运行 `--scenes TEEsavR23oF` 时的关键参数：

### 环境设置
```python
env_frame_width = 640        # 环境帧宽度
env_frame_height = 480       # 环境帧高度
frame_width = 160            # 处理帧宽度（下采样）
frame_height = 120           # 处理帧高度
max_episode_length = 500     # 最大时间步
```

### 深度与地图
```python
min_depth = 0.5              # 最小深度 (米)
max_depth = 5.0              # 最大深度 (米)
map_size_cm = 4800           # 地图大小 (厘米)
map_resolution = 5           # 地图分辨率 (厘米/像素)
vision_range = 100           # 视觉范围
```

### 模型配置
```python
path_detection_config = './configs/detection/HM3D.yaml'
cog_model_path = 'model/pretained_model/cogvlm2-llama3-chat-19B'
```

---

## 4. Episode 初始化

**文件**: `episode.py` - `init_episode()`

### 4.1 环境创建

```python
self.envs, scenes = make_vec_envs(args)  # 创建 Habitat 环境
obs, infos = self.envs.reset()           # 获取初始观测
```

**返回内容**：
- `obs`: RGBD 观测 (480×640 RGB + 深度)
- `infos`: 包含 `distance_to_goal`, `success`, `goal_cat_id` 等

### 4.2 地图初始化

```python
# 全地图: 960×960 空间网格 (4800cm / 5cm 分辨率)
full_map = torch.zeros(num_scenes, nc, full_w, full_h)

# 地图通道:
# [0] 障碍物地图
# [1] 已探索区域
# [2-3] 当前/历史智能体位置
# [4+] 语义物体地图
```

### 4.3 首次语义建图

```python
full_map = self.sem_map_module(obs, infos[0]['ori_obs'], poses, ...)
```

调用 `Semantic_Mapping_SG.forward()`：
1. 预处理深度图像
2. 投影到 3D 点云
3. 转换到世界坐标系
4. 运行 OpenSeeD 物体检测
5. 创建检测物体的 3D 体素表示
6. 投影到智能体高度的 2D 语义地图

---

## 5. 主循环: forward()

**文件**: `episode.py` - `start()` 和 `forward()`

```python
for step in range(num_training_frames // num_scenes + 1):
    init_data_var_dict, init_data_map_dict, ... = self.forward(step, ...)
```

每次迭代处理一个时间步。

---

## 6. 语义建图与目标检测

### 6.1 OpenSeeD 物体检测

**文件**: `detection/openseed.py`

```python
def detection(image_ori, idx, visual_save_path, detection_save_path):
    # 输入: RGB 图像
    # 输出:
    #   - 全景分割掩码 (为每个像素分配实例 ID)
    #   - 实例信息 (边界框、类别名、置信度)
    #   - 视觉 & 文本嵌入 (用于跨帧匹配)
```

### 6.2 远程推理 (可选)

**文件**: `detection/remote_inference.py`

如果 `USE_REMOTE_OPENSEED=1`：
```python
def openseed_predict(image):
    # 发送图像到远程服务器 (Colab)
    # 返回 base64 编码的掩码和实例元数据
```

### 6.3 3D 物体重建

**文件**: `slam/cfslam.py`

对每个检测到的物体：
1. **反投影掩码**: 将 2D 掩码转换为 3D 体素索引
2. **跨帧累积**: 使用 CLIP 嵌入进行物体匹配和跟踪
3. **存储元数据**:
   - `voxel_index`: 稀疏 3D 坐标
   - `class_name`: OpenSeeD 类别名
   - `clip_ft`: 视觉嵌入
   - `bbox`: 3D 边界框

物体存储在 `self.objects` (MapObjectList)

---

## 7. 场景图构建

### 7.1 场景图生成 (每 10 步)

**文件**: `model/Semantic_Mapping_SG.py` - `updateSceneGraph()`

当 `(step % 10 == 0)` 时：

1. **去除重叠物体**: 合并跨帧的相同物体检测
2. **准备关系查询**:
   - 对每对相邻物体，捕获显示两者的图像
3. **查询 LLM 获取关系**:
   ```python
   relation = self.llm.get_relationship_prompt(obj1, obj2, image)
   # 返回: "on", "under", "next to" 等
   ```
4. **构建场景图**: 字典映射物体对到其关系

### 7.2 Voronoi 导航图生成

**文件**: `utils/voronoi.py` - `generateVoronoi()`

从已探索空间创建导航路网：

1. **获取边界点**: 提取已探索/未探索区域之间的所有边界点
2. **Voronoi 分解**: 划分自由空间
3. **构建导航图**: 顶点是 Voronoi 站点，边表示连通性
4. **简化图**: 合并共线节点
5. **分类节点**:
   - **已探索节点**: 已访问
   - **边界节点**: 在探索边界上
   - **叶子节点**: 度为 1 的节点，指向特定物体或未探索区域

---

## 8. 状态机与决策

### 8.1 状态定义

智能体在 4 种状态下运行：

| 状态 | 英文 | 目标 | 动作 | 触发条件 |
|------|------|------|------|----------|
| 广泛搜索 | Broad Search | 找到目标可能在的大致区域 | 导航到边界/叶子节点 | 目标未检测到 或 智能体移动太远 |
| 上下文搜索 | Contextual Search | 导航到与目标语义相关的区域 | 找到与相关物体同房间的节点 | 检测到与目标语义相关的物体 |
| 观察目标 | Observe Target | 获取检测目标的更好视角 | 导航到有清晰视线的附近节点 | 检测到目标但需要确认 |
| 目标确认 | Target Confirmation | 接近已确认的目标 | 移动到目标位置附近的最近导航节点 | 高置信度检测到目标 |

### 8.2 状态转换逻辑

**文件**: `scenegraph/querypath.py` - `state_prompt()`

```python
user = state_prompt(
    goal_name,          # "bed", "chair" 等
    obstacle_map,       # 当前障碍物地图
    objects,            # 检测到的 3D 物体
    bg_objects,         # 墙壁、地板
    graph,              # Voronoi 导航图
    node_rooms,         # 节点的房间分配
    frontier_nodes,     # 边界位置
    state,              # 当前状态
    target_level        # 确认级别 (如果找到目标)
)
state_new, relative = self.llm.query_state_transition(user)
```

LLM 返回：
- `state_new`: 下一个状态
- `relative`: 用于上下文搜索的相关物体/房间名称

---

## 9. LLM 集成

**文件**: `scenegraph/qwen3.py`

### 9.1 LLM 查询类型

| 函数 | 输入 | 输出 | 用途 |
|------|------|------|------|
| `get_room()` | 当前视图图像 | 房间名称 (如 "living room") | 每 10 步分配节点房间 |
| `query_target_obj()` | 显示检测物体的图像 + 目标名 | "Yes" / "No" | 确认检测物体是否匹配目标 |
| `query_state_transition()` | 详细场景描述 | JSON {状态, 相关物体} | 核心决策机制 |
| `query_node_txt()` | 节点描述列表 | 选择的节点 ID | 所有搜索状态中使用 |
| `get_relationship_prompt()` | 两个物体的图像 | 关系类型 | 构建场景图 |

### 9.2 API 配置

```python
# Qwen API (阿里云)
api_key = os.environ.get("QWEN_API_KEY")
base_url = "https://dashscope.aliyuncs.com/compatible-mode/v1"
model_name = "qwen-vl-max"       # 视觉模型
text_model_name = "qwen-plus"    # 纯文本模型
```

**远程推理选项**:
- 如果 `USE_REMOTE_INFERENCE=1`，调用本地 CogVLM2 服务器

---

## 10. 导航规划与动作执行

### 10.1 规划管道

**文件**: `agents/sem_exp.py` - `_plan()`

每个时间步：

1. **更新地图上的目标**:
   ```python
   goal_maps[e][self.global_position[1], self.global_position[0]] = 1
   ```

2. **运行 FMM 规划器**:
   - 将障碍物视为不可通行
   - 使用快速行进法计算最短路径
   - 返回短期目标 (STG): 视野范围内的下一个航点

3. **确定动作**:
   | 条件 | 动作 | 代码 |
   |------|------|------|
   | STG 在正前方 | 前进 | `action = 1` |
   | STG 在右边 | 右转 | `action = 3` |
   | STG 在左边 | 左转 | `action = 2` |
   | 到达目标 | 停止 | `action = 0` |
   | 需要向下看 | 向下看 | `action = 5` |
   | 需要向上看 | 向上看 | `action = 4` |

### 10.2 碰撞检测与重规划

```python
if dist(prev_position, curr_position) < collision_threshold:
    collision_n += 1
    # 将区域标记为障碍物 (碰撞地图)

if collision_n > 40 or replan_count > 20:
    # 强制重规划 (清除当前地图区域)
    clear_flag = 1
```

---

## 11. 完整时间步流程

每次 `forward()` 调用中发生的事情：

```
1. [观测] 从 Habitat 环境获取 RGBD
   ├─ obs: (1, 4, 120, 160) [RGB + 深度]
   ├─ infos: {'distance_to_goal', 'success', 'goal_cat_id', ...}
   └─ done: episode 是否结束?

2. [语义建图]
   ├─ 使用 OpenSeeD 检测物体
   ├─ 反投影到 3D 体素
   ├─ 在 self.objects (MapObjectList) 中累积物体
   └─ 投影到智能体高度的 2D 语义地图

3. [每 10 步: 场景图更新]
   ├─ 去除重叠检测
   ├─ 生成 Voronoi 导航图
   ├─ 构建场景图 (物体关系)
   ├─ 将历史位置投影到图 (房间分配)
   └─ 分类节点 (已探索/边界/叶子)

4. [目标检测]
   ├─ 检查是否检测到目标物体 (目标类别)
   ├─ 如果是: found_obj_id = 索引, found_obj_rate = 置信度
   └─ 跟踪 start_found = 首次检测时间步

5. [状态转换]
   ├─ 如果 found_obj_id != -1 且 target_reach 且 最近检测到:
   │  └─ 查询 LLM: "这是目标吗?" 使用最佳视图图像
   ├─ 查询 LLM: state_transition() 使用场景描述
   ├─ 获取新状态 & 相关物体/房间
   └─ 更新导航策略

6. [地标选择]
   ├─ 如果 state == "Broad Search":
   │  └─ 查询 LLM 选择边界节点
   ├─ 如果 state == "Contextual Search":
   │  └─ 查询 LLM 选择相关房间/物体附近的节点
   ├─ 如果 state == "Observe Target":
   │  └─ 查询 LLM 选择目标的观察位置
   └─ 设置 self.global_position = 选择的节点位置

7. [运动规划]
   ├─ 创建规划器输入:
   │  ├─ 障碍物地图 (墙壁)
   │  ├─ 已探索区域
   │  ├─ 当前位姿
   │  ├─ 地图上的目标位置 (self.global_position)
   │  └─ 是否找到目标物体
   ├─ 调用 FMM 规划器获取短期目标
   └─ 确定动作 (前进/转向/停止)

8. [动作执行]
   ├─ 发送动作到 Habitat
   ├─ 接收新观测
   ├─ 从里程计更新位姿
   ├─ 检查碰撞
   └─ 更新成功/失败指标

9. [Episode 终止检查]
   ├─ 如果 done (到达目标或超时):
   │  ├─ 记录: success, SPL, distance
   │  ├─ 重置地图 & 位姿进入下一个 episode
   │  └─ 如果达到 num_eval_episodes 则退出
   └─ 否则继续循环
```

---

## 12. 执行流程图

```
main.py
   ↓
get_args() ← 解析 --scenes TEEsavR23oF
   ↓
Episode.__init__()
   ↓
Episode.start()
   ├─ Episode.init_episode()
   │  ├─ make_vec_envs() → Habitat 环境
   │  ├─ envs.reset() → 初始观测
   │  ├─ sem_map_module.forward() → 首次检测
   │  └─ return init_data_var_dict, init_data_map_dict
   │
   └─ for step in range(num_steps):
       └─ Episode.forward(step, ...)
           ├─ 检查 episode 是否完成
           │
           ├─ sem_map_module() → 检测 & 建图
           │
           ├─ 每 10 步:
           │  ├─ updateSceneGraph() → 构建场景图
           │  ├─ generateVoronoi() → 导航图
           │  └─ projectHistoryToGraph() → 房间分配
           │
           ├─ 查询 LLM:
           │  ├─ state_transition() → 下一状态
           │  └─ landmark_BS/CS/OT() → 下一目标
           │
           ├─ Sem_Exp_Env_Agent.plan_act_and_preprocess()
           │  ├─ _plan() → FMM 规划
           │  ├─ _get_stg() → 短期目标
           │  └─ step() → 执行动作
           │
           └─ 返回新 obs, done, infos
```

---

## 13. 关键数据结构

### MapObjectList (objects)
```python
[{
    'class_id': [openseed_id, ...],
    'class_name': ['bed', 'bed', ...],
    'voxel_index': [3D 索引列表],
    'clip_ft': torch tensor (嵌入),
    'color_path': [看到物体的图像路径],
    'pcd': open3d 点云,
    'bbox': 3D 边界框,
    ...
}, ...]
```

### Voronoi Graph (networkx)
```
节点:
  - id: 唯一整数
  - pos: (x, y) 地图坐标

边:
  - Voronoi 站点之间的连通性
```

### Scene Graph
```python
{
    obj_id_1: {
        obj_id_2: "on",      # 关系
        obj_id_3: "next to"
    },
    ...
}
```

---

## 14. 成功/失败判定

**文件**: `envs/habitat/objectgoal_hm3d.py` - `step()`

从 Habitat 环境：

- **成功**: 智能体在目标物体 1.0m 范围内 (真实值)
- **失败原因**:
  - **探索失败**: 超时 (500 步) 未找到目标
  - **碰撞过多**: 碰撞次数 > 40
  - **规划失败**: 重规划次数 > 20
  - **超时**: 达到最大 episode 长度

记录的指标：
- **SPL** (Success weighted Path Length): `success * (optimal_length / actual_length)`
- **到目标距离**: episode 结束时的欧氏距离
- **成功率**: 在阈值内找到目标的 episode 百分比

---

## 15. 环境变量配置

```bash
# LLM 配置
QWEN_API_KEY=...           # Qwen API 密钥
QWEN_MODEL=qwen-vl-max     # 视觉模型
QWEN_TEXT_MODEL=qwen-plus  # 纯文本模型
INFERENCE_API_URL=https://dashscope.aliyuncs.com/compatible-mode/v1

# 远程推理 (可选)
USE_REMOTE_INFERENCE=0     # 1 = 使用本地 CogVLM2 服务器
USE_REMOTE_OPENSEED=1      # 1 = 使用远程 OpenSeeD (Colab)
LLM_API_URL=...
OPENSEED_API_URL=...
```

---

## 16. 输出文件

保存在 `results/{scene_id}/{skip_times}/`:

| 目录 | 内容 |
|------|------|
| `image/` | 原始 RGB 帧 |
| `detection/` | OpenSeeD 检测可视化 |
| `som/` | 语义物体地图图像 |
| `visual/` | 最终组合可视化 |
| `map/` | 路径规划地图 |
| `episodes/` | 完整可视化 (RGB + 分割 + 地图) |

---

## 附录: 文件路径索引

| 模块 | 文件路径 |
|------|----------|
| 入口点 | `main.py` |
| Episode 管理 | `episode.py` |
| 参数解析 | `utils/arguments.py` |
| 智能体 | `agents/sem_exp.py` |
| 语义建图 | `model/Semantic_Mapping_SG.py` |
| 物体检测 | `detection/openseed.py` |
| 远程推理 | `detection/remote_inference.py` |
| SLAM | `slam/cfslam.py` |
| 场景图查询 | `scenegraph/querypath.py` |
| LLM 接口 | `scenegraph/qwen3.py` |
| Voronoi 图 | `utils/voronoi.py` |
| Habitat 环境 | `envs/habitat/objectgoal_hm3d.py` |
| 可视化 | `agents/utils/visualization.py` |
