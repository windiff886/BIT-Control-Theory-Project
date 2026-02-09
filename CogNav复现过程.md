# CogNav 复现过程（基于当前仓库版本）

本文档对应仓库：

- 根目录：`/home/windiff/Code/BIT-Control-Theory-Project`
- CogNav 代码目录：`CogNav_ObjNav/`
- 入口脚本：`CogNav_ObjNav/main.py`

目标：在当前代码基础上，尽可能稳定地复现 CogNav 在 HM3D ObjectNav 任务上的单场景运行结果，并能批量统计 `success / SPL / distance_to_goal`。

## 1. 先说结论与推荐路线

当前仓库里，最容易跑通的是：

1. OpenSeeD 用远程推理（Colab/服务器）
2. LLM 用 Qwen API（`scenegraph/qwen3.py`）
3. 本地只负责 Habitat 仿真、建图、规划和结果落盘

对应环境变量组合（推荐）：

```bash
USE_REMOTE_OPENSEED=1
USE_REMOTE_INFERENCE=0
```

原因：

1. 本地 OpenSeeD + 本地 CogVLM2 对显存、依赖和编译环境要求都很高
2. 当前仓库已经内置 `configs/remote_inference.env` 与远程调用逻辑（`detection/remote_inference.py`）

## 2. 代码路径与运行机制（复现前必读）

核心执行链路：

1. `main.py` 解析参数并创建 `Episode`
2. `episode.py` 驱动时间步循环
3. `model/Semantic_Mapping_SG.py` 做语义建图与物体融合
4. `detection/openseed.py` 负责 OpenSeeD 检测（本地或远程）
5. `scenegraph/qwen3.py` 负责状态转移、目标确认、节点选择等 LLM 查询
6. `agents/sem_exp.py` 负责局部规划与动作执行

关键输出目录（默认 `-d results/`）：

`results/{scene}/{skip_times}/`

其中常见子目录：

1. `image/` 原始 RGB
2. `visual/` 分割可视化
3. `som/` 关系推理相关中间图
4. `pcd/` 点云与体素可视化
5. `map/` Voronoi/图相关输出
6. `episodes/` 拼接可视化大图
7. `train.log` 运行日志

## 3. 环境准备（Ubuntu 22.04 + Python 3.8）

### 3.1 系统依赖

```bash
sudo apt-get update
sudo apt-get install -y \
  git wget curl unzip \
  build-essential cmake pkg-config \
  libjpeg-dev libpng-dev libgl1-mesa-dev libegl1-mesa-dev \
  libglib2.0-0 libsm6 libxext6 libxrender-dev \
  ffmpeg ninja-build
```

### 3.2 Conda 环境

```bash
conda create -n cognav python=3.8 -y
conda activate cognav
pip install -U pip setuptools wheel
```

### 3.3 安装 Habitat（按仓库 README 推荐）

```bash
# habitat-sim
git clone https://github.com/facebookresearch/habitat-sim.git
cd habitat-sim
git checkout tags/challenge-2022
pip install -r requirements.txt
python setup.py install --headless
cd ..

# habitat-lab
git clone https://github.com/facebookresearch/habitat-lab.git
cd habitat-lab
git checkout tags/challenge-2022
pip install -e .
cd ..
```

### 3.4 安装 PyTorch（README 对齐版本）

```bash
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=11.8 -c pytorch -c nvidia -y
```

### 3.5 安装 CogNav 项目依赖

```bash
cd /home/windiff/Code/BIT-Control-Theory-Project/CogNav_ObjNav
pip install -r requirements.txt
```

`detectron2` 不在 `requirements.txt` 中，但 `OpenSeeD` 依赖它，建议补装：

```bash
pip install 'git+https://github.com/facebookresearch/detectron2.git'
```

## 4. OpenSeeD CUDA 扩展编译

### 4.1 编译命令

```bash
cd /home/windiff/Code/BIT-Control-Theory-Project/CogNav_ObjNav/openseed/body/encoder/ops
bash make.sh
```

### 4.2 若 PyTorch 2.x 报 `value.type()` 相关错误

文件：`openseed/body/encoder/ops/src/cuda/ms_deform_attn_cuda.cu`

把以下两处替换：

1. `value.type()` -> `value.scalar_type()`

可直接执行：

```bash
cd /home/windiff/Code/BIT-Control-Theory-Project/CogNav_ObjNav
sed -i 's/AT_DISPATCH_FLOATING_TYPES(value.type()/AT_DISPATCH_FLOATING_TYPES(value.scalar_type()/g' \
  openseed/body/encoder/ops/src/cuda/ms_deform_attn_cuda.cu

cd openseed/body/encoder/ops
bash make.sh
```

## 5. 数据准备（HM3D）

## 5.1 当前仓库内已有内容

当前 `CogNav_ObjNav/data/` 下已包含：

1. `objectgoal_hm3d/`（episode 标注）
2. `versioned_data/hm3d-1.0/`（含部分 minival 场景资源）
3. `scene_datasets/hm3d_v0.2/hm3d -> ../../versioned_data/hm3d-1.0/hm3d`（软链接）

注意：目前只看到少量 `minival` 场景资源完整（例如 `TEEsavR23oF`）。如果跑其他 `val` 场景，可能会缺 `.basis.glb/.semantic.glb`。

### 5.2 验证关键文件

```bash
cd /home/windiff/Code/BIT-Control-Theory-Project/CogNav_ObjNav
ls data/scene_datasets/hm3d_v0.2/hm3d/hm3d_annotated_basis.scene_dataset_config.json
find data/versioned_data/hm3d-1.0/hm3d/minival/00800-TEEsavR23oF -maxdepth 1 -type f
```

### 5.3 缺数据时下载（示例）

```bash
python -m habitat_sim.utils.datasets_download \
  --username <api-token-id> \
  --password <api-token-secret> \
  --uids hm3d_minival
```

如果要复现完整 `val` 场景集合，请按 Habitat 官方说明下载对应拆分并放到 `data/scene_datasets/hm3d_v0.2/hm3d/` 可解析的位置。

## 6. 模型准备

### 6.1 OpenSeeD 权重（本地推理时必须）

下载 `openseed_swinl_pano_sota.pt` 后放到：

`CogNav_ObjNav/model/pretrained_models/openseed_swinl_pano_sota.pt`

### 6.2 CogVLM2 权重（本地 LLM 时必须）

下载 `cogvlm2-llama3-chat-19B` 后放到：

`CogNav_ObjNav/model/pretained_model/cogvlm2-llama3-chat-19B`

注意目录名是 `pretained_model`（代码里就是这个拼写）。

## 7. 推理模式配置

配置文件：`CogNav_ObjNav/configs/remote_inference.env`

建议复制一份自己的本地配置，避免误提交密钥：

```bash
cd /home/windiff/Code/BIT-Control-Theory-Project/CogNav_ObjNav
cp configs/remote_inference.env configs/remote_inference.local.env
```

### 7.1 推荐模式（OpenSeeD 远程 + Qwen API）

```bash
export USE_REMOTE_OPENSEED=1
export USE_REMOTE_INFERENCE=0
export OPENSEED_API_URL="https://<your-openseed-ngrok-or-server>"
export QWEN_API_KEY="<your-qwen-key>"
export INFERENCE_API_URL="https://dashscope.aliyuncs.com/compatible-mode/v1"
export QWEN_MODEL="qwen-vl-max"
export QWEN_TEXT_MODEL="qwen-plus"
```

### 7.2 双远程模式（OpenSeeD + CogVLM2 都远程）

```bash
export USE_REMOTE_OPENSEED=1
export USE_REMOTE_INFERENCE=1
export OPENSEED_API_URL="https://<your-openseed-server>"
export LLM_API_URL="https://<your-cogvlm-server>"
```

### 7.3 全本地模式（最重）

```bash
export USE_REMOTE_OPENSEED=0
export USE_REMOTE_INFERENCE=0
```

同时需要确保本地 OpenSeeD/CogVLM2 权重和依赖都齐全。

## 8. 远程服务健康检查（推荐先测）

### 8.1 HTTP 直接检查

```bash
curl -X GET "${OPENSEED_API_URL}/health"
```

如果启用 `USE_REMOTE_INFERENCE=1`，也建议检查：

```bash
curl -X GET "${LLM_API_URL}/health"
```

### 8.2 Python 侧检查（调用仓库代码）

```bash
cd /home/windiff/Code/BIT-Control-Theory-Project/CogNav_ObjNav
python - << 'PY'
from detection.remote_inference import check_api_health
print("OpenSeeD:", check_api_health("openseed"))
print("LLM:", check_api_health("llm"))
PY
```

## 9. 正式运行

所有运行命令都在 `CogNav_ObjNav/` 目录下执行。

### 9.1 单场景（推荐先跑这个）

```bash
cd /home/windiff/Code/BIT-Control-Theory-Project/CogNav_ObjNav
source configs/remote_inference.local.env

python main.py \
  -d results/ \
  --skip_times 0 \
  --scenes TEEsavR23oF \
  --num_processes 1
```

说明：

1. `--skip_times` 用于跳过前 N 个 episode；建议从 `0` 开始
2. 由于当前仓库可见资源以 minival 为主，优先选 `TEEsavR23oF`

### 9.2 批量复现（按 skip_times 扫描）

仓库自带 `run.sh`，但请先创建日志目录：

```bash
cd /home/windiff/Code/BIT-Control-Theory-Project/CogNav_ObjNav
mkdir -p record
bash run.sh
```

`run.sh` 当前默认场景是 `bCPU9suPUw9`。若该场景资产不全，请先改成你本地确认有资源的场景（如 `TEEsavR23oF`）。

## 10. 如何判断复现成功

### 10.1 终端日志

每个 episode 结束会打印类似信息：

```text
scene <scene_id> episode: <skip_times> success: <0/1> dist: <float> spl: <float>
```

### 10.2 结果文件

检查以下路径是否持续更新：

1. `results/<scene>/<skip_times>/image/`
2. `results/<scene>/<skip_times>/visual/`
3. `results/<scene>/<skip_times>/episodes/`
4. `results/<scene>/<skip_times>/train.log`

### 10.3 指标文件（eval 统计）

当 `episode.py` 进入最终统计阶段时会写：

1. `results/<scene>/<skip_times>/val_spl_per_cat_pred_thr.json`
2. `results/<scene>/<skip_times>/val_success_per_cat_pred_thr.json`

## 11. 常见问题与排查

### 11.1 场景文件缺失（最常见）

现象：Habitat 报找不到 `.basis.glb` / `.semantic.glb`。  
处理：

1. 先改跑 `TEEsavR23oF` 测试链路
2. 补全 HM3D 对应 split 的数据资源
3. 确认 `data/scene_datasets/hm3d_v0.2/hm3d` 链接正确

### 11.2 OpenSeeD 远程超时

现象：日志出现 `OpenSeeD Request Failed`。  
处理：

1. 检查 `OPENSEED_API_URL` 可访问性
2. 检查远程服务是否仍在线（ngrok 链接可能失效）
3. 降低并发，保持 `--num_processes 1`

### 11.3 Qwen API 调用失败

现象：LLM 返回错误、状态解析回退到默认状态。  
处理：

1. 检查 `QWEN_API_KEY` 是否有效
2. 检查 `INFERENCE_API_URL` 是否为兼容 OpenAI 的地址
3. 核对 `QWEN_MODEL` 与 `QWEN_TEXT_MODEL` 是否可用

### 11.4 OpenSeeD CUDA 扩展编译失败

现象：`MultiScaleDeformableAttention` 构建失败。  
处理：

1. 按第 4.2 节替换 `value.type()` -> `value.scalar_type()`
2. 确认 CUDA 与 PyTorch 版本匹配
3. 重新执行 `bash make.sh`

### 11.5 显存不足 / 推理很慢

处理：

1. 优先走推荐模式（远程 OpenSeeD）
2. 降低并发：`--num_processes 1`
3. 减少可视化开销：可尝试 `--print_images 0 --visualize 0`

## 12. 一键检查清单（建议逐项打勾）

1. 已激活 `conda` 环境且 Python=3.8
2. `habitat-sim` 与 `habitat-lab` challenge-2022 已安装
3. `pip install -r requirements.txt` 成功
4. `detectron2` 已安装
5. `openseed` CUDA 扩展编译成功
6. 数据路径可解析（至少 `TEEsavR23oF` 可用）
7. 远程/本地模型配置与当前模式一致
8. 远程服务 `health` 检查通过
9. `python main.py --skip_times 0 --scenes TEEsavR23oF` 可启动并产生输出
10. `results/.../train.log` 持续写入

---

如果你要追求“论文级完整复现实验”（大规模场景、批量统计、固定随机种子对比），建议在本文件基础上再补两部分：

1. 固定软硬件版本清单（驱动/CUDA/cuDNN/torch/detectron2/habitat commit）
2. 固定场景与 episode 列表（避免 `skip_times` 变化引入偏差）
