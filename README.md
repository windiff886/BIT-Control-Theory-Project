# BIT-Control-Theory-Project
# BIT-Control-Theory-Project

<div align="center">

**基于ROS2的阿克曼结构智能车自然语言控制系统**

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/C%2B%2B-69.4%25-orange.svg)]()
[![Language](https://img.shields.io/badge/Python-16.1%25-blue.svg)]()

</div>

## 📖 项目简介

本项目是北京理工大学控制理论课程项目，实现了一个基于ROS2的阿克曼结构智能车控制系统，支持通过**自然语言指令**控制机器人的移动和导航。系统集成了大语言模型(LLM)API，可以将中文自然语言指令解析为结构化的控制命令，并在Gazebo仿真环境中执行。

### ✨ 核心特性

- 🤖 **阿克曼结构车辆仿真**:  基于Gazebo的真实物理仿真环境
- 🗣️ **自然语言控制**: 支持中文自然语言指令解析和执行
- 🧠 **LLM集成**: 集成DeepSeek/ChatGPT等大语言模型API
- 🎮 **手柄控制**: 支持通过游戏手柄进行实时控制
- 🎯 **多级执行器**: 提供简单和高级两种任务执行模式
- 📊 **轨迹可视化**: Python工具用于轨迹验证和可视化

---

## 🏗️ 系统架构

```
BIT-Control-Theory-Project/
├── src/
│   ├── ackermann_v2/          # 阿克曼车辆模型与仿真
│   │   ├── launch/            # ROS2启动文件
│   │   ├── model/             # URDF/Xacro车辆模型
│   │   ├── config/            # 参数配置文件
│   │   ├── rviz/              # RViz可视化配置
│   │   └── src/               # 手柄控制器源码
│   │
│   └── api_invocation/        # LLM API调用与任务执行
│       ├── src/
│       │   ├── llm_analyze.cpp        # LLM API调用节点
│       │   ├── test_publisher.cpp     # 测试发布节点
│       │   ├── simple_executor.cpp    # 简单任务执行器
│       │   ├── advanced_executor.cpp  # 高级任务执行器
│       │   └── plot_point.py          # 轨迹可视化工具
│       └── config/
│           └── llm_config.yaml        # LLM配置与Prompt
│
├── llm_params. yaml            # LLM参数配置
└── README. md

```

---

## 🚀 快速开始

### 📋 依赖环境

- **操作系统**: Ubuntu 22.04 (推荐)
- **ROS2**: Humble Hawksbill
- **Gazebo**:  Gazebo Classic 或 Gazebo Ignition
- **C++标准**: C++17
- **Python**: Python 3.10+

### 📦 依赖库

```bash
# ROS2核心依赖
sudo apt install ros-humble-rclcpp ros-humble-std-msgs ros-humble-geometry-msgs 
sudo apt install ros-humble-nav-msgs ros-humble-tf2 ros-humble-tf2-geometry-msgs

# Gazebo与仿真
sudo apt install ros-humble-gazebo-ros-pkgs

# 手柄控制
sudo apt install ros-humble-joy

# C++库
sudo apt install libcurl4-openssl-dev libjsoncpp-dev

# Python依赖
pip3 install matplotlib numpy
```

### ⚙️ 编译项目

```bash
# 克隆仓库
git clone https://github.com/windiff886/BIT-Control-Theory-Project.git
cd BIT-Control-Theory-Project

# 编译
colcon build

# 加载环境变量
source install/setup.bash
```

---

## 🎯 使用指南

### 1️⃣ 启动仿真环境

```bash
# 启动Gazebo仿真环境和阿克曼车辆
ros2 launch ackermann_v2 vehicle.launch.py

# 可选参数: 
# - world:  指定世界文件 (默认: empty. sdf)
# - x, y, z: 初始位置坐标
# - R, P, Y: 初始姿态 (roll, pitch, yaw)
```

### 2️⃣ 手柄控制模式

```bash
# 启动手柄控制
ros2 launch ackermann_v2 joystick.launch.py

# 手柄操作: 
# - 左摇杆横向: 控制转向角度
# - 右摇杆纵向: 控制车辆速度
```

### 3️⃣ 自然语言控制模式

#### 启动LLM解析器

```bash
# 启动LLM自然语言解析节点
ros2 run api_invocation llm_analyze --ros-args \
  --params-file src/api_invocation/config/llm_config.yaml
```

#### 启动任务执行器

```bash
# 简单执行器 (开环控制)
ros2 run api_invocation simple_executor

# 或者使用高级执行器 (支持闭环导航)
ros2 run api_invocation advanced_executor
```

#### 发送自然语言指令

```bash
# 方式1: 使用测试发布器
ros2 run api_invocation test_publisher

# 方式2: 手动发布指令
ros2 topic pub /voice_command std_msgs/msg/String \
  "data: '向前走2米然后左转'"
```

---

## 📝 支持的指令类型

系统支持以下几种类型的自然语言指令: 

| 指令类型 | Action | 参数 | 示例 |
|---------|--------|------|------|
| **基础移动** | `move_cmd` | `linear`, `angular`, `duration` | "向前走3秒"<br>"左转2秒" |
| **坐标导航** | `move_to` | `x`, `y` | "去坐标(2,2)"<br>"移动到原点" |
| **机械臂控制** | `control_arm` | `command` (lift/lower) | "抬起机械臂"<br>"放下机械臂" |
| **等待** | `wait` | `duration` | "等待5秒" |
| **停止** | `stop` | - | "停止" |

### 🎨 复杂指令示例

```bash
# 复合任务
"去坐标(2,2)，然后抬起机械臂"

# 曲线绘制
"画一条sinx曲线，等待3s后返回"

# 相对移动
"朝左前方沿与朝向30度方向前进2m"

# 连续动作
"前进5米然后左转90度"
```

---

## ⚙️ 配置说明

### LLM API配置

编辑 `src/api_invocation/config/llm_config.yaml`:

```yaml
llm_analyzer:
  ros__parameters:
    # 使用DeepSeek API (推荐)
    api_key: "your-api-key-here"
    api_url: "https://api.deepseek.com/chat/completions"
    model:  "deepseek-chat"
    
    # 或使用ChatGPT
    # api_url: "https://api.openai.com/v1/chat/completions"
    # model:  "gpt-4"
    
    temperature: 0.7
    max_tokens: 2000
    system_prompt: "你的系统提示词..."
```

### 重要约束

由于本车采用**阿克曼结构**,有以下物理限制:

1. ❌ **无法原地旋转** - 车辆必须有线速度才能转向
2. ✅ 任何转向指令必须同时包含非零线速度
3. ✅ 如果用户只说"左转",系统会自动设置默认线速度(0.5 m/s)

---

## 🔧 开发与调试

### 📊 可视化轨迹

```bash
# 运行轨迹绘制工具
cd src/api_invocation/src
python3 plot_point.py
```

该工具可以帮助你:
- 验证生成的轨迹坐标
- 可视化机器人的运动路径
- 调试复杂的运动规划

### 🛠️ 调试技巧

1. **Prompt调优**:  系统效果主要依赖于LLM的Prompt设计,如果解析结果不符合预期,请调整 `llm_config.yaml` 中的 `system_prompt`

2. **查看日志**:
```bash
ros2 run api_invocation llm_analyze --ros-args \
  --params-file config/llm_config.yaml --log-level DEBUG
```

3. **测试单个指令**:  修改 `test_publisher.cpp` 中的 `test_commands_` 数组来测试特定指令

---

## 📂 ROS2话题

### 订阅话题

| 话题名 | 类型 | 描述 |
|-------|------|------|
| `/voice_command` | `std_msgs/msg/String` | 自然语言指令输入 |
| `/joy` | `sensor_msgs/msg/Joy` | 手柄输入 |

### 发布话题

| 话题名 | 类型 | 描述 |
|-------|------|------|
| `/navigation_command` | `std_msgs/msg/String` | LLM解析后的JSON任务列表 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度控制指令 |
| `/desired_steering_angle` | `std_msgs/msg/Float64` | 期望转向角度 |
| `/desired_velocity` | `std_msgs/msg/Float64` | 期望速度 |

---

## 🤝 贡献指南

欢迎提交Issue和Pull Request! 

1. Fork本仓库
2. 创建你的特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交你的更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开一个Pull Request

---

## 📄 许可证

本项目基于 MIT License 开源 - 详见 [LICENSE](LICENSE) 文件

---

## 🙏 致谢

- ROS2社区
- DeepSeek & OpenAI

---

## 📧 联系方式

- 维护者: windiff886
- 项目链接: [https://github.com/windiff886/BIT-Control-Theory-Project](https://github.com/windiff886/BIT-Control-Theory-Project)

---

<div align="center">

**⭐ 如果这个项目对你有帮助,请给我们一个Star!  ⭐**

</div>
