<div align="center">

# Tomato-Orbbec-Vision

ROS 2 Humble RGB-D 3D target service for Orbbec Gemini 335L

面向番茄采摘机器人的 Orbbec Gemini 335L RGB-D 三维目标检测服务

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?style=flat-square)](https://docs.ros.org/en/humble/)
[![Orbbec](https://img.shields.io/badge/Orbbec-Gemini%20335L-00AEEF?style=flat-square)](https://www.orbbec.com/products/stereo-vision-camera/gemini-335l/)
[![YOLO](https://img.shields.io/badge/Detector-Ultralytics%20YOLO-111F68?style=flat-square)](https://docs.ultralytics.com/)

</div>

## 定位

Tomato-Orbbec-Vision 是番茄采摘系统中的独立视觉子系统，当前阶段只负责 **RGB-D 目标检测与相机坐标系下的三维位置估计**，不负责机械臂坐标转换、手眼标定、抓取姿态生成和采摘任务编排

系统使用 Orbbec 官方 ROS 2 驱动启动 Gemini 335L，通过同步 RGB、注册深度图和彩色相机内参，在收到服务请求时运行 YOLO，并返回当前画面中所有满足条件且深度有效的三维目标

视觉应用层对外只保留一个 ROS 2 Service：

```text
/vision/get_targets
```

Orbbec 驱动仍会发布 RGB、Depth、CameraInfo 和 PointCloud2 等传感器 Topic；其中 PointCloud2 当前不参与目标定位，保留给后续机械臂环境感知使用

## 核心能力

| 能力 | 作用 | 主要入口 |
| --- | --- | --- |
| Gemini 335L Bringup | 启动 RGB、Depth、CameraInfo 与 PointCloud2 | `vision_system.launch.py` |
| RGB-D Synchronization | 缓存最新同步的 RGB / Depth / CameraInfo | `vision_server.py` |
| YOLO Detection | 按请求执行目标检测，不持续占用 GPU 推理 | `detector.py` |
| Depth Validation | 中值深度、有效样本比例与 MAD 稳定性检查 | `rgbd_projector.py` |
| 2D → 3D | 使用相机内参将目标像素投影到相机坐标系 | `rgbd_projector.py` |
| Target Query | 按类别与置信度返回一组有效三维目标 | `/vision/get_targets` |
| PointCloud2 | 提供公共 Orbbec 点云，预留环境感知 | Orbbec Driver |

## 系统结构

```text
Orbbec Gemini 335L
├── RGB Image ──────────────┐
├── Registered Depth ───────┼──> vision_server
└── Color CameraInfo ───────┘        │
                                     │ cache only
                                     │
                              /vision/get_targets
                                     │
                                     ▼
                                  YOLO
                                     │
                              Depth validation
                                     │
                                 2D → 3D
                                     │
                                     ▼
                           TargetPoint[] + Header

Orbbec PointCloud2 ─────────────────────> 后续机械臂环境感知
```

平时 `vision_server` 只缓存最新同步 RGB-D 帧，不运行 YOLO；只有收到 `/vision/get_targets` 请求时才执行一次推理

## 仓库结构

OrbbecSDK_ROS2 使用官方仓库，不在本项目中维护修改版；首次部署时将官方源码拉取到 `src/OrbbecSDK_ROS2/`

```text
Tomato-Orbbec-Vision/
├── README.md
├── best.pt                              # 项目权重；构建时安装到 share/vision_target_server/models/
│
└── src/
    ├── OrbbecSDK_ROS2/                  # 官方依赖，git clone 获取
    │   ├── orbbec_camera/
    │   ├── orbbec_camera_msgs/
    │   └── orbbec_description/
    │
    ├── vision_interfaces/               # ROS 2 Service / Message 定义
    │   ├── msg/
    │   │   └── TargetPoint.msg
    │   └── srv/
    │       ├── GetTargets.srv
    │       └── ManageHarvest.srv         # 运动学采摘场景管理
    │
    └── vision_target_server/            # RGB-D 三维目标服务
        ├── config/
        │   └── vision.yaml
        ├── launch/
        │   ├── vision_system.launch.py
        │   ├── vision_server.launch.py   # 仅视觉服务，不启动或查找 Orbbec 驱动
        │   └── sim_targets.launch.py     # 运动学仿真目标
        └── vision_target_server/
            ├── detector.py
            ├── rgbd_projector.py
            ├── sim_target_server.py
            └── vision_server.py
```

## 使用入口

```text
第一次部署
    ↓
拉取 OrbbecSDK_ROS2 v2-main
    ↓
安装 udev rules 与依赖
    ↓
配置 best.pt / vision.yaml
    ↓
colcon build

单独检查相机
    ↓
list_devices_node
    ↓
gemini_330_series.launch.py

运行完整视觉系统
    ↓
vision_system.launch.py
    ↓
/vision/get_targets

后续机械臂集成
    ↓
手眼标定
    ↓
camera optical frame → base_link
    ↓
目标选择 / 抓取姿态 / PickTask
```

## Quick Start

### 1 获取 OrbbecSDK_ROS2

本项目基于 Orbbec 官方 ROS 2 Wrapper：

https://github.com/orbbec/OrbbecSDK_ROS2

Gemini 335L 推荐使用官方 `v2-main` 分支；官方当前设备支持表中，Gemini 335L 对应启动文件为 `gemini_330_series.launch.py`

在项目根目录执行：

```bash
cd src

git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2-main

cd ../..
```

如果已经拉取过：

```bash
git -C src/OrbbecSDK_ROS2 fetch origin
git -C src/OrbbecSDK_ROS2 checkout v2-main
git -C src/OrbbecSDK_ROS2 pull --ff-only origin v2-main
```

### 2 安装 Orbbec 依赖与 udev rules

先加载 ROS 2 Humble：

```bash
source /opt/ros/humble/setup.bash
```

安装 Orbbec 官方依赖：

```bash
sudo apt update
sudo apt install -y \
  libgflags-dev \
  nlohmann-json3-dev \
  ros-$ROS_DISTRO-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-$ROS_DISTRO-image-publisher \
  ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-diagnostic-updater \
  ros-$ROS_DISTRO-diagnostic-msgs \
  ros-$ROS_DISTRO-statistics-msgs \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-backward-ros \
  libdw-dev \
  libssl-dev \
  mesa-utils \
  libgl1 \
  libgoogle-glog-dev
```

安装 Linux udev rules：

```bash
cd src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
cd ../../../..
```

udev rules 是 Linux 下正常访问 Orbbec 设备的必要步骤

### 3 安装视觉 Python 依赖

YOLO 由 Ultralytics 提供：

```bash
python3 -m pip install --user ultralytics
```

确认可导入：

```bash
python3 -c "import ultralytics; print(ultralytics.__version__)"
```

ROS 2 其余依赖优先交给 rosdep：

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --rosdistro humble
```

### 4 配置模型与视觉参数

视觉配置文件：

```text
src/vision_target_server/config/vision.yaml
```

当前主要参数：

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `color_topic` | `/camera/color/image_raw` | RGB 输入 |
| `depth_topic` | `/camera/depth/image_raw` | 已注册深度输入 |
| `camera_info_topic` | `/camera/color/camera_info` | 彩色相机内参 |
| `service_name` | `/vision/get_targets` | 唯一视觉应用接口 |
| `model_path` | 空字符串 | 通过 ament 定位已安装的项目 `models/best.pt`；非空值优先 |
| `device` | `cpu` | 需要 CUDA 时显式设置设备编号 |
| `default_confidence` | `0.40` | 请求未指定时的默认置信度 |
| `default_max_frame_age_sec` | `0.50` | 默认允许的最大图像年龄 |
| `min_depth_m` | `0.25` | 最小有效深度 |
| `max_depth_m` | `6.00` | 最大有效深度 |
| `max_depth_mad_m` | `0.10` | 深度 MAD 稳定性阈值 |
| `sample_radius_px` | `7` | 目标中心深度采样半径 |

构建前将项目训练的 `best.pt` 放在仓库根目录；构建会将它复制到包的 `share/vision_target_server/models/best.pt`，运行时不依赖源码绝对路径。权重不随 Git 提交；缺文件会使构建或启动明确失败，不会下载或回退到通用 YOLO 模型

需要覆盖默认模型或设备时修改配置：

```yaml
model_path: ""    # 使用已安装的项目权重，也可写本地模型文件路径
device: "cpu"    # NVIDIA CUDA 可改为 "0"
```

当前仓库默认按单台 Orbbec 相机使用，不配置 `/dev/ttyUSB*` 或 `/dev/ttyACM*`；Gemini 335L 是 USB 相机，不是串口设备

多相机场景需要在 `vision_system.launch.py` 中进一步暴露并固定 Orbbec `serial_number` 或 USB 设备选择参数

### 5 构建

在项目根目录：

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

修改 Python 代码后使用 `--symlink-install` 可以减少重复复制安装文件

### 6 检查 Gemini 335L

连接相机后：

```bash
source install/setup.bash
ros2 run orbbec_camera list_devices_node
```

确认能识别设备后，可以先单独启动官方相机节点：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

检查传感器 Topic：

```bash
ros2 topic list | grep camera
```

至少确认 RGB、Depth 和 CameraInfo 正常输出

### 7 启动完整视觉系统

```bash
source install/setup.bash
ros2 launch vision_target_server vision_system.launch.py
```

`vision_system.launch.py` 当前会同时启动：

```text
Orbbec Gemini 330 series driver
├── Color: 1280 × 800 @ 30 Hz
├── Depth: 1280 × 800 @ 30 Hz
├── Depth registration: ON
├── Align target: COLOR
├── Frame sync: ON
├── PointCloud2: ON
└── Colored PointCloud: OFF

vision_server
└── /vision/get_targets
```

应用层不会周期性发布目标 Topic，也不会持续运行 YOLO

### 仅启动视觉服务

已有 RGB、对齐深度与 CameraInfo 输入时，使用不依赖 Orbbec 包的入口：

```bash
ros2 launch vision_target_server vision_server.launch.py
```

可通过 `vision_config:=/absolute/path/vision.yaml` 覆盖配置；三个输入必须有一致的 optical frame、匹配的尺寸与有效采样时间戳。`use_sim_time` 默认 `false`；播放 rosbag 时应使发布方与服务使用相同的时钟

### 运动学仿真目标

运动学仿真使用同一 `GetTargets` 接口，不加载 YOLO 或相机驱动；机械臂由 Robot 的仿真入口另行启动；服务持续按 30 Hz 采样机械臂 TF，返回缓存采样的时间戳，不把请求时间写成图像时间

```bash
ros2 launch vision_target_server sim_targets.launch.py \
  calibration_file:=/absolute/path/handeye.yaml
```

直接运行 `ros2 run vision_target_server sim_target_server --ros-args -p calibration_file:=/absolute/path/handeye.yaml` 也可使用相同参数；保持 `use_sim_time=false`，并与机械臂及编排程序使用同一个 `ROS_DOMAIN_ID`

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `calibration_file` | 必填 | `camera_to_gripper (^gripper T_camera)` 手眼外参文件 |
| `world_frame` / `tool_frame` / `tcp_frame` | `base_link` / `tool0` / `tip` | 世界、外参父系与 TCP |
| `camera_frame` | `camera_color_optical_frame` | 返回坐标系，必须匹配外参子系 |
| `initial_target_distance_m` | `0.30` | 首次 TF 下沿 TCP 局部 Z 放置果实，随后世界位置固定 |
| `world_target` | `[]` | 可覆盖为世界坐标 `[x, y, z]`，单位 m；通过 ROS 参数或 `sim_config` 文件传入 |
| `sample_rate_hz` | `30.0` | 独立于查询的 TF 采样频率 |
| `confidence` / `default_confidence` | `0.95` / `0.40` | 模拟置信度与请求默认阈值 |
| `default_max_frame_age_sec` | `0.50` | 请求未指定时的最大帧年龄 |
| `service_name` | `/vision/get_targets` | 与真实视觉相同的接口 |
| `dropout_start_request` / `dropout_requests` | `2` / `0` | 从第几次有效查询起连续返回几次空目标，0 为关闭 |
| `ambiguous_start_request` / `ambiguous_requests` | `2` / `0` | 从第几次有效查询起连续返回两个相距 10 mm 的成熟果，0 为关闭 |

`sim_targets.launch.py` 支持表中标量 launch 参数；`sim_config:=/absolute/path/sim.yaml` 可提供完整 ROS 参数文件；将 `dropout_requests:=2` 或 `ambiguous_requests:=2` 用于短暂漏检或关联歧义；将 `initial_target_distance_m:=0.50` 用于首次行程预算拒绝场景；请求计数从节点启动后第一次成功且新鲜的查询开始，因此修改场景后应重启该模块，再运行完整流程

固定果实以 `visualization_msgs/msg/Marker` 发布到 `/simulation/targets`，可在 RViz 添加 Marker 显示；模拟目标没有渲染图像，像素、bbox 与深度 MAD 字段保持零值；该入口覆盖 TF、关联与运动闭环，RGB-D/YOLO 仍使用真实视觉服务

服务存在可作为节点就绪探测；目标可用需等待带时间戳的 `world_frame → tool_frame` 与 `world_frame → tcp_frame` TF；尚无样本返回 `NO_FRAME`，停止更新后返回 `STALE_FRAME`，正常漏检返回 `success=true` 的空目标；类别和置信度过滤与真实服务一致

### 多果实采摘场景

设置 `harvest_mode:=true` 后，`/simulation/harvest` 提供 `vision_interfaces/srv/ManageHarvest`，在机械臂到达观察位后调用 `RESET`，以该时刻的新鲜实测 TCP 建立固定果实场景；配置 `harvest_scene_file` 时，篮位使用共享文件中的绝对世界坐标；实际机械臂规划、运动和末端执行器指令由 Robot 流程编排负责

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `harvest_mode` | `false` | 保留原单目标模式；启用后只向视觉查询返回当前选中果实 |
| `fruit_offsets_m` | `[-0.055,0,0.10, 0,0,0.12, 0.055,0,0.10]` | 依次排列的 XYZ，单位 m，相对 `RESET` 记录的观察位 TCP；通过 `sim_config` 传入 |
| `harvest_scene_file` | 空字符串 | 共享采摘场景 YAML，定义世界坐标系、篮底中心与尺寸、槽向与槽距、释放姿态和间隙；与 MoveIt 场景节点使用同一文件；配置后覆盖下面三个相对篮位参数 |
| `basket_offset_frame` | `world` | `world` 使用世界坐标轴并保持篮底水平；`tcp` 使用 `RESET` 记录的观察位 TCP 坐标轴与姿态，供旧场景显式选择；通过 `sim_config` 传入 |
| `basket_offset_m` | `[0.20,0.05,-0.06]` | 放果中心相对 `RESET` 记录的观察位 TCP 原点的 XYZ 偏移，单位 m，坐标轴由 `basket_offset_frame` 决定；各果实沿 `basket_slot_axis` 指定的轴间隔 55 mm；通过 `sim_config` 传入 |
| `basket_slot_axis` | `x_axis` | 使用 `x_axis` 或 `y_axis` 指定篮槽沿所选坐标系的 X 或 Y 轴排列，槽间距 55 mm，篮底长边随排列轴调整 |
| `standoff_m` | `0.08` | RViz 中预接近点距果实中心的距离 |
| `retreat_distance_m` | `0.10` | 撤离点距果实中心的距离，沿 `RESET` 记录的观察位 TCP 的负 Z |
| `grasp_tolerance_m` | `0.006` | 实测 TCP 距果实中心的最大夹取距离 |
| `release_tolerance_m` | `0.01` | 实测 TCP 距对应篮子槽位的最大释放距离 |
| `harvest_service_name` | `/simulation/harvest` | 场景服务名称 |

Robot 的连续采摘仿真工程将 `retreat_distance_m` 覆盖为 `0.06`，并将 `standoff_m` 覆盖为 `0.02`，对应一段连续预接近后的 20 mm 果前停距；表中 `0.08` 仍为视觉节点单独启动时的默认显示值，实际预接近与末段修正距离由 Robot 的流程任务配置决定

该静态运动学仿真工程还将节点和查询的观测有效期覆盖为 3 秒，与坐标变换及目标计算的调度预算一致；样本仍保留真实 TF 采样时间，Robot 仍要求运动停止后的新采样和夹取前连续 3 个新帧确认，真实视觉配置保持独立

流程依次调用 `RESET → SELECT_NEXT → GRASP → RELEASE`，每次释放后再次 `SELECT_NEXT`，直至 `finished=true`；`RESET` 重新记录当前观察位 TCP 并重建场景，取消后可重新开始；`fruit_index` 从 1 起算，0 表示未选中，响应同时给出总果数、入篮数、观察位姿、撤离位姿和当前果实的放置位姿

Robot 完整流程使用 `Bringup/src/robot-profile-sim/assets/harvest_scene.yaml`，篮底在机器人侧方，与 MoveIt BOX 采用相同中心和尺寸；三果沿观察位 TCP 的局部 Y 轴竖排，放果时使用共享文件中的专属工具姿态，撤离位和观察位保留观测姿态

共享文件的 `basket.position` 是水平篮底的中心，`dimensions` 为 XYZ 尺寸，`slot_axis` 和 `slot_spacing_m` 指定各槽位，`release_orientation_xyzw` 为放果工具四元数；释放 TCP 高度为篮底中心 Z 加半厚度、25 mm 果实半径及 `release_clearance_m`，工具到位释放后果实保留实际 XY，并落至篮底上表面；未配置共享文件时沿用相对篮位参数

`GRASP` 只在新鲜实测 TCP 距选中果实满足夹取容差时附着果实；附着后果实随实际 TCP 的位置与姿态运动；`RELEASE` 只在已附着且 TCP 到达对应篮子槽位后增加入篮计数，失败保留原场景状态；这是运动学夹取与释放仿真，不包含物理接触、夹持力或番茄脱离枝梗的动力学

多果场景将全部果实、标签、TCP、目标点、篮子与计数作为一帧 `MarkerArray` 发布到 `/simulation/targets_array`，避免逐条 Marker 更新时球体、标签与计数处在不同场景帧；GRASP 与 RELEASE 返回前立即发布新状态，入篮后果实变绿，标签按果号错开高度

RViz Humble 的 `Marker` 显示在订阅 `/simulation/targets` 时会自动订阅 `/simulation/targets_array`，已有显示配置可继续使用；也可直接添加 `MarkerArray` 显示订阅新话题；整帧使用 Transient Local、深度 1，可在流程开始后打开 RViz，RESET 会清理本仿真命名空间的旧标记；非多果模式仍使用原 `/simulation/targets` 单目标 `Marker`

多果场景在 `RESET` 前不固定果实位置；`home_pose` 始终表示本轮场景的参考观察位，逐果关节返回和最终回零由 Robot 流程定义

## Service API

### `/vision/get_targets`

接口类型：

```text
vision_interfaces/srv/GetTargets
```

查看完整定义：

```bash
ros2 interface show vision_interfaces/srv/GetTargets
```

请求：

```text
string[] class_filter
float32 min_confidence
float32 max_frame_age_sec
```

响应：

```text
uint8 OK=0
uint8 NO_FRAME=1
uint8 STALE_FRAME=2
uint8 BAD_FRAME=3
uint8 INFERENCE_ERROR=4

bool success
uint8 status_code
string message
std_msgs/Header header
vision_interfaces/TargetPoint[] targets
```

一次请求返回 **一组目标**，不是单个目标

例如：

```bash
ros2 service call \
  /vision/get_targets \
  vision_interfaces/srv/GetTargets \
  "{class_filter: ['tomato'], min_confidence: 0.40, max_frame_age_sec: 0.50}"
```

如果 `min_confidence <= 0`，使用 `vision.yaml` 中的 `default_confidence`

如果 `max_frame_age_sec <= 0`，使用 `default_max_frame_age_sec`

### `TargetPoint`

```text
string semantic_label
int32 class_id
float32 confidence
geometry_msgs/Point position
int32[4] bbox_xyxy
int32 pixel_u
int32 pixel_v
float32 depth_mad_m
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `semantic_label` | YOLO 类别名，例如 `tomato` |
| `class_id` | YOLO 类别编号 |
| `confidence` | 检测置信度 |
| `position` | 相机 optical frame 下的目标 XYZ，单位 m |
| `bbox_xyxy` | RGB 图像中的 `[xmin, ymin, xmax, ymax]` |
| `pixel_u / pixel_v` | 用于三维投影的中心像素 |
| `depth_mad_m` | 深度采样的 MAD，单位 m，用于判断深度稳定性 |

所有 `targets[]` 共用 `response.header`

```text
response.header.frame_id
        ↓
表示 targets[].position 所在坐标系
```

在标准 ROS optical frame 中：

```text
+x 向右
+y 向下
+z 向前
```

当前阶段不会转换到 `base_link`

### 返回示例

```yaml
success: true
status_code: 0
message: "OK: 2 target(s); 1 detection(s) skipped for invalid depth"
header:
  stamp: ...
  frame_id: camera_color_optical_frame
targets:
- semantic_label: tomato
  class_id: 0
  confidence: 0.91
  position:
    x: 0.052
    y: -0.031
    z: 0.624
  bbox_xyxy: [412, 255, 518, 364]
  pixel_u: 465
  pixel_v: 309
  depth_mad_m: 0.006
- semantic_label: tomato
  class_id: 0
  confidence: 0.84
  position:
    x: -0.083
    y: 0.017
    z: 0.711
  bbox_xyxy: [603, 281, 691, 370]
  pixel_u: 647
  pixel_v: 325
  depth_mad_m: 0.009
```

## RGB-D 目标生成流程

一次服务请求的内部流程：

```text
最新同步 RGB-D 帧
    ↓
检查 frame age
    ↓
YOLO 检测 N 个目标
    ↓
逐目标取 bbox 中心像素
    ↓
中心邻域深度采样
    ↓
有效样本比例 + Median + MAD
    ↓
无效深度目标剔除
    ↓
CameraInfo 投影
    ↓
得到 M 个有效三维目标
    ↓
class_filter + confidence filter
    ↓
GetTargets.Response
```

通常：

```text
M <= N
```

YOLO 检测成功但深度无效的目标不会进入最终 `targets[]`

## PointCloud2

`vision_system.launch.py` 已打开 Orbbec 普通点云：

```text
enable_point_cloud = true
enable_colored_point_cloud = false
```

点云由 Orbbec 驱动直接发布，视觉服务当前不订阅、不滤波，也不用于番茄三维位置计算

后续机械臂环境感知建议直接复用这一公共 PointCloud2：

```text
Orbbec PointCloud2
    ↓
workspace crop
    ↓
voxel / outlier filter
    ↓
相机 → base_link TF
    ↓
MoveIt Planning Scene / 环境感知
```

实际点云 Topic 名称以当前 Orbbec 驱动版本为准，可通过下面命令确认：

```bash
ros2 topic list | grep -E "points|cloud"
```

## 当前边界

当前版本负责：

```text
Gemini 335L
    ↓
RGB-D
    ↓
YOLO
    ↓
相机坐标系三维目标集合
```

当前版本不负责：

```text
手眼标定
camera → base_link
最佳目标选择
抓取姿态生成
MoveIt 规划
机械臂运动
采摘任务编排
```

推荐后续集成顺序：

```text
视觉服务验收
    ↓
手眼标定
    ↓
TF 接入
    ↓
TargetPoint[] → 目标选择
    ↓
抓取 PoseStamped
    ↓
Tomato-Picker / PickTask
```

## 验收

第一次部署建议按下面顺序检查：

```bash
# 1. 相机是否识别
ros2 run orbbec_camera list_devices_node

# 2. 完整系统是否启动
ros2 launch vision_target_server vision_system.launch.py

# 3. Service 是否存在
ros2 service list | grep vision

# 4. 接口定义是否正确
ros2 interface show vision_interfaces/srv/GetTargets

# 5. 请求一次三维目标
ros2 service call \
  /vision/get_targets \
  vision_interfaces/srv/GetTargets \
  "{class_filter: [], min_confidence: 0.40, max_frame_age_sec: 0.50}"
```

当前视觉子系统的核心验收标准是：

```text
RGB 正常
+ Depth 正常
+ YOLO 检测正确
+ response.header.frame_id 明确
+ targets[].position 稳定且距离正确
```

完成这一阶段后，再进入手眼标定和机械臂坐标系集成

## 上游依赖

- OrbbecSDK ROS2 Wrapper: https://github.com/orbbec/OrbbecSDK_ROS2
- ROS 2 Humble: https://docs.ros.org/en/humble/
- Ultralytics YOLO: https://docs.ultralytics.com/

