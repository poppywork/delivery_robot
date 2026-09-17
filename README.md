# GraspGenX 瓶类视觉抓取 ROS 2 部署指南

本项目将原来的脚本式流程改造成 ROS 2 Action 管线：

```text
RealSense D435
  -> 左右红外/RGB 同步
  -> FoundationStereo 深度
  -> YOLO11 + MobileSAM 实例分割
  -> 物体点云生成与轴对称补全
  -> GraspGenX / GraspMoE 抓取生成
  -> 场景碰撞过滤与排序
  -> GenerateGrasps Action 结果
```

模型依赖继续运行在 Docker 中，宿主机只负责相机、ROS 2 通信、点云处理和
Action 管理。原脚本入口仍然保留，不影响旧流程。

## 1. 已验证环境

推荐使用以下组合：

| 项目 | 版本或要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 x86_64 |
| ROS 2 | Humble Desktop，Python 3.10 |
| GPU | NVIDIA GPU，建议显存不低于 8 GB |
| 驱动 | 支持 CUDA 12.4 的 NVIDIA 驱动 |
| Docker | Docker Engine 24 或更新版本 |
| GPU 容器 | NVIDIA Container Toolkit |
| 相机 | Intel RealSense D435，开启 infra1、infra2 和 color |

当前验证机器使用 RTX 4060 Laptop GPU、PyTorch 2.6.0+cu124。

## 2. 获取代码

```bash
git clone YOUR_REPOSITORY_URL graspgen-X_v1_bottle_vision
cd graspgen-X_v1_bottle_vision
```

主要目录：

```text
repo/ros2/graspgenx_interfaces/   ROS 2 消息和 GenerateGrasps Action
repo/ros2/graspgenx_ros/          ROS 2 节点、launch 和参数
repo/graspgenx/                   GraspGenX Python 源码
run_ros2_gpu_container.sh         GPU 模型容器统一启动脚本
```

## 3. 安装宿主机依赖

先按照 ROS 2 官方文档安装 ROS 2 Humble，然后安装本项目需要的包：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-realsense2-camera \
  ros-humble-sensor-msgs-py \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-pip \
  python3-rosdep \
  python3-scipy
```

首次使用 `rosdep` 时执行：

```bash
sudo rosdep init
rosdep update
```

如果系统提示 `rosdep init` 已执行过，可以忽略该提示。

## 4. 安装 Docker GPU 环境

安装 Docker Engine 和 NVIDIA Container Toolkit，并重启 Docker。安装方式以
NVIDIA 官方文档为准：

- Docker Engine: https://docs.docker.com/engine/install/ubuntu/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

验证 Docker 能访问 GPU：

```bash
docker run --rm --gpus all \
  nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

## 5. 准备模型镜像

ROS 2 管线使用以下镜像名：

| 镜像 | 内容 | 本机验证大小 |
| --- | --- | --- |
| `graspgenx:fs` | PyTorch、FoundationStereo、DINOv2 和 FS 权重 | 约 31 GB |
| `graspgenx:mobilesam` | `graspgenx:fs` 加 Ultralytics、YOLO/MobileSAM | 约 32 GB |
| `graspgenx:infer` | PyTorch、GraspGenX、Trimesh 等推理依赖 | 约 25 GB |

### 本地构建精简镜像（推荐）

如果拿不到发布者提供的镜像包，可以直接在本机重建。精简版只装推理链路需要的
依赖，不包含仿真和训练组件：

```bash
# 1) 推理镜像（基于 pytorch 2.6.0 + CUDA 12.4）
docker build -f docker/Dockerfile.infer -t graspgenx:infer .
docker tag graspgenx:infer graspgenx:e2e

# 2) FoundationStereo 镜像（代码 + 23-51-11 权重 + 离线 DINOv2）
#    权重可从官方 Google Drive 或 hf-mirror 镜像获取后放进构建上下文
docker build -f docker/Dockerfile.fs -t graspgenx:fs <fs-build-context>

# 3) 分割镜像（Ultralytics YOLO + MobileSAM）
docker build -f Dockerfile.mobilesam -t graspgenx:mobilesam .
```

注意：`docker/Dockerfile.infer` 会删除基础镜像自带的 forward-compat
`libcuda`，否则在 GeForce 显卡上会报 CUDA error 804。

这些镜像体积较大，不应提交进 Git。仓库发布者需要通过容器镜像仓库或离线
镜像包提供它们。

### 只用点云生成 grasp pose

如果不需要相机和深度链路，只要“点云文件 + 夹爪 -> grasp pose”，可以直接用：

```bash
./grasp_from_pc.sh <点云文件或目录> <夹爪名> [输出 JSON]

# 例：
./grasp_from_pc.sh repo/assets/sample_data/object_pc/1740787815_545011.json xarm_hand
```

点云文件是含 `pc` 字段（N×3，单位米，相机系）的 JSON；输入目录时会处理目录内
所有 `.json`。输出 JSON 每个物体包含 `top_grasps`（position、quaternion_xyzw、
confidence、source）。

如果发布者提供的是镜像包：

```bash
docker load -i graspgenx-fs.tar
docker load -i graspgenx-mobilesam.tar
docker load -i graspgenx-infer.tar
```

如果镜像位于容器 Registry，拉取后改成本项目使用的名字：

```bash
docker pull YOUR_REGISTRY/graspgenx-fs:VERSION_TAG
docker pull YOUR_REGISTRY/graspgenx-mobilesam:VERSION_TAG
docker pull YOUR_REGISTRY/graspgenx-infer:VERSION_TAG

docker tag YOUR_REGISTRY/graspgenx-fs:VERSION_TAG graspgenx:fs
docker tag YOUR_REGISTRY/graspgenx-mobilesam:VERSION_TAG graspgenx:mobilesam
docker tag YOUR_REGISTRY/graspgenx-infer:VERSION_TAG graspgenx:infer
```

确认镜像存在：

```bash
docker image inspect graspgenx:fs >/dev/null
docker image inspect graspgenx:mobilesam >/dev/null
docker image inspect graspgenx:infer >/dev/null
```

也可以通过环境变量使用其他镜像名：

```bash
export GRASPGENX_FS_IMAGE=YOUR_REGISTRY/graspgenx-fs:VERSION_TAG
export GRASPGENX_MOBILE_SAM_IMAGE=YOUR_REGISTRY/graspgenx-mobilesam:VERSION_TAG
export GRASPGENX_INFER_IMAGE=YOUR_REGISTRY/graspgenx-infer:VERSION_TAG
```

## 6. 下载模型和夹爪资产

安装 Git LFS：

```bash
sudo apt install -y git-lfs
git lfs install
mkdir -p assets_cache/models
```

下载 GraspGenX 模型：

```bash
git clone https://huggingface.co/adithyamurali/GraspGenXModel \
  assets_cache/graspgenx_checkpoints
```

下载夹爪描述：

```bash
git clone https://huggingface.co/datasets/adithyamurali/gripper_descriptions \
  assets_cache/gripper_descriptions
```

使用 MobileSAM 镜像下载 YOLO11n 权重：

```bash
docker run --rm \
  -v "$PWD/assets_cache/models:/models" \
  -w /models \
  graspgenx:mobilesam \
  python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"
```

准备完成后应至少存在：

```text
assets_cache/graspgenx_checkpoints/release/gen/
assets_cache/graspgenx_checkpoints/release/dis/
assets_cache/gripper_descriptions/gripper_descriptions/assets/
assets_cache/models/yolo11n.pt
assets_cache/models/mobile_sam.pt
```

下载 MobileSAM 权重（约 40 MB）：

```bash
curl -fL --retry 3 \
  https://github.com/ChaoningZhang/MobileSAM/releases/download/v1.0/mobile_sam.pt \
  -o assets_cache/models/mobile_sam.pt
```

FoundationStereo 权重已经包含在对应的预构建镜像中；本流程不再下载或使用 SAM2。

如果资产存放在其他位置，启动前设置：

```bash
export GRASPGENX_CHECKPOINT_DIR=/绝对路径/graspgenx_checkpoints
export GRASPGENX_GRIPPER_CFG_DIR=/绝对路径/gripper_descriptions
export GRASPGENX_MODEL_DIR=/绝对路径/models
```

## 7. 构建 ROS 2 工作区

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths repo/ros2 --ignore-src -r -y \
  --skip-keys "ament_python ament_pytest"
colcon build --base-paths repo/ros2 --symlink-install
source install/setup.bash
```

验证接口和节点：

```bash
ros2 interface show graspgenx_interfaces/action/GenerateGrasps
ros2 pkg executables graspgenx_ros
```

每次打开新终端都需要执行：

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/graspgen-X_v1_bottle_vision/install/setup.bash
```

## 8. 配置 ROS 网络

所有宿主进程和容器必须使用相同的 `ROS_DOMAIN_ID`。例如：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

建议将这两行加入所有启动终端。GPU 容器脚本会自动继承这些变量，并使用
`--network host` 加入宿主 ROS 2 DDS 网络。容器会使用
`repo/ros2/fastdds_udp.xml` 强制 Fast DDS 通过 UDPv4 通信，避免容器 root
用户与宿主用户之间的共享内存权限冲突。

## 9. 启动 RealSense

方式一，在宿主机启动：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_color:=true \
  align_depth.enable:=true
```

如果 RealSense 也运行在 Docker 中，该容器同样必须使用 `--network host`、相同
的 `ROS_DOMAIN_ID`，并映射相机 USB 设备。

检查相机话题：

```bash
ros2 topic list | grep camera
```

默认话题配置位于：

```text
repo/ros2/graspgenx_ros/config/pipeline.yaml
```

如果相机命名空间不同，请修改其中的 `left_topic`、`right_topic`、
`color_topic` 和三个 `camera_info` 话题。

## 10. 启动 ROS 2 抓取管线

### 一键启动

推荐用一条 `ros2 launch` 启动全部环节：RealSense 相机、宿主节点、三个 GPU
容器（FoundationStereo、YOLO+MobileSAM、GraspGenX）和 Viser。

```bash
cd /absolute/path/to/graspgen-X_v1_bottle_vision
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch graspgenx_ros full_pipeline.launch.py
```

等三个容器都打印模型加载完成后，另开终端发送抓取请求：

```bash
export ROS_DOMAIN_ID=42
ros2 action send_goal --feedback /graspgenx/generate_grasps \
  graspgenx_interfaces/action/GenerateGrasps \
  "{gripper_name: xarm_hand, complete_clouds: true, timeout_sec: 120.0}"
```

常用参数：

```bash
# 相机已在别的终端启动时，跳过相机
ros2 launch graspgenx_ros full_pipeline.launch.py start_camera:=false
# 只用宿主节点调试
ros2 launch graspgenx_ros full_pipeline.launch.py start_containers:=false
# 不开 Viser
ros2 launch graspgenx_ros full_pipeline.launch.py start_viser:=false
# 手动指定相机流配置（默认按 USB 链路速度自动选择）
ros2 launch graspgenx_ros full_pipeline.launch.py infra_profile:=640x480x30 color_profile:=640x480x30
```

相机流配置会按真实 USB 链路速度自动选择：探测到 USB 3.x（5000 Mbps 及以上）
时使用 `640x480x30`，探测到 USB 2.x（480 Mbps）时自动降级为 `640x480x15`；
万一按 USB 3.0 启动后相机节点立刻退出，还会自动用 USB 2.0 配置重试一次。启动
日志会打印本次判断结果，例如：

```text
[full_pipeline] 相机 USB 链路 480 Mbps (USB 2.x) → 自动降级为 640x480x15
```

退出时脚本会自动 `docker rm -f` 掉本次启动的容器，释放显存。流程跑通后在浏览器
打开 `http://localhost:8080` 查看抓取结果。

也可以使用总控脚本（需要相机已单独启动）：

```bash
./run_ros2_all.sh --once
```

日志保存在 `repo/outputs/ros2_logs/<时间戳>/`。

需要四个终端。所有终端使用相同的 `ROS_DOMAIN_ID`。

终端 1，启动宿主节点和 Action 服务：

```bash
cd /absolute/path/to/graspgen-X_v1_bottle_vision
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch graspgenx_ros host.launch.py
```

终端 2，启动 FoundationStereo：

```bash
cd /absolute/path/to/graspgen-X_v1_bottle_vision
export ROS_DOMAIN_ID=42
./run_ros2_gpu_container.sh stereo
```

终端 3，启动 YOLO + MobileSAM：

```bash
cd /absolute/path/to/graspgen-X_v1_bottle_vision
export ROS_DOMAIN_ID=42
./run_ros2_gpu_container.sh segmentation
```

终端 4，启动 GraspGenX 和碰撞过滤：

```bash
cd /absolute/path/to/graspgen-X_v1_bottle_vision
export ROS_DOMAIN_ID=42
./run_ros2_gpu_container.sh grasp
```

可选终端 5，启动 Viser 实时可视化（不使用 GPU）：

```bash
cd /absolute/path/to/graspgen-X_v1_bottle_vision
export ROS_DOMAIN_ID=42
./run_ros2_gpu_container.sh viser
```

如果提示 `graspgenx_ros_viser` 容器名称已被占用，通常表示 Viser 已经在运行；
再次执行同一命令会自动复用正在运行的容器，不需要手动创建第二个实例。

保持这个终端运行，然后用浏览器打开：

```text
http://localhost:8080
```

Viser 必须在发送 Action 前启动，这样它才能收到同一个 `request_id` 的 RGB 帧、
清理/补全后的物体点云和最终抓取结果。显示内容与 Python Viser 流程一致：只显示
物体点云，不显示地面和背景；可见点使用 RGB 图像颜色，轴对称补全生成的不可见点
使用补全脚本的灰色。抓取姿态按组合分数由红到绿着色，每次请求会自动刷新。如果
从局域网内另一台电脑访问，请使用
`http://宿主机IP:8080`，并允许防火墙访问 TCP 8080 端口。

首次启动模型需要一定时间。看到以下节点后再发送请求：

```bash
ros2 node list
```

预期至少包含：

```text
/graspgenx/frame_sync
/graspgenx/stereo_depth
/graspgenx/segmentation
/graspgenx/object_cloud
/graspgenx/completion
/graspgenx/grasp_inference
/graspgenx/grasp_filter
/graspgenx/pipeline_manager
/graspgenx/viser_visualizer
```

不要在上述分容器模式下同时启动 `pipeline.launch.py`，否则会产生重名节点。

## 11. 请求一次抓取

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 action send_goal --feedback \
  /graspgenx/generate_grasps \
  graspgenx_interfaces/action/GenerateGrasps \
  "{gripper_name: xarm_hand, complete_clouds: true, timeout_sec: 120.0}"
```

结果中包含：

- 目标编号和类别；
- RGB 相机光学坐标系下的 6-DOF 抓取位姿；
- ROS 四元数，顺序为 `x, y, z, w`；
- 网络置信度、接近方向权重、组合分数和碰撞结果。

最终结果同时发布在：

```text
/graspgenx/grasps
```

任一阶段失败时，Action 会立即返回 `ABORTED`，错误原因写在 `message` 中，
不会无提示地一直等待。

### 数据记录位置

`full_pipeline.launch.py` 每次启动会新建一个批次目录，并把本次抓取的所有中间
结果写在里面（目录在启动日志中打印）：

```text
repo/outputs/ros2/<启动时间戳>/
└── request_001/              # 每个抓取请求一个子目录
    ├── frames/               # left/right/color 图像 + camera_info.json
    ├── fs/                   # FoundationStereo 深度（depth_meter.npy、depth_vis.png）
    ├── seg/                  # 分割结果（见下表）
    ├── cloud/                # 物体点云 + 场景点云
    ├── grasp/                # 抓取候选与最终排序结果
    ├── timings/              # 各阶段耗时 JSON（SYNC/FS/YOLO/SAM/…/PIPELINE）
    └── summary.json          # 本次请求的成败、耗时与消息
```

`seg/` 目录保存全部检测与分割产物，便于回看“YOLO 当时到底看到了什么”：

| 文件 | 内容 |
| --- | --- |
| `color.png` | 送入 YOLO 的彩色原图 |
| `yolo_raw.png` | YOLO 自带渲染结果（它自己的框、标签、配色），**0 检出时也会保存** |
| `yolo_raw.json` | 阈值、类别过滤、是否回退，以及 YOLO 在阈值上的全部原始检出（类别、置信度、bbox） |
| `yolo_detections.png` | 管线最终保留目标的标注图 |
| `sam_overlay.png` | 最终 MobileSAM 掩码的彩色叠加图（回退分割时一眼看出掩码质量） |
| `masks/mask_xxx.png` | 逐个物体的二值掩码 |
| `masks/meta.json` | 每个掩码对应的标签、置信度与 bbox |
| `meta.json` | 本次分割汇总：物体数、标签、置信度、YOLO 原始检出数、被丢弃的超大框数、是否回退 |

注意：只有真正执行了一次抓取请求，上述文件才会产生；只启动 launch 而不发请求
时，批次目录保持为空。若 `meta.json` 里 `used_sam_fallback` 为 `true`，说明 YOLO
在该阈值下 0 检出、已回退到 MobileSAM 全图分割，此时应当调整
`repo/ros2/graspgenx_ros/config/pipeline.yaml` 里的 `confidence` / `classes`
或改变相机与被抓物体的相对位置。

关于"整屏大框"：回退分割的标签一律是 `sam<编号>`（不是 YOLO 类别）且置信度为
`0.00`，它们的框是**掩码的真实包围盒**；面积占比 ≥ `max_box_area_ratio`
（默认 `0.95`）的框会被丢弃并记入 `yolo_raw.json` 的 `oversized_detections`。
所以标注图上再不会出现由回退分支伪造的整屏矩形——若仍看到整屏框，那一定是
YOLO 自己的检出，可在 `yolo_raw.json` 里核对类别与 `area_ratio`。

## 12. 启动前自检

下面的命令只检查容器导入、ROS 2 动态库和 DDS，不加载模型权重：

```bash
GRASPGENX_ROS2_CHECK_ONLY=1 ./run_ros2_gpu_container.sh stereo
GRASPGENX_ROS2_CHECK_ONLY=1 ./run_ros2_gpu_container.sh segmentation
GRASPGENX_ROS2_CHECK_ONLY=1 ./run_ros2_gpu_container.sh grasp
```

运行测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q repo/ros2/graspgenx_ros/test

GRASPGENX_ROS2_INTEGRATION=1 python3 -m pytest -q \
  repo/ros2/graspgenx_ros/test/test_pipeline_manager.py
```

## 13. 常见问题

### 容器提示 `Missing ... asset directory`

检查 `assets_cache` 目录结构，或通过 `GRASPGENX_CHECKPOINT_DIR`、
`GRASPGENX_GRIPPER_CFG_DIR` 和 `GRASPGENX_MODEL_DIR` 指定绝对路径。

### `ros2 node list` 看不到容器节点

确认宿主与所有容器的 `ROS_DOMAIN_ID` 一致，并确认没有设置
`ROS_LOCALHOST_ONLY=1`。防火墙需要允许 DDS 的 UDP 通信。

### Action 超时

依次检查：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo --once /graspgenx/frames
```

同时查看四个启动终端中的模型加载或推理错误。

### 提示找不到 `/opt/ros/humble` 或宿主动态库

当前容器包装方案在 Ubuntu 22.04 x86_64、ROS 2 Humble 和 Python 3.10 上
验证通过。其他发行版或架构应把 ROS 2 直接安装进模型镜像，不要挂载不兼容的
宿主 ROS 2 二进制文件。

### CUDA 不可用

```bash
docker run --rm --gpus all graspgenx:fs \
  python3 -c "import torch; print(torch.cuda.is_available())"
```

如果输出为 `False`，检查 NVIDIA 驱动、Docker 和 NVIDIA Container Toolkit。

### CUDA out of memory

FoundationStereo 默认使用 `model_precision: fp16` 常驻 GPU，并从 CPU 加载
检查点，避免加载阶段在 GPU 中保留重复权重。容器脚本同时启用了
`PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`。不要把该参数改回 `fp32`，
否则 8 GB 显卡无法同时常驻 FoundationStereo、MobileSAM 和 GraspGenX。

如果仍然显存不足，先关闭浏览器等占用独显的程序，再重启三个 GPU 容器；可用
`nvidia-smi` 确认没有旧容器进程残留。

### 分割结果为空

调整 `repo/ros2/graspgenx_ros/config/pipeline.yaml` 中的 `confidence`、
`classes`、`min_pixels` 和 `best_only`。修改后重启分割容器。

## 14. 仓库发布者注意事项

要真正实现其他用户 clone 后按文档启动，发布仓库前还必须完成以下工作：

1. 将三个 Docker 镜像推送到可访问的 Registry，或在 Release 页面提供镜像包；
2. 将本 README 中的 `YOUR_REPOSITORY_URL`、`YOUR_REGISTRY` 和
   `VERSION_TAG` 替换成真实值；
3. 不要把模型权重、`assets_cache`、ROS `build/install/log` 提交到 Git；
4. 在一台全新 Ubuntu 22.04 机器上按本文从头验证一次。

模型和镜像属于外部大文件。只有代码仓库、可下载的镜像地址和模型来源三者都
可访问时，才能称为“clone 后可部署”。

## 15. 更多资料

- ROS 2 包说明：`repo/ros2/README.md`
- GraspGenX 上游说明：`repo/README.md`
- ROS 参数：`repo/ros2/graspgenx_ros/config/pipeline.yaml`
- 完整单环境 launch：`repo/ros2/graspgenx_ros/launch/pipeline.launch.py`

## 16. 底盘移动 + 抓取 + 放置一体化流程（move_grasp_demo）

在 graspgen-X 抓取管线之上，新增了 delivery_robot `move_grasp_demo.py` 的
ROS 2 版本：**导航到目标点 → 抓取物体 → 导航 → 放置物体**，其中抓取位姿由
本工作区的 YOLO + MobileSAM + FoundationStereo + GraspGenX 管线给出
（`/graspgenx/generate_grasps`），替代 delivery_robot 原来的点云聚类识别方法。

```bash
# 一键运行（工作区根目录）
./run_move_grasp_demo.sh
# 无硬件演示流程顺序
./run_move_grasp_demo.sh dry_run:=true
```

包位置与说明：

```text
repo/ros2/move_grasp_demo/move_grasp_demo/move_grasp_demo_node.py   全流程节点
repo/ros2/move_grasp_demo/launch/move_grasp_demo.launch.py           一键 launch
repo/ros2/move_grasp_demo/config/move_grasp_demo.yaml                现场调参
repo/ros2/move_grasp_demo/README.md                                  说明与内存评估
```

该 launch 会启动相机 + graspgen-X 宿主节点，并**复用已在运行的 GPU 容器**
（不会重复创建，退出时也不销毁容器）；机械臂、夹爪与触觉传感器驱动仍复用
`delivery_robot`。

## 17. move_grasp_demo 依赖安装（系统 python3.10）

`move_grasp_demo` 复用 delivery_robot 的机械臂 / 夹爪驱动，这些代码跑在**系统
python3.10（与 ROS 2 Humble 同一解释器）**里，因此需要在该解释器补齐第三方库：

```bash
./install_move_grasp_demo_deps.sh            # 装到 ~/.local（推荐，免 sudo）
./install_move_grasp_demo_deps.sh --system   # 装到 /usr/local（需 sudo，多用户共享）
```

最小依赖集（实测得出，已全部验证可同时导入 rclpy）：

| 包 | 版本 | 为什么需要 |
|---|---|---|
| open3d | 0.18.0 | delivery_robot 点云 / 刚体变换 |
| pyqtgraph + PyQt5 | 0.13.7 / 5.15 | capGrasp → GUI → displayDataWindow 显示链 |
| urx + math3d | 0.11.0 / 3.4.1 | robotic_arm_dal 无条件导入 UR 驱动 |
| plotly + dash | 最新 | `import open3d` 连带加载 visualization.draw_plotly |
| scikit-learn | 最新 | `import open3d` → open3d.ml.datasets.semantickitti |
| addict | 最新 | open3d._ml3d.utils.config |
| pyquaternion、pillow≥9.3 | | open3d 声明依赖 |
| pymodbus 3.7.4 + pyudev | | DH 夹爪（本现场）Modbus 串口 |

**坑 1（必看）：numpy 必须停在 1.x。** open3d 的依赖会把 numpy 拉到 2.x（实测
1.24.4 → 2.2.6）；ROS 2 与 graspgen 宿主节点都按 numpy 1.24 验证。安装命令里始终带
`"numpy<2"`（脚本已内置）。

**坑 2：不要 sudo 重装 open3d。** 实测 `~/.local` 在 sys.path 里排在
`/usr/local/lib/python3.10/dist-packages` **之前**，用户级安装即生效；而 sudo 重装
需重新下载 400MB（弱网下 33 kB/s ≈ 3.4 小时）。确需系统级时用 `--system`
（脚本已带 `--cache-dir` 复用下载缓存）。

**不要装**：torch / torchvision / torchaudio / snntorch（约 1 GB+，那是 delivery_robot
老检测方法用的，graspgen-X 已替代）、pygame / pyinstaller / plyfile、jupyter 全家桶
（open3d 声明里有，但导入期实测不需要，故脚本对 open3d 用 `--no-deps`）。

磁盘/内存：open3d 下载 448 MB、解压约 1.1 GB；其余约 80 MB。运行时 open3d 约
+200~300 MB RAM，pyqtgraph + Qt 约 +100~200 MB。

GPU 说明：open3d 的 pip wheel 自带 CUDA 11.7 运行时，导入时探测 CUDA 设备——本机
（RTX 4060 Laptop）有 GPU 走 CUDA 绑定，无 GPU 自动回退 CPU 绑定，两种都能用。
若个别终端里 open3d CUDA 初始化报错（例如导入被 Ctrl+C 打断），可强制 CPU 模式：
`CUDA_VISIBLE_DEVICES="" python3 ...`（delivery_robot 的用法只需 CPU 侧几何计算）。

