# move_grasp_demo

ROS 2 版「底盘移动 + 识别抓取 + 运送放置」全流程包。

抓取位姿不再用 delivery_robot 的老方法（点云聚类），而是调用本工作区
**graspgen-X 管线**（YOLO + MobileSAM + FoundationStereo + GraspGenX）的
Action 服务 `/graspgenx/generate_grasps` 得到 6-DOF grasp pose，
再变换到机械臂基座坐标系执行抓取；底盘导航、机械臂和夹爪(触觉)仍复用
delivery_robot 的驱动。

## 流程

```
安全检查
  -> 底盘导航到取物点 delivery_marker
  -> [graspgen-X] 请求 grasp pose（相机拍摄 -> YOLO/SAM 分割 -> FS 深度 -> GraspGenX）
  -> 相机坐标系位姿 变换到 机械臂基座坐标系
  -> 机械臂移动 -> 夹爪(触觉)闭合抓稳 -> 慢抬 -> 保持位姿
  -> 底盘导航到放置点 task_marker
  -> 递接位姿 -> 等待 -> 夹爪松开 -> 回初始位姿
```

## 依赖

- 宿主机：ROS 2 Humble、`graspgenx_interfaces`（工作区已构建）、
  numpy/requests（已安装）。**不需要新增大库/模型下载**——AI 模型全部跑在
  已有的 GPU 容器里。
- 现场机器人环境：需要 `delivery_robot` 及其驱动依赖（open3d、pyqtgraph、
  PyQt5、pyserial 等，原项目运行环境即已具备）；需要底盘、机械臂、
  夹爪+触觉传感器硬件在线。

## 构建（工作区根目录）

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths repo/ros2 --symlink-install
source install/setup.bash
```

## 一键运行（工作区根目录）

```bash
# 确保 GPU 模型容器已启动（未启动会自动拉起）：
#   已有容器：docker ps | grep graspgenx_ros_   （运行中会被自动复用）

./run_move_grasp_demo.sh

# 常用变体：
./run_move_grasp_demo.sh dry_run:=true                    # 无硬件演示流程
./run_move_grasp_demo.sh delivery_marker:=xx task_marker:=yy
./run_move_grasp_demo.sh start_camera:=false              # 相机已单独启动
./run_move_grasp_demo.sh start_containers:=false          # 只调宿主/节点逻辑
```

等价于：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
source install/setup.bash
ros2 launch move_grasp_demo move_grasp_demo.launch.py
```

## 现场调参

参数集中在 [`config/move_grasp_demo.yaml`](config/move_grasp_demo.yaml)，
含中文注释：取物/放置点位、底盘地址、递接/保持位姿、物体高度、
放置高度计算参数、相机外参文件等。launch 命令行参数优先级更高。

## 内存评估（本工作区实测，2026-09）

| 项目 | 占用 |
| --- | --- |
| 4 个 GPU 模型容器（stereo/grasp/segmentation/viser） | ≈ 5.0 GB RAM |
| 新增：相机 + 3 个宿主节点 + demo 节点 | ≈ 1.3–1.8 GB RAM |
| 新增代码/模型下载 | 无（<100 KB 代码，模型均在已有容器） |

本机 14 GB 内存可容纳（当前可用约 3.6 GB，容器已占约 5 GB；建议关闭其他
大内存程序；显存紧张时用 `start_viser:=false`）。磁盘剩余 23 GB，每次请求
只在 `repo/outputs/ros2/<时间戳>/` 写入数 MB 图片/日志，无压力。

## 注意事项

- `ROS_DOMAIN_ID` 必须与 GPU 容器一致（当前机器为 20）。
- 所有示教位姿（init/hold/handover）务必用现场示教值替换。
- 容器退出时本 launch 不销毁它们（保持模型常驻，复用加速）。
- 机械臂/夹爪驱动在抓取阶段才初始化，dry_run 模式下完全不碰硬件。