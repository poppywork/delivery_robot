#!/usr/bin/env bash
# =============================================================================
# run_move_grasp_demo.sh —— 一键启动「底盘移动 + 识别抓取 + 运送放置」全流程
#
# 1. 加载 ROS 2 Humble 与本工作区（colcon install）
# 2. 设定 ROS_DOMAIN_ID（默认 20，与已运行的 GPU 模型容器一致）
# 3. 用 ros2 launch 启动：相机 + graspgen-X 宿主节点 + GPU 容器(已运行则复用)
#    + move_grasp_demo 节点
#
# 用法：
#     ./run_move_grasp_demo.sh                      # 一键全流程
#     ./run_move_grasp_demo.sh dry_run:=true        # 无硬件演示流程
#     ./run_move_grasp_demo.sh delivery_marker:=xx task_marker:=yy base_url:=http://...
# =============================================================================
set -e
cd "$(dirname "$0")"   # 定位到 graspgen-X_v1_bottle_vision 根目录

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/.roslog}"

echo "[move_grasp_demo] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  工作区=$(pwd)"
exec ros2 launch move_grasp_demo move_grasp_demo.launch.py "$@"