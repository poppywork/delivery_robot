"""move_grasp_demo.launch.py —— 一键启动「底盘移动 + 识别抓取 + 运送放置」全流程。

启动内容：
    1. RealSense D435 相机（infra1/infra2/color）
    2. graspgen-X 宿主节点（frame_sync / object_cloud / completion / pipeline_manager）
    3. GPU 模型容器（stereo/segmentation/grasp/viser）—— 已运行的容器会被复用，
       不会重复创建，也不会在退出时被你杀掉
    4. move_grasp_demo 节点：底盘导航 -> 请求 grasp pose -> 机械臂抓取 -> 导航 -> 放置

用法：
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=20          # 与已运行的 GPU 容器一致
    ros2 launch move_grasp_demo move_grasp_demo.launch.py

常用参数：
    delivery_marker:=take_delivery_point   task_marker:=place_coffee_point
    base_url:=http://192.168.10.10:9001    runs 相关参数在 config 里
    start_camera:=false    # 相机已单独启动时跳过
    start_containers:=false # 只调试宿主节点/节点逻辑时跳过容器
    dry_run:=true          # 不连接任何硬件，只演示流程
"""

from pathlib import Path
from time import strftime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# graspgen-X 的 GPU 容器角色（run_ros2_gpu_container.sh 支持）
CONTAINER_ROLES = ["stereo", "segmentation", "grasp", "viser"]


def _default_profiles():
    """按相机 USB 链路速度选择流配置：USB 3.x 用 640x480x30，USB 2.x 用 640x480x15。"""
    best = 0.0
    try:
        for entry in Path("/sys/bus/usb/devices").glob("*"):
            try:
                vendor = (entry / "idVendor").read_text().strip().lower()
                product = (entry / "idProduct").read_text().strip().lower()
                speed = float((entry / "speed").read_text().strip())
            except (OSError, ValueError):
                continue
            if vendor == "8086" and product.startswith("0b"):
                best = max(best, speed)
    except OSError:
        pass
    if best >= 5000.0:
        return "640x480x30", "640x480x30"
    return "640x480x15", "640x480x15"  # USB 2.x 或探测不到相机时用低帧率


def _make_run_dir(context, *args, **kwargs):
    """给宿主节点和容器一个共同的批次日志目录（GRASPGENX_RUN_DIR）。"""
    workspace = context.launch_configurations.get("workspace")
    run_dir = Path(workspace) / "repo" / "outputs" / "ros2" / strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"[move_grasp_demo] 本次流程数据将保存到: {run_dir}")
    return [SetEnvironmentVariable("GRASPGENX_RUN_DIR", str(run_dir))]


def _container(workspace, role):
    """启动一个 GPU 容器；已运行的容器会被 run_ros2_gpu_container.sh 自动复用。"""
    return ExecuteProcess(
        cmd=["bash", PathJoinSubstitution([workspace, "run_ros2_gpu_container.sh"]), role],
        output="screen",
        emulate_tty=True,
    )


def generate_launch_description():
    share = FindPackageShare("move_grasp_demo")
    grasp_share = FindPackageShare("graspgenx_ros")

    workspace = LaunchConfiguration("workspace")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    start_camera = LaunchConfiguration("start_camera")
    start_containers = LaunchConfiguration("start_containers")
    start_viser = LaunchConfiguration("start_viser")
    container_delay = LaunchConfiguration("container_delay")
    demo_delay = LaunchConfiguration("demo_delay")
    start_demo = LaunchConfiguration("start_demo")

    # 常用 demo 参数（其余调参在 config/move_grasp_demo.yaml）
    base_url = LaunchConfiguration("base_url")
    delivery_marker = LaunchConfiguration("delivery_marker")
    task_marker = LaunchConfiguration("task_marker")
    dry_run = LaunchConfiguration("dry_run")
    delivery_robot_dir = LaunchConfiguration("delivery_robot_dir")
    gripper_name = LaunchConfiguration("gripper_name")

    default_config = PathJoinSubstitution([share, "config", "move_grasp_demo.yaml"])
    default_infra, default_color = _default_profiles()

    # 相机节点（与 full_pipeline.launch.py 一致）
    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        parameters=[
            {
                "enable_infra1": True,
                "enable_infra2": True,
                "enable_color": True,
                "enable_depth": True,
                "align_depth.enable": True,
                "depth_module.infra_profile": LaunchConfiguration("infra_profile"),
                "depth_module.depth_profile": LaunchConfiguration("infra_profile"),
                "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                "publish_tf": True,
                "log_level": "warn",
            }
        ],
        output="screen",
        condition=IfCondition(start_camera),
    )

    # graspgen-X 宿主节点（frame_sync/object_cloud/completion/pipeline_manager）
    host_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([grasp_share, "launch", "host.launch.py"])
        ),
        launch_arguments={"config": LaunchConfiguration("pipeline_config")}.items(),
    )

    # GPU 模型容器：晚几秒启动，等相机话题/宿主节点就绪；已运行的被自动复用
    containers = TimerAction(
        period=container_delay,
        actions=[
            _container(workspace, "stereo"),
            _container(workspace, "segmentation"),
            _container(workspace, "grasp"),
            TimerAction(
                period=20.0,
                actions=[_container(workspace, "viser")],
                condition=IfCondition(start_viser),
            ),
        ],
        condition=IfCondition(start_containers),
    )

    # demo 节点：晚一点启动，等管线就绪（节点内部还会再等 Action 服务可用）
    demo_node = Node(
        package="move_grasp_demo",
        executable="move_grasp_demo_node",
        name="move_grasp_demo",
        output="screen",
        parameters=[
            default_config,
            {
                "base_url": base_url,
                "delivery_marker": delivery_marker,
                "task_marker": task_marker,
                "dry_run": dry_run,
                "delivery_robot_dir": delivery_robot_dir,
                "gripper_name": gripper_name,
            },
        ],
    )
    demo = TimerAction(
        period=demo_delay,
        actions=[demo_node],
        condition=IfCondition(start_demo),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workspace",
                default_value=EnvironmentVariable(
                    "GRASPGENX_WORKSPACE",
                    default_value="/home/zrk/tashan/graspgen-X_v1_bottle_vision",
                ),
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="20"),
                description="DDS domain，须与 GPU 容器一致",
            ),
            DeclareLaunchArgument(
                "pipeline_config",
                default_value=PathJoinSubstitution(
                    [grasp_share, "config", "pipeline.yaml"]
                ),
            ),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("infra_profile", default_value=default_infra),
            DeclareLaunchArgument("color_profile", default_value=default_color),
            DeclareLaunchArgument("start_containers", default_value="true"),
            DeclareLaunchArgument("start_viser", default_value="false"),
            DeclareLaunchArgument("container_delay", default_value="6.0"),
            DeclareLaunchArgument("start_demo", default_value="true"),
            DeclareLaunchArgument("demo_delay", default_value="8.0"),
            # ---------- demo 常用参数 ----------
            DeclareLaunchArgument("delivery_robot_dir",
                                  default_value="/home/zrk/tashan/delivery_robot"),
            DeclareLaunchArgument("base_url", default_value="http://192.168.10.10:9001"),
            DeclareLaunchArgument("delivery_marker", default_value="take_delivery_point"),
            DeclareLaunchArgument("task_marker", default_value="place_coffee_point"),
            DeclareLaunchArgument("gripper_name", default_value="xarm_hand"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
            SetEnvironmentVariable(
                "ROS_LOG_DIR",
                PathJoinSubstitution([workspace, ".roslog"]),
            ),
            SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", "{message}"),
            OpaqueFunction(function=_make_run_dir),
            camera,
            host_nodes,
            containers,
            demo,
        ]
    )