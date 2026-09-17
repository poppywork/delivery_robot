#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
move_grasp_demo_node.py
=======================

ROS 2 版「底盘移动 + 识别抓取 + 运送放置」全流程节点。

流程（与 delivery_robot/move_grasp_demo.py 一致，仅抓取位姿来源不同）：

    1. 安全检查（底盘急停/状态）
    2. 底盘导航到 取物点（delivery_marker）
    3. 调用 graspgen-X 管线 Action（/graspgenx/generate_grasps）得到物体 grasp pose
       —— YOLO + MobileSAM + FoundationStereo + GraspGenX（yolo-sam-fs-graspgenx 方式）
    4. 把相机坐标系下的 grasp pose 变换到机械臂基座坐标系
    5. 机械臂移动到抓取点 -> 夹爪(触觉)闭合抓稳 -> 慢抬 -> 保持位姿
    6. 底盘导航到 放置点（task_marker）
    7. 机械臂依次经过递接位姿 -> 等待 -> 夹爪松开 -> 回初始位姿

复用的 delivery_robot 模块（只换掉"识别+位姿估计"这一环）：
    - robotic_arm/   机械臂驱动（RealMan / UR5）
    - armCom.py      机械臂 <-> 夹爪消息队列
    - capGrasp.py    夹爪 + 触觉传感器线程（threadCapGrasp）
    - config/camera_param_ext*.json   相机(光轴) -> 机械臂基座外参

运行方式见包内 README.md 与根目录 run_move_grasp_demo.sh。
"""

import json
import os
import sys
import time

import numpy as np
import requests

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from graspgenx_interfaces.action import GenerateGrasps


# ---------------------------------------------------------------------------
# 默认参数（与 delivery_robot/move_grasp_demo.py 保持一致；现场可在
# config/move_grasp_demo.yaml 中覆盖）
# ---------------------------------------------------------------------------
DEFAULT_BASE_URL = "http://192.168.10.10:9001"      # 底盘服务
DEFAULT_DELIVERY_MARKER = "take_delivery_point"     # 取物点
DEFAULT_TASK_MARKER = "place_coffee_point"          # 放置点
DEFAULT_HOLD_SECONDS = 2.0                          # 递接时等待秒数
DEFAULT_MOVEMENT_TIMEOUT = 3600.0                   # 单段导航超时（秒）
DEFAULT_GRASP_ATTEMPTS = 3                          # 抓取尝试次数（空抓/失败后重试）
DEFAULT_GRIPPER_NAME = "xarm_hand"                  # graspgen-X 夹爪名
DEFAULT_ACTION_TIMEOUT_SEC = 120.0                  # 单次抓取请求管线超时（秒）

# 机械臂示教位姿（现场读取，务必确认；默认值与 move_grasp_demo.py 一致）
DEFAULT_INIT_POSE = [-0.202335, 0.021550, 0.268639, 1.799546, -0.036349, -3.128607]
DEFAULT_HOLD_POSE = [-0.171741, 0.050245, 0.268887, 1.584683, 0.021031, -3.126630]
DEFAULT_HANDOVER_POSES = [
    [-0.243511, 0.468121, 0.337032, 1.547183, -0.017308, -3.108060],
    [-0.240270, 0.499860, 0.204129, 1.591725, 0.013015, -3.010266],
]

# 放置高度计算（与旧流程一致）
DEFAULT_TABLE_CLEARANCE = 0.24   # 递接位姿1 距桌面高度（米），用于反推桌面高度
DEFAULT_RELEASE_GAP = 0.01       # 物体底部与桌面保留间隙（米）
DEFAULT_OBJECT_HEIGHT = 0.10     # 物体高度（米），用于估算"物体底部高度"，需现场校准

# 抓取点阈值修正（单位米，来自 config/config.json robot_arms）
DEFAULT_THRESHOLDS = {"x": 0.001, "y": -0.005, "z": -0.015}

# 夹爪抓稳 / 空抓等待超时（秒）
GRASP_WAIT_TIMEOUT = 60.0
RELEASE_WAIT_TIMEOUT = 30.0


# ---------------------------------------------------------------------------
# 几何小工具（只用 numpy，避免额外依赖）
# ---------------------------------------------------------------------------
def quat_to_rot(q_xyzw):
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_euler_xyz(R):
    """旋转矩阵 -> 欧拉角 [rx, ry, rz]（内旋 x-y-z / RPY，与 RealMan SDK 约定一致）。"""
    rx = np.arctan2(R[2, 1], R[2, 2])
    ry = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    rz = np.arctan2(R[1, 0], R[0, 0])
    return np.array([rx, ry, rz])


# ---------------------------------------------------------------------------
# 底盘导航客户端（与 move_grasp_demo.py 的 ChassisClient 相同，带网络重试）
# ---------------------------------------------------------------------------
class ChassisClient:
    def __init__(self, base_url, request_timeout=10):
        self.base_url = base_url.rstrip("/")
        self.request_timeout = request_timeout
        self.session = requests.Session()

    def get_status(self, retries=3, backoff=1.0):
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                resp = self.session.get(
                    f"{self.base_url}/api/robot_status", timeout=self.request_timeout
                )
                resp.raise_for_status()
                return resp.json()
            except requests.exceptions.RequestException as exc:
                last_exc = exc
                if attempt < retries:
                    time.sleep(backoff)
        raise last_exc

    def move_to(self, marker, retries=3, backoff=2.0):
        last_exc = None
        for attempt in range(1, retries + 1):
            try:
                resp = self.session.get(
                    f"{self.base_url}/api/move",
                    params={"marker": marker},
                    timeout=self.request_timeout,
                )
                resp.raise_for_status()
                result = resp.json()
                if result.get("status") not in (None, "OK"):
                    raise RuntimeError(
                        f"底盘拒绝移动指令 {marker}: "
                        f"{result.get('status')} {result.get('error_message', '')}".strip()
                    )
                return result
            except requests.exceptions.RequestException as exc:
                last_exc = exc
                if attempt < retries:
                    time.sleep(backoff)
        raise last_exc

    def wait_until_arrived(self, marker, movement_timeout, poll_interval=1.0):
        started = time.monotonic()
        while time.monotonic() - started < movement_timeout:
            status = self.get_status()
            results = status.get("results", {})
            error_code = results.get("error_code")
            if error_code not in (None, "", "00000000"):
                raise RuntimeError(
                    f"机器人错误：{error_code} {results.get('error_message', '')}".strip()
                )
            if results.get("move_status") == "succeeded" and results.get("move_target") == marker:
                self._print_status(results)
                return
            self._print_status(results)
            time.sleep(poll_interval)
        raise TimeoutError(f"等待到达 {marker} 超时（{movement_timeout} 秒）")

    @staticmethod
    def _print_status(results):
        print(
            f"  楼层={results.get('current_floor')}, 运行={results.get('running_status')}, "
            f"移动={results.get('move_status')}, 目标={results.get('move_target')}"
        )


# ---------------------------------------------------------------------------
# 主节点
# ---------------------------------------------------------------------------
class MoveGraspDemoNode(Node):
    def __init__(self):
        super().__init__("move_grasp_demo")

        # ---------- 参数（launch 传入，或 config/move_grasp_demo.yaml 默认值） ----------
        self.declare_parameter("delivery_robot_dir", "/home/zrk/tashan/delivery_robot")
        self.declare_parameter("base_url", DEFAULT_BASE_URL)
        self.declare_parameter("delivery_marker", DEFAULT_DELIVERY_MARKER)
        self.declare_parameter("task_marker", DEFAULT_TASK_MARKER)
        self.declare_parameter("hold_seconds", DEFAULT_HOLD_SECONDS)
        self.declare_parameter("movement_timeout", DEFAULT_MOVEMENT_TIMEOUT)
        self.declare_parameter("grasp_attempts", DEFAULT_GRASP_ATTEMPTS)
        self.declare_parameter("gripper_name", DEFAULT_GRIPPER_NAME)
        self.declare_parameter("action_timeout_sec", DEFAULT_ACTION_TIMEOUT_SEC)
        self.declare_parameter("init_pose", json.dumps(DEFAULT_INIT_POSE))
        self.declare_parameter("hold_pose", json.dumps(DEFAULT_HOLD_POSE))
        self.declare_parameter("handover_poses", json.dumps(DEFAULT_HANDOVER_POSES))
        self.declare_parameter("table_clearance", DEFAULT_TABLE_CLEARANCE)
        self.declare_parameter("release_gap", DEFAULT_RELEASE_GAP)
        self.declare_parameter("object_height", DEFAULT_OBJECT_HEIGHT)
        self.declare_parameter("extrinsic_file", "auto")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("use_grasp_orientation", False)

        self.delivery_robot_dir = self._s("delivery_robot_dir")
        self.base_url = self._s("base_url")
        self.delivery_marker = self._s("delivery_marker")
        self.task_marker = self._s("task_marker")
        self.hold_seconds = self._f("hold_seconds")
        self.movement_timeout = self._f("movement_timeout")
        self.grasp_attempts = int(self._f("grasp_attempts"))
        self.gripper_name = self._s("gripper_name")
        self.action_timeout_sec = self._f("action_timeout_sec")
        self.init_pose = self._j("init_pose")
        self.hold_pose = self._j("hold_pose")
        self.handover_poses = self._j("handover_poses")
        self.table_clearance = self._f("table_clearance")
        self.release_gap = self._f("release_gap")
        self.object_height = self._f("object_height")
        self.dry_run = bool(self._j("dry_run"))
        self.use_grasp_orientation = bool(self._j("use_grasp_orientation"))

        # 阈值修正（默认从 delivery_robot config 读取，可在 YAML 覆盖）
        self.thresholds = DEFAULT_THRESHOLDS
        self.x_threshold = float(self.thresholds["x"])
        self.y_threshold = float(self.thresholds["y"])
        self.z_threshold = float(self.thresholds["z"])

        # ---------- 句柄 ----------
        self._action = ActionClient(self, GenerateGrasps, "/graspgenx/generate_grasps")
        self._chassis = None if self.dry_run else ChassisClient(self.base_url)

        self._arm = None
        self._gripper_thread = None
        self._gripper_started = False

        # 抓取阶段记录（放置高度计算用）
        self.grasp_z = None
        self.object_bottom_z = None
        self.grasp_bottom_delta = None

        # 相机 -> 机械臂基座外参（延迟加载）
        self._ext_R = None
        self._ext_t = None

        self._log("节点初始化完成")
        self._log(f"取物点={self.delivery_marker}  放置点={self.task_marker}  "
                  f"dry_run={self.dry_run}")

    # ---------------- 参数解析小工具 ----------------
    def _s(self, name):
        return str(self.get_parameter(name).value)

    def _f(self, name):
        v = self.get_parameter(name).value
        return float(v) if not isinstance(v, (int, float)) else float(v)

    def _j(self, name):
        """JSON 参数：既能读 YAML 里的真 JSON 类型，也能读 launch 传来的字符串。"""
        v = self.get_parameter(name).value
        if isinstance(v, str):
            try:
                return json.loads(v)
            except (ValueError, TypeError):
                return v
        return v

    def _log(self, msg):
        self.get_logger().info(msg)

    # ======================================================================
    # 1. 底盘
    # ======================================================================
    def _phase_safety_check(self):
        self._log("[Phase 0] 底盘安全检查 ...")
        results = self._chassis.get_status().get("results", {})
        if results.get("estop_state") is True:
            raise RuntimeError("机器人处于急停状态，取消执行。")

    def _phase_navigate(self, marker, label):
        self._log(f"[Phase] 底盘前往 {label}：{marker}")
        self._chassis.move_to(marker)
        self._chassis.wait_until_arrived(marker, self.movement_timeout)
        self._log(f"[Phase] 已到达 {label}：{marker}")

    # ======================================================================
    # 2. 通过 graspgen-X 管线获取 grasp pose（替代旧识别方法）
    # ======================================================================
    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self._log(f"[graspgen-x] stage={fb.stage} progress={fb.progress:.0%}")

    def _request_grasp_pose(self):
        """调用 /graspgenx/generate_grasps，返回 (位置[3], 四元数[4], 分数)。"""
        if not self._action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "graspgen-X Action 服务不可用：请确认相机/宿主节点/GPU 容器已启动"
                "（ros2 node list 应能看到 /graspgenx/pipeline_manager）"
            )

        goal = GenerateGrasps.Goal()
        goal.gripper_name = self.gripper_name
        goal.complete_clouds = True
        goal.timeout_sec = float(self.action_timeout_sec)

        self._log(f"发送抓取请求：gripper={goal.gripper_name} ...")
        send_fut = self._action.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=15.0)
        if not send_fut.done():
            raise RuntimeError("发送抓取目标超时。")
        goal_handle = send_fut.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("抓取目标被管线拒绝。")

        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_fut, timeout_sec=float(self.action_timeout_sec) + 60.0
        )
        if not result_fut.done():
            raise RuntimeError(f"等待抓取结果超时（>{self.action_timeout_sec}s）。")
        result = result_fut.result().result  # GenerateGrasps.Result
        if result.success is not True:
            raise RuntimeError(f"抓取请求失败：{result.message}")

        # 管线已按分数排序：取第一个物体的第一个抓取
        best = result.grasps.objects[0].grasps[0]
        p = best.pose.position
        q = best.pose.orientation
        self._log(
            f"得到 grasp pose：pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})m "
            f"score={best.combined_score:.3f} source={best.source}"
        )
        return (
            np.array([p.x, p.y, p.z]),
            np.array([q.x, q.y, q.z, q.w]),
            float(best.combined_score),
        )

    # ======================================================================
    # 3. 相机坐标系 -> 机械臂基座坐标系
    # ======================================================================
    def _load_extrinsics(self):
        """加载相机->机械臂基座外参：p_base = R @ p_cam + t。

        extrinsic_file="auto" 时按 delivery_robot/config/config.json 的相机/夹爪类型选择：
          REALSENSE + DH_ROBOTICS -> camera_param_ext_d435.json
          REALSENSE + CTAG        -> camera_param_ext_d435_CTAG.json
          ORBBEC                  -> camera_param_ext.json
        """
        if self._ext_R is not None:
            return
        config_path = os.path.join(self.delivery_robot_dir, "config", "config.json")
        gripper = ""
        camera = ""
        if os.path.exists(config_path):
            with open(config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            camera = cfg.get("cameras", [{}])[0].get("type", "")
            gripper = cfg.get("gripper", [{}])[0].get("type", "")

        ext_path = self._s("extrinsic_file")
        if ext_path.lower() == "auto":
            if "ORBBEC" in camera:
                ext_path = os.path.join(self.delivery_robot_dir, "config", "camera_param_ext.json")
            elif "CTAG" in gripper:
                ext_path = os.path.join(self.delivery_robot_dir, "config", "camera_param_ext_d435_CTAG.json")
            else:
                ext_path = os.path.join(self.delivery_robot_dir, "config", "camera_param_ext_d435.json")
        if not os.path.exists(ext_path):
            raise RuntimeError(f"相机外参文件不存在：{ext_path}")

        with open(ext_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        index = str(0)  # 当前现场只有第 1 台相机（D435）
        self._ext_R = np.array(data[index]["r"], dtype=float)
        self._ext_t = np.array(data[index]["t"], dtype=float).reshape(3)
        self._log(f"加载相机外参：{ext_path}")

    def _to_base(self, p_cam, q_cam):
        """相机光轴坐标系下的 6D 位姿 -> 机械臂基座坐标系（位置 + 可选欧拉角）。"""
        self._load_extrinsics()
        p_base = self._ext_R @ p_cam + self._ext_t
        eul = None
        if self.use_grasp_orientation and q_cam is not None:
            r_base = self._ext_R @ quat_to_rot(q_cam)
            eul = rot_to_euler_xyz(r_base)
        return p_base, eul

    # ======================================================================
    # 4. 机械臂 / 夹爪（复用 delivery_robot 驱动）
    # ======================================================================
    def _load_hardware(self):
        """延迟导入 delivery_robot 的机械臂/夹爪驱动。"""
        if self._arm is not None:
            return
        d = self.delivery_robot_dir
        if not os.path.isdir(d):
            raise RuntimeError(f"delivery_robot 目录不存在：{d}")
        if d not in sys.path:
            sys.path.insert(0, d)
        os.chdir(d)  # 驱动内部按相对路径读取 config/config.json

        # 读取机器人/夹爪类型
        cfg_path = os.path.join(d, "config", "config.json")
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        arm_type_name = cfg["robot_arms"][0]["type"]

        from robotic_arm.robotic_arm_dal import robotic_arm_register, eRoboticArmType
        from capGrasp import threadCapGrasp
        from armCom import (eArmMoveStep, eMArmCmd, mechanicalArmSendMsg,
                            mechanicalArmReadGraspMsg)

        self._arm_msgs = {
            "send": mechanicalArmSendMsg,
            "read": mechanicalArmReadGraspMsg,
            "ARRIVED": eArmMoveStep.ROBOT_MSG_ARRIVED_OBJECT_POS,
            "SLOW_RAISE": eArmMoveStep.ROBOT_MSG_SLOW_RAISE,
            "STOP_PUTDOWN": eArmMoveStep.ROBOT_MSG_STOP_PUTDOWN,
            "RAISE": eMArmCmd.GRASP_ROBOT_COM_RAISE,
            "RELEASE_DONE": eMArmCmd.GRASP_ROBOT_COM_RELEASE_DONE,
            "NO_OBJ": eMArmCmd.GRASP_ROBOT_COM_NO_OBJ,
        }
        self._gripper_thread = threadCapGrasp

        # 阈值修正量（与 delivery_robot config/config.json 保持一致）
        arm_cfg = cfg.get("robot_arms", [{}])[0]
        self.x_threshold = float(arm_cfg.get("x_threshold", DEFAULT_THRESHOLDS["x"]))
        self.y_threshold = float(arm_cfg.get("y_threshold", DEFAULT_THRESHOLDS["y"]))
        self.z_threshold = float(arm_cfg.get("z_threshold", DEFAULT_THRESHOLDS["z"]))

        # 机械臂速度（RealMan: 位移速度 75 / 缓抬 10；UR5: 0.15 / 0.007）
        if "REALMAN" in arm_type_name:
            self._arm_speed_move = 75
            self._arm_speed_slow = 10
        else:
            self._arm_speed_move = 0.15
            self._arm_speed_slow = 0.007

        # 注册机械臂（第 2 个参数是相机对象，仅用于标定；这里传入一个最小桩对象）
        class _CameraStub:
            num_of_cameras_ = 1

        end_pose = [1.57079, 0.0, 3.14159]  # 夹爪朝下的末端姿态
        arm_type = (eRoboticArmType.ROBOTICARM_TYPE_REALMAN
                    if "REALMAN" in arm_type_name
                    else eRoboticArmType.ROBOTICARM_TYPE_UR5)
        self._arm = robotic_arm_register(arm_type, _CameraStub(), end_pose)
        if self._arm is None:
            raise RuntimeError("机械臂驱动注册失败。")
        self._log("机械臂驱动初始化完成")

    def _start_gripper(self):
        """启动夹爪 + 触觉传感器线程（与旧流程一致：回到初始位姿之后再启动）。"""
        if self._gripper_started:
            return
        self._gripper_thread.start()
        self._gripper_started = True
        self._log("夹爪/触觉传感器线程已启动")

    def _wait_arm_ready(self, interval=2.0, timeout=600.0):
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            if self._arm.getArmPose() is not None:
                return
            self._log("[等待] 机械臂链路不可用，请检查网线/机械臂电源 ...")
            time.sleep(interval)
        raise RuntimeError("等待机械臂链路超时。")

    def _back_to_init_pose(self):
        result = self._arm.moveArm(list(self.init_pose), self._arm_speed_move)
        if result is not True:
            raise RuntimeError("机械臂回到初始位姿失败。")

    def _grasp_once(self):
        """单次抓取：请求 grasp pose -> 移动 -> 夹爪抓稳。返回 True 抓稳 / False 空抓。"""
        # 1) graspgen-X 管线给出相机坐标系下的抓取位姿
        p_cam, q_cam, _score = self._request_grasp_pose()

        # 2) 变换到机械臂基座，并叠加阈值修正（与旧流程一致）
        p_base, eul = self._to_base(p_cam, q_cam)
        target = list(p_base)
        target[0] -= self.x_threshold
        target[1] -= self.y_threshold
        target[2] -= self.z_threshold
        self._log(f"[Phase] 抓取点(机械臂基座系)：{[round(v, 4) for v in target]}")

        # 3) 记录高度：抓取点 z 与"物体底部 z"的差值（放置时计算下降量）
        self.grasp_z = float(target[2])
        self.object_bottom_z = self.grasp_z - self.object_height / 2.0
        self.grasp_bottom_delta = self.grasp_z - self.object_bottom_z
        self._log(
            f"[Phase] 高度记录：抓取点 z={self.grasp_z:.4f}m, "
            f"物体底部 z≈{self.object_bottom_z:.4f}m, 差值={self.grasp_bottom_delta:.4f}m"
        )

        # 4) 移动机械臂到目标上方并抓取（复用旧流程的 moveArmToTargets）
        result = self._arm.moveArmToTargets(np.array(target), self._arm_speed_move)
        if result is not True:
            raise RuntimeError("移动机械臂到抓取点失败。")
        self._arm_msgs["send"](self._arm_msgs["ARRIVED"])
        self._log("[Phase] 已到达物体位置，夹爪开始闭合 ...")

        # 5) 等待夹爪抓稳（触觉检测）：RAISE=抓稳，RELEASE_DONE/NO_OBJ=空抓
        deadline = time.monotonic() + GRASP_WAIT_TIMEOUT
        while time.monotonic() < deadline:
            msg = self._arm_msgs["read"](False)
            if msg == self._arm_msgs["RAISE"]:
                self._log("[Phase] 夹爪抓稳（GRASP_ROBOT_COM_RAISE）")
                return True
            if msg in (self._arm_msgs["RELEASE_DONE"], self._arm_msgs["NO_OBJ"]):
                self._log("[Phase] 夹爪空抓（未抓到物体）")
                return False
            time.sleep(0.2)
        raise TimeoutError(f"等待夹爪抓取超时（{GRASP_WAIT_TIMEOUT}s）")

    def _phase_grasp_and_hold(self):
        self._log(f"[Phase] 在 {self.delivery_marker} 识别并抓取物体（抓取后一直保持）...")
        self._wait_arm_ready()
        for attempt in range(1, self.grasp_attempts + 1):
            self._log(f"[Phase] ====== 第 {attempt}/{self.grasp_attempts} 次抓取尝试 ======")
            self._back_to_init_pose()
            try:
                grasped = self._grasp_once()
            except Exception as exc:
                if attempt >= self.grasp_attempts:
                    self._back_to_init_pose()
                    raise RuntimeError(f"第 {attempt} 次尝试失败：{exc}") from exc
                self._log(f"[Phase] 第 {attempt} 次失败（{exc}），回初始位姿重试...")
                continue
            if not grasped:
                if attempt >= self.grasp_attempts:
                    self._back_to_init_pose()
                    raise RuntimeError(f"连续 {self.grasp_attempts} 次空抓，取消抓取。")
                self._log("[Phase] 夹爪空抓，回到初始位姿重新识别抓取...")
                continue

            # 抓稳成功：慢抬 -> 保持位姿
            self._arm_msgs["send"](self._arm_msgs["SLOW_RAISE"])
            if self._arm.moveArmZ(0.045, self._arm_speed_slow) is not True:
                raise RuntimeError("抬升机械臂失败。")
            if self._arm.moveArm(list(self.hold_pose), self._arm_speed_move) is not True:
                raise RuntimeError("移动到保持位姿失败。")
            self._log(f"[Phase] 物体已抓取并保持在位姿：{self.hold_pose}")
            return

        raise RuntimeError("抓取流程结束（未抓取成功）")

    # ======================================================================
    # 5. 放置（递接 -> 等待 -> 松爪）
    # ======================================================================
    def _phase_place_and_release(self):
        self._log(f"[Phase] 到达 {self.task_marker}，开始放置物体 ...")
        self._wait_arm_ready()

        poses = [list(p) for p in self.handover_poses if p is not None]
        if not poses:
            raise RuntimeError("未配置有效的递接位姿（handover_poses）。")

        # 放置高度：位姿1 距桌面 table_clearance -> 反推桌面高度；
        # 位姿2 下降使物体底部停在桌面上方 release_gap 处。
        if len(poses) >= 2:
            table_z = poses[0][2] - self.table_clearance
            if self.grasp_bottom_delta is not None:
                release_z = table_z + self.release_gap + self.grasp_bottom_delta
                drop = poses[0][2] - release_z
                self._log(
                    f"[Phase] 放置高度：桌面 z≈{table_z:.4f}m，位姿2 下降 {drop * 1000:.1f}mm "
                    f"-> z={release_z:.4f}m"
                )
                poses[1][2] = release_z

        for idx, pose in enumerate(poses, start=1):
            self._log(f"[Phase] 移动到递接位姿 {idx}/{len(poses)}：{pose}")
            if self._arm.moveArm(pose, self._arm_speed_move) is not True:
                raise RuntimeError(f"移动到位姿 {idx} 失败。")
            if idx < len(poses):
                time.sleep(1.0)

        # 等待交接，然后通知夹爪线程松爪（触觉/滑移检测会先确认再张开）
        self._log(f"[Phase] 等待 {self.hold_seconds}s 后松开夹爪 ...")
        time.sleep(self.hold_seconds)
        self._arm_msgs["send"](self._arm_msgs["STOP_PUTDOWN"])

        deadline = time.monotonic() + RELEASE_WAIT_TIMEOUT
        released = False
        while time.monotonic() < deadline:
            if self._arm_msgs["read"](False) == self._arm_msgs["RELEASE_DONE"]:
                released = True
                break
            time.sleep(0.2)
        if not released:
            raise TimeoutError(f"等待夹爪松开超时（{RELEASE_WAIT_TIMEOUT}s）")

        self._log("[Phase] 夹爪已松开，物体放置完成。")
        self._back_to_init_pose()
        self._log("[Phase] 机械臂已回到初始位姿。")

    # ======================================================================
    # 主流程
    # ======================================================================
    def run_flow(self):
        if self.dry_run:
            self._dry_run_flow()
            return

        print("\n########## move_grasp_demo 全流程开始 ##########")
        # 0. 安全检查
        self._phase_safety_check()

        # 1. 初始化机械臂 + 夹爪（先回到初始位姿再开始夹爪线程，与旧流程一致）
        self._load_hardware()
        self._wait_arm_ready()
        self._back_to_init_pose()
        self._start_gripper()

        # 2. 底盘前往取物点
        self._phase_navigate(self.delivery_marker, "取物点")

        # 3. 通过 graspgen-X 获取 grasp pose 并抓取、保持
        self._phase_grasp_and_hold()

        # 4. 底盘前往放置点（物体一直抓在手上）
        self._phase_navigate(self.task_marker, "放置点")

        # 5. 递接放置
        self._phase_place_and_release()

        print("########## move_grasp_demo 全流程执行完毕 ##########")

    # ------------------------------------------------------------------
    # dry-run：不连接任何硬件/管线，只演示流程顺序（用于无硬件环境调试）
    # ------------------------------------------------------------------
    def _dry_run_flow(self):
        print("\n########## dry-run（不连接硬件/管线）##########")
        steps = [
            ("安全检查", 1.0),
            ("初始化机械臂 + 夹爪线程", 1.0),
            (f"底盘导航到取物点 {self.delivery_marker}", 1.0),
            ("调用 /graspgenx/generate_grasps 获取 grasp pose（模拟）", 1.0),
            ("机械臂抓取 + 慢抬 + 保持位姿（模拟）", 1.0),
            (f"底盘导航到放置点 {self.task_marker}", 1.0),
            ("递接位姿 -> 等待 -> 松爪 -> 回初始位姿（模拟）", 1.0),
        ]
        for label, duration in steps:
            print(f"[Phase] {label}（等待 {duration}s）")
            time.sleep(duration)
        print("########## dry-run 执行完毕 ##########")

    # ------------------------------------------------------------------
    # 清理（机械臂停止/断开，夹爪线程退出）
    # ------------------------------------------------------------------
    def cleanup(self):
        try:
            if self._arm is not None:
                destroy = getattr(self._arm, "destroy", None) or getattr(self._arm, "destory", None)
                if destroy is not None:
                    destroy()
        except Exception as exc:
            self._log(f"清理机械臂异常：{exc}")
        if self._chassis is not None:
            try:
                self._chassis.session.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = MoveGraspDemoNode()
    exit_code = 0
    try:
        node.run_flow()
    except KeyboardInterrupt:
        print("\n收到中断，流程停止。")
        exit_code = 130
    except Exception as exc:
        node.get_logger().error(f"流程失败：{type(exc).__name__}: {exc}")
        exit_code = 1
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())